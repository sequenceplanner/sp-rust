use super::sp_runner::*;
use super::transition_planner::*;
use super::operation_planner::*;
use crossbeam::{channel, Receiver, Sender};
use failure::Error;
use sp_domain::*;
use sp_formal::CompiledModel;
use sp_ros::*;
use std::collections::HashMap;
use std::panic;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

pub fn launch_model(model: Model, mut initial_state: SPState) -> Result<(), Error> {
    // we use this as the main entry point for SP.
    // so here we register our panic handler to send out
    // fatal messages to ROS
    let default_panic = std::panic::take_hook();
    panic::set_hook(Box::new(move |panic_info| {
        let msg = if let Some(s) = panic_info.payload().downcast_ref::<&str>() {
            s
        } else if let Some(s) = panic_info.payload().downcast_ref::<String>() {
            &s[..]
        } else {
            ""
        };
        let (file, line) = if let Some(location) = panic_info.location() {
            (location.file(), location.line())
        } else {
            ("", 0)
        };
        log_fatal(msg, file, line);
        println!("\n\n\nSP PANIC: {}\n\n\nfile: {}:{}", msg, file, line);

        default_panic(panic_info);
        std::process::exit(1);
    }));

    println!("Model");
    println!("{}", serde_json::to_string_pretty(&model).unwrap());
    println!("********");

    let (mut node, comm) = set_up_ros_comm(&model)?;

    let (tx_runner, rx_runner): (Sender<SPRunnerInput>, Receiver<SPRunnerInput>) =
        channel::unbounded();

    // let (runner_out_tx, runner_out_rx) = watch::channel(RunnerOutput::default());
    let runner_out = Arc::new(Mutex::new(RunnerOutput::default()));

    let resource_list_path = SPPath::from_string("registered_resources");
    resource_handler(
        comm.rx_resources,
        comm.rx_commands,
        tx_runner.clone(),
        &resource_list_path,
    );
    initial_state.add_variable(
        resource_list_path,
        SPValue::Array(SPValueType::Path, vec![]),
    );

    merger(comm.rx_mess.clone(), tx_runner.clone());
    ticker(Duration::from_millis(100), tx_runner.clone());

    runner(
        &model,
        initial_state,
        rx_runner,
        comm.tx_state_out,
        comm.tx_runner_info,
        runner_out.clone(),
    );

    planner(&model, tx_runner.clone(), runner_out.clone());

    loop {
        // blocking ros spinning
        spin(&mut node);
    }
}

/// For the time being, this is what the runner emits.
/// This is then picked up by the various tasks that need access to the state.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RunnerOutput {
    pub state: SPState,
    pub disabled_paths: Vec<SPPath>
}

fn planner(model: &Model, tx_input: Sender<SPRunnerInput>,
           // mut runner_out_rx: watch::Receiver<RunnerOutput>
           runner_out: Arc<Mutex<RunnerOutput>>
) {
    let compiled_model = CompiledModel::from(model.clone()); // TODO: temporarily created here
    let mut transition_planner = TransitionPlanner::from(&compiled_model);
    let mut operation_planner = OperationPlanner::from(&compiled_model);

    let t_runner_out = runner_out.clone();
    let t_tx_input = tx_input.clone();
    thread::spawn(move || {
        // Initiall block all our transitions.
        let block_transition_plan = transition_planner.block_all();
        let cmd = SPRunnerInput::NewPlan("transition_planner".to_string(), block_transition_plan);
        t_tx_input.try_send(cmd).expect("could not send to runner...");
        loop {
            let ro = {
                let ro = t_runner_out.lock().unwrap();
                ro.clone()
            };
            if let Some(plan) = transition_planner.compute_new_plan(ro.state, &ro.disabled_paths) {
                println!("new plan computed");
                let cmd = SPRunnerInput::NewPlan("transition_planner".to_string(), plan);
                t_tx_input.try_send(cmd).expect("could not send to runner...");
            }
        }
    });

    let o_runner_out = runner_out.clone();
    let o_tx_input = tx_input.clone();
    thread::spawn(move || {
        // Initiall block all our transitions.
        let block_operation_plan = operation_planner.block_all();
        let cmd = SPRunnerInput::NewPlan("operation_planner".to_string(), block_operation_plan);
        tx_input.try_send(cmd).expect("could not send to runner...");
        loop {
            let ro = {
                let ro = o_runner_out.lock().unwrap();
                ro.clone()
            };
            if let Some(plan) = operation_planner.compute_new_plan(ro.state, &ro.disabled_paths) {
                println!("new plan computed");
                let cmd = SPRunnerInput::NewPlan("operation_planner".to_string(), plan);
                o_tx_input.try_send(cmd).expect("could not send to runner...");
            }
        }
    });

}

fn runner(
    model: &Model, initial_state: SPState, rx_input: Receiver<SPRunnerInput>,
    tx_state_out: Sender<SPState>, tx_runner_info: Sender<SPState>,
    // runner_out_tx: watch::Sender<RunnerOutput>,
    runner_out: Arc<Mutex<RunnerOutput>>,
) {
    let model = model.clone();
    thread::spawn(move || {
        let mut runner = SPRunner::from(&model, initial_state);

        // TODO: move planner specific setup to the respective planner...

        // planning active or not
        let planner0 = SPPath::from_slice(&["runner", "planner", "0"]);
        let planner1 = SPPath::from_slice(&["runner", "planner", "1"]);
        let planners_initially_on =
            SPState::new_from_values(&[(planner0, true.to_spvalue()), (planner1, true.to_spvalue())]);
        runner.update_state_variables(planners_initially_on);

        // monotonic planning mode
        let mono = SPPath::from_slice(&["runner", "planner", "monotonic"]);
        let monotonic_initially_on = SPState::new_from_values(&[(mono, true.to_spvalue())]);
        runner.update_state_variables(monotonic_initially_on);

        // planning counters
        let planner0_counter = SPPath::from_slice(&["runner", "plans", "0"]);
        let planner0_counter = SPState::new_from_values(&[(planner0_counter, 0.to_spvalue())]);
        runner.update_state_variables(planner0_counter);

        let planner1_counter = SPPath::from_slice(&["runner", "plans", "1"]);
        let planner1_counter = SPState::new_from_values(&[(planner1_counter, 0.to_spvalue())]);
        runner.update_state_variables(planner1_counter);

        let mut now = Instant::now();

        loop {
            let elapsed_ms = now.elapsed().as_millis();
            if elapsed_ms > 1000 {
                log_debug!("RUNNER TICK TIME: {}ms", elapsed_ms);
            }
            let input = rx_input.recv();
            let mut state_has_probably_changed = false;
            let mut ticked = false;
            let mut last_fired_transitions = vec![];
            if let Ok(msg) = input {
                match msg {
                    SPRunnerInput::StateChange(s) => {
                        if !runner.state().are_new_values_the_same(&s) {
                            last_fired_transitions = runner.take_a_tick(s, false);
                            state_has_probably_changed = true;
                        } else {
                            runner.update_state_variables(s);
                        }
                    }
                    SPRunnerInput::NodeChange(s) => {
                        if !runner.state().are_new_values_the_same(&s) {
                            last_fired_transitions = runner.take_a_tick(s, true);
                            state_has_probably_changed = true;
                        }
                    }
                    SPRunnerInput::Tick => {
                        last_fired_transitions = runner.take_a_tick(SPState::new(), true);
                        ticked = true;
                    }
                    SPRunnerInput::NewPlan(plan_name, plan) => {
                        runner.set_plan(plan_name, plan);
                    }
                }
            } else {
                println!("The runner channel broke? - {:?}", input);
                break;
            }

            now = Instant::now();

            //println!("tick: {} ms", timer.elapsed().as_millis());

            if !last_fired_transitions.is_empty() {
                println!("fired:");
                last_fired_transitions
                    .iter()
                    .for_each(|x| println!("{:?}", x));
            }

            // if there's nothing to do in this cycle, continue
            if !state_has_probably_changed && last_fired_transitions.is_empty() && !ticked {
                continue;
            } else {
                // println!("state changed? {}", state_has_probably_changed);
                // println!("transition fired? {}", !runner.last_fired_transitions.is_empty());
                // println!("ticked? {}", ticked);
            }

            let disabled = runner.disabled_paths();
            let mut enabled_state = runner.state().projection();
            let l1 = enabled_state.state.len();
            enabled_state.state.retain(|(p, v)| {
                !disabled.iter().any(|d| p.is_child_of(d)) && v.current_value() != &SPValue::Unknown
            });
            let disabled_states = enabled_state.state.len() < l1;
            tx_state_out
                .send(enabled_state.clone_state())
                .expect("tx_state:out");

            // send out runner info.
            let mut runner_modes = vec![];
            if disabled_states {
                runner_modes.push("resource(s) offline".to_spvalue());
            }
            // if planning_state.bad_state {
            //     runner_modes.push("bad state".to_spvalue());
            // }
            runner.ticker.state.add_variable(
                SPPath::from_string("mode"),
                SPValue::Array(SPValueType::String, runner_modes),
            );
            let runner_info = runner.state().clone();

            tx_runner_info.send(runner_info).expect("tx_runner_info");


            // send out the state to the planning task
            let output = RunnerOutput {
                state: runner.state().clone(),
                disabled_paths: runner.disabled_paths().clone()
            };

            println!("sending out runner state...");
            (*runner_out.lock().unwrap()) = output;

            if disabled_states {
                println!("still waiting... do nothing");
                continue;
            }
        }
    });
}

struct RosCommSetup {
    rx_mess: Receiver<RosMessage>,
    rx_commands: Receiver<SPState>,
    rx_resources: Receiver<ROSResource>,
    tx_state_out: Sender<SPState>,
    tx_runner_info: Sender<SPState>,
}

fn set_up_ros_comm(model: &Model) -> Result<(RosNode, RosCommSetup), Error> {
    // start ros node
    let mut node = start_node()?;

    // data from resources to runner
    // setup ros pub/subs. tx_out to send out to network
    let (tx_in, rx_mess) = channel::bounded(20);
    let tx_state_out: channel::Sender<SPState> = roscomm_setup(&mut node, model, tx_in)?;

    // misc runner data to/from the network.
    // setup ros pub/subs. tx_out to send out to network
    let (tx_in_misc, rx_commands) = channel::bounded(20);
    let tx_runner_info = roscomm_setup_misc(&mut node, tx_in_misc)?;

    let (tx_in_resources, rx_resources) = channel::unbounded();
    ros_resource_comm_setup(&mut node, tx_in_resources, model.path())?;

    Ok((
        node,
        RosCommSetup {
            rx_mess,
            rx_commands,
            rx_resources,
            tx_state_out,
            tx_runner_info,
        },
    ))
}

fn resource_handler(
    rx_resources: Receiver<ROSResource>, rx_commands: Receiver<SPState>,
    tx_runner: Sender<SPRunnerInput>, registered_resources: &SPPath,
) {
    fn ros_resource(
        msg: Result<ROSResource, channel::RecvError>, resources: &mut Vec<SPPath>,
        registered_resources: &SPPath, tx_runner: Sender<SPRunnerInput>,
    ) -> bool {
        match msg {
            Ok(x) => {
                //log_info!("Got resource: {:?}", x);
                if !resources.contains(&x.path) {
                    resources.push(x.path.clone());
                    log_info!("Got new resource: {:?}", x);
                    let res_list = SPValue::Array(
                        SPValueType::Path,
                        resources.iter().map(|p| SPValue::Path(p.clone())).collect(),
                    );
                    let mut state_to_send =
                        SPState::new_from_values(&[(registered_resources.clone(), res_list)]);
                    if let Some(goal) = x.last_goal_from_sp {
                        state_to_send.extend(goal);
                    }
                    log_info!("resource state: {}", state_to_send);
                    let res = tx_runner.send(SPRunnerInput::StateChange(state_to_send));
                    if res.is_err() {
                        println!("The runner channel is dead (in the merger)!: {:?}", res);
                        return false;
                    }
                }
                true
            }
            Err(e) => {
                println!("The ticker is dead (in ticker)!: {:?}", e);
                false
            }
        }
    }

    fn cmd_from_node(
        cmd: Result<SPState, channel::RecvError>, tx_runner: Sender<SPRunnerInput>,
    ) -> bool {
        if let Ok(cmd) = cmd {
            // TODO: Handle bad commands here and reply to commander if needed. Probably use service?
            tx_runner
                .send(SPRunnerInput::StateChange(cmd))
                .expect("cmd from node could not talk to the runner");
            true
        } else {
            true
        }
    }

    let mut resources: Vec<SPPath> = vec![];
    let registered_resources = registered_resources.clone();
    thread::spawn(move || loop {
        let res;
        crossbeam::select! {
            recv(rx_resources) -> resource => res = ros_resource(resource, &mut resources, &registered_resources, tx_runner.clone()),
            recv(rx_commands) -> cmd => res = cmd_from_node(cmd, tx_runner.clone()),
        };
        if !res {
            break;
        }
    });
}

struct MergedState {
    pub states: Vec<SPState>,
}
impl MergedState {
    pub fn new() -> MergedState {
        MergedState{states: vec!()}
    }
}

/// Merging states if many states arrives at the same time
fn merger(
    rx_mess: tokio::sync::mpsc::Receiver<SPState>,
    tx_runner: tokio::sync::mpsc::Sender<SPRunnerInput>,
) {
    let (tx, rx) = tokio::sync::watch::channel(false);
    let ms_arc = Arc::new(Mutex::new(MergedState::new()));
    
    let ms_in = ms_arc.clone();
    tokio::spawn(async move {
        loop {
            let s = rx_mess.recv().await.expect("The state channel should always work!");
            {
                ms_in.lock().unwrap().states.push(s);
            }
            tx.send(true).expect("internal channel in merge should always work!");
        }
    });

    let ms_out = ms_arc.clone();
    tokio::spawn(async move {
        loop {
            rx.changed().await;
            let mut states = {
                let x = ms_out.lock().unwrap();
                let res = x.states.clone();
                x.states = vec!();
                res
            };
            states.reverse();
            if !states.is_empty() {
                let mut x = states.pop().unwrap();
                for y in states {
                    if let Some(other) =  try_extend(&mut x, y) {
                        // Can not be merged so sending what we have
                        tx_runner.send(SPRunnerInput::StateChange(x.clone())).await;
                        x = other;
                    }
                }
                tx_runner.send(SPRunnerInput::StateChange(x)).await;
            }
        }
    });


}

/// Tries to extend the state only if the state does not contain the same
/// path or if that path has the same value, else will leave the state unchanged
/// and returns false.
fn try_extend(state: &mut SPState, other_state: SPState) -> Option<SPState> {
    let can_extend = other_state.projection().state.iter().all(|(p, v)| {
        let self_v = state.sp_value_from_path(p);
        p.leaf() == "timestamp" || self_v.map(|x| x == v.value()).unwrap_or(true)
    });
    if can_extend {
        state.extend(other_state);
        None
    } else {
        Some(other_state)
    }
}

async fn ticker_async(freq: Duration, tx_runner: tokio::sync::mpsc::Sender<SPRunnerInput>) {
    let mut ticker = tokio::time::interval(freq);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        ticker.tick().await;
        tx_runner.send(SPRunnerInput::Tick).await;
    }
}



