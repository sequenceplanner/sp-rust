use super::sp_runner::*;
use super::transition_planner::*;
use super::operation_planner::*;
use sp_domain::*;
use sp_formal::CompiledModel;
use sp_ros::*;
use std::panic;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub async fn launch_model(model: Model, mut initial_state: SPState) -> Result<(), SPError> {
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
    //println!("{}", serde_json::to_string_pretty(&model).unwrap());
    println!("********");

    log_info!("startar SP!");

    let (tx_runner, rx_runner) = tokio::sync::mpsc::channel(2);
    let (tx_new_state, rx_new_state) = tokio::sync::mpsc::channel(2);
    let (tx_runner_state, rx_runner_state) = tokio::sync::watch::channel(initial_state.clone());


    tokio::spawn(merger(rx_new_state, tx_runner.clone()));
    tokio::spawn(ticker_async(std::time::Duration::from_millis(1000), tx_runner.clone()));

    let ros_comm = sp_ros::RosComm::new(
        rx_runner_state.clone(),
        tx_new_state.clone(),
        model.clone(),
    ).await?;

    let model_watcher = ros_comm.model_watcher();
    let runner_handle = tokio::spawn(async move {
        runner(
            model_watcher,
            initial_state,
            rx_runner,
            tx_runner_state,
        ).await;
    });

    let model_watcher = ros_comm.model_watcher();
    let planner_handle = tokio::spawn(async move {
        planner(
            model_watcher,
            tx_runner.clone(),
            rx_runner_state.clone()
        ).await;
    });

    let err = runner_handle.await; //let err = tokio::try_join!(runner_handle, planner_handle);

    println!("The runner terminated!: {:?}", err);
    log_error!("The SP runner terminated: {:?}", err);
    Ok(())

}


async fn planner(
    model_watcher: tokio::sync::watch::Receiver<Model>,
    tx_input: tokio::sync::mpsc::Sender<SPRunnerInput>,
    runner_out: tokio::sync::watch::Receiver<SPState>
) {
    let model = model_watcher.borrow().clone();
    let compiled_model = CompiledModel::from(model); // TODO: temporarily created here
    let mut transition_planner = TransitionPlanner::from(&compiled_model);
    let mut operation_planner = OperationPlanner::from(&compiled_model);

    let mut t_runner_out = runner_out.clone();
    let t_tx_input = tx_input.clone();
    tokio::spawn(async move {
        // Initiall block all our transitions.
        let block_transition_plan = transition_planner.block_all();
        let cmd = SPRunnerInput::NewPlan("transition_planner".to_string(), block_transition_plan);
        t_tx_input.send(cmd).await;
        loop {
            t_runner_out.changed().await;
            let ro = t_runner_out.borrow().clone();
            let mut tpc = transition_planner.clone();
            let x = tokio::task::spawn_blocking(move || {
                let plan = tpc.compute_new_plan(ro);
                (plan, tpc)
            }).await;
            if let Ok((plan, tpc)) = x {
                transition_planner = tpc;
                if let Some(plan) = plan {
                    println!("new plan computed");
                    let cmd = SPRunnerInput::NewPlan("transition_planner".to_string(), plan);
                    t_tx_input.send(cmd).await;
                }
            }
        }
    });

    let mut o_runner_out = runner_out.clone();
    let o_tx_input = tx_input.clone();
    tokio::spawn(async move {
        // Initiall block all our transitions.
        let block_operation_plan = operation_planner.block_all();
        let cmd = SPRunnerInput::NewPlan("operation_planner".to_string(), block_operation_plan);
        o_tx_input.send(cmd).await;
        loop {
            o_runner_out.changed().await;
            let s = o_runner_out.borrow().clone();
            let mut opc = operation_planner.clone();
            let x = tokio::task::spawn_blocking(move || {
                let plan = opc.compute_new_plan(s);
                (plan, opc)
            }).await;
            if let Ok((plan, opc)) = x {
                operation_planner = opc;
                if let Some(plan) = plan {
                    println!("new plan computed");
                    let cmd = SPRunnerInput::NewPlan("operation_planner".to_string(), plan);
                    o_tx_input.send(cmd).await;
                }
            }
        }
    });

}

async fn runner(
    mut model_watcher: tokio::sync::watch::Receiver<Model>,
    initial_state: SPState,
    mut rx_input: tokio::sync::mpsc::Receiver<SPRunnerInput>,
    tx_state_out: tokio::sync::watch::Sender<SPState>
) {
    log_info!("Runner start");
    let model = model_watcher.borrow().clone();
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

        let input = tokio::select! {
            Some(input) = rx_input.recv() => {input},
            Ok(_) = model_watcher.changed() => {
                let m = model_watcher.borrow().clone();
                SPRunnerInput::ModelChange(m)
            }
        };

        let mut state_has_probably_changed = false;
        let mut ticked = false;
        let mut last_fired_transitions = vec![];

        // log_info!("Runner got: {:?}", &input);
        match input {
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
            SPRunnerInput::ModelChange(m) => {
                println!("A new model TODO");
                log_info!("A new model in the runner. TODO");
            }
        }

        now = Instant::now();

        // if there's nothing to do in this cycle, continue
        if !state_has_probably_changed && last_fired_transitions.is_empty() && !ticked {
            continue;
        } else {
            // println!("state changed? {}", state_has_probably_changed);
            // println!("transition fired? {}", !runner.last_fired_transitions.is_empty());
            // println!("ticked? {}", ticked);
        }

        let mut s = runner.ticker.state.clone();

        if !last_fired_transitions.is_empty() {
            let f = last_fired_transitions.iter().fold(String::new(), |a, t| {
                if a.is_empty() {
                    t.to_string()
                } else {
                    format!{"{}, {}", a, t}
                }
            });
            s.add_variable(SPPath::from_string("sp/fired"), f.to_spvalue());
            println!("fired:");
            last_fired_transitions
                .iter()
                .for_each(|x| println!("{:?}", x));
        }


        tx_state_out.send(s);

    }
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
async fn merger(
    mut rx_mess: tokio::sync::mpsc::Receiver<SPState>,
    tx_runner: tokio::sync::mpsc::Sender<SPRunnerInput>,
) {
    let (tx, mut rx) = tokio::sync::watch::channel(false);
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
                let mut x = ms_out.lock().unwrap();
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
        //log_info!("Ticker");
        ticker.tick().await;
        tx_runner.send(SPRunnerInput::Tick).await;
    }
}
