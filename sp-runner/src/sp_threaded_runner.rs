use super::sp_runner::*;
use super::transition_planner::*;
use super::operation_planner::*;
use crate::planning;
use crate::*;
use crossbeam::{channel, Receiver, Sender};
use failure::Error;
use sp_domain::*;
use sp_ros::*;
use std::collections::{HashSet, HashMap};
use std::panic;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use rayon::prelude::*;

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
    let old_runner = make_new_runner(&model, false);

    let store_async = Arc::new(Mutex::new(planning::AsyncPlanningStore::load(
        &old_runner.transition_system_models[1],
    )));

    let mut transition_planner = TransitionPlanner {
        plan: SPPlan::default(),
        model: old_runner.transition_system_models[0].clone(),
        operations: old_runner.operations.clone(),
        bad_state: false,
        prev_state: SPState::new(),
        prev_goals: vec![],
        store: planning::PlanningStore::default(),
        disabled_operation_check: std::time::Instant::now(),
    };

    let mut operation_planner = OperationPlanner {
        plan: SPPlan::default(),
        model: old_runner.transition_system_models[1].clone(),
        operations: old_runner.operations.clone(),
        intentions: old_runner.intentions.clone(),
        replan_specs: old_runner.replan_specs.clone(),
        prev_state: SPState::new(),
        prev_goals: vec![],
        store_async: store_async.clone(),
        disabled_operation_check: std::time::Instant::now(),
        prev_disabled_operations: HashSet::new(),
    };

    // block all transitions intially. (hmmm)
    let _block_transition_plan = transition_planner.block_all();
    let _block_operation_plan = operation_planner.block_all();

    let t_runner_out = runner_out.clone();
    let t_tx_input = tx_input.clone();
    thread::spawn(move || {
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
        let old_runner = make_new_runner(&model, false);
        let mut runner = NewSPRunner::from_oldrunner(&old_runner);

        // perform additional setup...
        runner.update_state_variables(initial_state);

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

        let store_async = Arc::new(Mutex::new(planning::AsyncPlanningStore::load(
                &old_runner.transition_system_models[1],
            )));

        let mut transition_planner = TransitionPlanner {
            plan: SPPlan::default(),
            model: old_runner.transition_system_models[0].clone(),
            operations: old_runner.operations.clone(),
            bad_state: false,
            prev_state: SPState::new(),
            prev_goals: vec![],
            store: planning::PlanningStore::default(),
            disabled_operation_check: std::time::Instant::now(),
        };

        let mut operation_planner = OperationPlanner {
            plan: SPPlan::default(),
            model: old_runner.transition_system_models[1].clone(),
            operations: old_runner.operations.clone(),
            intentions: old_runner.intentions.clone(),
            replan_specs: old_runner.replan_specs.clone(),
            prev_state: SPState::new(),
            prev_goals: vec![],
            store_async: store_async.clone(),
            disabled_operation_check: std::time::Instant::now(),
            prev_disabled_operations: HashSet::new(),
        };

        // block all transitions intially
        let block_transition_plan = transition_planner.block_all();
        let block_operation_plan = operation_planner.block_all();
        runner.set_plan("transition_planner".to_string(), block_transition_plan);
        runner.set_plan("operation_planner".to_string(), block_operation_plan);

        // experiment with timeout on effects...
        old_runner.transition_system_models[0].transitions.iter().for_each(|t| {
            if t.type_ == TransitionType::Effect {
                let path = t.path().add_parent("effects");
                let effect_enabled = SPState::new_from_values(&[(path, true.to_spvalue())]);
                runner.update_state_variables(effect_enabled);
            }
        });

        // add extra operation goal variables to the initial state.
        // TODO: maybe not necessary...
        for o in &old_runner.operations {
            let op = o.path().clone();
            let mut new_state = SPState::new();
            o.get_goal_state_paths().iter().for_each(|p| {
                let np = op.add_child_path(p);
                let v = runner.state().sp_value_from_path(p).expect(&format!("no such path {}", p));
                new_state.add_variable(np, v.clone());
            });
            runner.update_state_variables(new_state);
        }

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

fn merger(rx_mess: Receiver<RosMessage>, tx_runner: Sender<SPRunnerInput>) {
    thread::spawn(move || {
        let mut s: SPState = SPState::new();
        let mut temp_q: Option<SPState> = None;
        let mut resource_map: HashMap<SPPath, Instant> = HashMap::new();

        loop {
            if rx_mess.is_empty() && temp_q.is_some() {
                s = temp_q.take().unwrap();
            } else if rx_mess.is_empty() {
                match rx_mess.recv() {
                    Ok(mess) => {
                        let x = resource_map
                            .entry(mess.resource.clone())
                            .or_insert(mess.time_stamp.clone());
                        //println!{"resource: {}, tick: {}, timer: {}", mess.resource, x.elapsed().as_millis(), ticker.elapsed().as_millis()};
                        *x = mess.time_stamp.clone();
                        s = mess.state;
                    }
                    Err(e) => {
                        eprintln!(
                            "Something whent wrong in the ROS comm in recv in merger: {:?}",
                            e
                        );
                        break;
                    }
                }
            } else {
                if temp_q.is_some() {
                    s = temp_q.take().unwrap();
                }
                for mess in rx_mess.try_iter() {
                    let x = resource_map
                        .entry(mess.resource.clone())
                        .or_insert(mess.time_stamp.clone());
                    //println!{"M: resource: {}, tick: {}, timer: {}", mess.resource, x.elapsed().as_millis(), ticker.elapsed().as_millis()};
                    *x = mess.time_stamp.clone();
                    let res = s.try_extend(mess.state);
                    if res.is_some() {
                        // The message could not be merged
                        temp_q = res;
                        break;
                    }
                }
            }
            let res = tx_runner.send(SPRunnerInput::StateChange(s.clone()));
            if res.is_err() {
                println!("The runner channel is dead (in the merger)!: {:?}", res);
                break;
            }
            s = SPState::new();
        }
    });
}

fn ticker(freq: Duration, tx_runner: Sender<SPRunnerInput>) {
    let t = channel::tick(freq);
    thread::spawn(move || loop {
        match t.recv() {
            Ok(_) => {
                let res = tx_runner.send(SPRunnerInput::Tick);
                if res.is_err() {
                    println!("The runner channel is dead (in the merger)!: {:?}", res);
                    break;
                }
            }
            Err(e) => {
                println!("The ticker is dead (in ticker)!: {:?}", e);
                break;
            }
        }
    });
}

#[derive(Debug)]
struct PlannerTask {
    namespace: i32,
    ts: TransitionSystemModel,
    state: SPState,
    goals: Vec<(Predicate, Option<Predicate>)>,
    disabled_paths: Vec<SPPath>,
}

pub fn make_new_runner(
    model: &Model, generate_mc_problems: bool,
) -> SPRunner {
    let mut ts_model = TransitionSystemModel::from(&model);

    // refine invariants
    println!("refining model invariants");
    let tsm = ts_model.clone();
    ts_model.specs.par_iter_mut().for_each(|s| {
        s.invariant = refine_invariant(tsm.clone(), s.invariant.clone())
            .expect("crash in refine sp-fm");
        println!("spec done...");
    });
    println!("refining invariants done");

    // add runner transitions
    let runner_transitions = model.all_runner_transitions();

    // add global op transitions
    let global_ops: Vec<&Operation> = model.all_operations();
    let global_ops_trans: Vec<Transition> = global_ops
        .iter()
        .map(|o| o.make_runner_transitions())
        .flatten()
        .collect();

    let global_ops_ctrl: Vec<_> = global_ops_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Controlled)
        .cloned()
        .collect();
    let global_ops_un_ctrl: Vec<_> = global_ops_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Auto)
        .cloned()
        .collect();

    let operations: Vec<Operation> = model.all_operations().into_iter().cloned().collect();

    // unchanged. todo
    let global_intentions: Vec<&Intention> = model.all_intentions();
    let global_int_trans: Vec<_> = global_intentions
        .iter()
        .map(|i| i.make_runner_transitions())
        .flatten()
        .collect();
    let global_int_ctrl: Vec<_> = global_int_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Controlled)
        .cloned()
        .collect();
    let global_int_un_ctrl: Vec<_> = global_int_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Auto)
        .cloned()
        .collect();

    // debug high level model

    let ts_model_op = TransitionSystemModel::from_op(&model);

    if generate_mc_problems {
        crate::planning::generate_offline_nuxvm(&ts_model_op, &Predicate::TRUE);
        crate::planning::generate_offline_nuxvm(&ts_model, &Predicate::TRUE);

        // debug low level model
        let all_op_trans = global_ops
            .iter()
            .map(|o| (o.clone(), o.make_lowlevel_transitions()))
            .collect::<Vec<_>>();
        println!("refining operation forbidden specs");
        global_ops.iter().for_each(|o| {
            // check if a "real" operation or really just an autotransition
            if o.make_runner_transitions().is_empty() {
                return;
            }
            // here we should find all unctronllable actions that can
            // modify the variables of GUARD and disallow them.
            println!("CHECKING OP: {}", o.name());
            let mut temp_ts_model = ts_model.clone();

            let mut new_invariants = vec![];
            temp_ts_model.transitions.retain(|t| {
                let belongs_to_other_op = all_op_trans.iter().find(|(op, ts)| {
                    o.path() != op.path() && ts.iter().any(|x| x.path() == t.path())
                });

                if let Some((op, _)) = belongs_to_other_op {
                    if op.make_runner_transitions().is_empty() {
                        // "auto transition operation", keep this
                        return true;
                    }
                    // println!("FOR OP: {}, filtering transition: {}", op.path(), t.path());
                    if t.type_ == TransitionType::Auto {
                        // this also means we need to forbid this state!
                        let opg = op.make_verification_goal();
                        // println!("FOR OP: {}, forbidding: {}", op.path(), opg);
                        new_invariants.push((op.path(), opg));
                    }
                    false
                } else {
                    true
                }
            });

            let new_specs = new_invariants
                .par_iter()
                .map(|(op, p)| {
                    let i = Predicate::NOT(Box::new(p.clone()));
                    // println!("REFINING: {}", i);
                    let ri = refine_invariant(temp_ts_model.clone(), i)
                        .expect("crash in refine sp-fm");
                    println!("spec done...");
                    let mut s = Spec::new("extended", ri);
                    s.node_mut().update_path(op);
                    s
                })
                .collect::<Vec<_>>();
            temp_ts_model.specs.extend(new_specs);

            // here we check if the operation has an assertion
            // which is assumed to be fulfilled for nominal behavior
            // (eg "resource not in failure mode")
            let guard = if let Some(c) = o.mc_constraint.as_ref() {
                Predicate::AND(vec![o.guard.clone(), c.clone()])
            } else {
                o.guard.clone()
            };
            let op = vec![(o.path().to_string(), guard, o.make_verification_goal())];
            temp_ts_model.name += &format!("_{}", o.name());
            crate::planning::generate_offline_nuxvm_ctl(&temp_ts_model, &Predicate::TRUE, &op);
        });
        println!("refining operation forbidden specs done");
    }

    // old runner model as code here.
    let rm_ab_transitions_ctrl: Vec<Transition> = ts_model
        .transitions
        .iter()
        .filter(|t| t.type_ == TransitionType::Controlled)
        .cloned()
        .collect();

    let rm_ab_transitions_un_ctrl: Vec<Transition> = ts_model
        .transitions
        .iter()
        .filter(|t| t.type_ == TransitionType::Auto || t.type_ == TransitionType::Runner)
        .cloned()
        .collect();

    let mut trans = vec![];
    trans.extend(runner_transitions);
    let mut restrict_controllable = vec![];
    let mut restrict_op_controllable = vec![];
    let false_trans = Transition::new(
        "empty",
        Predicate::FALSE,
        vec![],
        TransitionType::Controlled,
    );
    global_ops_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_op_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    global_ops_un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_op_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });

    // intentions are never restricted
    global_int_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });
    global_int_un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    rm_ab_transitions_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    rm_ab_transitions_un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    let intentions = global_intentions.into_iter().cloned().collect();

    let mut all_vars = ts_model.vars.clone();
    all_vars.extend(ts_model.state_predicates.iter().cloned());

    // add runner variables.
    // TODO: also look in resources/sub-models
    let runner_vars: Vec<Variable> = model
        .items()
        .iter()
        .flat_map(|i| match i {
            SPItem::Variable(s) if s.type_ == VariableType::Runner => Some(s.clone()),
            _ => None,
        })
        .collect();
    all_vars.extend(runner_vars.iter().cloned());

    println!("refining replan specs");
    let replan_specs: Vec<Spec> = operations
        .par_iter()
        .map(|o| {
            let mut s = o.make_replan_specs();
            for mut s in &mut s {
                // Hmm this probably does not belong here...
                s.invariant = refine_invariant(ts_model.clone(), s.invariant.clone())
                    .expect("crash in refine sp-fm");
                println!("spec done...");
            }
            s
        })
        .flatten()
        .collect();
    println!("refining replan specs done");

    let runner = SPRunner::new(
        "test",
        trans,
        all_vars,
        vec![ts_model.clone(), ts_model_op],
        model
            .all_resources()
            .iter()
            .map(|r| r.path().clone())
            .collect(),
        replan_specs,
        operations,
        intentions,
    );

    runner
}
