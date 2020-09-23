use super::sp_runner::*;
use crossbeam::{channel, Receiver, Sender};
use failure::Error;
use sp_domain::*;
use sp_ros::*;
use sp_runner_api::*;
use std::collections::{HashMap, HashSet};
use std::thread;
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};
use std::panic;
use crate::formal_model::*;
use crate::planning;

// some planning constants
const LVL0_MAX_STEPS: u32 = 25;
const LVL1_MAX_STEPS: u32 = 40;
const LVL1_CUTOFF: u32 = 20;
const LVL1_LOOKOUT: f32 = 0.75;
const LVL1_MAX_TIME: Duration = Duration::from_secs(5);

pub fn launch_model(model: Model, mut initial_state: SPState) -> Result<(), Error> {
    // we use this as the main entry point for SP.
    // so here we register our panic handler to send out
    // fatal messages to ROS
    panic::set_hook(Box::new(|panic_info| {
        let msg = if let Some(s) = panic_info.payload().downcast_ref::<&str>() {
            s
        } else if  let Some(s) = panic_info.payload().downcast_ref::<String>() {
            &s[..]
        } else {
            ""
        };
        let (file, line) = if let Some(location) = panic_info.location() {
            (location.file(), location.line())
        } else {
            ("",0)
        };
        log_fatal(msg, file, line);
        println!("\n\n\nSP PANIC: {}\n\n\nfile: {}:{}", msg, file, line);
    }));

    let (mut node, comm) = set_up_ros_comm(&model)?;

    let (tx_runner, rx_runner): (Sender<SPRunnerInput>, Receiver<SPRunnerInput>) =
        channel::bounded(3);
    let (tx_planner, _rx_planner): (Sender<PlannerTask>, Receiver<PlannerTask>) =
        channel::unbounded();

    let resource_list_path = SPPath::from_string("registered_resources");
    resource_handler(comm.rx_resources, comm.rx_commands, tx_runner.clone(), &resource_list_path);
    initial_state.add_variable(resource_list_path, SPValue::Array(SPValueType::Path, vec!()));

    merger(comm.rx_mess.clone(), tx_runner.clone());
    ticker(Duration::from_millis(100), tx_runner.clone());

    // node_handler(
    //     Duration::from_millis(1000),
    //     Duration::from_secs(1000),
    //     &model,
    //     comm.rx_node.clone(),
    //     comm.tx_node_cmd.clone(),
    //     tx_runner.clone(),
    //     comm.rx_commands.clone()
    // );
    runner(
        &model,
        initial_state,
        rx_runner,
        comm.tx_state_out,
        comm.tx_runner_info,
        tx_planner,
    );



    loop {
        // blocking ros spinning
        spin(&mut node);
    }
}

fn runner(
    model: &Model, initial_state: SPState, rx_input: Receiver<SPRunnerInput>,
    tx_state_out: Sender<SPState>, tx_runner_info: Sender<RunnerInfo>,
    _tx_planner: Sender<PlannerTask>,
) {
    let model = model.clone();
    thread::spawn(move || {
        let mut runner = make_new_runner(&model, initial_state, false);
        let mut prev_goals: HashMap<usize,
                                    Vec<(Predicate, Option<Predicate>)>> = HashMap::new();
        let store_async = Arc::new(Mutex::new(
            planning::AsyncPlanningStore::load(&runner.transition_system_models[1])));
        // let timer = Instant::now();

        let mut bad_state = false;

        let mut disabled_operations = Vec::new();

        let mut now = Instant::now();

        'outer: loop {
            let elapsed_ms = now.elapsed().as_millis();
            if elapsed_ms > 1000 {
                log_debug!("RUNNER TICK TIME: {}ms", elapsed_ms);
            }
            let input = rx_input.recv();
            let mut state_has_probably_changed = false;
            let mut ticked = false;
            runner.last_fired_transitions = vec![];
            if let Ok(msg) = input {
                match msg {
                    SPRunnerInput::StateChange(s) => {
                        if !runner.state().are_new_values_the_same(&s) {
                            runner.input(SPRunnerInput::StateChange(s));
                            state_has_probably_changed = true;
                        } else {
                            runner.update_state_variables(s);
                        }
                    }
                    SPRunnerInput::NodeChange(s) => {
                        if !runner.state().are_new_values_the_same(&s) {
                            runner.input(SPRunnerInput::NodeChange(s));
                            state_has_probably_changed = true;
                        }
                    }
                    SPRunnerInput::Tick => {
                        runner.input(SPRunnerInput::Tick);
                        ticked = true;
                    }
                    SPRunnerInput::NewPlan(_, _) => {
                        panic!("not used at the moment, planner called synchronously");
                    }
                    SPRunnerInput::Settings(cmd) => {
                        // hack to test out the parser -- add new specs to a living system.
                        if let RunnerCommand::Mode(str) = &cmd {
                            log_debug!("got mode string: {}", str);
                            let v: Vec<_> = str.splitn(3, ":").collect();
                            if v.len() == 3 && v[0] == "add_spec" {
                                let name = v[1];
                                let pred_str = v[2];
                                let pred = Predicate::from_string(pred_str);
                                match pred {
                                    Some(pred) => { // for now we assume only lvl0 spec
                                        // got a new specifiction, add it to the system and
                                        // reset the planning store
                                        log_info!("Added new specification: {}", pred);
                                        let refined_pred =
                                            crate::formal_model::refine_invariant(
                                                &runner.transition_system_models[0], &pred);
                                        let spec = Spec::new(name, refined_pred);
                                        runner.transition_system_models[0].specs.push(spec);
                                    },
                                    None => log_error!("Failed to parse predicate {}: {}", name, pred_str)
                                }
                            }
                            if v.len() == 2 && v[0] == "remove_spec" {
                                let name = v[1];
                                let ol = runner.transition_system_models[0].specs.len();
                                runner.transition_system_models[0].specs.retain(|s| s.name() != name);
                                if runner.transition_system_models[0].specs.len() != ol {
                                    log_info!("Removed specification {}.", name);
                                }
                            }
                        }

                        state_has_probably_changed = true;
                        runner.input(SPRunnerInput::Settings(cmd));
                        runner.input(SPRunnerInput::Tick);

                        // this could be done all the time, or
                        // periodically, but for now we only trigger
                        // this check when the state is manually
                        // updated
                        let mut something_was_fixed = false;

                        disabled_operations.retain(|p: &SPPath| {
                            let e = "error".to_spvalue();
                            let os = runner.state().sp_value_from_path(p).unwrap_or(&e);
                            println!("CHECKING {} {}", p, os);
                            if os != &e {
                                // user manually reset the operation
                                something_was_fixed = true;
                                false
                            } else {
                                true
                            }
                        });
                        disabled_operations.retain(|p: &SPPath| {
                            // check if the state if OK so we can remove any error states.
                            println!("checking if we can remove error on low level operation: {}", p);
                            let goal = runner.operation_goals.get(p).expect("no goal on disabled op");
                            let goal = vec![(goal.clone(), None)];

                            let pr = planning::plan(&runner.transition_system_models[0], goal.as_slice(),
                                                    runner.state(), LVL0_MAX_STEPS);

                            if !pr.plan_found {
                                println!("operation still problematic...");
                            } else {
                                runner.ticker.state.force_from_path(&p, &"i".to_spvalue()).unwrap();
                                something_was_fixed = true;
                            }

                            !pr.plan_found
                        });

                        if something_was_fixed {
                            // check all high level ops with error states. maybe we can move some of them
                            // back to their executing state.
                            // HACKS!

                            for p in &runner.intentions {
                                if runner.state().sp_value_from_path(p).unwrap() == &"error".to_spvalue() {
                                    println!("checking if we can remove error on high level operation: {}", p);
                                    let goal_path = p.clone().add_child("goal");
                                    println!("goal name {}", goal_path);

                                    let g1 = &runner.intention_goals;
                                    let i: &IfThen = g1.iter().find(|g| g.path() == &goal_path).unwrap();
                                    let goal = vec![(i.goal().clone(), None)];

                                    let pr = planning::plan_async_with_cache(&runner.transition_system_models[1], &goal, runner.state(), &disabled_operations,
                                                                             LVL1_MAX_STEPS, LVL1_CUTOFF, LVL1_LOOKOUT, LVL1_MAX_TIME, store_async.clone());

                                    if pr.plan_found {
                                        log_info!("automatically restarting operation {}", p);
                                        runner.ticker.state
                                            .force_from_path(&p,
                                                             &"e".to_spvalue()).unwrap();
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                println!("The runner channel broke? - {:?}", input);
                break;
            }

            now = Instant::now();

            //println!("tick: {} ms", timer.elapsed().as_millis());

            if !runner.last_fired_transitions.is_empty() {
                println!("fired:");
                runner
                    .last_fired_transitions
                    .iter()
                    .for_each(|x| println!("{:?}", x));
            }

            // if there's nothing to do in this cycle, continue
            if !state_has_probably_changed && runner.last_fired_transitions.is_empty() &&
                !ticked {
                continue;
            } else {
                // println!("state changed? {}", state_has_probably_changed);
                // println!("transition fired? {}", !runner.last_fired_transitions.is_empty());
                // println!("ticked? {}", ticked);
            }

            let disabled = runner.disabled_paths();
            let mut enabled_state = runner.state().projection();
            let l1 = enabled_state.state.len();
            enabled_state.state.retain(|(p,v)| !disabled.iter().any(|d| p.is_child_of(d)) &&
                                       v.current_value() != &SPValue::Unknown);
            let disabled_states = enabled_state.state.len() < l1;
            tx_state_out
                .send(enabled_state.clone_state())
                .expect("tx_state:out");

            // send out runner info.
            let mut runner_modes = vec![];
            if disabled_states { runner_modes.push("waiting");}
            if bad_state { runner_modes.push("bad state");}
            if !disabled_operations.is_empty() { runner_modes.push("disabled operations"); }
            let runner_info = RunnerInfo {
                state: runner.state().clone(),
                mode: runner_modes.join(" | "),
                ..RunnerInfo::default()
            };

            tx_runner_info.send(runner_info).expect("tx_runner_info");

            if disabled_states {
                println!("still waiting... do nothing");
                continue;
            }

            if !bad_state {
                let bad: Vec<_> = runner.transition_system_models[0].specs.iter()
                    .filter_map(|s| if !s.invariant().eval(runner.state()) {
                        Some(s)
                    } else {
                        None
                    }).collect();

                if !bad.is_empty() {
                    // try to find a way out of this situation by temporarily relaxing the specs
                    // and instead planning to a new state where the specs holds.
                    let mut temp_ts = runner.transition_system_models[0].clone();
                    temp_ts.specs.retain(|spec| !bad.iter().any(|b| b.path() == spec.path()));
                    let goals = bad.iter().map(|b| (b.invariant().clone(), None)).collect::<Vec<_>>();
                    let pr = planning::plan(&temp_ts, goals.as_slice(), runner.state(), LVL0_MAX_STEPS);

                    let plan = pr.trace.iter().map(|x| x.transition.to_string()).collect::<Vec<_>>().join("\n");

                    let spec_names = bad.iter().map(|b| b.path().to_string()).collect::<Vec<_>>().join(", ");
                    log_error!("We are in a forbidden state! Spec(s) {} are violated. A way to get out could be \n{}", spec_names, plan);

                    bad_state = true;
                }
            } else {
                bad_state = SPRunner::bad_state(runner.state(), &runner.transition_system_models[0]);
            }

            if bad_state {
                continue;
            }

            println!("The State:\n{}", runner.state());

            let ts_models = runner.transition_system_models.clone();
            let goals = runner.goal();
            // for (i,g) in goals.iter().enumerate() {
            //     if !g.is_empty() {
            //         println!("Goals for {}", i);
            //         for g in g {
            //             println!("{}", g.0);
            //         }
            //         println!("--");
            //     }
            // }

            for (i, (ts, goals)) in ts_models.iter().zip(goals.iter()).enumerate().rev() {
                //println!("TS {}", i);
                let mut ts = ts.clone();
                if i == 1 {
                    ts.transitions.retain(|t| !disabled_operations.contains(&t.path().parent()));
                }
                let ts = &ts;

                let planner = SPPath::from_slice(&["runner", "planner", &i.to_string()]);
                if runner.ticker.state.sp_value_from_path(&planner)
                    .unwrap_or(&false.to_spvalue()) != &true.to_spvalue() {
                        prev_goals.insert(i, Vec::new());
                        runner.input(SPRunnerInput::NewPlan(i as i32, SPPlan {
                            plan: crate::planning::block_all(ts),
                            included_trans: Vec::new(),
                            state_change: SPState::new(),
                        }));

                        continue;
                    }

                //println!("TS {} NOT SKIPPED", i);

                // for each namespace, check if we need to replan because
                // 1. got new goals from the runner or
                // 2. can no longer reach our goal (because the state has changed)

                // Because the plan is already encoded in the guards
                // of the runner transitions we can use the runner for
                // this purpose.

                // This is also true for the goals -> they are a function of the state.

                let prev_goal = prev_goals.get(&i);

                let gr: Vec<&Predicate> = goals
                    .iter()
                    .map(|g| &g.0).collect(); // dont care about the invariants for now

                //println!("TS {} GOT GOALS", i);

                let replan = {
                    let is_empty = prev_goal.is_none();
                    if is_empty {
                        let pg = Predicate::AND(gr.iter().map(|&c|c.clone()).collect());
                        log_info!("replanning because previous goal was empty. {}: new goal: {}", i, pg);
                    }
                    is_empty
                } || {
                    let ok = &prev_goal.map(|g| g == goals).unwrap_or(false);
                    // let all low-level effects complete before computing a new high level goal.
                    if i == 1 && !ok {
                        let active_effects = runner.replan_specs.iter()
                            .any(|t| {
                                let x = !t.invariant().eval(runner.state());
                                if x { println!("Effect in progress: {}", t.path()); }
                                x
                            });
                        if active_effects {
                            println!("XXX effects in progress, replanning later....");
                            continue 'outer;
                        }
                    }
                    if !ok {
                        let pg = Predicate::AND(gr.iter().map(|&c|c.clone()).collect());
                        log_info!("replanning because goal changed. {}: new goal: {}", i, pg);
                        prev_goal.map(|g| g.iter().for_each(|g| {
                            println!("prev goals {}", g.0)
                        }));
                    }
                    !ok
                } || {
                    let now = std::time::Instant::now();

                    //println!("TS {} CHECKING GOALS", i);

                    let ok = if i == 1 {
                        runner
                            .check_goals_op_model(runner.state(), &gr, &runner.plans[i],
                                                  &runner.transition_system_models[i])
                    } else {
                        runner
                            .check_goals_fast(runner.state(), &gr, &runner.plans[i],
                                              &runner.transition_system_models[i])
                    };

                    //println!("TS {} CHECKING GOALS DONE", i);

                    if now.elapsed().as_millis() > 100 {
                        println!("WARNINIG goal check for {}: {} (took {}ms)", i, ok, now.elapsed().as_millis());
                    }

                    if !ok {
                        println!("goal check for {}: {} (took {}ms)", i, ok, now.elapsed().as_millis());
                        let pg = Predicate::AND(gr.iter().map(|&c|c.clone()).collect());
                        log_info!("replanning because we cannot reach goal. {}: {}", i, pg);
                    }
                    !ok
                };

                if replan {

                    //println!("TS {} REPLAN", i);

                    // temporary hack -- actually probably not so
                    // temporary, this is something we need to deal with
                    if i == 1 {
                        println!("resetting all operation state");
                        for op in &runner.operations {
                            if disabled_operations.contains(op.path()) { continue }
                            let path = op.node().path();
                            if runner.ticker.state.sp_value_from_path(path)
                                .map(|v| v == &"e".to_spvalue()).unwrap_or(false) {
                                    runner.ticker.state.force_from_path(path,
                                                                        &"i".to_spvalue()).unwrap();
                                }
                        }
                    }

                    println!("computing plan for namespace {}", i);

                    let planner_result = if i == 0 {
                        // in the default case we should ensure monotonicity of the planning problem,
                        // because 1) sops and high level plans need it
                        // 2) operations (and errors!) are more intuitive this way.

                        let mono = SPPath::from_slice(&["runner", "planner", "monotonic"]);
                        let mut pr = if runner.ticker.state.sp_value_from_path(&mono).unwrap_or(&false.to_spvalue()) == &true.to_spvalue() {
                            let modifies: HashSet<SPPath> = goals.iter().map(|(a,_)|a.support()).flatten().collect();
                            let ts_1 = &runner.transition_system_models[1];
                            let mut all: HashSet<SPPath> = ts_1.vars.iter().map(|v|v.path().clone()).collect();
                            // HACK below!
                            all.retain(|p| p.path.contains(&"product_state".to_string()));
                            let no_change: HashSet<&SPPath> = all.difference(&modifies).collect();
                            let no_change_specs: Vec<Predicate> = no_change.iter().flat_map(|p| {
                                runner.state().sp_value_from_path(p)
                                    .map(|val|
                                         Predicate::EQ(PredicateValue::SPPath((*p).clone(), None),
                                                       PredicateValue::SPValue(val.clone())))
                            }).collect();
                            // TODO: these specs should really be extended to include any auto trans which can change them.
                            // the extending should be computed whenever the model changes
                            // this is temporary.
                            let mut tts = ts.clone();
                            if !no_change_specs.is_empty() {
                                let no_change_pred = Predicate::AND(no_change_specs);
                                tts.specs.push(Spec::new("monotonicity_constraints", no_change_pred));
                            }

                            // skip heuristic for the low level (cannot use cache if we dont serialize the extra invariants)
                            planning::plan(&tts, &goals, runner.state(), LVL0_MAX_STEPS)
                        } else {
                            // skip heuristic for the low level
                            planning::plan(ts, &goals, runner.state(), 2 * LVL0_MAX_STEPS)
                        };

                        planning::bubble_up_delibs(ts, &gr, &mut pr);
                        pr
                    } else {
                        planning::plan_async_with_cache(&ts,
                                                        &goals,
                                                        runner.state(),
                                                        &disabled_operations,
                                                        LVL1_MAX_STEPS,
                                                        LVL1_CUTOFF,
                                                        LVL1_LOOKOUT,
                                                        LVL1_MAX_TIME,
                                                        store_async.clone())
                    };

                    let plan_p = SPPath::from_slice(&["runner", "plans", &i.to_string()]);

                    // hack to reset the user visible planning steps when we replan
                    // because sp_ui cannot remove states anyway (TODO!), we set dummy values
                    let statevals = runner.ticker.state.clone().extract();
                    let filtered_state: Vec<_> = statevals.into_iter().map(|(p,v)| {
                        if p.is_child_of(&plan_p) {
                            let mut nv = v.clone(); nv.force("-".to_spvalue());
                            (p, nv)
                        } else {
                            (p, v)
                        }}).collect();
                    runner.force_new_state(SPState::new_from_state_values(filtered_state.as_slice()));

                    let (tr, s) = if i == 1 {
                        planning::convert_planning_result_with_packing_heuristic(&ts, &planner_result, &plan_p)
                    } else {
                        planning::convert_planning_result(&ts, &planner_result, &plan_p)
                    };

                    let trans = planner_result.trace.iter()
                        .filter_map(|f|
                                    if f.transition.is_empty() {
                                        None
                                    } else {
                                        Some(f.transition.clone())
                                    }).collect();

                    let no_plan = !planner_result.plan_found;

                    if no_plan && i == 0 {
                        // temp
                        std::fs::copy("./last_planning_request.bmc",
                                      "./last_failed_planning_request.bmc")
                            .expect("file copy failed");
                        // no low level plan found, we are in trouble.

                        // look for the problematic goals
                        for (op_path,goal) in &runner.operation_goals {
                            println!("checking low level operation: {} -- {:?}", op_path, goal);
                            println!("disabled ops {:?}", disabled_operations);
                            if disabled_operations.contains(op_path) { continue; }
                            let goal = vec![(goal.clone(), None)];
                            let pr = planning::plan(&ts, goal.as_slice(), runner.state(), LVL0_MAX_STEPS);
                            if !pr.plan_found {
                                println!("offending low level operation: {}", op_path);
                                log_warn!("offending low level operation: {}", op_path);
                                runner.ticker.state
                                    .force_from_path(op_path, &"error".to_spvalue()).unwrap();
                                disabled_operations.push(op_path.clone());
                            }
                        }
                        if disabled_operations.is_empty() {
                            // panic!("NO PLAN FOUND BUT ALSO NOW OFFENDING OPS");
                            log_error!("NO PLAN FOUND BUT ALSO NO OFFENDING OPS.\n\
                                       It could be that the planning horizon needs to be increased.");
                        }
                    }

                    if no_plan && i == 1 {
                        // no high level plan found, we are in trouble.

                        // look for the problematic goals
                        let g1 = &runner.intention_goals;
                        let ifthens: Vec<&IfThen> = g1.iter().filter(|g| g.condition.eval(runner.state())).collect();

                        for i in ifthens {
                            let goal = vec![(i.goal().clone(), None)];
                            let pr = planning::plan_async_with_cache(&ts, &goal,
                                                                     runner.state(),
                                                                     &disabled_operations,
                                                                     LVL1_MAX_STEPS,
                                                                     LVL1_CUTOFF,
                                                                     LVL1_LOOKOUT,
                                                                     LVL1_MAX_TIME,
                                                                     store_async.clone());

                            if !pr.plan_found {
                                let offending_op = i.path().parent();
                                log_warn!("offending high level operation: {}", offending_op);
                                runner.ticker.state
                                    .force_from_path(&offending_op, &"error".to_spvalue()).unwrap();
                            }
                        }
                    }


                    let plan = SPPlan {
                        plan: tr,
                        included_trans: trans,
                        state_change: s,
                    };

                    // update last set of goals
                    if no_plan {
                        log_warn!("No plan was found for namespace {}! time to fail {}ms",
                                  i, planner_result.time_to_solve.as_millis());
                        prev_goals.remove(&i);
                    } else {
                        log_info!("New plan was found for namespace {}! time to solve {}ms",
                                  i, planner_result.time_to_solve.as_millis());
                        prev_goals.insert(i, goals.clone());
                    }

                    runner.input(SPRunnerInput::NewPlan(i as i32, plan));

                    // because we probably need to do something else,
                    // wait until next iteration to check lower levels, so
                    // that the plan has time to be "ticked in".
                    continue 'outer;
                }
            }
        }
    });
}

struct RosCommSetup {
    rx_mess: Receiver<RosMessage>,
    rx_commands: Receiver<RunnerCommand>,
    rx_resources: Receiver<ROSResource>,
    tx_state_out: Sender<SPState>,
    tx_runner_info: Sender<RunnerInfo>,
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
            rx_commands: rx_commands,
            rx_resources,
            tx_state_out,
            tx_runner_info,
        },
    ))
}

fn resource_handler(
    rx_resources: Receiver<ROSResource>,
    rx_commands: Receiver<RunnerCommand>,
    tx_runner: Sender<SPRunnerInput>,
    registered_resources: &SPPath

) {


    fn ros_resource(
        msg: Result<ROSResource, channel::RecvError>,
        resources: &mut Vec<SPPath>,
        registered_resources:  &SPPath,
        tx_runner: Sender<SPRunnerInput>
    ) -> bool {
        match msg {
            Ok(x) => {
                //log_info!("Got resource: {:?}", x);
                if !resources.contains(&x.path) {
                    resources.push(x.path.clone());
                    log_info!("Got new resource: {:?}", x);
                    let res_list = SPValue::Array(SPValueType::Path, resources.iter().map(|p| SPValue::Path(p.clone())).collect());
                    let mut state_to_send = SPState::new_from_values(&[(registered_resources.clone(), res_list)]);
                    if let Some(goal) = x.last_goal_from_sp {
                        state_to_send.extend(goal);
                    }
                    log_info!("resource state: {}", state_to_send);
                    let res = tx_runner.send(SPRunnerInput::StateChange(state_to_send));
                    if res.is_err() {
                        println!("The runner channel is dead (in the merger)!: {:?}", res);
                        return false
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
        cmd: Result<RunnerCommand, channel::RecvError>,
        tx_runner: Sender<SPRunnerInput>,
    ) -> bool {
        if let Ok(cmd) = cmd {
            // TODO: Handle bad commands here and reply to commander if needed. Probably use service?
            tx_runner.send(SPRunnerInput::Settings(cmd)).expect("cmd from node could not talk to the runner");
            true
        } else {
            true
        }
    }

    let mut resources: Vec<SPPath> = vec!();
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

pub fn make_new_runner(model: &Model, initial_state: SPState, generate_mc_problems: bool) -> SPRunner {
    let ts_model = TransitionSystemModel::from(&model);

    // add global op transitions
    let global_ops: Vec<&Operation> = model.all_operations();
    let global_ops_trans: Vec<Transition> = global_ops.iter().
        map(|o|o.make_runner_transitions()).flatten().collect();

    let global_ops_ctrl: Vec<_> = global_ops_trans
        .iter()
        .filter(|t|t.type_ == TransitionType::Controlled)
        .cloned()
        .collect();
    let global_ops_un_ctrl: Vec<_> = global_ops_trans
        .iter()
        .filter(|t|t.type_ == TransitionType::Auto)
        .cloned()
        .collect();

    let operations: Vec<Operation> = model.all_operations().into_iter().cloned().collect();

    // unchanged. todo
    let global_intentions: Vec<&Intention> = model.all_intentions();
    let global_int_trans: Vec<_> = global_intentions
        .iter().map(|i| i.make_runner_transitions())
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
    let global_hl_goals: Vec<IfThen> = global_intentions
        .iter()
        .map(|o| o.make_goal())
        .collect();

    // debug high level model

    let ts_model_op = TransitionSystemModel::from_op(&model);

    if generate_mc_problems {
        crate::planning::generate_offline_nuxvm(&ts_model_op, &Predicate::TRUE);
        crate::planning::generate_offline_nuxvm(&ts_model, &Predicate::TRUE);

        // debug low level model
        let all_op_trans = global_ops.iter().map(|o|(o.clone(),o.make_lowlevel_transitions()))
            .collect::<Vec<_>>();
        global_ops.iter().for_each(|o| {
            // check if a "real" operation or really just an autotransition
            if o.make_runner_transitions().is_empty() { return; }
            // here we should find all unctronllable actions that can
            // modify the variables of GUARD and disallow them.
            println!("CHECKING OP: {}", o.name());
            let mut temp_ts_model = ts_model.clone();

            let mut new_invariants = vec![];
            temp_ts_model.transitions.retain(|t| {
                let belongs_to_other_op = all_op_trans.iter()
                    .find(|(op,ts)|
                          o.path() != op.path() &&
                          ts.iter().any(|x|x.path() == t.path()));

                if let Some((op,_)) = belongs_to_other_op {
                    if op.make_runner_transitions().is_empty() {
                        // "auto transition operation", keep this
                        return true;
                    }
                    println!("FOR OP: {}, filtering transition: {}", op.path(), t.path());
                    if t.type_ == TransitionType::Auto {
                        // this also means we need to forbid this state!
                        let opg = op.make_verification_goal();
                        println!("FOR OP: {}, forbidding: {}", op.path(), opg);
                        new_invariants.push((op.path(), opg));
                    }
                    false
                } else {
                    true
                }
            });

            let new_specs = new_invariants.iter().map(|(op, p)| {
                let i = Predicate::NOT(Box::new(p.clone()));
                println!("REFINING: {}", i);
                let ri = refine_invariant(&temp_ts_model, &i);
                let mut s = Spec::new("extended", ri);
                s.node_mut().update_path(op);
                s
            }).collect::<Vec<_>>();
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

            let inits: Vec<_> = ts_model_op.specs.iter().map(|s|s.invariant.clone()).collect();
            let initial_states = if inits.is_empty() {
                Predicate::TRUE
            } else {
                Predicate::AND(inits)
            };


            crate::planning::generate_offline_nuxvm_ctl(&temp_ts_model, &initial_states, &op);
        });
    }

    // old runner model as code here.

    let rm_op_transitions = RunnerTransitions {
        ctrl: global_ops_ctrl,
        un_ctrl: global_ops_un_ctrl,
    };

    let rm_hl_op_transitions = RunnerTransitions {
        ctrl: global_int_ctrl,
        un_ctrl: global_int_un_ctrl,
    };


    let rm_ab_transitions = RunnerTransitions {
        ctrl: ts_model
            .transitions
            .iter()
            .filter(|t| t.type_ == TransitionType::Controlled)
            .cloned()
            .collect(),
        un_ctrl: ts_model
            .transitions
            .iter()
            .filter(|t| t.type_ == TransitionType::Auto)
            .cloned()
            .collect(),
    };

    let mut trans = runner_transitions;
    let mut restrict_controllable = vec![];
    let mut restrict_op_controllable = vec![];
    let false_trans = Transition::new("empty", Predicate::FALSE, vec![], TransitionType::Controlled);
    rm_op_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_op_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    rm_op_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_op_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });

    // intentions are never restricted
    rm_hl_op_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });
    rm_hl_op_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    rm_ab_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    rm_ab_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    let intentions = global_intentions.iter().map(|v| v.path().clone()).collect();

    let mut all_vars = ts_model.vars.clone();
    all_vars.extend(ts_model.state_predicates.iter().cloned());

    let replan_specs: Vec<Spec> = operations.iter().map(|o| {
        let mut s = o.make_replan_specs();
        for mut s in &mut s {
            // Hmm this probably does not belong here...
            s.invariant = ts_model.refine_invariant(&s.invariant);
        }
        s
    }).flatten().collect();

    let mut runner = SPRunner::new(
        "test",
        trans,
        all_vars,
        global_hl_goals,
        vec![],
        vec![],
        vec![ts_model, ts_model_op],
        model.all_resources().iter().map(|r| r.path().clone()).collect(),
        intentions,
        replan_specs,
        operations
    );

    runner.input(SPRunnerInput::NewPlan(0, SPPlan {
        plan: restrict_controllable.clone(),
        included_trans: Vec::new(),
        state_change: SPState::new(),
    }));
    runner.input(SPRunnerInput::NewPlan(1, SPPlan {
        plan: restrict_op_controllable.clone(),
        included_trans: Vec::new(),
        state_change: SPState::new(),
    }));

    runner.update_state_variables(initial_state);

    // planning active or not
    let planner0 = SPPath::from_slice(&["runner", "planner", "0"]);
    let planner1 = SPPath::from_slice(&["runner", "planner", "1"]);
    let planners_initially_on = SPState::new_from_values(&[
        (planner0, true.to_spvalue()),
        (planner1, true.to_spvalue()),
    ]);
    runner.update_state_variables(planners_initially_on);

    // monotonic planning mode
    let mono = SPPath::from_slice(&["runner", "planner", "monotonic"]);
    let monotonic_initially_on = SPState::new_from_values(&[
        (mono, true.to_spvalue()),
    ]);
    runner.update_state_variables(monotonic_initially_on);

    runner
}
