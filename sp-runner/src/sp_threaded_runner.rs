use super::sp_runner::*;
use crossbeam::{channel, Receiver, Sender};
use failure::Error;
use sp_domain::*;
use sp_ros::*;
use sp_runner_api::*;
use std::collections::HashMap;
use std::thread;
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};
use crate::{helpers,planning};

// some planning constants
const LVL0_MAX_STEPS: u32 = 25;
const LVL1_MAX_STEPS: u32 = 50;
const LVL1_CUTOFF: u32 = 15;
const LVL1_LOOKOUT: f32 = 0.5;
const LVL1_MAX_TIME: Duration = Duration::from_secs(30);

pub fn launch_model(model: Model, initial_state: SPState) -> Result<(), Error> {
    let (mut node, comm) = set_up_ros_comm(&model)?;

    let (tx_runner, rx_runner): (Sender<SPRunnerInput>, Receiver<SPRunnerInput>) =
        channel::bounded(3);
    let (tx_planner, _rx_planner): (Sender<PlannerTask>, Receiver<PlannerTask>) =
        channel::unbounded();

    merger(comm.rx_mess.clone(), tx_runner.clone());
    ticker(Duration::from_millis(100), tx_runner.clone());

    node_handler(
        Duration::from_millis(1000),
        Duration::from_secs(1000),
        &model,
        comm.rx_node.clone(),
        comm.tx_node_cmd.clone(),
        tx_runner.clone(),
        comm.rx_commands.clone()
    );
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
        let runner_model = helpers::make_runner_model(&model);
        let mut runner = make_new_runner(&model, runner_model, initial_state);
        let mut prev_goals: HashMap<usize,
                                    Vec<(Predicate, Option<Predicate>)>> = HashMap::new();
        let mut store = planning::PlanningStore::default();
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
                                        store = planning::PlanningStore::default();
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
                                    store = planning::PlanningStore::default();
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
                            let p = p.parent().add_child("state");
                            let os = runner.state().sp_value_from_path(&p).unwrap_or(&e);
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
                            let goal_name = p.parent().add_child("goal");
                            println!("goal name {}", goal_name);

                            let g0 = &runner.goals[0];
                            let i: &IfThen = g0.iter().find(|g| g.path() == &goal_name).unwrap();

                            let goal =
                                if let Some(actions) = &i.actions {
                                    Predicate::AND(actions.iter().map(|a| a.to_concrete_predicate(runner.state())
                                                                      .expect("weird goal")).collect())
                                } else {
                                    Predicate::FALSE
                                };

                            let goal = vec![(goal, None)];

                            let pr = planning::plan_with_cache(&runner.transition_system_models[0], goal.as_slice(),
                                                               runner.state(), LVL0_MAX_STEPS, &mut store);

                            if !pr.plan_found {
                                println!("operation still problematic...");
                            } else {
                                let state = p.parent().add_child("state");
                                runner.ticker.state.force_from_path(&state, "i".to_spvalue()).unwrap();
                                something_was_fixed = true;
                            }

                            !pr.plan_found
                        });

                        if something_was_fixed {
                            // check all high level ops with error states. maybe we can move some of them
                            // back to their executing state.
                            // HACKS!

                            for p in &runner.hl_operation_states {
                                if runner.state().sp_value_from_path(p).unwrap() == &"error".to_spvalue() {
                                    println!("checking if we can remove error on high level operation: {}", p);
                                    let goal_path = p.parent().add_child("goal");
                                    println!("goal name {}", goal_path);

                                    let g1 = &runner.goals[1];
                                    let i: &IfThen = g1.iter().find(|g| g.path() == &goal_path).unwrap();
                                    let goal = vec![(i.goal().clone(), None)];

                                    let pr = planning::plan_async_with_cache(&runner.transition_system_models[1], &goal, runner.state(), &disabled_operations,
                                                                             LVL1_MAX_STEPS, LVL1_CUTOFF, LVL1_LOOKOUT, LVL1_MAX_TIME, store_async.clone());

                                    if pr.plan_found {
                                        let state_path = p.parent().add_child("state");
                                        log_info!("automatically restarting operation {}", p);
                                        runner.ticker.state
                                            .force_from_path(&state_path,
                                                             "e".to_spvalue()).unwrap();
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
                println!("state changed? {}", state_has_probably_changed);
                println!("transition fired? {}", !runner.last_fired_transitions.is_empty());
                println!("ticked? {}", ticked);
            }

            let disabled = runner.disabled_paths();
            let mut enabled_state = runner.state().projection();
            enabled_state.state.retain(|(p,_)| !disabled.iter().any(|d| p.is_child_of(d)));
            tx_state_out
                .send(enabled_state.clone_state())
                .expect("tx_state:out");

            // send out runner info.
            let mut runner_modes = vec![];
            if !runner.disabled_paths().is_empty() { runner_modes.push("waiting");}
            if bad_state { runner_modes.push("bad state");}
            if !disabled_operations.is_empty() { runner_modes.push("disabled operations"); }
            let runner_info = RunnerInfo {
                state: runner.state().clone(),
                mode: runner_modes.join(" | "),
                ..RunnerInfo::default()
            };

            tx_runner_info.send(runner_info).expect("tx_runner_info");

            let disabled = runner.disabled_paths();
            if !disabled.is_empty() {
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

                for s in runner.transition_system_models[0].specs.iter() {
                    if !s.invariant().eval(runner.state()) {

                    }
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
            for (i,g) in goals.iter().enumerate() {
                println!("Goals for {}", i);
                for g in g {
                    println!("{}", g.0);
                }
                println!("--");
            }

            for (i, (ts, goals)) in ts_models.iter().zip(goals.iter()).enumerate().rev() {
                let mut ts = ts.clone();
                if i == 1 {
                    ts.transitions.retain(|t| !disabled_operations.contains(t.path()));
                }
                let ts = &ts;

                let planner = SPPath::from_slice(&["runner", "planner", &i.to_string()]);
                if runner.ticker.state.sp_value_from_path(&planner)
                    .unwrap_or(&false.to_spvalue()) != &true.to_spvalue() {
                        continue;
                    }

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

                let replan = {
                    let is_empty = prev_goal.is_none();
                    if is_empty {
                        log_info!("replanning because previous goal was empty. {}", i);
                    }
                    is_empty
                } || {
                    let ok = &prev_goal.map(|g| g == goals).unwrap_or(false);
                    if !ok {
                        log_info!("replanning because goal changed. {}", i);
                        prev_goal.map(|g| g.iter().for_each(|g| {
                            println!("prev goals {}", g.0)
                        }));
                    }
                    !ok
                } || {
                    let now = std::time::Instant::now();

                    let ok = if i == 1 {
                        runner
                            .check_goals_op_model(runner.state(), &gr, &runner.plans[i],
                                                  &runner.transition_system_models[i])
                    } else {
                        runner
                            .check_goals_fast(runner.state(), &gr, &runner.plans[i],
                                              &runner.transition_system_models[i])
                    };

                    if now.elapsed().as_millis() > 100 {
                        println!("WARNINIG goal check for {}: {} (took {}ms)", i, ok, now.elapsed().as_millis());
                    }

                    if !ok {
                        println!("goal check for {}: {} (took {}ms)", i, ok, now.elapsed().as_millis());
                        log_info!("replanning because we cannot reach goal. {}", i);
                    }
                    !ok
                };

                if replan {
                    // temporary hack -- actually probably not so
                    // temporary, this is something we need to deal with
                    if i == 1 {
                        println!("resetting all operation state");
                        for p in &runner.operation_states {
                            let start = p.parent().add_child("start"); // go from /state to /start
                            if disabled_operations.contains(&start) { continue }
                            runner.ticker.state.force_from_path(&p, "i".to_spvalue()).unwrap();
                        }
                    }

                    println!("computing plan for namespace {}", i);

                    let planner_result = if i == 0 {
                        // skip heuristic for the low level
                        let mut pr = planning::plan_with_cache(&ts, &goals, runner.state(),
                                                               LVL0_MAX_STEPS, &mut store);
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
                        // no low level plan found, we are in trouble.

                        // look for the problematic goals
                        let g0 = &runner.goals[i];
                        let ifthens: Vec<&IfThen> = g0.iter()
                            .filter(|g| g.condition.eval(runner.state())).collect();

                        for i in ifthens {
                            let goal = vec![(i.goal().clone(), None)];

                            let pr = planning::plan_with_cache(&ts, goal.as_slice(), runner.state(), LVL0_MAX_STEPS, &mut store);
                            if !pr.plan_found {
                                let offending_op = i.path().parent();
                                log_warn!("offending low level operation: {}", offending_op);
                                runner.ticker.state
                                    .force_from_path(&offending_op.clone().add_child("state"),
                                                     "error".to_spvalue()).unwrap();
                                disabled_operations.push(offending_op.add_child("start"));
                            }
                        }
                    }

                    if no_plan && i == 1 {
                        // no high level plan found, we are in trouble.

                        // look for the problematic goals
                        let g1 = &runner.goals[i];
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
                                    .force_from_path(&offending_op.clone().add_child("state"),
                                                     "error".to_spvalue()).unwrap();
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
    rx_node: Receiver<NodeMode>,
    tx_state_out: Sender<SPState>,
    tx_runner_info: Sender<RunnerInfo>,
    tx_node_cmd: Sender<NodeCmd>,
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

    // Node handler comm
    let (tx_in_node, rx_node) = channel::unbounded();
    let tx_node_cmd = ros_node_comm_setup(&mut node, model, tx_in_node)?;

    Ok((
        node,
        RosCommSetup {
            rx_mess,
            rx_commands: rx_commands,
            rx_node,
            tx_state_out,
            tx_runner_info,
            tx_node_cmd,
        },
    ))
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

#[derive(Debug)]
struct NodeState {
    resource: SPPath,
    cmd: String,
    mode: String,
    time: Instant,
    cmd_msg: Option<MessageField>,
}

fn node_handler(
    freq: Duration,
    deadline: Duration,
    model: &Model,
    rx_node: Receiver<NodeMode>,
    tx_node: Sender<NodeCmd>,
    tx_runner: Sender<SPRunnerInput>,
    rx_commands: Receiver<RunnerCommand>
) {
    let mut nodes: HashMap<SPPath, NodeState> = model
        .all_resources()
        .iter()
        .map(|r| {
            let r: &Resource = r;
            let cmd_msg = r.get_message("command");
            (
                r.path().clone(),
                NodeState {
                    resource: r.path().clone(),
                    cmd: "init".to_string(),
                    mode: String::new(),
                    time: Instant::now(),
                    cmd_msg,
                },
            )
        })
        .collect();

    let tick = channel::tick(freq);

    fn mode_from_node(
        mode: Result<NodeMode,
        channel::RecvError>,
        nodes: &mut HashMap<SPPath, NodeState>,
        tx_runner: Sender<SPRunnerInput>,
    ) -> bool {
        match mode {
            Ok(n) => {
                let x = nodes.entry(n.resource.clone()).or_insert(NodeState {
                    resource: n.resource.clone(),
                    cmd: "init".to_string(),
                    mode: n.mode.clone(),
                    time: n.time_stamp.clone(),
                    cmd_msg: None,
                });

                x.mode = n.mode;
                if x.cmd == "init" {
                    // update goal state vars
                    tx_runner
                        .send(SPRunnerInput::NodeChange(n.echo))
                        .expect("Hmm, why is the runner dead?");
                }
                x.cmd = "run".to_string();
                x.time = n.time_stamp;
                true
            }
            Err(e) => {
                println!(
                    "The node ROS channel is dead (in the node handler)!: {:?}",
                    e
                );
                false
            }
        }
    }
    fn tick_node(
        time: Result<Instant, channel::RecvError>,
        nodes: &mut HashMap<SPPath, NodeState>,
        deadline: Duration,
        tx_node: Sender<NodeCmd>,
        tx_runner: Sender<SPRunnerInput>,
    ) -> bool {
        match time {
            Ok(time) => {
                let mut resource_state: Vec<(SPPath, SPValue)> = vec![];
                for (r, n) in nodes {
                    // TODO: Handle handshake with SP and node and change cmd when echo is written

                    // Handle lack of response
                    if n.time.elapsed() > deadline {
                        println!("Node {} is not responding", r.clone());
                        n.mode = String::new();
                        n.cmd = "init".to_string();
                    }

                    let cmd = NodeCmd {
                        resource: r.clone(),
                        mode: n.cmd.clone(),
                        time_stamp: time.clone(),
                    };

                    tx_node.send(cmd).unwrap();

                    let enabled = n.cmd == "run".to_string()
                        && (!n.mode.is_empty() || n.mode != "init".to_string());
                    resource_state.push((r.clone(), enabled.to_spvalue()));
                }
                let rs = SPState::new_from_values(&resource_state);
                tx_runner.send(SPRunnerInput::NodeChange(rs)).unwrap();

                true
            }
            Err(e) => {
                println!("The ticker is dead (in the node handler)!: {:?}", e);
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

    thread::spawn(move || loop {
        let res;
        crossbeam::select! {
            recv(rx_node) -> mode => res = mode_from_node(mode, &mut nodes, tx_runner.clone()),
            recv(tick) -> tick_time => res = tick_node(tick_time, &mut nodes, deadline, tx_node.clone(), tx_runner.clone()),
            recv(rx_commands) -> cmd => res = cmd_from_node(cmd, tx_runner.clone()),
        };
        if !res {
            break;
        }
    });
}

// TODO: do we keep the old RunnerModel?
fn make_new_runner(model: &Model, rm: RunnerModel, initial_state: SPState) -> SPRunner {
    let mut trans = vec![];
    let mut restrict_controllable = vec![];
    let mut restrict_op_controllable = vec![];
    let false_trans = Transition::new("empty", Predicate::FALSE, vec![], vec![], true);
    rm.op_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_op_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    rm.op_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    // high level ops are never restricted
    rm.hl_op_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });
    rm.hl_op_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    rm.ab_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    rm.ab_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });


    let ops = rm.op_states.iter().map(|v| v.path().clone()).collect();
    let hl_ops = rm.hl_op_states.iter().map(|v| v.path().clone()).collect();

    let mut all_vars = rm.model.vars.clone();
    all_vars.extend(rm.state_predicates);

    let mut runner = SPRunner::new(
        "test",
        trans,
        all_vars,
        vec![rm.goals.clone(), rm.hl_goals.clone()],
        vec![],
        vec![],
        vec![rm.model.clone(), rm.op_model.clone()],
        model.all_resources().iter().map(|r| r.path().clone()).collect(),
        ops,
        hl_ops,
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

    runner
}
