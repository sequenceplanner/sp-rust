use super::sp_runner::*;
use crossbeam::{channel, Receiver, Sender};
use failure::Error;
use sp_domain::*;
use sp_ros;
use sp_runner_api::*;
use std::collections::HashMap;
use std::thread;
use std::time::{Duration, Instant};

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
        sp_ros::spin(&mut node);
    }
}

fn runner(
    model: &Model, initial_state: SPState, rx_input: Receiver<SPRunnerInput>,
    tx_state_out: Sender<SPState>, tx_runner_info: Sender<RunnerInfo>,
    _tx_planner: Sender<PlannerTask>,
) {
    let model = model.clone();
    thread::spawn(move || {
        let runner_model = crate::helpers::make_runner_model(&model);
        let mut runner = make_new_runner(&model, runner_model, initial_state);
        let mut prev_goals: HashMap<usize, Option<Vec<(Predicate, Option<Predicate>)>>> = HashMap::new();
        let mut store = crate::planning::PlanningStore::default();
        let mut store2 = crate::planning::PlanningStore::default();
        // let timer = Instant::now();

        let mut now = Instant::now();

        let mut low_fail = false;
        'outer: loop {
            let elapsed_ms = now.elapsed().as_millis();
            if elapsed_ms > 5 {
                println!("RUNNER TICK TIME: {}ms", elapsed_ms);
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
                    SPRunnerInput::NewPlan(idx, _) => {
                        state_has_probably_changed = true;

                        // temporary hack
                        if idx == 1 {
                            println!("new plan, resetting all operation state");
                            for p in &runner.operation_states {
                                runner.ticker.state.force_from_path(&p, "i".to_spvalue()).unwrap();
                            }
                        }

                        runner.input(msg.clone());
                        runner.input(SPRunnerInput::Tick);
                    }
                    SPRunnerInput::Settings(_) => {
                        runner.input(msg);
                        state_has_probably_changed = true;
                        low_fail = true; // force replan
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
            if !state_has_probably_changed && runner.last_fired_transitions.is_empty() && !ticked {
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
            let runner_info = RunnerInfo {
                state: runner.state().clone(),
                ..RunnerInfo::default()
            };

            tx_runner_info.send(runner_info).expect("tx_runner_info");

            let disabled = runner.disabled_paths();
            if !disabled.is_empty() {
                println!("still waiting... do nothing");
                continue;
            }

            if SPRunner::bad_state(runner.state(), &runner.transition_system_models[0]) {
                continue;
            }

            // println!("The State:\n{}", runner.state());

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
                // for each namespace, check if we need to replan because
                // 1. got new goals from the runner or
                // 2. can no longer reach our goal (because the state has changed)

                // Because the plan is already encoded in the guards
                // of the runner transitions we can use the runner for
                // this purpose.

                // This is also true for the goals -> they are a function of the state.

                let prev_goal = prev_goals.get(&i).unwrap_or(&None).clone();

                let gr: Vec<&Predicate> = goals
                    .iter()
                    .map(|g| &g.0).collect(); // dont care about the invariants for now

                let replan = {
                    let is_empty = prev_goal.is_none();
                    if is_empty {
                        println!("replanning because previous goal was empty. {}", i);
                    }
                    is_empty
                } || {
                    let ok = &prev_goal.as_ref().map(|g| g == goals).unwrap_or(false);
                    if !ok && goals.len() > 0 {
                        println!("replanning because goal changed. {}", i);
                        prev_goal.as_ref().iter().for_each(|g| g.iter().for_each(|g| {
                            println!("prev goals {}", g.0);
                        }));
                    }
                    !ok
                } || (!runner.plans[i].is_blocked && {
                    let now = std::time::Instant::now();

                    let mut state = runner.state().clone();
                    let rpi = SPPath::from_string("runner/plans/1");
                    let mut v = state.sp_value_from_path(&rpi).unwrap().clone();
                    if let SPValue::Int32(i) = &mut v {
                        v = SPValue::Int32(*i-1);
                    }
                    let _res = state.force_from_path(&rpi, v.clone());

                    let ok =
                        if i == 0 {
                            runner
                                .check_goals_fast(&state, &gr, &runner.plans[i],
                                                  &runner.transition_system_models[i])
                        } else {
                            if low_fail {
                                println!("replanning because planning failed at the lower level");
                                low_fail = false;
                                if gr.is_empty() {
                                    true
                                } else {
                                    false
                                }
                            } else {
                                true
                            }
                        };

                    if now.elapsed().as_millis() > 100 {
                        println!("WARNINIG goal check for {}: {} (took {}ms)", i, ok, now.elapsed().as_millis());
                    }

                    if !ok {
                        println!("goal check for {}: {} (took {}ms)", i, ok, now.elapsed().as_millis());
                        println!("replanning because we cannot reach goal. {}", i);
                    }
                    !ok
                });

                if replan {

                    if false // goals.len() == 0
                    {
                        // println!("NO ACTIVE GOALS...");
                    } else {

                        // temporary hack -- actually probably not so
                        // temporary, this is something we need to deal with
                        if i == 1 {
                            println!("resetting all operation state");
                            for p in &runner.operation_states {
                                runner.ticker.state.force_from_path(&p, "i".to_spvalue()).unwrap();
                            }
                        }

                        println!("computing plan for namespace {}", i);

                        let mut planner_result = if i == 0 {
                            // skip heuristic for the low level
                            let max_steps = 40; // low to be able to fail quickly
                            crate::planning::plan_with_cache(&ts, &goals, runner.state(), max_steps, &mut store)
                        } else {
                            let max_steps = 50;
                            let cutoff = 15;
                            let lookout = 1.0;
                            let max_time = Duration::from_secs(15);
                            // crate::planning::plan_async(&ts, &goals, runner.state(), max_steps, cutoff, lookout, max_time)
                            crate::planning::plan_async_with_cache(&ts, &goals, runner.state(), max_steps, cutoff, lookout, max_time, &mut store2)
                            // crate::planning::plan_with_cache(&ts, &goals, runner.state(), max_steps, &mut store)
                        };

                        // check length > 2 because for the heuristic
                        // to improve the situation sense we need at
                        // least two deliberation trans + 1 other
                        if i == 0 && planner_result.plan_length > 2 {
                            // try out our greedy packing algorithm.
                            let plan = planner_result.trace.iter()
                                .filter(|f| f.transition != SPPath::new())
                                .flat_map(|f| ts.transitions.iter().find(|t| t.path() == &f.transition))
                                .collect::<Vec<_>>();

                            let delib = plan.iter().filter(|t| t.controlled()).collect::<Vec<_>>();

                            println!("original plan");
                            for t in &plan {
                                println!("{}", t.path());
                            }

                            //let mut new_plans = Vec::new();
                            let mut new_plan = plan.clone();
                            let mut idx = 1; // start at 2
                            loop {
                                if idx >= new_plan.len() {
                                    println!("done...");
                                    break;
                                }

                                if !delib.contains(&&new_plan[idx]) {
                                    println!("not moving {} to {}, this is not delib", new_plan[idx].path(), idx - 1);
                                    idx += 1;
                                    continue;
                                }

                                if delib.contains(&&new_plan[idx-1]) {
                                    // don't move delib transitions to before other delib transitions
                                    println!("not moving {} to {}, previous is delib", new_plan[idx].path(), idx - 1);
                                    idx += 1;
                                    continue;
                                }

                                // else try to bubble it up.
                                let mut p = new_plan.clone();
                                let t = p.remove(idx);

                                println!("moving {} to {}", t.path(), idx - 1);
                                p.insert(idx - 1, t);

                                // check if the new plan successfully reaches the goals
                                let plan: Vec<SPPath> = p.iter().map(|t|t.path().clone()).collect();

                                let result = runner.check_goals_exact(runner.state(), &gr, &plan, &ts);
                                println!("plan takes us to goal? {}", result);

                                if result {
                                    new_plan = p;
                                    // can only move up to zero
                                    if idx > 1 {
                                        idx-=1;
                                    }
                                } else {
                                    idx+=1;
                                }
                            }

                            println!("Final plan after heuristic");
                            for t in &new_plan {
                                println!("{}", t.path());
                            }
                            println!("--------");


                            // change original planner result
                            let plan: Vec<SPPath> = new_plan.iter().map(|t|t.path().clone()).collect();
                            let trace = crate::planning::make_planning_trace(&ts, &plan, &planner_result.trace[0].state);
                            planner_result.trace = trace;
                        }

                        let is_empty = !planner_result.plan_found;
                        if is_empty {
                            println!("No plan was found for namespace {}! time to fail {}ms", i, planner_result.time_to_solve.as_millis());
                            if i == 0 {
                                low_fail = true;
                            }
                            prev_goals.insert(i, None);
                        }

                        let plan_p = SPPath::from_slice(&["runner", "plans", &i.to_string()]);
                        let (tr, s) = if i == 1 {
                            // test packing
                            crate::planning::convert_planning_result_with_packing_heuristic(&ts, &planner_result, &plan_p)
                            //crate::planning::convert_planning_result(&ts, &planner_result, &plan_p)
                        } else {
                            crate::planning::convert_planning_result(&ts, &planner_result, &plan_p)
                        };

                        let trans = planner_result.trace.iter()
                            .filter_map(|f|
                                        if f.transition.is_empty() {
                                            None
                                        } else {
                                            Some(f.transition.clone())
                                        }).collect();

                        let plan = SPPlan {
                            is_blocked: is_empty,
                            plan: tr,
                            included_trans: trans,
                            state_change: s,
                        };

                        // update last set of goals
                        prev_goals.insert(i, Some(goals.clone()));

                        runner.input(SPRunnerInput::NewPlan(i as i32, plan));

                        // if we had goals, skip the next level
                        if goals.len() > 0 {
                            continue 'outer; // wait until next iteration to check lower levels
                        } else {
                            // else we need to check them also
                            low_fail = true;
                        }
                    }
                }
            }
        }
    });
}

struct RosCommSetup {
    rx_mess: Receiver<sp_ros::RosMessage>,
    rx_commands: Receiver<RunnerCommand>,
    rx_node: Receiver<sp_ros::NodeMode>,
    tx_state_out: Sender<SPState>,
    tx_runner_info: Sender<RunnerInfo>,
    tx_node_cmd: Sender<sp_ros::NodeCmd>,
}

fn set_up_ros_comm(model: &Model) -> Result<(sp_ros::RosNode, RosCommSetup), Error> {
    // start ros node
    let mut node = sp_ros::start_node()?;

    // data from resources to runner
    // setup ros pub/subs. tx_out to send out to network
    let (tx_in, rx_mess) = channel::bounded(20);
    let tx_state_out: channel::Sender<SPState> = sp_ros::roscomm_setup(&mut node, model, tx_in)?;

    // misc runner data to/from the network.
    // setup ros pub/subs. tx_out to send out to network
    let (tx_in_misc, rx_commands) = channel::bounded(20);
    let tx_runner_info = sp_ros::roscomm_setup_misc(&mut node, tx_in_misc)?;

    // Node handler comm
    let (tx_in_node, rx_node) = channel::unbounded();
    let tx_node_cmd = sp_ros::ros_node_comm_setup(&mut node, model, tx_in_node)?;

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

fn merger(rx_mess: Receiver<sp_ros::RosMessage>, tx_runner: Sender<SPRunnerInput>) {
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

use sp_ros::{NodeCmd, NodeMode};
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
    );

    runner.input(SPRunnerInput::NewPlan(0, SPPlan {
        is_blocked: true,
        plan: restrict_controllable.clone(),
        included_trans: Vec::new(),
        state_change: SPState::new(),
    }));
    runner.input(SPRunnerInput::NewPlan(1, SPPlan {
        is_blocked: true,
        plan: restrict_op_controllable.clone(),
        included_trans: Vec::new(),
        state_change: SPState::new(),
    }));
    runner.update_state_variables(initial_state);

    runner
}
