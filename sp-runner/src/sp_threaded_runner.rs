use super::sp_runner::*;
use crossbeam::{channel, Receiver, Sender};
use failure::Error;
use sp_domain::*;
use sp_ros;
use sp_runner_api::*;
use std::collections::{HashMap, HashSet};
use std::thread;
use std::time::{Duration, Instant};

pub fn launch_model(model: Model, initial_state: SPState) -> Result<(), Error> {
    let (mut node, comm) = set_up_ros_comm(&model)?;

    let (tx_runner, rx_runner): (Sender<SPRunnerInput>, Receiver<SPRunnerInput>) = channel::bounded(3);
    let (tx_planner, rx_planner): (Sender<PlannerTask>, Receiver<PlannerTask>) = channel::unbounded();

    merger(comm.rx_mess.clone(), tx_runner.clone());
    ticker(Duration::from_millis(500), tx_runner.clone());
    planner(tx_runner.clone(), rx_planner);
    node_handler(
        Duration::from_millis(1000),
        Duration::from_secs(3),
        &model,
        comm.rx_node.clone(),
        comm.tx_node_cmd.clone(),
        tx_runner.clone(),
    );
    runner(&model, initial_state, rx_runner, comm.tx_state_out, comm.tx_runner_info, tx_planner);

    loop {
        // blocking ros spinning
        sp_ros::spin(&mut node);
    }
}


fn runner(
    model: &Model,
    initial_state: SPState,
    rx_input: Receiver<SPRunnerInput>,
    tx_state_out: Sender<SPState>,
    tx_runner_info: Sender<RunnerInfo>,
    tx_planner: Sender<PlannerTask>,
) {
    let model = model.clone();
    thread::spawn(move || {
        let runner_model = crate::helpers::make_runner_model(&model);
        let mut runner = make_new_runner(&model, runner_model, initial_state);
        let resources: Vec<SPPath> = model.resources().iter().map(|r| r.path().clone()).collect();

        loop {
            let mut tick = false;
            let mut state_has_probably_changed = false;
            let input = rx_input.recv();
            if let Ok(msg) = input {
                match msg {
                    SPRunnerInput::StateChange(s) => {
                        if !runner.state().are_new_values_the_same(&s) {
                            runner.input(SPRunnerInput::StateChange(s));
                            state_has_probably_changed = true;
                        }
                    },
                    SPRunnerInput::NodeChange(s) => {
                        if !runner.state().are_new_values_the_same(&s) {
                            runner.input(SPRunnerInput::NodeChange(s));
                            state_has_probably_changed = true;
                        }
                    },
                    SPRunnerInput::Tick => {
                        runner.input(SPRunnerInput::Tick);
                        tick = true;
                    },
                    SPRunnerInput::AbilityPlan(plan) => {
                        runner.input(SPRunnerInput::AbilityPlan(plan));
                        runner.input(SPRunnerInput::Tick);
                    },
                    SPRunnerInput::OperationPlan(plan) => {
                        runner.input(SPRunnerInput::OperationPlan(plan));
                        runner.input(SPRunnerInput::Tick);
                    },
                    SPRunnerInput::Settings => {}// TODO},
                }

            } else {
                println!("The runner channel broke? - {:?}", input);
                break;
            }


            if !runner.last_fired_transitions.is_empty() {
                println!("fired:");
                runner
                    .last_fired_transitions
                    .iter()
                    .for_each(|x| println!("{:?}", x));
            }

            if state_has_probably_changed || !runner.last_fired_transitions.is_empty() {
                println! {""};
                println!("The State: {}", runner.state());
                println! {""};
            }


            // For now, we will send out all the time, but soon we should check this
            tx_state_out.send(runner.state().clone()).expect("tx_state:out");
            // send out runner info.
            let runner_info = RunnerInfo {
                state: runner.state().clone(),
                ..RunnerInfo::default()
            };

            tx_runner_info.send(runner_info).expect("tx_runner_info");

            let planning = PlannerTask{
                ts: runner.transition_system_model.clone(),
                ots: runner.operation_planning_model.clone(),
                state: runner.state().clone(),
                goal: runner.goal(),
                disabled_paths: runner.disabled_paths(),
            };
            tx_planner.send(planning);


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
            rx_commands,
            rx_node,
            tx_state_out,
            tx_runner_info,
            tx_node_cmd,
        },
    ))
}

fn merger(rx_mess: Receiver<sp_ros::RosMessage>, tx_runner: Sender<SPRunnerInput>) {
    thread::spawn(move || {
        let mut hack = 0;
        let mut prev_state = SPState::new();

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
    ts: TransitionSystemModel,
    ots: TransitionSystemModel,
    state: SPState,
    goal: Vec<(Predicate, Option<Predicate>)>,
    disabled_paths: Vec<SPPath>,
}


            // // simulate the operation planning/scheduling layer
            // let buffer1 = SPPath::from_string("cubes/buffer1_holding");
            // let buffer2 = SPPath::from_string("cubes/buffer2_holding");
            // // two configurations to plan between
            // let c1 = p!([ p:buffer1 == 2] && [ p:buffer2 == 1 ]);
            // let c2 = p!([ p:buffer1 == 1] && [ p:buffer2 == 2 ]);

            // // meta operations...
            // if hack == 0 && c1.eval(&runner.state()) {
            //     hack = 1;
            //     println!("in configuration 1, start op planning");

            //     let max_steps = 20;
            //     let planner_result = crate::planning::plan(&runner.operation_planning_model,
            //                                                &[(c2,None)], &runner.state(), max_steps);
            //     println!("operation plan is");
            //     planner_result.trace.iter().for_each(|f| { println!("Transition: {}", f.transition); });

            //     let (tr, s) = crate::planning::convert_planning_result(&runner.operation_planning_model, planner_result, true);
            //     let plan = SPPlan{plan: tr, state_change: s};
            //     runner.input(SPRunnerInput::OperationPlan(plan));

            // } else if hack == 1 && c2.eval(&runner.state()) {
            //     println!("in configuration 1, start op planning");
            //     hack = 0;

            //     let max_steps = 20;
            //     let planner_result = crate::planning::plan(&runner.operation_planning_model,
            //                                                &[(c1,None)], &runner.state(), max_steps);
            //     println!("operation plan is");
            //     planner_result.trace.iter().for_each(|f| { println!("Transition: {}", f.transition); });

            //     let (tr, s) = crate::planning::convert_planning_result(&runner.operation_planning_model, planner_result, true);
            //     let plan = SPPlan{plan: tr, state_change: s};
            //     runner.input(SPRunnerInput::OperationPlan(plan));
            // }

            // //println!("WE GOT:\n{}", &s);
            // let old_g = runner.goal();
            // //if runner.state().are_new_values_the_same(&s)
            // {
            //     runner.input(SPRunnerInput::StateChange(s));
            //     let new_g = runner.goal();
            //     // println!("GOALS");
            //     // new_g.iter().for_each(|x| println!("{:?}", x));
            //     if !runner.last_fired_transitions.is_empty() {
            //         println!("fired:");
            //         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
            //     }

            //     if prev_state.clone().extract() != runner.state().clone().extract() {
            //         println!{""};
            //         println!("The State:\n{}", runner.state());
            //         println!{""};
            //         prev_state = runner.state().clone();
            //     }

            //     tx_out.send(runner.state().clone()).unwrap();
            //     // send out runner info.
            //     let runner_info = RunnerInfo {
            //         state: runner.state().clone(),
            //         .. RunnerInfo::default()
            //     };

            //     tx_out_misc.send(runner_info).unwrap();


            //     if old_g != new_g {
            //         println!("NEW GOALS");
            //         println!("*********");

            //         let max_steps = 100; // arbitrary decision
            //         let planner_result = crate::planning::plan(&runner.transition_system_model, &new_g, &runner.state(), max_steps);
            //         assert!(planner_result.plan_found);
            //         //println!("new plan is");
            //         //planner_result.trace.iter().for_each(|f| { println!("Transition: {}\nState:\n{}", f.transition, f.state); });
            //         let (tr, s) = crate::planning::convert_planning_result(&runner.transition_system_model, planner_result, false);
            //         let plan = SPPlan{plan: tr, state_change: s};
            //         runner.input(SPRunnerInput::AbilityPlan(plan));

fn planner(tx_runner: Sender<SPRunnerInput>, rx_planner: Receiver<PlannerTask>) {
    thread::spawn(move || {
        let mut prev_state = SPState::new();
        let mut prev_goal: Vec<(Predicate, Option<Predicate>)> = vec![];
        let mut hack = 0;
        loop {
            let pt: PlannerTask = if rx_planner.is_empty() {
                match rx_planner.recv() {
                    Ok(pt) => pt,
                    Err(e) => {
                        println!(
                            "The planning channel from the runner is dead (in planner)!: {:?}",
                            e
                        );
                        return;
                    }
                }
            } else {
                let mut xs: Vec<PlannerTask> = rx_planner.try_iter().collect();
                if xs.is_empty() {
                    println!("The planning channel from the runner is bad (in planner)!");
                    return;
                }
                xs.remove(xs.len() - 1)
            };



            // simulate the operation planning/scheduling layer
            let buffer1 = SPPath::from_string("cubes/buffer1_holding");
            let buffer2 = SPPath::from_string("cubes/buffer2_holding");
            // two configurations to plan between
            let c1 = p!([ p:buffer1 == 2] && [ p:buffer2 == 1 ]);
            let c2 = p!([ p:buffer1 == 1] && [ p:buffer2 == 2 ]);

            // meta operations...
            if hack == 0 && c1.eval(&pt.state) && pt.disabled_paths.is_empty() {
                hack = 1;
                println!("in configuration 1, start op planning");

                let max_steps = 20;
                let planner_result = crate::planning::plan(&pt.ots,
                                                           &[(c2,None)], &pt.state, max_steps);
                println!("operation plan is");
                planner_result.trace.iter().for_each(|f| { println!("Transition: {}", f.transition); });

                let (tr, s) = crate::planning::convert_planning_result(&pt.ots, planner_result, true);
                let plan = SPPlan{plan: tr, state_change: s};
                tx_runner.send(SPRunnerInput::OperationPlan(plan)).unwrap();
            } else if hack == 1 && c2.eval(&pt.state) && pt.disabled_paths.is_empty() {
                println!("in configuration 1, start op planning");
                hack = 0;

                let max_steps = 20;
                let planner_result = crate::planning::plan(&pt.ots,
                                                           &[(c1,None)], &pt.state, max_steps);
                println!("operation plan is");
                planner_result.trace.iter().for_each(|f| { println!("Transition: {}", f.transition); });

                let (tr, s) = crate::planning::convert_planning_result(&pt.ots, planner_result, true);
                let plan = SPPlan{plan: tr, state_change: s};
                tx_runner.send(SPRunnerInput::OperationPlan(plan)).unwrap();
            }




            if prev_goal != pt.goal && pt.disabled_paths.is_empty() {
                println!("NEW GOALS");
                println!("*********");

                let max_steps = 100; // arbitrary decision
                let planner_result = crate::planning::plan(&pt.ts, &pt.goal, &pt.state, max_steps);
                assert!(planner_result.plan_found);
                //println!("new plan is");
                //planner_result.trace.iter().for_each(|f| { println!("Transition: {}\nState:\n{}", f.transition, f.state); });
                let (tr, s) = crate::planning::convert_planning_result(&pt.ts, planner_result, false);
                let plan = SPPlan {
                    plan: tr,
                    state_change: s,
                };

                let res = tx_runner.send(SPRunnerInput::AbilityPlan(plan));
                if res.is_err() {
                    println!("The runner channel is dead (in the planner)!: {:?}", res);
                    break;
                }
                prev_goal = pt.goal;
                prev_state = pt.state;
            }
        }
    });
}

#[derive(Debug)]
struct NodeState {
    resource: SPPath,
    cmd: String,
    mode: String,
    time: Instant,
    echo: Option<serde_json::Value>,
}

use sp_ros::{NodeCmd, NodeMode};
fn node_handler(
    freq: Duration,
    deadline: Duration,
    model: &Model,
    rx_node: Receiver<NodeMode>,
    tx_node: Sender<NodeCmd>,
    tx_runner: Sender<SPRunnerInput>,
) {
    let resources: Vec<SPPath> = model.resources().iter().map(|r| r.path().clone()).collect();
    let mut nodes: HashMap<SPPath, NodeState> = resources
        .into_iter()
        .map(|r| {
            (
                r.clone(),
                NodeState {
                    resource: r,
                    cmd: "init".to_string(),
                    mode: String::new(),
                    time: Instant::now(),
                    echo: None,
                },
            )
        })
        .collect();
    let tick = channel::tick(freq);

    fn mode_from_node(
        mode: Result<NodeMode, channel::RecvError>,
        nodes: &mut HashMap<SPPath, NodeState>,
    ) -> bool {
        match mode {
            Ok(n) => {
                let x = nodes.entry(n.resource.clone()).or_insert(NodeState {
                    resource: n.resource.clone(),
                    cmd: "init".to_string(),
                    mode: n.mode.clone(),
                    time: n.time_stamp.clone(),
                    echo: Some(n.echo.clone()),
                });

                // TODO: Handle handshake with SP and node and upd using echo if needed
                x.mode = n.mode;
                x.time = n.time_stamp;
                x.echo = Some(n.echo);
                x.cmd = "run".to_string();

                // println!(
                //     "Node mode: {}, since: {}, node_mode: {:?}",
                //     x.resource.clone(),
                //     x.time.elapsed().as_millis(),
                //     x.mode.clone()
                // );

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
                let mut resource_state: Vec<(SPPath, SPValue)> = vec!();
                for (r, n) in nodes {
                    // TODO: Handle handshake with SP and node and change cmd when echo is written

                    // Handle lack of response
                    if n.time.elapsed() > deadline {
                        println!("Node {} is not responding", r.clone());
                        n.mode = String::new();
                        n.cmd = "init".to_string();
                        n.echo = None;
                    }

                    let cmd = NodeCmd {
                        resource: r.clone(),
                        mode: n.cmd.clone(),
                        time_stamp: time.clone(),
                    };

                    tx_node.send(cmd); //.unwrap();

                    let enabled = n.cmd == "run".to_string() && (!n.mode.is_empty() || n.mode != "init".to_string());
                    resource_state.push((r.clone(), enabled.to_spvalue()));
                }
                let rs = SPState::new_from_values(&resource_state);
                tx_runner.send(SPRunnerInput::NodeChange(rs));

                true
            }
            Err(e) => {
                println!("The ticker is dead (in the node handler)!: {:?}", e);
                false
            }
        }
    }

    thread::spawn(move || loop {
        let res;
        crossbeam::select! {
            recv(rx_node) -> mode => res = mode_from_node(mode, &mut nodes),
            recv(tick) -> tick_time => res = tick_node(tick_time, &mut nodes, deadline, tx_node.clone(), tx_runner.clone()),
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
    let mut runner = SPRunner::new(
        "test",
        trans,
        rm.state_predicates.clone(),
        rm.goals.clone(),
        vec!(),
        vec!(),
        rm.model.clone(),
        rm.op_model.clone(),
        model.resources().iter().map(|r| r.path().clone()).collect()
    );

    runner.input(SPRunnerInput::AbilityPlan(SPPlan {
        plan: restrict_controllable.clone(),
        state_change: SPState::new(),
    }));
    runner.input(SPRunnerInput::OperationPlan(SPPlan {
        plan: restrict_op_controllable.clone(),
        state_change: SPState::new(),
    }));
    runner.update_state_variables(initial_state);

    runner
}
