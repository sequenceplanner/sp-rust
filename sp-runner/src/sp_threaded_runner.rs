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

    let (tx_runner, rx_runner): (Sender<SPRunnerInput>, Receiver<SPRunnerInput>) =
        channel::bounded(3); // The runner should backpressure
    let (tx_planner, rx_planner): (Sender<PlannerTask>, Receiver<PlannerTask>) =
        channel::bounded(3); // The runner should backpressure

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

    loop {
        // blocking ros spinning
        sp_ros::spin(&mut node);
    }
}

struct RunnerComm {}

fn runner(
    model: &Model,
    initial_state: SPState,
    rx_input: Receiver<SPRunnerInput>,
    tx_state_out: Sender<SPState>,
    tx_runner_info: Sender<RunnerInfo>,
    tx_planner: Sender<PlannerTask>,
) {
    thread::spawn(move || {
        let runner_model = crate::helpers::make_runner_model(&model);
        let mut runner = make_new_runner(runner_model, initial_state);

        // hack to wait for all measured to show at least once.
        let mut measured_states: HashSet<SPPath> = runner
            .transition_system_model
            .vars
            .iter()
            .filter_map(|v| {
                if v.variable_type() == VariableType::Measured {
                    Some(v.path().clone())
                } else {
                    None
                }
            })
            .collect();
        let mut first_complete_state = SPState::new();
        let mut waiting = true;

        // we extend the hack to also first listen to initial states for our command variables.
        let cm: HashMap<_, _> = model
            .resources()
            .iter()
            .flat_map(|r| r.get_command_mirrors())
            .collect();
        let cm_rev: HashMap<_, _> = model
            .resources()
            .iter()
            .flat_map(|r| r.get_command_mirrors_rev())
            .collect();

        let mut command_with_echoes: HashSet<SPPath> = runner
            .transition_system_model
            .vars
            .iter()
            .filter_map(|v| {
                if v.variable_type() == VariableType::Command && cm.contains_key(v.path()) {
                    Some(v.path().clone())
                } else {
                    None
                }
            })
            .collect();

        let mut resource_map = HashMap::new();
        let ticker = Instant::now();
        loop {
            // hack to wait for resources initially
            if waiting {
                if !measured_states.is_empty() || !command_with_echoes.is_empty() {
                    println!(
                        "Waiting for measured... {}",
                        measured_states
                            .iter()
                            .map(|x| x.to_string())
                            .collect::<Vec<String>>()
                            .join(", ")
                    );
                    println!(
                        "Waiting for command... {}",
                        command_with_echoes
                            .iter()
                            .map(|x| x.to_string())
                            .collect::<Vec<String>>()
                            .join(", ")
                    );

                    let s = s.extract();
                    for (p, v) in &s {
                        if let Some(&command) = cm_rev.get(p) {
                            if command_with_echoes.remove(command) {
                                first_complete_state.add_state_variable(command.clone(), v.clone());
                            }
                        } else {
                            if measured_states.remove(p) {
                                first_complete_state.add_state_variable(p.clone(), v.clone());
                            }
                        }
                    }
                    continue;
                } else {
                    waiting = false;
                    s = first_complete_state.clone();

                    // s.add_variable(SPPath::from_string("cubes/r1/command/0/ref_pos"),
                    //                "r1buffer".to_spvalue());
                    // s.add_variable(SPPath::from_string("cubes/r2/command/0/ref_pos"),
                    //                "r2table".to_spvalue());
                    println!("FIRST COMPLETE INITIAL STATE:\n{}", &s);
                }
            }

            //println!("WE GOT:\n{}", &s);
            let old_g = runner.goal();
            //if runner.state().are_new_values_the_same(&s)
            {
                runner.input(SPRunnerInput::StateChange(s));
                let new_g = runner.goal();
                // println!("GOALS");
                // new_g.iter().for_each(|x| println!("{:?}", x));
                if !runner.last_fired_transitions.is_empty() {
                    println!("fired:");
                    runner
                        .last_fired_transitions
                        .iter()
                        .for_each(|x| println!("{:?}", x));
                }

                println! {""};
                println!("The State: {}", runner.state());
                println! {""};

                comm.tx_state_out.send(runner.state().clone()).unwrap();
                // send out runner info.
                let runner_info = RunnerInfo {
                    state: runner.state().clone(),
                    ..RunnerInfo::default()
                };

                comm.tx_runner_info.send(runner_info).unwrap();

                if old_g != new_g {
                    println!("NEW GOALS");
                    println!("*********");

                    let max_steps = 100; // arbitrary decision
                    let planner_result = crate::planning::plan(
                        &runner.transition_system_model,
                        &new_g,
                        &runner.state(),
                        max_steps,
                    );
                    assert!(planner_result.plan_found);
                    //println!("new plan is");
                    //planner_result.trace.iter().for_each(|f| { println!("Transition: {}\nState:\n{}", f.transition, f.state); });
                    let (tr, s) = crate::planning::convert_planning_result(
                        &runner.transition_system_model,
                        planner_result,
                    );
                    let plan = SPPlan {
                        plan: tr,
                        state_change: s,
                    };
                    runner.input(SPRunnerInput::AbilityPlan(plan));

                    comm.tx_state_out.send(runner.state().clone()).unwrap();

                    // send out runner info.
                    let runner_info = RunnerInfo {
                        state: runner.state().clone(),
                        ..RunnerInfo::default()
                    };
                    comm.tx_runner_info.send(runner_info).unwrap();
                }
            }

            // let mess: Result<SPState, RecvError> = rx_input.recv();
            // if let Ok(s) = mess {

            // }
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
    state: SPState,
    goal: Vec<(Predicate, Option<Predicate>)>,
}

fn planner(tx_runner: Sender<SPRunnerInput>, rx_planner: Receiver<PlannerTask>) {
    thread::spawn(move || {
        let mut prev_state = SPState::new();
        let mut prev_goal: Vec<(Predicate, Option<Predicate>)> = vec![];
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

            if prev_goal != pt.goal {
                println!("NEW GOALS");
                println!("*********");

                let max_steps = 100; // arbitrary decision
                let planner_result = crate::planning::plan(&pt.ts, &pt.goal, &pt.state, max_steps);
                assert!(planner_result.plan_found);
                //println!("new plan is");
                //planner_result.trace.iter().for_each(|f| { println!("Transition: {}\nState:\n{}", f.transition, f.state); });
                let (tr, s) = crate::planning::convert_planning_result(&pt.ts, planner_result);
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
    let resources: Vec<SPPath> = model.resources().map(|r| r.path().clone()).collect();
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

                println!(
                    "Node mode: {}, since: {}, node_mode: {:?}",
                    n.resource.clone(),
                    n.time_stamp.elapsed().as_millis(),
                    n.clone()
                );

                // TODO: Handle handshake with SP and node and upd using echo if needed
                x.mode = n.mode;
                x.time = n.time_stamp;
                x.echo = Some(n.echo);

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
                }

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
fn make_new_runner(m: RunnerModel, initial_state: SPState) -> SPRunner {
    let mut trans = vec![];
    let mut restrict_controllable = vec![];
    let false_trans = Transition::new("empty", Predicate::FALSE, vec![], vec![], true);
    m.op_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });
    m.op_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });
    m.ab_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    m.ab_transitions.un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });
    let mut runner = SPRunner::new(
        "test",
        trans,
        m.state_predicates.clone(),
        m.goals.clone(),
        vec![],
        vec![],
        m.model.clone(),
    );
    runner.input(SPRunnerInput::AbilityPlan(SPPlan {
        plan: restrict_controllable,
        state_change: SPState::new(),
    }));
    runner.update_state_variables(initial_state);

    runner
}
