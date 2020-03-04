use super::sp_runner::*;
use sp_domain::*;
use sp_runner_api::*;
use failure::Error;
use sp_ros;
use std::thread;
use crossbeam::channel;
use std::collections::{HashSet,HashMap};
use std::time::{Instant, Duration};

pub fn launch_model(model: Model, initial_state: SPState) -> Result<(), Error> {
    let runner_model = crate::helpers::make_runner_model(&model);

    // start ros node
    let mut node = sp_ros::start_node()?;

    // data from resources to runner
    let (tx_in, rx_in) = channel::unbounded();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out: channel::Sender<SPState> = sp_ros::roscomm_setup(&mut node, &model, tx_in)?;

    // misc runner data to/from the network.
    let (tx_in_misc, rx_in_misc) = channel::unbounded();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out_misc = sp_ros::roscomm_setup_misc(&mut node, tx_in_misc)?;

    let mut runner = make_new_runner(runner_model, initial_state);

    // "runner"
    thread::spawn(move || {

        // hack to wait for all measured to show at least once.
        let mut measured_states: HashSet<SPPath> = runner.transition_system_model.vars.iter().
            filter_map(|v| if v.variable_type() == VariableType::Measured {
                Some(v.path().clone())
            } else { None }).collect();
        let mut first_complete_state = SPState::new();
        let mut waiting = true;

        // we extend the hack to also first listen to initial states for our command variables.
        let cm: HashMap<_,_> = model.resources().iter().flat_map(|r|r.get_command_mirrors()).collect();
        let cm_rev: HashMap<_,_> = model.resources().iter().flat_map(|r|r.get_command_mirrors_rev()).collect();

        let mut command_with_echoes: HashSet<SPPath> = runner.transition_system_model.vars.iter().
            filter_map(|v| if v.variable_type() == VariableType::Command && cm.contains_key(v.path()) {
                Some(v.path().clone())
            } else { None }).collect();

        let mut resource_map = HashMap::new();
        let ticker = Instant::now();

        let mut hack = 0;
        let mut prev_state = SPState::new();

        loop {
            // println!("STARTING");


            let mut s = SPState::new();
            if rx_in.is_empty() {
                if let Ok(mess) = rx_in.recv() {
                    let x = resource_map.entry(mess.resource.clone()).or_insert(mess.time_stamp.clone());
                    //println!{"resource: {}, tick: {}, timer: {}", mess.resource, x.elapsed().as_millis(), ticker.elapsed().as_millis()};
                    *x = mess.time_stamp.clone();
                    s = mess.state;
                }
            } else {
                for mess in rx_in.try_iter() {
                    let x = resource_map.entry(mess.resource.clone()).or_insert(mess.time_stamp.clone());
                    //println!{"M: resource: {}, tick: {}, timer: {}", mess.resource, x.elapsed().as_millis(), ticker.elapsed().as_millis()};
                    *x = mess.time_stamp.clone();
                    s.extend(mess.state);  // Do the merge only if not overwriting, else save mess for next iteration
                }
            }

            // hack to wait for resources initially
            if waiting {
                if !measured_states.is_empty() || !command_with_echoes.is_empty() {
                    println!("Waiting for measured... {}", measured_states.iter().
                             map(|x|x.to_string()).collect::<Vec<String>>().join(", "));
                    println!("Waiting for command... {}", command_with_echoes.iter().
                             map(|x|x.to_string()).collect::<Vec<String>>().join(", "));

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

            // simulate the operation planning/scheduling layer
            let buffer1 = SPPath::from_string("cubes/buffer1_holding");
            let buffer2 = SPPath::from_string("cubes/buffer2_holding");
            // two configurations to plan between
            let c1 = p!([ p:buffer1 == 2] && [ p:buffer2 == 1 ]);
            let c2 = p!([ p:buffer1 == 1] && [ p:buffer2 == 2 ]);

            // meta operations...
            if hack == 0 && c1.eval(&runner.state()) {
                hack = 1;
                println!("in configuration 1, start op planning");

                let max_steps = 20;
                let planner_result = crate::planning::plan(&runner.operation_planning_model,
                                                           &[(c2,None)], &runner.state(), max_steps);
                println!("operation plan is");
                planner_result.trace.iter().for_each(|f| { println!("Transition: {}", f.transition); });

                let (tr, s) = crate::planning::convert_planning_result(&runner.operation_planning_model, planner_result, true);
                let plan = SPPlan{plan: tr, state_change: s};
                runner.input(SPRunnerInput::OperationPlan(plan));

            } else if hack == 1 && c2.eval(&runner.state()) {
                println!("in configuration 1, start op planning");
                hack = 0;

                let max_steps = 20;
                let planner_result = crate::planning::plan(&runner.operation_planning_model,
                                                           &[(c1,None)], &runner.state(), max_steps);
                println!("operation plan is");
                planner_result.trace.iter().for_each(|f| { println!("Transition: {}", f.transition); });

                let (tr, s) = crate::planning::convert_planning_result(&runner.operation_planning_model, planner_result, true);
                let plan = SPPlan{plan: tr, state_change: s};
                runner.input(SPRunnerInput::OperationPlan(plan));
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
                    runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
                }

                if prev_state.clone().extract() != runner.state().clone().extract() {
                    println!{""};
                    println!("The State:\n{}", runner.state());
                    println!{""};
                    prev_state = runner.state().clone();
                }

                tx_out.send(runner.state().clone()).unwrap();
                // send out runner info.
                let runner_info = RunnerInfo {
                    state: runner.state().clone(),
                    .. RunnerInfo::default()
                };

                tx_out_misc.send(runner_info).unwrap();


                if old_g != new_g {
                    println!("NEW GOALS");
                    println!("*********");

                    let max_steps = 100; // arbitrary decision
                    let planner_result = crate::planning::plan(&runner.transition_system_model, &new_g, &runner.state(), max_steps);
                    assert!(planner_result.plan_found);
                    //println!("new plan is");
                    //planner_result.trace.iter().for_each(|f| { println!("Transition: {}\nState:\n{}", f.transition, f.state); });
                    let (tr, s) = crate::planning::convert_planning_result(&runner.transition_system_model, planner_result, false);
                    let plan = SPPlan{plan: tr, state_change: s};
                    runner.input(SPRunnerInput::AbilityPlan(plan));

                    tx_out.send(runner.state().clone()).unwrap();

                    // send out runner info.
                    let runner_info = RunnerInfo {
                        state: runner.state().clone(),
                        .. RunnerInfo::default()
                    };
                    tx_out_misc.send(runner_info).unwrap();
                }
            }

            // let mess: Result<SPState, RecvError> = rx_in.recv();
            // if let Ok(s) = mess {

            // }

        }

    });

    loop {
        // blocking ros spinning
        sp_ros::spin(&mut node);
    }
}



// TODO: do we keep the old RunnerModel?
fn make_new_runner(m: RunnerModel, initial_state: SPState) -> SPRunner {
    let mut trans = vec!();
    let mut restrict_controllable = vec!();
    let false_trans = Transition::new("empty", Predicate::FALSE, vec!(), vec!(), true);
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
            vec!(t.path().clone())
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
        vec!(),
        vec!(),
        m.model.clone(),
        m.op_model.clone(),
    );
    runner.input(SPRunnerInput::AbilityPlan(SPPlan{
        plan: restrict_controllable,
        state_change: SPState::new(),
    }));
    runner.update_state_variables(initial_state);

    runner
}
