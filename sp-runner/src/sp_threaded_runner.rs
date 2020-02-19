use super::sp_runner::*;
use sp_domain::*;
use sp_runner_api::*;
use failure::Error;
use sp_ros;
use std::thread;
use crossbeam::channel;
use std::collections::{HashSet,HashMap};


pub fn launch() -> Result<(), Error> {
    //let (model, initial_state) = crate::testing::two_dummy_robots();
    let (model, initial_state, _) = crate::testing::cubes();
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

    let mut runner = make_dummy_robot_runner(runner_model, initial_state);

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

        loop {
            println!("STARTING");


            let mut s = SPState::new();
            if rx_in.is_empty() {
                if let Ok(mess) = rx_in.recv() {
                    s = mess;
                }
            } else {
                for mess in rx_in.try_iter() {
                    s.extend(mess);
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

            println!("WE GOT:\n{}", &s);
            let old_g = runner.goal();
            //if runner.state().are_new_values_the_same(&s)
            {
                runner.input(SPRunnerInput::StateChange(s));
                let new_g = runner.goal();
                println!("GOALS");
                new_g.iter().for_each(|x| println!("{:?}", x));
                println!{""};
                println!("fired:");
                println!{""};
                runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));

                println!{""};
                println!("The State: {}", runner.state());
                println!{""};

                tx_out.send(runner.state().clone()).unwrap();

                if old_g != new_g {
                    println!("NEW GOALS");
                    println!("*********");

                    let planner_result = crate::planning::plan(&runner.transition_system_model, &new_g, &runner.state());
                    assert!(planner_result.plan_found);
                    println!("new plan is");
                    planner_result.trace.iter().for_each(|f| { println!("Transition: {}\nState:\n{}", f.transition, f.state); });
                    let (tr, s) = crate::planning::convert_planning_result(&runner.transition_system_model, planner_result);
                    let plan = SPPlan{plan: tr, state_change: s};
                    runner.input(SPRunnerInput::AbilityPlan(plan));

                    tx_out.send(runner.state().clone()).unwrap();

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



fn make_dummy_robot_runner(m: RunnerModel, mut initial_state: SPState) -> SPRunner {
    //println!("{:?}", m);
    let mut trans = vec!();
    let mut restrict_controllable = vec!();
    let false_trans = Transition::new("empty", Predicate::FALSE, vec!(), vec!(), true);
    m.op_transitions.ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        // restrict_controllable.push(TransitionSpec::new(
        //     &format!("s_{}_false", t.name()),
        //     false_trans.clone(),
        //     vec!(t.path().clone())
        // ))
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
    );
    runner.input(SPRunnerInput::AbilityPlan(SPPlan{
        plan: restrict_controllable,
        state_change: SPState::new(),
    }));
    // let the_upd_state = state!(
    //     ["dummy_robot_model", "r1", "State", "act_pos"] => "away",
    //     ["dummy_robot_model", "r1", "Control", "ref_pos"] => "away",
    //     ["dummy_robot_model", "r2", "State", "act_pos"] => "away",
    //     ["dummy_robot_model", "r2", "Control", "ref_pos"] => "away"
    // );
    // initial_state.extend(the_upd_state);
    runner.update_state_variables(initial_state);

    runner
}


#[cfg(test)]
mod launch_new_runner {
    use super::*;
    use crate::planning::*;

    #[test]
    fn go() {
        super::launch();
    }
}
