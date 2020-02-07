//! The runners in sp-runner


use sp_domain::*;
use sp_runner_api::*;
use tokio::prelude::*;
use tokio::*;
use sync::mpsc;
use std::collections::{HashMap,HashSet};

use serde::{Deserialize, Serialize};


#[derive(Debug)]
pub struct Runner {
    model: RunnerModel,
    state: SPState,
    ctrl: RunnerCtrl,
    comm: RunnerCommInternal,
    delayed_ticks: timer::DelayQueue<RunnerTicker>,
    tick_in_que: bool,
    // hack to wait until we have the current state for all resources
    // we start taking transitions only when this is empty.
    untouched_state_paths: HashSet<SPPath>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
struct RunnerCtrl {
    pause: bool,
    override_ability_transitions: Vec<SPPath>,
    override_operation_transitions: Vec<SPPath>,
}


#[derive(Debug)]
struct RunnerCommInternal {
    state_input: mpsc::Receiver<SPState>,
    command_input: mpsc::Receiver<RunnerCommand>,
    planner_input: mpsc::Receiver<PlannerResult>,
    state_output: mpsc::Sender<SPState>,
    runner_output: mpsc::Sender<RunnerInfo>,
    planner_output: mpsc::Sender<PlannerCommand>,
}

#[derive(Debug)]
pub struct RunnerComm {
    pub state_input: mpsc::Sender<SPState>,
    pub command_input: mpsc::Sender<RunnerCommand>,
    pub planner_input: mpsc::Sender<PlannerResult>,
    pub state_output: mpsc::Receiver<SPState>,
    pub runner_output: mpsc::Receiver<RunnerInfo>,
    pub planner_output: mpsc::Receiver<PlannerCommand>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
enum RunnerTicker {
    Tick,
}

#[allow(dead_code)]
impl RunnerCommInternal {
    fn new() -> (RunnerCommInternal, RunnerComm) {
        let (state_to, state_input) = tokio::sync::mpsc::channel::<SPState>(2);
        let (command_to, command_input) = tokio::sync::mpsc::channel::<RunnerCommand>(2);
        let (planner_to, planner_input) = tokio::sync::mpsc::channel::<PlannerResult>(2);
        let (state_output, state_from) = tokio::sync::mpsc::channel::<SPState>(2);
        let (runner_output, command_from) = tokio::sync::mpsc::channel::<RunnerInfo>(2);
        let (planner_output, planner_from) = tokio::sync::mpsc::channel::<PlannerCommand>(2);

        (RunnerCommInternal{
            state_input,
            command_input,
            planner_input,
            state_output,
            runner_output,
            planner_output,
        },
        RunnerComm{
            state_input: state_to,
            command_input: command_to,
            planner_input: planner_to,
            state_output: state_from,
            runner_output: command_from,
            planner_output: planner_from,
        })
    }
}

impl Runner {
    pub fn new(model: RunnerModel, initial_state: SPState) -> (Runner, RunnerComm) {
        let (comm, external_comm) = RunnerCommInternal::new();

        let items = model.model.items();
        let resources: Vec<&Resource> = items
            .iter()
            .flat_map(|i| match i {
                SPItem::Resource(r) => Some(r),
                _ => None,
            })
            .collect();
        let vars: HashMap<SPPath, Variable> = resources.iter().
            flat_map(|r| r.get_variables()).
            map(|v| (v.path().clone(), v.clone())).collect();
        let measured_states: HashSet<SPPath> = vars.iter().
            filter_map(|(p,v)|
                if v.variable_type() == VariableType::Measured {
                    Some(p.clone())
                } else { None }).collect();

        let mut r = Runner {
            model,
            state: SPState::default(),
            ctrl: RunnerCtrl { pause: false,
                               override_ability_transitions: vec![],
                               override_operation_transitions: vec![]
            },
            comm,
            delayed_ticks: timer::DelayQueue::new(),
            tick_in_que: false,
            untouched_state_paths: measured_states
        };

        // Set state here to allow for predicates to run first
        r.state = initial_state;
        r.model.upd_state_paths(&r.state);
        Runner::upd_state_predicates(&mut r.model.state_predicates, &mut r.state);

        (r, external_comm)
    }

    /// Upd the runner based on incoming command
    fn upd_command(&mut self, cmd: Option<RunnerCommand>) {
        if let Some(c) = cmd {
            self.ctrl.pause = c.pause;
            self.ctrl.override_ability_transitions = c.override_ability_transitions;
            self.ctrl.override_operation_transitions = c.override_operation_transitions;
        }
    }

    /// Upd the runner based on incoming plans
    fn upd_plan(&mut self, cmd: Option<PlannerResult>) {
        // for now we always take the plan. Soon we should do it smart
        if let Some(p) = cmd {
            match p {
                PlannerResult::OpPlan(x) => self.model.plans.op_plan = x,
                PlannerResult::AbPlan(x) => { panic!("NOT YET"); self.model.plans.ab_plan =
                                              x.iter().map(|p| AbPlanItem { transition: SPPath::default(), guard: Predicate::TRUE }).collect() },
            }
        }
    }

    /// Upd the runner based on incoming state
    fn upd_state(&mut self, assign: Option<SPState>) -> bool {
        // println!("upd state: {:?}", state);
        if let Some(s) = assign {
            // temp-fix to handle uninitialized measured
            if !self.untouched_state_paths.is_empty() {
                s.projection().state.iter().for_each(|(p, _)| {self.untouched_state_paths.remove(&p);});
            }
            if !self.state.are_new_values_the_same(&s) {
                self.state.extend(s);
                Runner::upd_state_predicates(&mut self.model.state_predicates, &mut self.state);
                return true;
            }
        }
        false
    }

    /// Upd the runner based on the tick
    fn upd_from_tick(&mut self, cmd: Option<timer::delay_queue::Expired<RunnerTicker>>) {
        if let Some(x) = cmd {
            // println!("Got a tick: {:?}", x);
            match x.into_inner() {
                RunnerTicker::Tick => self.tick_in_que = false,
            }
        };
    }

    /// Tick the runner one step trying to run the transitions that are enabled.
    /// the transitions that was fired
    fn tick(&mut self) -> RunnerPlans {
        if !self.untouched_state_paths.is_empty() {
            let rn = self.untouched_state_paths.iter().map(|s|s.to_string()).collect::<Vec<_>>().join(", ");
            println!("WAITING FOR RESOURCES: {}", rn);
            return RunnerPlans::default();
        }


        let mut fired = if !self.ctrl.pause {
            // for now, fake the queue...
            let mut temp_q = Runner::enabled_ctrl(&self.state, &self.model.op_transitions);
            println!("enabled operations: {:?}", temp_q);
            // self.tick_transitions(&mut state, &mut plans.op_plan, &self.model.op_transitions)
            Runner::tick_transitions(&mut self.state, temp_q, &self.model.op_transitions, &self.model.state_predicates)
        } else {
            // if we are paused, we can take transitions only if they are in the override list
            let mut temp: Vec<&SPPath> = self.ctrl.override_operation_transitions.iter().collect();
            Runner::tick_transitions(&mut self.state, temp, &self.model.op_transitions, &self.model.state_predicates)
        };

        let goals = self.next_op_functions();
        // validate plan with new goals here and if needed, clear the plan
        if !goals.is_empty() {
            println!("we have goals! {:?}", goals.iter().map(|g|g.node().path().to_string()).collect::<Vec<_>>().join(","));
        }

        let goal_invs: Vec<_> = goals.iter().map(|x| (x.goal().clone(), x.invariant().clone())).collect();
        let result = crate::planning::compute_plan(&self.model.model, &goal_invs, &self.state, 20);
        println!("we have a plan? {} -- got it in {}ms",
                 result.plan_found, result.time_to_solve.as_millis());

        let mut new_ab_plan: Vec<AbPlanItem> = Vec::new();

        // the following is super hacky. orig is just to have a state
        // with all correct paths... since we can no longer add
        // variables using action.next.
        let mut wait_for = self.state.clone();
        let orig = self.state.clone();
        for f in &result.trace {
            if let Some(mut uc) = self.model.ab_transitions.un_ctrl.iter_mut().find(|c| c.path() == &f.transition) {
                    uc.actions.iter_mut().for_each(|a| {
                        a.next(&mut wait_for);
                        wait_for.take_transition();
                    });
                    uc.effects.iter_mut().for_each(|a| {
                        a.next(&mut wait_for);
                        wait_for.take_transition();
                    });
                    println!("ADDING EFFECT OF TRANS {}: {:?}", uc.path(), wait_for);
                }
            if let Some(c) = self.model.ab_transitions.ctrl.iter_mut().find(|c| c.path() == &f.transition) {
                // turn current wait for state into a guard.
                let guard = Predicate::AND(wait_for.projection().clone_vec_value().into_iter().flat_map(|(k,v)| {
                    // only collect if value has changed.
                    match orig.sp_value_from_path(&k) {
                        Some(nv) if nv != &v => Some(Predicate::EQ(PredicateValue::path(k), PredicateValue::value(v))),
                        _ => None
                    }
                }).collect());

                let api = AbPlanItem { transition: f.transition.clone(), guard: guard};
                println!("GUARD FOR TRANS {}: {:?}", api.transition, api.guard);
                new_ab_plan.push(api);


                // reset wait_for state
                wait_for = orig.clone();

                // add any effects of taking this trans to new wait for
                c.effects.iter_mut().for_each(|a| {
                    a.next(&mut wait_for);
                    wait_for.take_transition();
                });
                println!("ADDING EFFECT OF TRANS {}: {:?}", c.path(), wait_for);
            }
        }

        println!("plan is: {:?}", new_ab_plan.iter().map(|p|p.transition.clone()).collect::<Vec<_>>());
        let mut plans = RunnerPlans::default();
        plans.ab_plan = new_ab_plan;

        let ab_fired = if !self.ctrl.pause {
            Runner::tick_transitions_with_plan_guards(&mut self.state, &mut plans.ab_plan, &mut self.model.ab_transitions, &mut self.model.state_predicates)
        } else {
            // if we are paused, we can take transitions only if they are in the override list
            let mut temp: Vec<&SPPath> = self.ctrl.override_ability_transitions.iter().collect();
            Runner::tick_transitions(&mut self.state, temp, &mut self.model.ab_transitions,  &mut self.model.state_predicates)
        };
        fired.extend(ab_fired);

        plans
    }

    /// Enabled controllable transitions
    fn enabled_ctrl<'a>(state: &SPState, trans: &'a RunnerTransitions) -> Vec<&'a SPPath> {
        let mut res = vec!();
        for t in &trans.ctrl {
            if t.eval(state) {
                res.push(t.path());
            }
        }
        res
    }

    fn tick_transitions_with_plan_guards(state: &mut SPState, plan: &mut Vec<AbPlanItem>, trans: &RunnerTransitions, predicates: &Vec<Variable>) -> Vec<SPPath> {
        let (ctrl, un_ctrl) = (&trans.ctrl, &trans.un_ctrl);

        let to_run: Option<&Transition> = if !plan.is_empty() && plan.first().unwrap().guard.eval(&state) {
            let pi = plan.first().unwrap();
            println!("taking trans: {:?}", pi.transition);
            ctrl.iter().find(|t| t.path() == &pi.transition)
        } else {
            if !plan.is_empty() {
                println!("guard not satisfied: {:?}", plan.first().unwrap().guard);
            }
            None
        };
        let mut fired = Vec::new();
        if to_run.is_some() {
            let tt = to_run.unwrap();
            tt.next(state).expect(&format!("In tick transition ctrl with plan, next must be ok, {:?}", tt)); // Must return Ok, else something is bad
            Runner::upd_state_predicates(predicates, state);

            println!("TAKING TRANSITION: {}", tt.path());
            fired.push(tt.path().clone());

            plan.remove(0);
        }

        //println!("state: {:?}", &state);
        for t in un_ctrl.iter() {
            //println!("{:?}, t: {:?}", t.eval(&state), t);
            if t.eval(&state) {
                let n = t.next(state).expect(&format!("In tick transition un_ctrl with plan, next must be ok, {:?}", t)); // Must return Ok, else something is bad
                Runner::upd_state_predicates(predicates, state);
                fired.push(t.path().clone());
            }
        };

        fired

    }

    /// Ticks all transitions that are enabled, starting first with the controlled that is first in the
    /// plan. Mutates the runner state
    fn tick_transitions(state: &mut SPState, mut plan: Vec<&SPPath>, trans: &RunnerTransitions, predicates: &Vec<Variable>) -> Vec<SPPath> {
        let (ctrl, un_ctrl) = (&trans.ctrl, &trans.un_ctrl);

        let mut first = plan.first().and_then(|path| {
            ctrl.iter().find(|t| t.is_eq(path))
        });
        let mut fired = {
            if first.is_some() {
                let t = first.unwrap();
                if t.eval(state) {
                    let n = t.next(state).expect(&format!("In tick transition ctrl, next must be ok, {:?}", t)); // Must return Ok, else something is bad
                    Runner::upd_state_predicates(predicates, state);
                    plan.remove(0);
                    vec!(t.path().clone())
                } else {
                    Vec::new()
                }
            } else {
                Vec::new()
            }
        };

        //println!("state: {:?}", &state);
        for t in un_ctrl.iter() {
            //println!("{:?}, t: {:?}", t.eval(&state), t);
            if t.eval(&state) {
                let n = t.next(state).expect(&format!("In tick transition unctrl, next must be ok, {:?}", t)); // Must return Ok, else something is bad
                Runner::upd_state_predicates(predicates, state);
                fired.push(t.path().clone());
            }
        };

        fired

    }

    fn next_op_functions(&mut self) -> Vec<IfThen> {

        let mut goals: Vec<IfThen> = vec!();
        for x in self.model.goals.iter() {
            if x.condition.eval(&self.state) {
                goals.push(x.clone());
            }
        }

        goals
    }




    fn upd_state_predicates(predicates: &Vec<Variable>, state: &mut SPState) {
        predicates.iter().for_each(|v| match v.variable_type() {
            VariableType::Predicate(p) => {
                let upd = p.eval(state).to_spvalue();
                if let Some(x) = state.sp_value_from_path(v.path()) {
                    if x != &upd {
                        state.force_from_path(v.path(), upd);
                    }
                }
            },
            _ => {},
        });
    }



}

fn extr_option<T>(x: Async<Option<T>>) -> Option<T> {
    match x {
        Async::Ready(x) => x,
        _ => None,
    }
}

// fn extr_option_res<T, U>(x: Result<Async<Option<T>>, U>) -> Option<T> {
//     x.ok().and_then(|y| {
//         match y {
//             Async::Ready(x) => x,
//             _ => None,
//         }
//     })
// }


// fn is_not_ready<T>(x: &Async<Option<T>>) -> bool {
//     match x {
//         Async::NotReady => true,
//         Async::Ready(None) => true,
//         _ => false,
//     }
// }

fn got_something<T>(x: &Async<Option<T>>) -> bool {
    match x {
        Async::Ready(None) => false,
        Async::Ready(_x) => true,
        _ => false,
    }
}

fn is_completed<T>(x: &Async<Option<T>>) -> bool {
    match x {
        Async::Ready(_) => true,
        _ => false,
    }
}


impl Future for Runner {
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        // currently will crash / terminate if any of the channels fail. Maybe we should not do that. Let us test
        let upd_s = self.comm.state_input.poll().unwrap();
        let upd_cmd = self.comm.command_input.poll().unwrap();
        let upd_plan = self.comm.planner_input.poll().unwrap();
        let tick = self.delayed_ticks.poll().unwrap();


        // println!("******************************");
        // println!("******************************");
        // println!("state_input: {:?}", upd_s);
        // println!("upd_cmd: {:?}", upd_cmd);
        // println!("upd_plan: {:?}", upd_plan);
        // println!("tick: {:?}", tick);
        // println!("-----------------------------");

        // // Do nothing if no stream is ready
        // if is_not_ready(&upd_s) && is_not_ready(&upd_cmd) && is_not_ready(&upd_plan) && is_not_ready(&tick) {
        //     println!("none is ready!");
        //     return Ok(Async::NotReady)
        // }

        // If all have completed, trigger again since tick needs to be polled
        // Also if one channel had something, check again now
        if (is_completed(&tick)) ||
           (got_something(&upd_s) || got_something(&upd_cmd) || got_something(&upd_plan) || got_something(&tick))
        {
            // println!("WE ARE TRIGGERING");
            task::current().notify();
        }

        if !self.tick_in_que || is_completed(&tick) {
            self.delayed_ticks.insert(RunnerTicker::Tick, std::time::Duration::from_millis(2000));
            self.tick_in_que = true;
        }


        let did_upd = self.upd_state(extr_option(upd_s));
        self.upd_command(extr_option(upd_cmd));
        self.upd_plan(extr_option(upd_plan));
        self.upd_from_tick(extr_option(tick));

        let plans = self.tick();

        let enabled_ab_ctrl = Runner::enabled_ctrl(&self.state, &self.model.ab_transitions).into_iter().cloned().collect();
        let enabled_op_ctrl = Runner::enabled_ctrl(&self.state, &self.model.op_transitions).into_iter().cloned().collect();
        let ri = RunnerInfo {
            state: self.state.clone(),
            ability_plan: plans.ab_plan.iter().map(|abpi|abpi.transition.clone()).collect(),
            enabled_ability_transitions: enabled_ab_ctrl,
            operation_plan: plans.op_plan.clone(),
            enabled_operation_transitions: enabled_op_ctrl,
        };

        let _can_not_send = self.comm.runner_output.try_send(ri);

        let _can_not_send = self.comm.state_output
            .try_send(self.state.clone());
            //.expect("For now, the consumer after the runner must keep up");


        // println!("COULD WE SEND? {:?}", can_not_send);

        // TODO: handle delay here. add them to the delayQueue


        self.state.take_transition();
        self.model.plans = plans;
        // println!("Fired: {:?}", &fired);


        Ok(Async::NotReady)
    }
}




/// ********** TESTS ***************
///



#[cfg(test)]
mod runner_tests {
    use super::*;



    // #[test]
    // fn dummy_robot_model_tests() {
    //     let (model, state) = crate::testing::two_robots();
    //     let (mut runner, _comm) = Runner::new(model.clone(), state.clone());
    //     println!("{:?}", state);

    //     assert_eq!(runner.state.sp_value_from_path(
    //         &SPPath::from_slice(&["r1", "ref"])),
    //         Some(&0.to_spvalue())
    //     );

    //     let mut res = runner.tick();
    //     println!("{:?}", res);
    //     runner.state.take_transition();

    //     assert_eq!(runner.state.sp_value_from_path(
    //         &SPPath::from_slice(&["r1", "ref"])),
    //         Some(&10.to_spvalue())
    //     );

    //     println!("The runner: {:?}", runner);



    // }


}
