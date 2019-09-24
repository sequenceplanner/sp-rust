//! The runners in sp-runner


use sp_domain::*;

use tokio::prelude::*;
use tokio::*;
use sync::mpsc;

use std::collections::HashMap;

use serde::{Deserialize, Serialize};


#[derive(Debug)]
pub struct Runner {
    model: RunnerModel,
    state: SPState,
    ctrl: RunnerCtrl,
    comm: RunnerCommInternal,
    delayed_ticks: timer::DelayQueue<RunnerTicker>,
    tick_in_que: bool,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerModel {
    pub op_transitions: RunnerTransitions,
    pub ab_transitions: RunnerTransitions,
    pub plans: RunnerPlans,
    pub state_functions: Vec<Variable>,
    pub op_functions: Vec<OperationFunction>,
    pub vars: HashMap<SPPath, Variable>,  // needed for planning
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerPlans {
    pub op_plan: Vec<SPPath>,         // maybe have spids here?
    pub ab_plan: Vec<SPPath>,
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerTransitions {
    pub ctrl: Vec<Transition>,
    pub un_ctrl: Vec<Transition>
}

impl RunnerTransitions {
    pub fn extend(&mut self, other: RunnerTransitions) {
        self.ctrl.extend(other.ctrl);
        self.un_ctrl.extend(other.un_ctrl)
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
struct RunnerCtrl {
    pause: bool
}


#[derive(Debug)]
struct RunnerCommInternal {
    state_input: mpsc::Receiver<AssignState>,
    command_input: mpsc::Receiver<RunnerCommand>,
    planner_input: mpsc::Receiver<PlannerResult>,
    state_output: mpsc::Sender<SPState>,
    runner_output: mpsc::Sender<RunnerInfo>,
    planner_output: mpsc::Sender<PlannerCommand>,
}

#[derive(Debug)]
pub struct RunnerComm {
    pub state_input: mpsc::Sender<AssignState>,
    pub command_input: mpsc::Sender<RunnerCommand>,
    pub planner_input: mpsc::Sender<PlannerResult>,
    pub state_output: mpsc::Receiver<SPState>,
    pub runner_output: mpsc::Receiver<RunnerInfo>,
    pub planner_output: mpsc::Receiver<PlannerCommand>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
enum RunnerTicker {
    Tick,
    Delay(states::Delay)
}

#[allow(dead_code)]
impl RunnerCommInternal {
    fn new() -> (RunnerCommInternal, RunnerComm) {
        let (state_to, state_input) = tokio::sync::mpsc::channel::<AssignState>(2);
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

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum RunnerCommand {
    ToDO,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PlannerResult {
    OpPlan(Vec<SPPath>),
    AbPlan(Vec<SPPath>)
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum RunnerInfo {
    ToDO,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PlannerCommand {
    ToDO,
}




impl Runner {
    pub fn new(model: RunnerModel, initial_state: SPState) -> (Runner, RunnerComm) {
        let (comm, external_comm) = RunnerCommInternal::new();

        let r = Runner {
            model,
            state: initial_state,
            ctrl: RunnerCtrl::default(),
            comm,
            delayed_ticks: timer::DelayQueue::new(),
            tick_in_que: false,
        };

        (r, external_comm)
    }

    /// Upd the runner based on incoming command
    fn upd_command(&mut self, cmd: Option<RunnerCommand>) {

    }

    /// Upd the runner based on incoming plans
    fn upd_plan(&mut self, cmd: Option<PlannerResult>) {
        // for now we always take the plan. Soon we should do it smart
        if let Some(p) = cmd {
            match p {
                PlannerResult::OpPlan(x) => self.model.plans.op_plan = x,
                PlannerResult::AbPlan(x) => self.model.plans.ab_plan = x,
            }
        }
    }

    /// Upd the runner based on incoming state
    fn upd_state(&mut self, state: Option<AssignState>) {
        println!("upd state: {:?}", state);
        if let Some(s) = state {
            let res = self.state.insert_map(s);
            if res.is_err() {
                println!("Tried to overwrite in upd_state in runner: {:?}", res);
            }
        }
    }

    /// Upd the runner based on the tick
    fn upd_from_tick(&mut self, cmd: Option<timer::delay_queue::Expired<RunnerTicker>>) {
        if let Some(x) = cmd {
            println!("Got a tick: {:?}", x);
            match x.into_inner() {
                RunnerTicker::Tick => self.tick_in_que = false,
                RunnerTicker::Delay(x) => {
                    // TODO: do something here
                }
            }
        };
    }

    /// Tick the runner one step trying to run the transitions that are enabled.
    /// Returns an updated state, updated plans and the transitions that was fired
    fn tick(&self, mut state: SPState, mut plans: RunnerPlans) -> (SPState, RunnerPlans, Vec<SPPath>) {
        let mut fired = self.tick_transitions(&mut state, &mut plans.op_plan, &self.model.op_transitions);

        let (goal, inv) = self.next_op_functions(&state);
        // validate plan with new goals here and if needed, clear the plan
        println!("we have a goal! {:?}", goal);

        fired.extend(self.tick_transitions(&mut state, &mut plans.ab_plan, &self.model.ab_transitions));

        (state, plans, fired)
    }

    /// Ticks all transitions that are enabled, starting first with the controlled that is first in the
    /// plan. Mutates the runner state
    fn tick_transitions(&self, state: &mut SPState, plan: &mut Vec<SPPath>, trans: &RunnerTransitions) -> Vec<SPPath> {
        let (ctrl, un_ctrl) = (&trans.ctrl, &trans.un_ctrl);

        let first = plan.first().and_then(|path| {
            ctrl.iter().find(|t| t.path == *path)
        });
        let mut fired = match first {
            Some(t) if t.eval(&state) => {
                let n = t.next(&state).unwrap(); // Must return Ok, else something is bad
                self.insert_into_state(state, n);
                plan.remove(0);
                vec!(t.path.clone())
            },
            _ => Vec::new()
        };

        //println!("state: {:?}", &state);
        for t in un_ctrl.iter() {
            //println!("{:?}, t: {:?}", t.eval(&state), t);
            if t.eval(&state) {
                let n = t.next(&state).unwrap(); // should always work since eval works
                self.insert_into_state(state, n);
                fired.push(t.path.clone());
            }
        };

        fired

    }

    fn next_op_functions(&self, state: &SPState) -> (Vec<Predicate>, Vec<Predicate>) {
        self.model.op_functions.iter().fold((Vec::new(), Vec::new()), |(mut goal, mut inv), x| match x {
            OperationFunction::Goal(p, g) if p.eval(state) => {
                println!("pushin goal!");
                goal.push(g.clone());
                (goal, inv)
            },
            OperationFunction::Invariant(p, i) if p.eval(state) => {
                inv.push(i.clone());
                (goal, inv)
            }
            _ => (goal, inv)
         })
    }



    fn insert_into_state(&self, state: &mut SPState, map: AssignState) {
        state.insert_map(map).unwrap();  // should always work here, else eval is not ok
        self.upd_state_functions(state);
    }

    fn upd_state_functions(&self,state: &mut SPState) {
        let s = self.model.state_functions.iter()
            .fold(HashMap::new(), |mut aggr, v| {
                match v {
                    Variable::StatePredicate(_, a) => {
                        aggr.extend( a.next(&self.state).unwrap().s.into_iter().map(|(key, value)| {
                                (key, match value {
                                    AssignStateValue::SPValue(x) => AssignStateValue::Force(x),
                                    x => x,
                                })
                            }
                        ).collect::<HashMap<SPPath, AssignStateValue>>());
                        aggr
                    },
                    _ => aggr
                }
        });
        state.insert_map(AssignState{s}).unwrap();
    }



}

fn extr_option<T>(x: Async<Option<T>>) -> Option<T> {
    match x {
        Async::Ready(x) => x,
        _ => None,
    }
}

fn extr_option_res<T, U>(x: Result<Async<Option<T>>, U>) -> Option<T> {
    x.ok().and_then(|y| {
        match y {
            Async::Ready(x) => x,
            _ => None,
        }
    })
}


fn is_not_ready<T>(x: &Async<Option<T>>) -> bool {
    match x {
        Async::NotReady => true,
        Async::Ready(None) => true,
        _ => false,
    }
}

fn got_something<T>(x: &Async<Option<T>>) -> bool {
    match x {
        Async::Ready(None) => false,
        Async::Ready(x) => true,
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
        // curretly will crash / terminate if any of the channels fail. Maybe we should not do that. Let us test
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
            println!("WE ARE TRIGGERING");
            task::current().notify();
        }

        if !self.tick_in_que || is_completed(&tick) {
            self.delayed_ticks.insert(RunnerTicker::Tick, std::time::Duration::from_millis(2000));
            self.tick_in_que = true;
        }


        self.upd_state(extr_option(upd_s));
        self.upd_command(extr_option(upd_cmd));
        self.upd_plan(extr_option(upd_plan));
        self.upd_from_tick(extr_option(tick));


        let (mut state, plans, fired) = self.tick(self.state.clone(), self.model.plans.clone());

        let can_not_send = self.comm.state_output
            .try_send(state.clone());
            //.expect("For now, the consumer after the runner must keep up");


        println!("COULD WE SEND? {:?}", can_not_send);

        // TODO: handle delay here. add them to the delayQueue


        state.take_all_next();
        self.state = state;
        self.model.plans = plans;
        println!("Fired: {:?}", &fired);


        Ok(Async::NotReady)
    }
}




/// ********** TESTS ***************
///



#[cfg(test)]
mod runner_tests {
    use super::*;



    #[test]
    fn dummy_robot_model_tests() {
        let (model, state, resources) = crate::testing::two_robots();
        let (runner, comm) = Runner::new(model.clone(), state.clone());
        println!("{:?}", state);

        assert_eq!(runner.state.get_value(
            &SPPath::from_str(&["r1", "ref"])),
            Some(&0.to_spvalue())
        );

        let mut res = runner.tick(runner.state.clone(), runner.model.plans.clone());
        println!("{:?}", res);
        res.0.take_all_next();

        let newS = res.0;
        assert_eq!(newS.get_value(
            &SPPath::from_str(&["r1", "ref"])),
            Some(&10.to_spvalue())
        );

        let res = runner.tick(newS.clone(), runner.model.plans.clone());
        assert_eq!(res.0,
            newS
        );



    }

    #[test]
    fn create_a_runner() {
        let ab = SPPath::from_str(&["a", "b"]);
        let ac = SPPath::from_str(&["a", "c"]);
        let kl = SPPath::from_str(&["k", "l"]);
        let xy = SPPath::from_str(&["x", "y"]);

        let s = state!(
            ab => false,
            ac => false,
            kl => false,
            xy => false
        );

        let p_ab = p!(ab);
        let p_ac = p!(ac);
        let p_kl = p!(kl);
        let p_xy = p!(xy);
        let p_ab_not = p!(!ab);
        let p_ac_not = p!(!ac);
        let p_kl_not = p!(!kl);
        let p_xy_not = p!(!xy);

        let a_ab = a!(ab);
        let a_ac = a!(ac);
        let a_kl = a!(kl);
        let a_xy = a!(xy);
        let a_ab_not = a!(!ab);
        let a_ac_not = a!(!ac);
        let a_kl_not = a!(!kl);
        let a_xy_not = a!(!xy);

        let p = SPPath::from_str(&["t_ab"]);
        let t_ab = transition!(p, &p_ab_not, &a_ab);
        let t_ab_not = transition!(p, &p_ab, &a_ab_not);
        let t_ab_ac = transition!(p, &p_ab, &a_ac);
        let t_ac_kl = transition!(p, &p_ac, &a_kl);
        let t_kl_xy = transition!(p, &p_kl, &a_xy);
        let t_reset = transition!(p, &p_xy, &a_ab_not, &a_ac_not, &a_kl_not, &a_xy_not);

        let ts = RunnerTransitions{
            ctrl: vec!(t_ab, t_reset),
            un_ctrl: vec!(
                t_ab_not,
                t_ab_ac,
                t_ac_kl,
                t_kl_xy,
            )
        };

        // let sp_p = pr!(ab && ac);
        // let sf = vec!(variable!(SP "test", sp_p));

        // let mut plan = vec!();
        // let mut state = r.state.clone();
        // let res = r.tick_transitions(&mut state, &mut plan, &ts);
        // assert_eq!(res, vec!());

        // r.state = s.clone();

        // plan.push(t1.spid.id);

        // let res = r.tick_transitions(&mut state, &mut plan, &ts);
        // assert_eq!(res, vec!(t1.spid));
        // println!("{:?}", res);
        // println!("{:?}", r.state);

    }

}
