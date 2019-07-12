//! The runners in sp-runner 


use sp_domain::*;
use uuid::Uuid;

use tokio::prelude::*;
use tokio::*;
use sync::mpsc;

use std::collections::HashMap;

use serde::{Deserialize, Serialize};


#[derive(Debug)]
pub struct Runner {
    variables: RunnerVariables,
    op_transitions: RunnerTransitions,
    ab_transitions: RunnerTransitions,
    state: State,
    plans: RunnerPlans,
    ctrl: RunnerCtrl,
    state_functions: Vec<Variable>,
    op_function: Vec<OperationFunction>,
    comm: RunnerCommInternal,
    external_comm: RunnerComm,
    // tick: Interval
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
struct RunnerVariables {
    variables: Vec<Variable>,
    paths: SPStruct,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
struct RunnerTransitions {
    ctrl: Vec<Transition>,
    un_ctrl: Vec<Transition>
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
struct RunnerCtrl {
    pause: bool
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
struct RunnerPlans {
    op_plan: Vec<Uuid>,         // maybe have spids here?
    ab_plan: Vec<Uuid>,
}

#[derive(Debug)]
struct RunnerCommInternal {
    state_input: mpsc::Receiver<State>,
    command_input: mpsc::Receiver<RunnerCommand>,
    planner_input: mpsc::Receiver<PlannerResult>,
    state_output: mpsc::Sender<State>,
    runner_output: mpsc::Sender<RunnerInfo>,
    planner_output: mpsc::Sender<PlannerCommand>,
}

#[derive(Debug)]
pub struct RunnerComm {
    state_input: mpsc::Sender<State>,
    command_input: mpsc::Sender<RunnerCommand>,
    planner_input: mpsc::Sender<PlannerResult>,
    state_output: mpsc::Receiver<State>,
    runner_output: mpsc::Receiver<RunnerInfo>,
    planner_output: mpsc::Receiver<PlannerCommand>,
}

enum TimeForTransitions {
    OP,
    AB
}

impl RunnerCommInternal {
    fn new() -> (RunnerCommInternal, RunnerComm) {
        let (state_to, state_input) = tokio::sync::mpsc::channel::<State>(2);
        let (command_to, command_input) = tokio::sync::mpsc::channel::<RunnerCommand>(2);
        let (planner_to, planner_input) = tokio::sync::mpsc::channel::<PlannerResult>(2);
        let (state_output, state_from) = tokio::sync::mpsc::channel::<State>(2);
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
enum RunnerCommand {
    ToDO,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
enum PlannerResult {
    ToDO,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
enum RunnerInfo {
    ToDO,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
enum PlannerCommand {
    ToDO,
}




impl Runner {

    fn new() -> Runner {
        let (comm, external_comm) = RunnerCommInternal::new();
        Runner {
            variables: RunnerVariables::default(),
            op_transitions: RunnerTransitions::default(),
            ab_transitions: RunnerTransitions::default(),
            state: State::default(),
            plans: RunnerPlans::default(),
            ctrl:  RunnerCtrl::default(),
            state_functions: Vec::default(),
            op_function: Vec::default(),
            comm,
            external_comm   
        }
    }

    /// Upd the runner based on incoming command
    fn upd_command(&mut self, cmd: Option<RunnerCommand>) {
        
    }

    /// Upd the runner based on incoming plans
    fn upd_plan(&mut self, cmd: Option<PlannerResult>) {

    }

    /// Upd the runner based on incoming state
    fn upd_state(&mut self, s: Option<State>) {

    }

    /// Ticks all transitions that are enabled, starting first with the controlled that is first in the 
    /// plan. Mutates the runner state
    fn tick_transitions(&self, state: &mut State, plan: &mut Vec<Uuid>, trans: &RunnerTransitions) -> Vec<SPID> {
        let (ctrl, un_ctrl) = (&trans.ctrl, &trans.un_ctrl);

        let first = plan.first().and_then(|id| {
            ctrl.iter().find(|t| t.spid.id == *id)
        });
        let mut fired = match first {
            Some(t) if t.eval(&state) => {
                let n = t.next(&state).unwrap(); // Must return Ok, else somthing is bad
                self.insert_into_state(state, n);
                //self.insert_into_state(n);  // shoul also always work, else t.eval is no ok
                plan.remove(0);
                vec!(t.spid.clone())
            },
            _ => Vec::new()
        };

        for t in un_ctrl.iter() {
            if t.eval(&state) {
                let n = t.next(&state).unwrap(); // should always work since eval works
                self.insert_into_state(state, n);    // should always work
                fired.push(t.spid.clone());
            }
        };

        fired

    }

    fn insert_into_state(&self, state: &mut State, map: HashMap<SPPath, AssignStateValue>) {
        state.insert_map(map);
        self.upd_state_functions(state);
    }


    fn upd_state_functions(&self,state: &mut State) {
        let mut res = HashMap::new(); 
        for v in self.state_functions.iter() {
            if let Variable::StatePredicate(_, a) = v {
                let n = a.next(&self.state).unwrap();       // A state functions should always work
                res.extend(n);
            }
        };
        let force: HashMap<SPPath, AssignStateValue> = res.into_iter().map(|(key, value)| {
            let new_value = match value {
                AssignStateValue::SPValue(x) => AssignStateValue::Force(x),
                x => x,
            };
            (key, new_value)
        }).collect();
        state.insert_map(force);
    }



}

fn extr_option<T>(x: Result<Async<Option<T>>, mpsc::error::RecvError>) -> Option<T> {
    x.ok().and_then(|y| {
        match y {
            Async::Ready(x) => x,
            _ => None,
        }
    })
} 

fn is_not_ready<T, E>(p: &Poll<T, E>) -> bool {
    match p {
        Ok(Async::NotReady) => true,
        _ => false,
    }
}


impl Future for Runner {
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        let upd_s = self.comm.state_input.poll();
        let upd_cmd = self.comm.command_input.poll();
        let upd_plan = self.comm.planner_input.poll();
        // add ticker here

        if is_not_ready(&upd_s) && is_not_ready(&upd_cmd) && is_not_ready(&upd_plan) {
            return Ok(Async::NotReady)
        }

        self.upd_state(extr_option(upd_s));
        self.upd_command(extr_option(upd_cmd));
        self.upd_plan(extr_option(upd_plan));

        let mut state = self.state.clone();
        let mut plans = self.plans.clone();

        let mut fired = self.tick_transitions(&mut state, &mut plans.op_plan, &self.op_transitions);

        



        Ok(Async::Ready(()))
    }
}




/// ********** TESTS ***************

#[cfg(test)]
mod runner_tests {
    use super::*;
    #[test]
    fn create_a_runner() {
        let mut r = Runner::new();

        let ab = SPPath::from_str(&["a", "b"]);
        let ac = SPPath::from_str(&["a", "c"]);
        let kl = SPPath::from_str(&["k", "l"]);
        let xy = SPPath::from_str(&["x", "y"]);

        let s = state!(ab => 2, ac => true, kl => true, xy => false);
        r.state = s.clone();
        let p = pr!{{p!(ac)} && {p!(kl)}};

        let a = a!(ac = false);
        let a2 = a!(ab <- kl);
        let a3 = a!(xy ? p);

        let t1 = Transition {
            spid: SPID::new("t1"),
            guard: Predicate::TRUE,
            action: vec!(a.clone()),
            effects: vec!(),
        };
        let t2 = Transition {
            spid: SPID::new("t2"),
            guard: p,
            action: vec!(a2.clone()),
            effects: vec!(),
        };
        let ts = RunnerTransitions{
            ctrl: vec!(t1.clone()),
            un_ctrl: vec!(t2.clone())
        };
        let mut plan = vec!();
        let mut state = r.state.clone();
        let res = r.tick_transitions(&mut state, &mut plan, &ts);
        assert_eq!(res, vec!(t2.spid));
        println!("{:?}", res);
        println!("{:?}", state);

        r.state = s.clone();

        plan.push(t1.spid.id);

        let res = r.tick_transitions(&mut state, &mut plan, &ts);
        assert_eq!(res, vec!(t1.spid));
        println!("{:?}", res);
        println!("{:?}", r.state);

    }

}