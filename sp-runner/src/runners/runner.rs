//! The runners in sp-runner 


use sp_domain::*;
use uuid::Uuid;

use tokio::prelude::*;
use tokio::*;
use sync::mpsc;

use std::collections::HashMap;
use petgraph::Graph;
use petgraph::prelude::*;
use petgraph::graph::node_index as n;
use petgraph::visit::depth_first_search;
use petgraph::visit::{DfsEvent, Control};
use petgraph::visit::Bfs;
use petgraph::visit::Dfs;

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
    pub variables: Vec<Variable>,
    pub paths: SPStruct,
    pub op_transitions: RunnerTransitions,
    pub ab_transitions: RunnerTransitions,
    pub plans: RunnerPlans,
    pub state_functions: Vec<Variable>,
    pub op_functions: Vec<OperationFunction>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerPlans {
    pub op_plan: Vec<Uuid>,         // maybe have spids here?
    pub ab_plan: Vec<Uuid>,
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerTransitions {
    ctrl: Vec<Transition>,
    un_ctrl: Vec<Transition>
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
    OpPlan(Vec<Uuid>),
    AbPlan(Vec<Uuid>)
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

    /// Tick tick the runner one step trying to run the transitions that are enabled.
    /// Returns an updated state, updated plans and the transitions that was fired
    fn tick(&self, mut state: SPState, mut plans: RunnerPlans) -> (SPState, RunnerPlans, Vec<SPID>) {
        let mut fired = self.tick_transitions(&mut state, &mut plans.op_plan, &self.model.op_transitions);

        let (goal, inv) = self.next_op_functions(&state);
        // validate plan with new goals here and if needed, clear the plan

        fired.extend(self.tick_transitions(&mut state, &mut plans.ab_plan, &self.model.ab_transitions));

        (state, plans, fired)
    }

    /// Ticks all transitions that are enabled, starting first with the controlled that is first in the 
    /// plan. Mutates the runner state
    fn tick_transitions(&self, state: &mut SPState, plan: &mut Vec<Uuid>, trans: &RunnerTransitions) -> Vec<SPID> {
        let (ctrl, un_ctrl) = (&trans.ctrl, &trans.un_ctrl);

        let first = plan.first().and_then(|id| {
            ctrl.iter().find(|t| t.spid.id == *id)
        });
        let mut fired = match first {
            Some(t) if t.eval(&state) => {
                let n = t.next(&state).unwrap(); // Must return Ok, else something is bad
                self.insert_into_state(state, n);
                plan.remove(0);
                vec!(t.spid.clone())
            },
            _ => Vec::new()
        };

        for t in un_ctrl.iter() {
            if t.eval(&state) {
                let n = t.next(&state).unwrap(); // should always work since eval works
                self.insert_into_state(state, n);  
                fired.push(t.spid.clone());
            }
        };

        fired

    }

    fn next_op_functions(&self, state: &SPState) -> (Vec<Predicate>, Vec<Predicate>) {
        self.model.op_functions.iter().fold((Vec::new(), Vec::new()), |(mut goal, mut inv), x| match x {
            OperationFunction::Goal(p, g) if p.eval(state) => {
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

fn is_not_ready<T, E>(p: &Poll<T, E>) -> bool {
    match p {
        Ok(Async::NotReady) => true,
        _ => false,
    }
}

fn is_completed<T>(x: &Async<Option<T>>) -> bool {
    match x {
        Async::Ready(None) => true,
        _ => false,
    }
}

// fn generate_graph(states: Vec<State>, trans: Vec<Transition>) -> Graph<>{
    // let mut graph = Graph::<State, Transition>::new();
// }

fn validate_plan(states: Vec<SPState>, curr: SPState, goal: SPState, trans: RunnerTransitions) -> (Vec<SPID>, Vec<SPState>) {

    let mut graph = Graph::<SPState, Transition>::new();

    // Add nodes to the graph
    let current_state = graph.add_node(curr);
    let goal_state = graph.add_node(goal);

    let ctrl_trans: Vec<Transition> = trans.ctrl;
    let unctrl_trans: Vec<Transition> = trans.un_ctrl;

    let mut all_trans: Vec<Transition> = Vec::new();

    for trans in ctrl_trans{
        all_trans.push(trans);
    }

    for trans in unctrl_trans{
        all_trans.push(trans);
    }
    
    // return the edge repr from trans and states
    let mut edges = vec!();

    for state in states {
        graph.add_node(state.clone());
        for tr in &all_trans {
            if tr.guard.eval(&state){
                let source = state.clone();
                if tr.action.iter().all(|a| a.eval(&state)) {
                    let sink = state.clone();
                    edges.push((source, sink));
                }
            }
        }
    }

    //This part is the thing left to make it work (hopefully)...
    //graph.extend_with_edges(&edges);

    // Collect this in the dfs later and return
    let mut state_trace = Vec::new();
    let mut trans_spid_trace_to_goal: Vec<SPID> = Vec::new();

    let mut predecessor = vec![NodeIndex::end(); graph.node_count()];

    let start = current_state;
    let goal = goal_state;

    depth_first_search(&graph, Some(start), |event| {
        if let DfsEvent::TreeEdge(u, v) = event {
            predecessor[v.index()] = u;
            if v == goal {
                return Control::Break(v);
            }
        }
        Control::Continue
    });

    let mut next = goal;
    let mut path = vec![next];
    while next != start {
        let pred = predecessor[next.index()];
        path.push(pred);
        next = pred;
    }
    path.reverse();

    //Just to test some prints...
    // for trans in all_trans{
        // trans_spid_trace_to_goal.push(trans.spid);
    // }
    
    return (trans_spid_trace_to_goal, state_trace)
} 

impl Future for Runner {
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        // We will terminate if any of the channels fail. Maybe not in the future. Let us test
        let upd_s = self.comm.state_input.poll().unwrap();
        let upd_cmd = self.comm.command_input.poll().unwrap();
        let upd_plan = self.comm.planner_input.poll().unwrap();
        let tick = self.delayed_ticks.poll().unwrap();

        // Do nothing if no stream is ready
        if upd_s.is_not_ready() && upd_cmd.is_not_ready() && upd_plan.is_not_ready() && tick.is_not_ready() {
            return Ok(Async::NotReady)
        }

        // Terminate runner if all streams have completed
        if is_completed(&upd_s) && is_completed(&upd_cmd) && is_completed(&upd_plan) && is_completed(&tick) {
            println!("The runner comleted since all streams completed. If this happened unexpected, maybe remove this");
            return Ok(Async::Ready(()))
        }

        self.upd_state(extr_option(upd_s));
        self.upd_command(extr_option(upd_cmd));
        self.upd_plan(extr_option(upd_plan));
        self.upd_from_tick(extr_option(tick));


        let (mut state, plans, fired) = self.tick(self.state.clone(), self.model.plans.clone());

        self.comm.state_output.try_send(state.clone())
            .expect("For now, the consumer after the runner must keep up");


        // TODO: handle delay here. add them to the delayQueue
        if !self.tick_in_que {
            self.delayed_ticks.insert(RunnerTicker::Tick, std::time::Duration::from_millis(1000));
            self.tick_in_que = true;
        }

        state.take_all_next();
        self.state = state;
        self.model.plans = plans;
        println!("Fired: {:?}", &fired);

        // Since one of the inputs was ready we will poll again. We should terminate 
        // later when we get a command above
        task::current().notify();
        Ok(Async::NotReady)
    }
}




/// ********** TESTS ***************

// ***** E: Testing the validate_plan****
// maybe have to create a runner first, not really

#[test]
fn validate(){
    let ab = SPPath::from_str(&["a", "b"]);
    let ac = SPPath::from_str(&["a", "c"]);
    let kl = SPPath::from_str(&["k", "l"]);
    let xy = SPPath::from_str(&["x", "y"]);

    let s = state!(ab => 2, ac => true, kl => true, xy => false);
    let s2 = state!(ab => 3, ac => false, kl => true, xy => false);

    // some simple states
    let st1 = state!(ab => 1);
    let st2 = state!(ab => 2);
    let st3 = state!(ab => 3);
    let st4 = state!(ab => 4);
    let st5 = state!(ab => 5);
    let st6 = state!(ab => 6);

    let mut all_states: Vec<SPState> = Vec::new();
    all_states.push(st1.clone());
    all_states.push(st2.clone());
    all_states.push(st3.clone());
    all_states.push(st4.clone());
    all_states.push(st5.clone());
    all_states.push(st6.clone());

    let asdf = all_states.clone();

    let ac1 = a!(ab = 2);
    let ac2 = a!(ab = 3);
    let ac3 = a!(ab = 4);
    let ac4 = a!(ab = 5);
    let ac5 = a!(ab = 6);

    let pr1 = p!(ab == 1);
    let pr2 = p!(ab == 2);
    let pr3 = p!(ab == 3);
    let pr4 = p!(ab == 2);
    let pr5 = p!(ab == 3);

    let tr1 = Transition {
        spid: SPID::new("tr1"),
        guard: pr1.clone(),
        action: vec!(ac1.clone()),
        effects: vec!(),
    };

    let tr2 = Transition {
        spid: SPID::new("tr2"),
        guard: pr2.clone(),
        action: vec!(ac2.clone()),
        effects: vec!(),
    };

    let tr3 = Transition {
        spid: SPID::new("tr3"),
        guard: pr3.clone(),
        action: vec!(ac3.clone()),
        effects: vec!(),
    };

    let tr4 = Transition {
        spid: SPID::new("tr4"),
        guard: pr2.clone(),
        action: vec!(ac4.clone()),
        effects: vec!(),
    };

    let tr5 = Transition {
        spid: SPID::new("tr5"),
        guard: pr3.clone(),
        action: vec!(ac5.clone()),
        effects: vec!(),
    };

    // let t2 = Transition {
        // spid: SPID::new("t2"),
        // guard: p.clone(),
        // action: vec!(a2.clone()),
        // effects: vec!(),
    // };
// 
    // let t3 = Transition {
        // spid: SPID::new("t3"),
        // guard: Predicate::TRUE,
        // action: vec!(a3.clone()),
        // effects: vec!(),
    // };

    // let p = pr!{{p!(ac)} && {p!(kl)}};
    // let p2 = pr!{{p!(ac)} && {p!(kl)}};
    // let a = a!(ac = false);
    // let a2 = a!(ab <- kl);
    // let a3 = a!(xy ? p);
    // let a4 = a!(kl ? p2);
// 
    // let t1 = Transition {
        // spid: SPID::new("t1"),
        // guard: Predicate::TRUE,
        // action: vec!(a.clone()),
        // effects: vec!(),
    // };
// 
    // let t2 = Transition {
        // spid: SPID::new("t2"),
        // guard: p.clone(),
        // action: vec!(a2.clone()),
        // effects: vec!(),
    // };
// 
    // let t3 = Transition {
        // spid: SPID::new("t3"),
        // guard: Predicate::TRUE,
        // action: vec!(a3.clone()),
        // effects: vec!(),
    // };
// 
    // let t4 = Transition {
        // spid: SPID::new("t4"),
        // guard: p2.clone(),
        // action: vec!(a4.clone()),
        // effects: vec!(),
    // };

    let ts = RunnerTransitions{
        ctrl: vec!(tr1.clone(), tr2.clone(), tr3.clone(), tr4.clone(), tr5.clone()),
        un_ctrl: vec!(),
    };
    println!("EEEEEEEEEEEEEEEEEE");
    println!("{:?}", validate_plan(all_states, st1, st4, ts));
    //println!("EEEEEEEEEEEEEEEEEE");
    //println!("{:?}", asdf);
    // println!("EEEEEEEEEEEEEEEEEE");
    // println!("{:?}", p2);
    // println!("EEEEEEEEEEEEEEEEEE");
    // println!("{:?}", a);
    // println!("EEEEEEEEEEEEEEEEEE");
    // println!("{:?}", a4);
    // println!("EEEEEEEEEEEEEEEEEE");
    println!("{:?}", tr1);
}

#[cfg(test)]
mod runner_tests {
    use super::*;
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

        let t_ab = transition!("t_ab", &p_ab_not, &a_ab);
        let t_ab_not = transition!("t_ab", &p_ab, &a_ab_not);
        let t_ab_ac = transition!("t_ab", &p_ab, &a_ac);
        let t_ac_kl = transition!("t_ab", &p_ac, &a_kl);
        let t_kl_xy = transition!("t_ab", &p_kl, &a_xy);
        let t_reset = transition!("t_ab", &p_xy, &a_ab_not, &a_ac_not, &a_kl_not, &a_xy_not);

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