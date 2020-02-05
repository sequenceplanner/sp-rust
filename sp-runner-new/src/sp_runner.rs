#![allow(dead_code)]

use super::sp_ticker::{RunnerPredicate, SPTicker};
use sp_domain::*;
use sp_runner_api::*;

/// The SPRunner that keep tracks of the state and the transition execution
/// When using the runner, call the input method on every state change and probably
/// also at a specific frequency
#[derive(Debug, PartialEq, Clone)]
pub struct SPRunner {
    name: String,
    ticker: SPTicker,
    variables: Vec<Variable>,
    predicates: Vec<Variable>,
    goals: Vec<IfThen>,
    global_transition_specs: Vec<TransitionSpec>,
    ability_plan: SPPlan,
    operation_plan: SPPlan,
    pub last_fired_transitions: Vec<SPPath>,
    old_model: RunnerModel,
    in_sync: bool,
}

/// The input to the runner. 
/// 
/// Tick is used to trigger one evaluation and execution of the transitions. Use
/// it to run the runner at a frequency.
/// 
/// TickAsync is used when the runner is in sync mode but youdo not want to
/// do the planning before executing the transition.
/// 
/// StateChange(upd_variables) is used when the input variables have changed. If
/// they are the same as the internal state, the runner will not execute the transitions.
/// 
/// Settings will change the internal settings of the runner. Maybe send in goals and specs using this?
/// 
/// AbilityPlan and OperationPlan will update the plans in the runner. If the runner
/// is in sync-mode, the AbilityPlan will be computed at each tick and the plan written 
/// via this input will be overwritten.
#[derive(Debug, PartialEq, Clone)]
pub enum SPRunnerInput {
    Tick,
    TickAsync,
    StateChange(SPState),
    Settings, // Will come later
    AbilityPlan(SPPlan),
    OperationPlan(SPPlan),
}
#[derive(Debug, PartialEq, Clone, Default)]
pub struct SPPlan {
    //id: usize, // probably use later. Or maybe we should include some kind of timestamp,
    pub plan: Vec<TransitionSpec>, // the plan in the form of transition specification
    //sequence: Vec<(usize, SPPath),  // probably need this when handling unsync planning
}

impl SPRunner {
    /// Creates a new runner
    pub fn new(
        name: &str,
        transitions: Vec<Transition>,
        variables: Vec<Variable>,
        goals: Vec<IfThen>,
        global_transition_specs: Vec<TransitionSpec>,
        forbidden: Vec<IfThen>,
        old_model: RunnerModel,
    ) -> Self {
        let mut vars = vec!();
        let mut preds = vec!();
        let mut initial_state_map = vec!();
        
        variables.into_iter().for_each(|v| {
            let v: Variable = v;
            initial_state_map.push((v.path().clone(), v.initial_value().clone()));
            match v.variable_type() {
                VariableType::Predicate(_) => preds.push(v),
                _ => vars.push(v),
            }

        });

        let mut state = SPState::new_from_values(&initial_state_map);
        state.add_variable(SPPath::from_slice(&["runner","ability_plan"]), 0.to_spvalue());
        state.add_variable(SPPath::from_slice(&["runner", "operation_plan"]), 0.to_spvalue());
        let runner_predicates: Vec<RunnerPredicate> = preds.iter().flat_map(|p| RunnerPredicate::new(p, &state)).collect();
        let mut ticker = SPTicker::new();
        ticker.state = state;
        ticker.transitions = transitions;
        ticker.forbidden = forbidden;
        ticker.predicates = runner_predicates;
        ticker.reload_state_paths();

        SPRunner {
            name: name.to_string(),
            ticker: ticker,
            variables: vars,
            predicates: preds,
            goals,
            global_transition_specs,
            ability_plan: SPPlan::default(),
            operation_plan: SPPlan::default(),
            last_fired_transitions: vec!(),
            old_model,
            in_sync: true,
        }
    }

    /// The main function to use when running the runner. Connect this to
    /// a channel either using async or standard threads
    pub fn input(&mut self, input: SPRunnerInput) {
        match input {
            SPRunnerInput::Tick => {
                self.take_a_tick(SPState::new(), self.in_sync);
            },
            SPRunnerInput::TickAsync => {
                self.take_a_tick(SPState::new(), false);
            },
            SPRunnerInput::StateChange(s) => {
                // We will not tick the runner when we got no changes. If you need
                // that, send a Tick.
                if !self.ticker.state.are_new_values_the_same(&s) {
                    self.take_a_tick(s, self.in_sync);
                }
            },
            SPRunnerInput::Settings => {}, // Will come later
            SPRunnerInput::AbilityPlan(plan) => {
                // Here we need to match with current plan, but let's do that later
                self.ability_plan = plan;
                self.load_plans();
            },
            SPRunnerInput::OperationPlan(plan) => {
                // Here we need to match with current plan, but let's do that later
                self.operation_plan = plan;
                self.load_plans();
            },
        }
    }

    /// Get the current state from the runner
    pub fn state(&self) -> SPState {
        self.ticker.state.clone()
    }

    /// Get the current goal that runner runner tries to reach. The predicate 
    /// in the result vec should be conjunted
    pub fn goal(&mut self) -> Vec<Predicate> {
        let goals: Vec<Predicate> = self.goals.iter().filter(|g| {
            g._if.eval(&self.ticker.state)
        }).map(|x| x._then.clone()).collect();
        //Predicate::AND(goals)
        goals
    }

    /// A special function that the owner of the runner can use to 
    /// add new state variables
    pub fn update_state_variables(&mut self, update: SPState) {
        let state_id = self.ticker.state.id();
        self.ticker.state.extend(update);
        if state_id != self.ticker.state.id() {
            // changed_variables have added a new variabel
            self.reload_state_paths();
        }
    }

    /// A special function that the owner of the runner can use to 
    /// force a complete new state. Must include all variables used 
    /// by the runner!
    pub fn force_new_state(&mut self, update: SPState) {
        self.ticker.state = update;
        self.reload_state_paths();
    }

    fn take_a_tick(&mut self, state: SPState, in_sync: bool) {
        self.update_state_variables(state);
        if in_sync {
            let g = self.goal();
            let result = crate::planning::compute_plan(&g, &self.ticker.state, &self.old_model, 20);
            println!("we have a plan? {} -- got it in {}ms", result.plan_found, result.time_to_solve.as_millis());
            
            self.include_planning_result(result);
        }
        let res = self.ticker.tick_transitions();
        self.last_fired_transitions = res.1;
    }

    fn load_plans(&mut self) {
        self.reload_state_paths_plans();
        self.ticker.specs = self.ability_plan.plan.clone();
        self.ticker.specs.extend(self.ability_plan.plan.clone());
        self.ticker.specs.extend(self.global_transition_specs.clone());
    }

    fn reload_state_paths(&mut self) {
        self.ticker.reload_state_paths();
        self.reload_state_paths_plans();
        for x in self.goals.iter_mut() {
            x.upd_state_path(&self.ticker.state)
        }
        for x in self.global_transition_specs.iter_mut() {
            x.spec_transition.upd_state_path(&self.ticker.state)
        }
    }
    fn reload_state_paths_plans(&mut self) {
        for x in self.ability_plan.plan.iter_mut() {
            x.spec_transition.upd_state_path(&self.ticker.state)
        }
        for x in self.operation_plan.plan.iter_mut() {
            x.spec_transition.upd_state_path(&self.ticker.state)
        }
    }

    fn include_planning_result(&mut self, res: crate::planning::PlanningResult) {
        let ctrl: Vec<SPPath> = self.old_model.ab_transitions.ctrl.iter().map(|t: &Transition| t.path().clone()).collect();
        let in_plan: Vec<SPPath> = res.trace.iter()
            .map(|x| x.transition.clone()).collect();
        let plan_p = SPPath::from_slice(&["runner","ability_plan"]);
        let mut tr: Vec<TransitionSpec> = in_plan.iter()
            .filter(|x| ctrl.contains(x))
            .enumerate()
            .map(|(i, p)| {
                let t = Transition::new(
                    &format!("step{:?}", i),
                    p!(plan_p == i),
                    vec!(a!(plan_p = {i+1})),
                    vec!(),
                    true,
                );
                TransitionSpec::new(
                    &format!("spec{:?}", i),
                    t,
                    vec!(p.clone())
                )
            })
            .collect();

            let blocked: Vec<TransitionSpec> = ctrl.iter()
                .filter(|x| !in_plan.contains(x))
                .map(|p| {
                    let t = Transition::new(
                        &format!("Blocked {}", p),
                        Predicate::FALSE,
                        vec!(),
                        vec!(),
                        true,
                    );
                    TransitionSpec::new(
                        &format!("Blocked {}", p),
                        t,
                        vec!(p.clone())
                    )
                })
                .collect();
            tr.extend(blocked);
            
            println!("THE PLAN");
            tr.iter().for_each(|x| println!("{:?}", x));
            println!("ctrl");
            ctrl.iter().for_each(|x| println!("{}", x));
            
            self.ticker.state.force_from_path(&plan_p, 0.to_spvalue()).unwrap();
            self.ability_plan = SPPlan{plan: tr};  
            self.load_plans();      

    }


}


#[cfg(test)]
mod test_new_runner {
    use super::*;
    use crate::planning::*;

    #[test]
    fn testing_me() { 
        let mut runner = make_dummy_robot_runner();
        

        println!{""};
        println!("Initial goal: {:?}", runner.goal());
        println!{""};
        println!("Initial State: {}", runner.state());
        println!{""};
        println!("Initial plan");
        runner.ability_plan.plan.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("Transitions plan");
        runner.ticker.transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        runner.input(SPRunnerInput::TickAsync);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        runner.input(SPRunnerInput::TickAsync);
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!("fired:");
        runner.input(SPRunnerInput::TickAsync);
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!("fired:");
        runner.input(SPRunnerInput::TickAsync);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        println!("goal:{:?}", runner.goal());
        runner.input(SPRunnerInput::Tick);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        runner.input(SPRunnerInput::TickAsync);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        runner.input(SPRunnerInput::TickAsync);
        runner.input(SPRunnerInput::TickAsync);
        runner.input(SPRunnerInput::TickAsync);
        runner.input(SPRunnerInput::TickAsync);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        runner.input(SPRunnerInput::TickAsync);
    }

    fn make_dummy_robot_runner() -> SPRunner {
        let (m, mut initial_state) = crate::testing::two_dummy_robots();
        let m: RunnerModel = m;
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
            m
        );
        runner.input(SPRunnerInput::AbilityPlan(SPPlan{
            plan: restrict_controllable,
        }));
        let the_upd_state = state!(
            ["dummy_robot_model", "r1", "State", "act_pos"] => "away",
            ["dummy_robot_model", "r1", "Control", "ref_pos"] => "away",
            ["dummy_robot_model", "r2", "State", "act_pos"] => "away",
            ["dummy_robot_model", "r2", "Control", "ref_pos"] => "away"
        );
        initial_state.extend(the_upd_state);
        runner.update_state_variables(initial_state);
        runner
    }
}

// name: &str,
//         transitions: Vec<Transition>,
//         variables: Vec<Variable>,
//         goals: Vec<IfThen>,
//         global_transition_specs: Vec<TransitionSpec>,
//         forbidden: Vec<IfThen>,
//         old_model: RunnerModel,

// pub struct RunnerModel {
//     pub op_transitions: RunnerTransitions,
//     pub ab_transitions: RunnerTransitions,
//     pub plans: RunnerPlans,
//     pub state_predicates: Vec<Variable>,
//     pub goals: Vec<IfThen>,
//     pub invariants: Vec<IfThen>,
//     pub model: Model,
// }