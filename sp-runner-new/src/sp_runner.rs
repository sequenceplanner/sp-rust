#![allow(dead_code)]

use super::sp_ticker::{RunnerPredicate, SPTicker};
use sp_domain::*;
use sp_runner_api::*;

#[derive(Debug, PartialEq, Clone, Default)]
pub struct SPPlan {
    //id: usize, // probably use later. Or maybe we should include some kind of timestamp,
    pub plan: Vec<TransitionSpec>, // the plan in the form of transition specification
    //sequence: Vec<(usize, SPPath),  // probably need this when handling unsync planning
}
#[derive(Debug, PartialEq, Clone)]
pub enum SPRunnerInput {
    Tick,
    StateChange(SPState),
    Settings, // Will come later
    AbilityPlan(SPPlan),
    OperationPlan(SPPlan),
}
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
    last_fired_transitions: Vec<SPPath>,
    old_model: RunnerModel,
    in_sync: bool,
}

impl SPRunner {
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

        let state = SPState::new_from_values(&initial_state_map);
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

    pub fn input(&mut self, input: SPRunnerInput) {
        match input {
            SPRunnerInput::Tick => {
                self.take_a_tick(SPState::new());
            },
            SPRunnerInput::StateChange(s) => {
                // We will not tick the runner when we got no changes. If you need
                // that, send a Tick.
                if !self.ticker.state.are_new_values_the_same(&s) {
                    self.take_a_tick(s);
                }
            },
            SPRunnerInput::Settings => {}, // Will come later
            SPRunnerInput::AbilityPlan(plan) => {
                // Here we need to match with current plan, but let's do that later
                self.ability_plan = plan;
            },
            SPRunnerInput::OperationPlan(plan) => {
                // Here we need to match with current plan, but let's do that later
                self.operation_plan = plan;
            },
        }
    }
    pub fn state(&self) -> SPState {
        self.ticker.state.clone()
    }

    pub fn goal(&mut self) -> Vec<Predicate> {
        let goals: Vec<Predicate> = self.goals.iter().filter(|g| {
            g._if.eval(&self.ticker.state)
        }).map(|x| x._then.clone()).collect();
        //Predicate::AND(goals)
        goals
    }

    pub fn insert_a_new_state(&mut self, new_variables: SPState) {
        self.ticker.state.extend(new_variables);
        self.reload_state_paths();
    }

    fn take_a_tick(&mut self, state: SPState) {
        if self.in_sync {
            let g = self.goal();
            let result = crate::planning::compute_plan(&g, &self.ticker.state, &self.old_model, 20);
            println!("we have a plan? {} -- got it in {}ms", result.plan_found, result.time_to_solve.as_millis());
            self.include_planning_result(result);
        }
        let res = self.ticker.tick_transitions(state);
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
        // do some
    }


}


#[cfg(test)]
mod test_new_runner {
    use super::*;
    use crate::planning::*;

    #[test]
    fn testing_me() {
        let (m, initial_state) = crate::testing::two_dummy_robots();
        let m: RunnerModel = m;
        //println!("{:?}", m);
        let mut trans = vec!();
        m.op_transitions.ctrl.iter().for_each(|t| {
            trans.push(t.clone());
        });
        m.op_transitions.un_ctrl.iter().for_each(|t| {
            trans.push(t.clone());
        });
        m.ab_transitions.ctrl.iter().for_each(|t| {
            trans.push(t.clone());
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

        let the_upd_state = state!(
            ["dummy_robot_model", "r1", "State", "act_pos"] => "away",
            ["dummy_robot_model", "r1", "Control", "ref_pos"] => "away",
            ["dummy_robot_model", "r2", "State", "act_pos"] => "away",
            ["dummy_robot_model", "r2", "Control", "ref_pos"] => "away"
        );

        println!{""};
        println!("Initial goal:{:?}", runner.goal());
        println!{""};
        runner.insert_a_new_state(initial_state);
        //println!("NU KÖR VI: {}", runner.state());
        runner.input(SPRunnerInput::StateChange(the_upd_state));
        println!{""};
        println!("{}", runner.state());
        println!{""};
        println!("goal:{:?}", runner.goal());
        println!{""};
        runner.input(SPRunnerInput::Tick);
        println!{""};
        println!("The state: {}", runner.state());
        println!("goal:{:?}", runner.goal());
        println!{""};
        runner.input(SPRunnerInput::Tick);
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