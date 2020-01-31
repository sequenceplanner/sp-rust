use super::sp_ticker::{RunnerPredicate, SPTicker};
use sp_domain::*;

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
    old_model: Model,
    in_sync: bool,
}

impl SPRunner {
    pub fn _new(
        name: &str,
        transitions: Vec<Transition>,
        variables: Vec<Variable>,
        goals: Vec<IfThen>,
        global_transition_specs: Vec<TransitionSpec>,
        forbidden: Vec<IfThen>,
        old_model: Model,
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

    fn take_a_tick(&mut self, state: SPState) {
        if self.in_sync {
            let g = self.goal();
            //let result = crate::planning::compute_plan(&g, &self.ticker.state, &self.old_model, 20);
            //println!("we have a plan? {} -- got it in {}ms",
            //result.plan_found, result.time_to_solve.as_millis());
            
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


}
