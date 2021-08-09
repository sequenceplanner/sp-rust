use super::sp_ticker::{RunnerPredicate, SPTicker};
use sp_domain::*;
use std::collections::HashMap;

/// The SPRunner that keep tracks of the state and the transition execution
/// When using the runner, call the input method on every state change and probably
/// also at a specific frequency
#[derive(Debug, PartialEq, Clone)]
pub struct SPRunner {
    pub name: String,
    pub ticker: SPTicker,

    pub plans: HashMap<String, SPPlan>,

    pub transition_system_models: Vec<TransitionSystemModel>,
    pub resources: Vec<SPPath>,
    pub replan_specs: Vec<Spec>,
    pub operations: Vec<Operation>,
    pub operation_goals: HashMap<SPPath, Predicate>, // todo: move to the state
    pub intentions: Vec<Intention>,
}

// Moving to this struct eventually...
#[derive(Debug, PartialEq, Clone)]
pub struct NewSPRunner {
    pub name: String,
    pub ticker: SPTicker,

    pub plans: HashMap<String, SPPlan>,

    // TODO: create a resource handler task?
    pub resources: Vec<SPPath>,
}

/// The input to the runner.
#[derive(Debug, PartialEq, Clone)]
pub enum SPRunnerInput {
    Tick,
    StateChange(SPState),
    NodeChange(SPState),
    NewPlan(String, SPPlan),
}
#[derive(Debug, PartialEq, Clone, Default)]
pub struct SPPlan {
    pub plan: Vec<TransitionSpec>, // the plan in the form of transition specification
    pub included_trans: Vec<SPPath>, // list of all included transitions in the plan (in order).
    pub state_change: SPState,     // for setting variables used in the plans (and others)
}

impl SPRunner {
    /// Creates a new runner
    pub fn new(
        name: &str, transitions: Vec<Transition>, variables: Vec<Variable>,
        transition_system_models: Vec<TransitionSystemModel>,
        resources: Vec<SPPath>, replan_specs: Vec<Spec>,
        operations: Vec<Operation>, intentions: Vec<Intention>,
    ) -> Self {
        let mut vars = vec![];
        let mut preds = vec![];
        let mut initial_state_map = vec![];

        variables.into_iter().for_each(|v| {
            initial_state_map.push((v.path().clone(), SPValue::Unknown));
            match v.variable_type() {
                VariableType::Predicate(_) => preds.push(v),
                _ => vars.push(v),
            }
        });

        let mut state = SPState::new_from_values(&initial_state_map);
        let runner_predicates: Vec<RunnerPredicate> = preds
            .iter()
            .flat_map(|p| RunnerPredicate::new(p, &state))
            .collect();
        let mut ticker = SPTicker::new();
        ticker.state = state;
        ticker.transitions = transitions;
        ticker.predicates = runner_predicates;
        ticker.reload_state_paths();
        ticker.disabled_paths = resources.clone();

        SPRunner {
            name: name.to_string(),
            ticker,
            plans: HashMap::new(),
            transition_system_models,
            resources,
            replan_specs,
            operations,
            operation_goals: HashMap::new(),
            intentions,
        }
    }
}

impl NewSPRunner {
    pub fn from_oldrunner(r: &SPRunner) -> Self {
        NewSPRunner {
            name: r.name.clone(),
            ticker: r.ticker.clone(),
            plans: r.plans.clone(),
            resources: r.resources.clone(),
        }
    }

    pub fn set_plan(&mut self, plan_identifier: String, plan: SPPlan) {
        self.update_state_variables(plan.state_change.clone());
        (*self.plans.entry(plan_identifier).or_default()) = plan;
        self.load_plans();
    }

    /// Get the current state from the runner
    pub fn state(&self) -> &SPState {
        &self.ticker.state
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
    /// force a new state. Must include all variables used
    /// by the runner!
    pub fn force_new_state(&mut self, update: SPState) {
        self.ticker.state = update;
        self.reload_state_paths();
    }

    pub fn disabled_paths(&self) -> Vec<SPPath> {
        self.ticker.disabled_paths.clone()
    }

    pub fn take_a_tick(&mut self, state: SPState, check_resources: bool) -> Vec<SPPath> {
        if check_resources {
            self.check_resources();
        }
        self.update_state_variables(state);

        self.ticker.tick_transitions()
    }

    fn load_plans(&mut self) {
        self.reload_state_paths_plans();
        self.ticker.specs = self.plans.values().map(|p|p.plan.clone()).flatten().collect();
    }

    fn reload_state_paths(&mut self) {
        self.ticker.reload_state_paths();
        self.reload_state_paths_plans();
    }
    fn reload_state_paths_plans(&mut self) {
        let s = &self.ticker.state;
        self.plans.iter_mut().for_each(|(_,p)| {
            p.plan.iter_mut().for_each(|x| {
                x.spec_transition.upd_state_path(&s);
            });
        });
    }

    fn check_resources(&mut self) {
        let old_id = self.ticker.state.id();
        self.upd_resource_enabled();
        let all_resources = self.resources.clone();
        let missing_resources: Vec<SPPath> = all_resources
            .iter()
            .filter(|r| {
                let r = r.clone();
                let is_not_enabled = Predicate::NEQ(
                    PredicateValue::path(r.clone()),
                    PredicateValue::value(SPValue::Bool(true)),
                );
                is_not_enabled.eval(&self.ticker.state)
            })
            .cloned()
            .collect();

        self.ticker.disabled_paths = missing_resources;
        if !self.ticker.disabled_paths.is_empty() {
            println!("Disabled paths: {:?}", self.ticker.disabled_paths);
        }
        if old_id != self.ticker.state.id() {
            self.reload_state_paths();
        }
    }

    fn upd_resource_enabled(&mut self) {
        // TODO: Move these to transitions into each resource
        let rs = self.resources.clone();
        let registered = SPPath::from_string("registered_resources");
        rs.iter().for_each(|r| {
            let is_member = Predicate::MEMBER(
                PredicateValue::SPValue(r.to_spvalue()),
                PredicateValue::path(registered.clone()),
            );
            let toff = Predicate::TOFF(
                PredicateValue::path(r.clone().add_child("timestamp")),
                PredicateValue::SPValue(SPValue::Int32(10000)),
            ); // Hardcoded 5 seconds
            let enabled_pred = Predicate::AND(vec![is_member.clone(), toff.clone()]);
            let enabled = enabled_pred.eval(&self.ticker.state);
            self.ticker
                .state
                .add_variable(r.clone(), SPValue::Bool(enabled));
        });
    }
}
