use super::sp_ticker::{RunnerPredicate, SPTicker};
use sp_domain::*;
use sp_formal::CompiledModel;
use sp_ros::RunnerModel;
use std::collections::HashMap;

/// The SPRunner that keep tracks of the state and the transition execution
/// When using the runner, call the input method on every state change and probably
/// also at a specific frequency
#[derive(Debug, PartialEq, Clone)]
pub struct SPRunner {
    pub name: String,
    pub ticker: SPTicker,
    pub plans: HashMap<String, SPPlan>,
}

/// The input to the runner.
#[derive(Debug, PartialEq, Clone)]
pub enum SPRunnerInput {
    Tick,
    StateChange(SPState),
    ModelChange(RunnerModel),
    NewPlan(String, SPPlan),
}

#[derive(Debug, PartialEq, Clone, Default)]
pub struct SPPlan {
    pub plan: Vec<TransitionSpec>, // the plan in the form of transition specification
    pub included_trans: Vec<SPPath>, // list of all included transitions in the plan (in order).
    pub state_change: SPState,     // for setting variables used in the plans (and others)
}

impl SPRunner {
    pub fn from(compiled_model: &CompiledModel, initial_state: SPState) -> Self {
        // collect all transitions from the model
        let mut transitions: Vec<_> = compiled_model.model.resources.iter()
            .map(|r| r.get_transitions())
            .flatten()
            .filter(|t| t.type_ != TransitionType::Effect)
            .collect();

        let global_transitions: Vec<_> = compiled_model.model.global_transitions.iter()
            .filter(|t| t.type_ != TransitionType::Effect)
            .cloned()
            .collect();

        let operations: Vec<Operation> = compiled_model.model.operations.clone();
        let operation_transitions: Vec<_> = operations
            .iter()
            .map(|o| o.make_runner_transitions())
            .flatten()
            .collect();

        let operation_lowlevel_transitions: Vec<_> = compiled_model.model.operations
            .iter()
            .map(|o|o.make_lowlevel_transitions())
            .flatten()
            .collect();

        let intentions = compiled_model.model.intentions.clone();
        let intention_transitions: Vec<_> = intentions
            .iter()
            .map(|i| i.make_runner_transitions())
            .flatten()
            .collect();

        transitions.extend(global_transitions);
        transitions.extend(operation_transitions);
        transitions.extend(operation_lowlevel_transitions);
        transitions.extend(intention_transitions);

        // collect all variables from the model.model.
        let mut all_variables: Vec<_> = compiled_model.model.resources.iter()
            .map(|r| r.get_variables())
            .flatten()
            .collect();

        all_variables.extend(compiled_model.model.global_variables.clone());

        // separate variables into variables and predicates and set them initially to unknown
        let mut variables = vec![];
        let mut preds = vec![];
        let mut initial_state_map = vec![];

        all_variables.into_iter().for_each(|v| {
            initial_state_map.push((v.path().clone(), SPValue::Unknown));
            match v.variable_type() {
                VariableType::Predicate(_) => preds.push(v),
                _ => variables.push(v),
            }
        });

        let mut state = SPState::new_from_values(&initial_state_map);

        // apply user-defined initial states
        state.extend(initial_state);

        // add extra operation goal variables to the initial state.
        let mut op_goal_state = SPState::new();
        for o in &compiled_model.model.operations {
            let op = o.path().clone();
            o.get_goal_state_paths().iter().for_each(|p| {
                let np = op.add_child_path(p);
                let v = state.sp_value_from_path(p).expect(&format!("no such path {}", p));
                op_goal_state.add_variable(np, v.clone());
            });
        }
        state.extend(op_goal_state);

        let runner_predicates: Vec<RunnerPredicate> = preds
            .iter()
            .flat_map(|p| RunnerPredicate::new(p, &state))
            .collect();

        // TODO: move this to respective planner.

        // TODO: remove disabled paths.
        let resource_paths: Vec<_> = compiled_model.model
            .resources
            .iter()
            .map(|r| r.path().clone())
            .collect();

        let mut ticker = SPTicker::new();
        ticker.state = state;
        ticker.transitions = transitions;
        ticker.predicates = runner_predicates;
        ticker.reload_state_paths();

        // initially block all controlled transitions until
        // we have seen them in an external plan.
        let mut restrict_controllable = vec![];
        let false_trans = Transition::new("empty",Predicate::FALSE,Predicate::FALSE,vec![],vec![],TransitionType::Controlled);
        ticker.transitions.iter().for_each(|t| {
            if t.type_ == TransitionType::Controlled {
                restrict_controllable.push(TransitionSpec::new(
                    &format!("s_{}_false", t.path()),
                    false_trans.clone(),
                    vec![t.path().clone()],
                ))
            }
        });

        let initial_plan_blocked = SPPlan {
            plan: restrict_controllable,
            .. SPPlan::default()
        };

        let mut plans = HashMap::new();
        plans.insert("internal".to_string(), initial_plan_blocked);

        let mut runner = SPRunner {
            name: compiled_model.model.path.to_string(),
            ticker,
            plans,
        };
        runner.load_plans();
        runner.reload_state_paths();


        // experiment with timeout on effects...
        let mut all_effects = vec![];
        let resource_effects: Vec<_> = compiled_model.model.resources.iter()
            .map(|r| r.get_transitions())
            .flatten()
            .filter(|t| t.type_ == TransitionType::Effect)
            .collect();
        let global_effects: Vec<_> = compiled_model.model.global_transitions.iter()
            .filter(|t| t.type_ == TransitionType::Effect)
            .cloned()
            .collect();
        all_effects.extend(resource_effects);
        all_effects.extend(global_effects);

        all_effects.iter().for_each(|t| {
            if t.type_ == TransitionType::Effect {
                let path = t.path().add_parent("effects");
                let effect_enabled = SPState::new_from_values(&[(path, true.to_spvalue())]);
                runner.update_state_variables(effect_enabled);
            }
        });

        //// end timeout effects

        runner
    }

    pub fn set_plan(&mut self, plan_identifier: String, plan: SPPlan) {
        // update our internal plan to remove blocks added by the new plan
        let paths_controlled_by_plan = plan.plan
            .iter()
            .map(|ts| ts.syncronized_with.clone())
            .flatten()
            .collect::<Vec<SPPath>>();

        if let Some(internal_plan) = self.plans.get_mut(&"internal".to_string()) {
            internal_plan.plan.retain(|ts| {
                let remove = ts.syncronized_with.iter().any(|p| paths_controlled_by_plan.contains(p));
                !remove
            });
        } else {
            panic!("no internal plan set");
        }

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


    pub fn take_a_tick(&mut self, state: SPState, check_resources: bool) -> Vec<SPPath> {
        self.update_state_variables(state);

        // TODO! At some point we don't udate the state paths
        // properly. For now we just recompute before ticking.
        self.reload_state_paths();
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

    // fn check_resources(&mut self) {
    //     let old_id = self.ticker.state.id();
    //     self.upd_resource_enabled();
    //     let all_resources = self.resources.clone();
    //     let missing_resources: Vec<SPPath> = all_resources
    //         .iter()
    //         .filter(|r| {
    //             let r = r.clone();
    //             let is_not_enabled = Predicate::NEQ(
    //                 PredicateValue::path(r.clone()),
    //                 PredicateValue::value(SPValue::Bool(true)),
    //             );
    //             is_not_enabled.eval(&self.ticker.state)
    //         })
    //         .cloned()
    //         .collect();


    //     if old_id != self.ticker.state.id() {
    //         self.reload_state_paths();
    //     }
    // }

    // fn upd_resource_enabled(&mut self) {
    //     // TODO: Move these to transitions into each resource
    //     let rs = self.resources.clone();
    //     rs.iter().for_each(|r| {
    //         let is_member = Predicate::MEMBER(
    //             PredicateValue::SPValue(r.to_spvalue()),
    //             PredicateValue::path(registered.clone()),
    //         );
    //         let toff = Predicate::TOFF(
    //             PredicateValue::path(r.clone().add_child("timestamp")),
    //             PredicateValue::SPValue(SPValue::Int32(10000)),
    //         ); // Hardcoded 5 seconds
    //         let enabled_pred = Predicate::AND(vec![is_member.clone(), toff.clone()]);
    //         let enabled = enabled_pred.eval(&self.ticker.state);
    //         self.ticker
    //             .state
    //             .add_variable(r.clone(), SPValue::Bool(enabled));
    //     });
    // }
}
