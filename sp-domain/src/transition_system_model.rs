use super::*;
/// This module contain a simple model type that we can use for the
/// formal verification stuff.
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct TransitionSystemModel {
    pub name: String,
    pub vars: Vec<Variable>,
    pub state_predicates: Vec<Variable>,
    pub transitions: Vec<Transition>,
    pub invariants: Vec<Specification>,
}

impl TransitionSystemModel {
    pub fn from(model: &Model) -> Self {
        let mut vars: Vec<Variable> = model
            .resources
            .iter()
            .flat_map(|r| r.get_variables())
            .filter(|v| !v.is_predicate())
            .collect();

        model.resources.iter().for_each(|r| {
            let resource_active = Variable::new_boolean(&r.path().to_string(),
                                                        VariableType::Estimated);
            vars.push(resource_active);
        });

        let global_vars: Vec<Variable> = model.global_variables.clone();

        vars.extend(global_vars.iter().cloned());

        vars.retain(|v| match v.type_ {
            VariableType::Runner => false,
            _ => true,
        });

        let mut transitions: Vec<Transition> = model.resources
            .iter()
            .flat_map(|r| r.transitions.clone())
            .collect();

        let global_transitions = model.global_transitions.clone();
        transitions.extend(global_transitions);

        transitions.retain(|t| match t.type_ {
            TransitionType::Controlled | TransitionType::Auto | TransitionType::Effect => true,
            _ => false, // for now this is just runner transitions but there may be more in the future.
        });

        let op_trans: Vec<Transition> = model.operations
            .iter()
            .map(|o|o.make_lowlevel_transitions())
            .flatten()
            .collect();

        transitions.extend(op_trans);

        let mut state_predicates: Vec<Variable> = model.resources
            .iter()
            .flat_map(|r| r.get_state_predicates())
            .collect();

        let mut invariants: Vec<_> = model.resources
            .iter()
            .map(|r| r.specifications.iter().flat_map(|s| {
                if let SpecificationType::TransitionInvariant(_) = &s.type_ {
                    Some(s.clone())
                } else {
                    None
                }
            }))
            .flatten()
            .collect();
        let global_invariants: Vec<_> = model.global_specs.iter().flat_map(|s| {
            if let SpecificationType::TransitionInvariant(_) = &s.type_ {
                Some(s.clone())
            } else {
                None
            }
        }).collect();

        invariants.extend(global_invariants);

        // recursively collect sub-models

        // EXPERIMENT:

        // pre-process the transitions to force all auto trans to be
        // taken before the controllable by adding guards to the
        // controllable.

        // if we leave it like this, we need to be sure that
        // there are no cycles among the auto trans
        // the benefit is that planning becomes more predicable.
        // the planner is forced to "switch off" any enabled auto trans,
        // just like the runner does.
        // otoh, planning horizons may increase slightly, as we no longer can
        // ignore "useless" (wrt the goal) auto trans.

        // actually we need this; otherwise the planner can "skip"
        // transitions that does not help it.

        // e.g. we get this plan:
        // drm/r1/move_to/start_with_at
        // drm/r1/move_to/finish
        // drm/r1/move_to/sync_prev       <-- notice this part
        // drm/r1/move_to/start_with_away
        // drm/r2/move_to/start_with_at
        // drm/r2/move_to/finish

        // instead of this plan:
        // drm/r2/move_to/start_with_at
        // drm/r2/move_to/finish
        // drm/r2/move_to/start_with_away   <-- this plan does not care about keeping prev_pos in sync.
        // drm/r1/activate/start
        // drm/r1/activate/finish
        // drm/r1/move_to/start_with_at
        // drm/r1/move_to/finish

        let auto = transitions
            .iter()
            .filter(|t| t.type_ == TransitionType::Auto);
        let auto_guards: Vec<Predicate> = auto
            .map(|a| Predicate::NOT(Box::new(a.guard().clone())))
            .collect();
        if !auto_guards.is_empty() {
            let auto_guards = Predicate::AND(auto_guards);
            let auto_pred = Variable::new_predicate("auto_guards", auto_guards);
            let auto_path = auto_pred.path().clone();
            state_predicates.push(auto_pred);
            transitions.iter_mut().for_each(|t| {
                if t.type_ == TransitionType::Controlled {
                    let orig = t.guard().clone();
                    let new = Predicate::AND(vec![orig, p!(p: auto_path)]);
                    *t.mut_guard() = new;
                }
            });
        }

        let mut ts_model = TransitionSystemModel {
            name: model.path.to_string(),
            vars,
            state_predicates,
            transitions,
            invariants,
        };

        ts_model
    }

    pub fn from_op(model: &Model) -> Self {
        let vars: Vec<Variable> = model.global_variables.iter()
            .flat_map(|i| match i.type_ {
                VariableType::Product => Some(i.clone()),
                _ => None,
            })
            .collect();

        let transitions: Vec<Transition> = model.operations.iter()
            .map(|o| o.make_planning_trans())
            .flatten()
            .collect();

        let global_invariants: Vec<_> = model
            .global_specs
            .iter()
            .flat_map(|s|
                if let SpecificationType::OperationInvariant(_) = &s.type_ {
                    Some(s.clone())
                } else {
                    None
                })
            .collect();

        TransitionSystemModel {
            name: format!("op_model_{}", model.path),
            vars,
            state_predicates: vec![],
            transitions,
            invariants: global_invariants,
        }
    }

    pub fn bad_state(&self, state: &SPState) -> bool {
        self.invariants.iter().any(|i| !i.invariant().eval(state))
    }

    /// Return a list of all paths used in states, e.g. the ones for
    /// variables and state predicates.
    pub fn get_state_paths(&self) -> Vec<SPPath> {
        let mut paths: Vec<_> = self.vars.iter().map(|p| p.path().clone()).collect();
        paths.extend(self.state_predicates.iter().map(|p| p.path().clone()).collect::<Vec<_>>());
        paths
    }
}
