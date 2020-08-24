/// This module contain a simple model type that we can use for the
/// formal verification stuff.
use serde::{Deserialize, Serialize};
use sp_domain::*;
use super::*;

// remember to also change in planner
// TODO: make this constant crate-wide and use in more places
const USE_GUARD_EXTRACTION: bool = false;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct TransitionSystemModel {
    pub name: String,
    pub vars: Vec<Variable>,
    pub state_predicates: Vec<Variable>,
    pub transitions: Vec<Transition>,
    pub specs: Vec<Spec>,
}

impl TransitionSystemModel {
    pub fn from(model: &Model) -> Self {
        let mut vars: Vec<Variable> = model
            .resources()
            .iter()
            .flat_map(|r| r.get_variables())
            .collect();

        let resource_sub_items: Vec<_> = model
            .resources()
            .iter()
            .flat_map(|r| r.sub_items())
            .collect();

        let model_item_vars: Vec<Variable> = model
            .items()
            .iter()
            .flat_map(|i| match i {
                SPItem::Variable(s) => Some(s.clone()),
                _ => None,
            })
            .collect();
        vars.extend(model_item_vars.iter().cloned());

        let mut transitions: Vec<Transition> = model
            .resources()
            .iter()
            .flat_map(|r| r.get_transitions())
            .collect();

        let model_transitions = model.items().iter().flat_map(|i| match i {
            SPItem::Transition(t) => Some(t.clone()),
            _ => None,
        });
        transitions.extend(model_transitions);

        let mut state_predicates: Vec<Variable> = model
            .resources()
            .iter()
            .flat_map(|r| r.get_state_predicates())
            .collect();

        let mut specs: Vec<Spec> = model
            .items()
            .iter()
            .flat_map(|i| match i {
                SPItem::Spec(s) => Some(s),
                _ => None,
            })
            .cloned()
            .collect();
        let resource_sub_item_specs: Vec<Spec> = resource_sub_items
            .iter()
            .flat_map(|i| match i {
                SPItem::Spec(s) => Some(s.clone()),
                _ => None,
            })
            .collect();
        specs.extend(resource_sub_item_specs.iter().cloned());


        // recursively collect sub-models
        model.items.iter().flat_map(|i| match i {
            SPItem::Model(m)
                if m.name() != "operations" => Some(TransitionSystemModel::from(&m)),
            _ => None
        }).for_each(|tsm| {
            vars.extend(tsm.vars);
            state_predicates.extend(tsm.state_predicates);
            transitions.extend(tsm.transitions);
            specs.extend(tsm.specs);
        });


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


        // for simplicity and clarity we remove this for now.

        let auto = transitions
            .iter()
            .filter(|t| !t.controlled() && !t.actions().is_empty());
        let auto_guards: Vec<Predicate> = auto
            .map(|a| Predicate::NOT(Box::new(a.guard().clone())))
            .collect();
        if !auto_guards.is_empty() {
            let auto_guards = Predicate::AND(auto_guards);
            transitions.iter_mut().for_each(|t| {
                if t.controlled() {
                    let orig = t.guard().clone();
                    let new = Predicate::AND(vec![orig, auto_guards.clone()]);
                    *t.mut_guard() = new;
                }
            });
        }

        let mut ts_model = TransitionSystemModel {
            name: model.name().into(),
            vars,
            state_predicates,
            transitions,
            specs,
        };

        // MD 2020-08-20: Moved "magic" from runner model helper to here.

        // TODO:
        // for now we dont do guard extraction because it appears we plan alot faster
        // using just the invariants instead. but we need to make proper measurements

        // each resource contains a supervisor defining its good states
        let inits: Vec<Predicate> = ts_model.specs.iter()
            .filter_map(|s| if s.name() == "supervisor" {
                Some(s.invariant().clone())
            } else {
                None
            })
            .collect();

        // we need to assume that we are in a state that adheres to the resources
        let initial = Predicate::AND(inits);

        if USE_GUARD_EXTRACTION {
            let (new_guards, supervisor) = extract_guards(&ts_model, &initial);

            // The specs are converted into guards + a global supervisor
            ts_model.specs.clear();
            ts_model.specs.push(Spec::new("global_supervisor", supervisor));

            // TODO: right now its very cumbersome to update the original Model.
            // but it would be nice if we could.
            update_guards(&mut ts_model, &new_guards);
        } else {
            // we can refine all invariants instead of performing GE
            let mut new_specs = Vec::new();
            for s in &ts_model.specs {
                println!("refining invariant {}", s.path());
                let ri = refine_invariant(&ts_model, s.invariant());
                let mut ns = s.clone();
                ns.invariant = ri;
                new_specs.push(ns);
            }
            ts_model.specs = new_specs;
        }

        ts_model
    }

    pub fn from_op(model: &Model) -> Self {
        let vars: Vec<Variable> =
            model.find_item("product_state",&[])
            .as_model()
            .map(|m| m.items.iter().flat_map(|i| match i {
                SPItem::Variable(s) => Some(s.clone()),
                _ => None,
            }).collect()).unwrap_or(vec![]);

        let transitions: Vec<Transition> =
            model.find_item("operations",&[])
            .as_model()
            .map(|m| m
                 .items().iter().flat_map(|i| match i {
                     // operations represented by a single transition
                     SPItem::Operation(o) => {
                         let mut t = o.planning_trans.clone();
                         t.node_mut().update_name("start");
                         Some(t)
                     },
                     // "auto planning" transitions. these exist only in the planning world
                     SPItem::Transition(t) => Some(t.clone()),
                     _ => None,
                 })
                 .collect()).unwrap_or(vec![]);

        TransitionSystemModel {
            name: format!("op_model_{}", model.name()),
            vars,
            state_predicates: vec![],
            transitions,
            specs: vec![],
        }
    }

    pub fn bad_state(&self, state: &SPState) -> bool {
        self.specs.iter().any(|s| !s.invariant().eval(state))
    }
}
