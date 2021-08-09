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
    pub specs: Vec<Spec>,
}

impl TransitionSystemModel {
    pub fn from(model: &Model) -> Self {
        let mut vars: Vec<Variable> = model
            .resources()
            .iter()
            .flat_map(|r| r.get_variables())
            .filter(|v| !v.is_predicate())
            .collect();

        model.resources().iter().for_each(|r| {
            let resource_active = Variable::new_boolean(&r.path().to_string(),
                                                        VariableType::Estimated);
            vars.push(resource_active);
        });

        let global_vars: Vec<Variable> = model
            .items()
            .iter()
            .flat_map(|i| match i {
                SPItem::Variable(s) => Some(s.clone()),
                _ => None,
            })
            .collect();
        vars.extend(global_vars.iter().cloned());

        vars.retain(|v| match v.type_ {
            VariableType::Runner => false,
            _ => true,
        });

        let mut transitions: Vec<Transition> = model
            .resources()
            .iter()
            .flat_map(|r| r.transitions.clone())
            .collect();

        let global_transitions = model.items().iter().flat_map(|i| match i {
            SPItem::Transition(t) => Some(t.clone()),
            _ => None,
        });
        transitions.extend(global_transitions);

        transitions.retain(|t| match t.type_ {
            TransitionType::Controlled | TransitionType::Auto | TransitionType::Effect => true,
            _ => false, // for now this is just runner transitions but there may be more in the future.
        });

        let op_trans: Vec<_> = model
            .find_item("operations", &[])
            .and_then(|m| {
                m.as_model().map(|m| {
                    m.items()
                        .iter()
                        .flat_map(|i| match i {
                            SPItem::Operation(o) => Some(o.make_lowlevel_transitions()),
                            _ => None,
                        })
                        .flatten()
                        .collect()
                })
            })
            .unwrap_or(vec![]);

        transitions.extend(op_trans);

        let mut state_predicates: Vec<Variable> = model
            .resources()
            .iter()
            .flat_map(|r| r.get_state_predicates())
            .collect();

        let mut specs: Vec<Spec> = model
            .resources()
            .iter()
            .flat_map(|r| r.specs.clone())
            .collect();
        let global_specs: Vec<Spec> = model
            .items()
            .iter()
            .flat_map(|i| match i {
                SPItem::Spec(s) => Some(s.clone()),
                // unsure if we should include these.
                // they always become very big expressions and are not needed
                // as long as we don't run only on the low level with mono = false.
                // perhaps its better to activate them when we need to in the runner
                // this allows us to start faster as they take some time to compute. TODO.
                // SPItem::ProductSpec(s) => {
                //     // convert "product spec" to "spec"
                //     let mut ns = Spec::new(s.name(), s.invariant.clone());
                //     ns.node_mut().update_path(&s.path().parent());
                //     Some(ns)
                // },
                _ => None,
            })
            .collect();

        specs.extend(global_specs);

        // recursively collect sub-models
        model
            .items
            .iter()
            .flat_map(|i| match i {
                SPItem::Model(m) if m.name() != "operations" => {
                    Some(TransitionSystemModel::from(&m))
                }
                _ => None,
            })
            .for_each(|tsm| {
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
            name: model.name().into(),
            vars,
            state_predicates,
            transitions,
            specs,
        };

        // MD 2020-08-20: Moved "magic" from runner model helper to here.

        // MD 2020-09-04: Removed guard extraction completely.
        // let mut new_specs = Vec::new();
        // let mut longest_refining = std::time::Duration::default();
        // for s in &ts_model.specs {
        //     println!("refining invariant {}", s.path());
        //     let now = std::time::Instant::now();
        //     let ri = refine_invariant(&ts_model, s.invariant());
        //     let dur = now.elapsed();
        //     if dur > longest_refining {
        //         longest_refining = dur;
        //     }
        //     let mut ns = s.clone();
        //     ns.invariant = ri;
        //     new_specs.push(ns);
        // }
        // ts_model.specs = new_specs;

        // println!("LONGEST_REFINEMENT_TIME: {}ms", longest_refining.as_millis());

        ts_model
    }

    pub fn from_op(model: &Model) -> Self {
        let vars: Vec<Variable> = model
            .find_item("product_state", &[])
            .and_then(|m| {
                m.as_model().map(|m| {
                    m.items
                        .iter()
                        .flat_map(|i| match i {
                            SPItem::Variable(s) => Some(s.clone()),
                            _ => None,
                        })
                        .collect()
                })
            })
            .unwrap_or(vec![]);

        let transitions: Vec<Transition> = model
            .find_item("operations", &[])
            .and_then(|m| {
                m.as_model().map(|m| {
                    m.items()
                        .iter()
                        .flat_map(|i| match i {
                            // operations represented by a single transition
                            SPItem::Operation(o) => Some(o.make_planning_trans()),
                            _ => None,
                        })
                        .flatten()
                        .collect()
                })
            })
            .unwrap_or(vec![]);

        let global_specs: Vec<Spec> = model
            .items()
            .iter()
            .flat_map(|i| match i {
                SPItem::ProductSpec(s) => {
                    // convert "product spec" to "spec"
                    let mut ns = Spec::new(&(s.name()), s.invariant.clone());
                    ns.node_mut().update_path(&s.path().parent());
                    Some(ns)
                }
                _ => None,
            })
            .collect();

        TransitionSystemModel {
            name: format!("op_model_{}", model.name()),
            vars,
            state_predicates: vec![],
            transitions,
            specs: global_specs,
        }
    }

    pub fn bad_state(&self, state: &SPState) -> bool {
        self.specs.iter().any(|s| !s.invariant().eval(state))
    }

    /// Return a list of all paths used in states, e.g. the ones for
    /// variables and state predicates.
    pub fn get_state_paths(&self) -> Vec<SPPath> {
        let mut paths: Vec<_> = self.vars.iter().map(|p| p.path().clone()).collect();
        paths.extend(self.state_predicates.iter().map(|p| p.path().clone()).collect::<Vec<_>>());
        paths
    }
}
