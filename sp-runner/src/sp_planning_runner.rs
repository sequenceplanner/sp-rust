use sp_domain::*;
use sp_ros::log_error;
use super::sp_ticker::{RunnerPredicate, SPTicker};
use super::sp_runner::*;

#[derive(Debug, PartialEq, Clone)]
pub struct PlanningState {
    pub plans: Vec<SPPlan>,
    pub transition_system_models: Vec<TransitionSystemModel>,
    pub intentions: Vec<SPPath>,
    pub intention_goals: Vec<IfThen>,
    pub replan_specs: Vec<Spec>,
    pub operations: Vec<Operation>,
}

pub fn bad_state(state: &SPState, ts_model: &TransitionSystemModel) -> bool {
    ts_model.specs.iter().any(|s| !s.invariant().eval(state))
}

/// Checks wheter we can reach the current goal As it doesnt
/// exactly follow the plan but instead simulated the runner, it
/// can fail if the plan contains two "choices" involving the same
/// transitions. Then the "wrong" one might be taken first.
pub fn check_goals_fast(
    s: &SPState, goals: &[&Predicate], plan: &SPPlan, ts_model: &TransitionSystemModel,
    disabled_paths: &[SPPath], predicates: &[RunnerPredicate]
) -> bool {
    if goals.iter().all(|g| g.eval(s)) {
        return true;
    }

    let trans: Vec<Transition> = ts_model
        .transitions
        .iter()
        .filter(|t| plan.included_trans.contains(t.path()))
        .cloned()
        .collect();

    let tm = SPTicker::create_transition_map(&trans, &plan.plan, disabled_paths);

    // we need to make sure goals can be ticked off when reached.
    let mut goals = goals.to_vec();

    let mut state = s.clone();
    let mut counter = 0;
    loop {
        let state_changed = tm
            .iter()
            .flat_map(|ts| {
                if ts.iter().all(|t| {
                    let x = t.eval(&state);
                    //println!("for {}: {}", t.path(), x);
                    x
                }) {
                    // transitions enabled.
                    counter += 1;
                    if counter > 1000 {
                        // there is a self loop in the model
                        let t_names = ts
                            .iter()
                            .map(|t| t.path().to_string())
                            .collect::<Vec<_>>()
                            .join(",");
                        panic!("self loop with transitions {}", t_names);
                    }

                    // take all actions
                    ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                        let _res = a.next(&mut state);
                    });

                    // update state predicates
                    SPTicker::upd_preds(&mut state, predicates);

                    // next -> cur
                    let changed = state.take_transition();

                    //println!("THE STATE\n{}", state);

                    if bad_state(&state, ts_model) {
                        None
                    } else {
                        Some(changed)
                    }
                } else {
                    None
                }
            })
            .any(|x| x);

        // if SPRunner::bad_state(&state, ts_model) {
        //     println!("final state is bad!");
        //     return false;
        // }

        goals.retain(|g| !g.eval(&state));
        if goals.is_empty() {
            return true;
        } else if !state_changed {
            // double check with complete search in the normal
            // case where we fail early in the plan this is still
            // fast, but it saves us from the issue descibed
            // above. worst case we waste some time here but but
            // it should be faster than replanning anyway.
            //
            // actually this becomes quite expensive... for now just replan

            println!("NOT CHECKING GOALS COMPLETE, replan instead.");
            // return self.check_goals_complete(s, &goals, plan, ts_model);
            return false; // think about how we want to do it...
        }
    }
}

/// A slower, but more forgiving forward search to goal.
/// This handles the case described in check_goals_fast.
pub fn check_goals_complete(
    state: &SPState, goals: &[&Predicate], plan: &SPPlan, ts_model: &TransitionSystemModel,
    disabled_paths: &[SPPath], predicates: &[RunnerPredicate]
) -> bool {
    if goals.iter().all(|g| g.eval(state)) {
        return true;
    }

    let trans: Vec<Transition> = ts_model
        .transitions
        .iter()
        .filter(|t| plan.included_trans.contains(t.path()))
        .cloned()
        .collect();

    let tm = SPTicker::create_transition_map(&trans, &plan.plan, &disabled_paths);

    fn rec<'a>(
        state: &SPState, goals: &[&Predicate], tm: &Vec<Vec<&'a Transition>>,
        visited: &mut Vec<SPState>, ts_model: &TransitionSystemModel,
        predicates: &[RunnerPredicate]
    ) -> bool {
        if goals.iter().all(|g| g.eval(&state)) {
            return true;
        }

        let new_states: Vec<SPState> = tm
            .iter()
            .flat_map(|ts| {
                if ts
                    .iter()
                    .all(|t| t.eval(&state) && !bad_state(&state, ts_model))
                {
                    // transitions enabled. clone the state to start a new search branch.
                    let mut state = state.clone();

                    // take all actions
                    ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                        // if actions write to the same path, only the first will be used
                        let _res = a.next(&mut state);
                    });

                    // update state predicates
                    SPTicker::upd_preds(&mut state, predicates);

                    // next -> cur
                    if state.take_transition() && !visited.contains(&state) {
                        visited.push(state.clone());
                        Some(state)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .collect();

        new_states
            .iter()
            .any(|s| rec(&s, goals, tm, visited, ts_model, predicates))
    }

    let mut visited = vec![state.clone()];
    rec(state, &goals, &tm, &mut visited, ts_model, predicates)
}

/// Same as above but with special hacks for level 1...
/// We need to branch whenever there are alternatives.
/// We only care about reaching the end goal.
pub fn check_goals_op_model(
    s: &SPState, goal_invs: &[(Predicate, Option<Predicate>)], plan: &SPPlan,
    ts_model: &TransitionSystemModel, disabled_paths: &[SPPath], predicates: &[RunnerPredicate],
    operations: &[Operation]
) -> bool {
    println!("CHECKING OP GOALS - {}", goal_invs.iter().map(|(g, _)| format!("{}",g)).collect::<Vec<String>>().join(" && "));

    let invar = goal_invs
        .iter()
        .all(|(_, i)| i.as_ref().map(|i| i.eval(&s)).unwrap_or(true));
    if !invar {
        println!("breaks invariant");
        return false;
    }

    // check invariants
    // filter out unrelated state
    let rp = SPPath::from_slice(&["runner", "plans", "1"]);
    let new_state = s.clone();
    let new_state: Vec<_> = new_state
        .extract()
        .into_iter()
        .filter(|(p, _)| p == &rp || ts_model.vars.iter().any(|v| v.path() == p))
        .map(|(p, v)| (p, v.extract()))
        .collect();
    let new_state = SPState::new_from_values(&new_state);
    let mut states = vec![new_state.clone()];

    // TODO: we need to do something about this, I spent a lot of time.... figuring out
    // why some actions were not applied. atleast we should panic if the state path is wrong.

    // we need to clone the plan dues to our state getting a new id above....
    let mut plan = plan.clone();
    for x in plan.plan.iter_mut() {
        x.spec_transition.upd_state_path(&new_state);
    }

    // first apply all effects that we are waiting
    // on for executing operations.
    let running = s
        .projection()
        .state
        .iter()
        .filter_map(|(k, v)| {
            if v.current_value() != &"e".to_spvalue() {
                return None;
            }

            operations.iter().find(|op| &op.node().path() == k)
        })
        .collect::<Vec<&Operation>>();
    let ok = running.iter().all(|op| {
        let tp = op.node().path().clone().add_child("start"); // hack
        let t: Vec<_> = ts_model
            .transitions
            .iter()
            .filter(|t| t.path().parent() == tp.parent())
            .collect();
        if t.is_empty() {
            panic!("no such transition: {}", tp);
        } else if t.len() == 1 {
            // no alternative, just apply.
            let t = t.first().unwrap();

            let mut ok = false;
            states.iter_mut().for_each(|mut state| {
                let x = t.guard.support();
                // TODO: again, does not handle "mixed" operations
                let iok = if x
                    .iter()
                    .any(|p| !p.path.contains(&"product_state".to_string()))
                {
                    true
                } else {
                    t.eval(s)
                };

                if iok {
                    t.actions.iter().for_each(|e| {
                        let _res = e.next(&mut state);
                    });
                    ok = true;
                }
            });
            ok
        } else {
            // alternatives, add them later.
            let mut states_to_add = vec![];
            let mut ok = false;
            t.iter().for_each(|t| {
                states.iter_mut().for_each(|state| {
                    let x = t.guard.support();
                    // TODO: again, does not handle "mixed" operations
                    let iok = if x
                        .iter()
                        .any(|p| !p.path.contains(&"product_state".to_string()))
                    {
                        true
                    } else {
                        t.eval(state)
                    };
                    if iok {
                        // println!("ADDING STATE FOR {}", t.path());
                        let mut ns = state.clone();
                        t.actions.iter().for_each(|e| {
                            let _res = e.next(&mut ns);
                        });
                        states_to_add.push(ns);
                        ok = true;
                    }
                });
            });
            states.clear();
            states.extend(states_to_add);
            ok
        }
    });

    if !ok {
        // could not apply all transitions of running transitions, the state must have
        // changed in an unexpected way.
        println!("could not apply transitions. false");
        return false;
    }

    for mut state in &mut states {
        state.take_transition();

        println!("NEW STARTING STATE:\n{}", state);

        let mut goal_invs = goal_invs.to_vec();
        goal_invs.retain(|(g, i)| {
            !g.eval(state) && i.as_ref().map(|i| i.eval(state)).unwrap_or(true)
        });
        if goal_invs.is_empty() {
            return true;
        }

        let trans: Vec<Transition> = ts_model
            .transitions
            .iter()
            .filter(|t| plan.included_trans.contains(t.path()))
            .cloned()
            .collect();

        let tm =
            SPTicker::create_transition_map(&trans, &plan.plan, disabled_paths);

        // create

        loop {
            let invar = goal_invs
                .iter()
                .all(|(_, i)| i.as_ref().map(|i| i.eval(&state)).unwrap_or(true));
            if !invar {
                println!("breaks invariant");
                return false;
            }

            println!("NEW STATE:\n{}", state);

            let state_changed = tm
                .iter()
                .flat_map(|ts| {
                    if ts.iter().all(|t| {
                        let x = t.eval(&state);
                        let bad = bad_state(&state, ts_model);
                        println!("for {}: eval: {} / bad state: {}", t.path(), x, bad);
                        x && !bad
                    }) {
                        // transitions enabled. clone the state to start a new search branch.

                        // take all actions
                        ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                            println!("applied action: {}", a);
                            let _res = a.next(&mut state);
                        });

                        // next -> cur
                        let changed = state.take_transition();
                        Some(changed)
                    } else {
                        None
                    }
                })
                .any(|x| x);

            goal_invs.retain(|(g, _)| !g.eval(&state));
            if goal_invs.is_empty() {

                let invar = goal_invs
                    .iter()
                    .all(|(_, i)| i.as_ref().map(|i| i.eval(&state)).unwrap_or(true));
                if !invar {
                    println!("breaks invariant final step");
                    return false;
                }

                return true;
            } else if !state_changed {
                break;
            }
        }
    }
    return false; // all branches break:d
}


impl PlanningState {
    /// For each planning level, get the current goal and respective invariant
    /// that runner tries to reach.
    pub fn goal(&mut self, state: &SPState) -> Vec<Vec<(Predicate, Option<Predicate>)>> {
        let low_level = self.operations.iter().filter_map(|op| {
            if op.is_executing(state) {
                Some((op.get_goal(Some(state)), None))
            } else {
                None
            }
        }).collect();
        let high_level = self.int_goal(state);

        vec![low_level, high_level]
    }

    /// Currently we don't perform any concretization on the high level
    pub fn int_goal(&self, state: &SPState) -> Vec<(Predicate, Option<Predicate>)> {
        self.intention_goals
            .iter()
            .filter(|g| g.condition.eval(state))
            .map(|x| (x.goal.clone(), x.invariant.clone()))
            .collect()
    }


    /// Preprocess operation plan. The operation plan contains a given
    /// assignment of which outcome to expect from each operation. But
    /// since we cannot chose this in reality, we remove this
    /// information before sending the plan to the
    /// runner. Specifically, this function replaces the specific
    /// transitions on the form /operation/X where X is the outcome to
    /// be /operation/start.
    pub fn preprocess_operation_plan(&self, plan: &SPPlan) -> SPPlan {
        let ts1 = &self.transition_system_models[1];
        let mut new_plan = plan.clone();

        for p in &mut new_plan.plan {
            let new_sync = p.syncronized_with
                .iter()
                .map(|p| {
                    if ts1.transitions.iter().any(|t| t.path() == p) {
                        p.parent().add_child("start")
                    } else {
                        p.clone()
                    }
                })
                .collect();
            (*p).syncronized_with = new_sync;
        }
        new_plan
    }

}
