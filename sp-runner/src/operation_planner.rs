#![allow(dead_code)]

use sp_domain::*;
use sp_ros::*;
use std::time::{Duration, Instant};
use super::sp_ticker::SPTicker;
use super::sp_runner::*;
use super::planning;
use std::collections::HashSet;
use std::sync::{Arc, Mutex};

// some planning constants
const LVL1_MAX_STEPS: u32 = 40;
const LVL1_CUTOFF: u32 = 20;
const LVL1_LOOKOUT: f32 = 1.25;
const LVL1_MAX_TIME: Duration = Duration::from_secs(5);

#[derive(Debug, Clone)]
pub struct OperationPlanner {
    pub plan: SPPlan, // current plan
    pub model: TransitionSystemModel, // planning model
    pub operations: Vec<Operation>, // our operations
    pub intentions: Vec<Intention>,
    pub replan_specs: Vec<Spec>,
    pub prev_state: SPState,
    pub prev_goals: Vec<(Predicate, Option<Predicate>)>, // previous goals
    pub store_async: Arc<Mutex<planning::AsyncPlanningStore>>,
    pub disabled_operation_check: Instant,
    pub prev_disabled_operations: HashSet<SPPath>,
}

fn bad_state(state: &SPState, ts_model: &TransitionSystemModel) -> bool {
    ts_model.specs.iter().any(|s| !s.invariant().eval(state))
}

/// Update the state predicate variables
/// TODO: this is very similar to the ticker code. Break out to its own crate perhaps.
fn update_predicates(state: &mut SPState, predicates: &[Variable]) {
    for v in predicates {
        if let VariableType::Predicate(p) = v.variable_type() {
            let value = p.eval(state).to_spvalue();
            if let Err(e) = state.force_from_path(v.path(), &value) {
                eprintln!(
                    "The predicate {:?} coult not be updated. Got error: {}",
                    p, e
                );
            }
        }
    }
}

/// Same as above but with special hacks for level 1...
/// We need to branch whenever there are alternatives.
/// We only care about reaching the end goal.
pub fn check_goals_op_model(
    s: &SPState, goal_invs: &[(Predicate, Option<Predicate>)], plan: &SPPlan,
    ts_model: &TransitionSystemModel, disabled_paths: &[SPPath],
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

            println!("is executing: {}", k);

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
                    let x = t.eval(s);
                    println!("eval for {}: {}", t.path(), x);
                    x
                };

                if iok {
                    t.actions.iter().for_each(|e| {
                        println!("applied {}", e);
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
                        let x = t.eval(state);
                        println!("eval for {}: {}", t.path(), x);
                        x
                    };
                    if iok {
                        // println!("ADDING STATE FOR {}", t.path());
                        let mut ns = state.clone();
                        t.actions.iter().for_each(|e| {
                            println!("applied {}", e);
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

        // println!("NEW STARTING STATE:\n{}", state);

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
                // println!("breaks invariant");
                return false;
            }

            // println!("NEW STATE:\n{}", state);

            let state_changed = tm
                .iter()
                .flat_map(|ts| {
                    if ts.iter().all(|t| {
                        let x = t.eval(&state);
                        let bad = bad_state(&state, ts_model);
                        // println!("for {}: eval: {} / bad state: {}", t.path(), x, bad);
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

impl OperationPlanner {
    /// Get the current goals and invariants given a state.
    fn goals(&mut self, state: &SPState) -> Vec<(Predicate, Option<Predicate>)> {
        self.intentions.iter().filter_map(|i| {
            if i.is_executing(state) {
                Some((i.get_goal(), None))
            } else {
                None
            }
        }).collect()
    }

    /// Preprocess operation plan. The operation plan contains a given
    /// assignment of which outcome to expect from each operation. But
    /// since we cannot chose this in reality, we remove this
    /// information before sending the plan to the
    /// runner. Specifically, this function replaces the specific
    /// transitions on the form /operation/X where X is the outcome to
    /// be /operation/start.
    pub fn preprocess_operation_plan(&self, plan: &SPPlan) -> SPPlan {
        let mut new_plan = plan.clone();

        for p in &mut new_plan.plan {
            let new_sync = p.syncronized_with
                .iter()
                .map(|p| {
                    if self.model.transitions.iter().any(|t| t.path() == p) {
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

    pub fn block_all(&mut self) -> SPPlan {
        self.prev_goals.clear();
        let block_plan = SPPlan {
            plan: crate::planning::block_all(&self.model),
            included_trans: Vec::new(),
            state_change: SPState::new(),
        };

        // update planning state plan
        self.plan = block_plan.clone();

        // rename paths before sending plan to the runner.
        self.preprocess_operation_plan(&block_plan)
    }


    /// Only keep parts of the state that are relevant to this planner.
    /// This will later be moved to a callback to the threaded runner so we
    /// can be notified upon change from there instead of busy waiting.
    pub fn filter_state(&self, state: SPState) -> SPState {
        // Planning index
        let mut to_keep = vec![SPPath::from_slice(&["runner", "plans", "1"])];
        // We want to keep any paths contained in our formal model.
        to_keep.extend(self.model.get_state_paths());
        // As well as the state of any operations in the model.
        to_keep.extend(self.operations.iter().map(|o|o.path().clone()).collect::<Vec<_>>());
        // As well as the state of any intentions in the model.
        to_keep.extend(self.intentions.iter().map(|o|o.path().clone()).collect::<Vec<_>>());
        state.filter_by_paths(&to_keep)
    }

    pub fn compute_new_plan(&mut self, state: SPState, disabled_paths: &[SPPath]) -> Option<SPPlan> {
        let new_state = self.filter_state(state.clone());

        // nothing has changed, no need to do anything.
        if new_state == self.prev_state {
            // back off a bit
            std::thread::sleep(Duration::from_millis(1));
            return None;
        }
        self.prev_state = new_state;

        // the reason we don't work on new_state below is that the statepaths won't be correct....

        if bad_state(&state, &self.model) {
            return None;
        }

        let goals = self.goals(&state);
        println!("Operation planner goals:");
        for g in &goals {
            println!("{}", g.0);
        }
        println!("--");

        let disabled_operations: HashSet<SPPath> = self.operations.iter()
            .filter(|o|o.is_error(&state))
            .map(|o|o.path().clone()).collect();

        // either something was fixed by the user or by this code the previous cycle.
        let something_was_fixed = disabled_operations.len() < self.prev_disabled_operations.len();
        self.prev_disabled_operations = disabled_operations.clone();

        if something_was_fixed {
            println!("atleast one operation has left its error state, checking all intentions");
            // check all high level ops with error states. maybe we can move some of them
            // back to their executing state.
            // HACKS!

            let mut fixed_its = vec![];
            for i in &self.intentions {
                if i.is_error(&state) {
                    println!("checking if we can remove error on intention: {}", i.path());
                    let goal = vec![(i.get_goal().clone(), None)];

                    let pr = planning::plan_async(
                        &self.model,
                        &goal,
                        &state,
                        LVL1_MAX_STEPS,
                        LVL1_CUTOFF,
                        LVL1_LOOKOUT,
                        LVL1_MAX_TIME,
                    );

                    if pr.plan_found {
                        fixed_its.push(i.path().clone());
                    }
                }
            }

            let block_plan = SPPlan {
                plan: crate::planning::block_all(&self.model),
                included_trans: Vec::new(),
                state_change: SPState::new(),
            };
            let mut block_plan = self.preprocess_operation_plan(&block_plan);
            fixed_its.iter().for_each(|p| {
                println!("fixed intention {} so we put it back in executing state", p);
                block_plan.state_change.add_variable(p.clone(), "e".to_spvalue());
            });
            self.plan = block_plan.clone();

            return Some(block_plan);
        }

        // TODO: fixme
        let mut ts = self.model.clone();
        ts.transitions.retain(|t| !disabled_operations.contains(&t.path().parent()));
        let ts = &ts; // ??

        // check if operation planner is disabled.
        let planner = SPPath::from_slice(&["runner", "planner", "1"]);
        if state
            .sp_value_from_path(&planner)
            .unwrap_or(&false.to_spvalue())
            != &true.to_spvalue()
        {
            let plan = self.block_all();
            return Some(plan);
        }

        let changed = {
            let ok = goals == self.prev_goals;
            // let all low-level effects complete before computing a new high level goal.
            if !ok {
                let active_effects = self.replan_specs.iter().any(|t| {
                    let x = !t.invariant().eval(&state);
                    if x {
                        println!("Effect in progress: {}", t.path());
                    }
                    x
                });
                if active_effects {
                    println!("XXX effects in progress, replanning later....");
                    return None;
                }
            }
            if !ok {
                let pg = Predicate::AND(goals.iter().map(|(c,_)| c.clone()).collect());
                log_info!("operation planner replanning because goal changed. new goal: {}", pg);
                self.prev_goals.iter().for_each(|g| println!("prev goals {}", g.0));
            }
            !ok
        };

        let goal_check = !changed && {
            let now = Instant::now();

            let ok = {
                check_goals_op_model(
                    &state,
                    &goals,
                    &self.plan,
                    &self.model,
                    disabled_paths,
                    &self.operations,
                )
            };

            if now.elapsed().as_millis() > 100 {
                println!(
                    "WARNINIG goal check for operation planner: {} (took {}ms)",
                    ok,
                    now.elapsed().as_millis()
                );
            }

            if !ok {
                println!(
                    "goal check for operation planner: {} (took {}ms)",
                    ok,
                    now.elapsed().as_millis()
                );
                let pg = Predicate::AND(goals.iter().map(|(c,_)| c.clone()).collect());
                log_info!("operation planner replanning because we cannot reach goal: {}", pg);
            }
            !ok
        };

        let replan = changed | goal_check;

        if replan {
            println!("OPERATION PLANNER REPLAN");

            let planner_result = {
                planning::plan_async(
                    &ts,
                    &goals,
                    &state,
                    LVL1_MAX_STEPS,
                    LVL1_CUTOFF,
                    LVL1_LOOKOUT,
                    LVL1_MAX_TIME,
                )
            };

            let plan_p = SPPath::from_slice(&["runner", "plans", "1"]);

            let (tr, s) =
                planning::convert_planning_result_with_packing_heuristic(
                    &ts,
                    &planner_result,
                    &plan_p
                );

            let trans: Vec<_> = planner_result
                .trace
                .iter()
                .filter_map(|f| {
                    if f.transition.is_empty() {
                        None
                    } else {
                        Some(f.transition.clone())
                    }
                })
                .collect();


            let mut plan = SPPlan {
                plan: tr,
                included_trans: trans,
                state_change: s,
            };

            // hack to reset the user visible planning steps when we replan
            // because sp_ui cannot remove states anyway (TODO!), we set dummy values
            let filtered_state: Vec<_> = state.projection().state
                .into_iter()
                .filter_map(|(p, _)| {
                    let in_new_plan = plan.state_change.projection()
                        .state
                        .iter()
                        .find(|(path,_)| &p == path).is_some();

                    if !in_new_plan && p.is_child_of(&plan_p) {
                        Some((p.clone(), "-".to_spvalue()))
                    } else {
                        None
                    }
                }).collect();
            plan.state_change.add_variables(filtered_state);

            let no_plan = !planner_result.plan_found;

            if no_plan {
                println!("no operation plan found...");

                // look for the problematic goals
                for i in &self.intentions {
                    let goal = vec![(i.get_goal().clone(), None)];
                    let pr = planning::plan_async(
                        &ts,
                        &goal,
                        &state,
                        LVL1_MAX_STEPS,
                        LVL1_CUTOFF,
                        LVL1_LOOKOUT,
                        LVL1_MAX_TIME,
                    );

                    if !pr.plan_found {
                        let offending = i.path().parent();
                        log_warn!("offending intention: {}", offending);
                        // TODO, verify that this works
                        plan.state_change.add_variable(offending, "error".to_spvalue());
                    }
                }
            }

            println!("resetting all operation state");

            for op in &self.operations {
                if disabled_operations.contains(op.path()) {
                    continue;
                }
                let path = op.node().path();
                if state
                    .sp_value_from_path(path)
                    .map(|v| v == &"e".to_spvalue())
                    .unwrap_or(false)
                {
                    plan.state_change.add_variable(path.clone(), "i".to_spvalue());
                }
            }

            // update last set of goals
            if no_plan {
                log_warn!(
                    "No plan was found for operation planner! Time to fail {}ms",
                    planner_result.time_to_solve.as_millis()
                );
                self.prev_goals.clear();
            } else {
                log_info!(
                    "New plan was found for operation planner!! Time to solve {}ms",
                    planner_result.time_to_solve.as_millis()
                );
                self.prev_goals = goals.clone();
            }

            // we save out own copy of the plan because we are
            // not necessarily in sync with the runner
            self.plan = plan.clone();

            // rename paths before sending plan to the runner.
            let plan_to_runner = self.preprocess_operation_plan(&plan);

            // return early as soon as we have a new plan
            return Some(plan_to_runner);
        }

        return None;
    }
}
