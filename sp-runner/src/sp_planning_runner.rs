#![allow(dead_code)]

use sp_domain::*;
use sp_ros::*;
use std::time::{Duration, Instant};
use super::sp_ticker::SPTicker;
use super::sp_runner::*;
use super::planning;
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

// some planning constants
const LVL0_MAX_STEPS: u32 = 25;
const LVL1_MAX_STEPS: u32 = 40;
const LVL1_CUTOFF: u32 = 20;
const LVL1_LOOKOUT: f32 = 1.25;
const LVL1_MAX_TIME: Duration = Duration::from_secs(5);



#[derive(Debug, Clone)]
pub struct PlanningState {
    pub plans: Vec<SPPlan>,
    pub transition_system_models: Vec<TransitionSystemModel>,
    pub intentions: Vec<SPPath>,
    pub intention_goals: Vec<IfThen>,
    pub replan_specs: Vec<Spec>,
    pub operations: Vec<Operation>,
    pub bad_state: bool,

    pub prev_goals: HashMap<usize, Vec<(Predicate, Option<Predicate>)>>,
    pub store_async: Arc<Mutex<planning::AsyncPlanningStore>>,
    pub store: planning::PlanningStore,
    pub disabled_operation_check: Instant,
    pub prev_disabled_operations: HashSet<SPPath>,
}

pub fn bad_state(state: &SPState, ts_model: &TransitionSystemModel) -> bool {
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

/// Checks wheter we can reach the current goal As it doesnt
/// exactly follow the plan but instead simulated the runner, it
/// can fail if the plan contains two "choices" involving the same
/// transitions. Then the "wrong" one might be taken first.
pub fn check_goals_fast(
    s: &SPState, goals: &[&Predicate], plan: &SPPlan, ts_model: &TransitionSystemModel,
    disabled_paths: &[SPPath]
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
        // println!("NEW STATE:\n{}", state);
        let state_changed = tm
            .iter()
            .flat_map(|ts| {
                if ts.iter().all(|t| {
                    let x = t.eval(&state);
                    // println!("for {}: eval: {}, trans: {}", t.path(), x, t);
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
                        // println!("applied action: {}", a);
                        let _res = a.next(&mut state);
                    });

                    // update state predicates
                    update_predicates(&mut state, &ts_model.state_predicates);

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
    disabled_paths: &[SPPath]
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
                    update_predicates(&mut state, &ts_model.state_predicates);

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
            .any(|s| rec(&s, goals, tm, visited, ts_model))
    }

    let mut visited = vec![state.clone()];
    rec(state, &goals, &tm, &mut visited, ts_model)
}

/// Same as above but with special hacks for level 1...
/// We need to branch whenever there are alternatives.
/// We only care about reaching the end goal.
pub fn check_goals_op_model(
    s: &SPState, goal_invs: &[(Predicate, Option<Predicate>)], plan: &SPPlan,
    ts_model: &TransitionSystemModel, disabled_paths: &[SPPath],
    operations: &[Operation]
) -> bool {
    // println!("CHECKING OP GOALS - {}", goal_invs.iter().map(|(g, _)| format!("{}",g)).collect::<Vec<String>>().join(" && "));

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

    pub fn compute_new_plan(&mut self, state: &SPState, disabled_paths: &[SPPath]) -> Option<(usize,SPPlan)> {
        if !self.bad_state {
            let mut temp_ts = self.transition_system_models[0].clone();
            temp_ts.specs.extend(self.transition_system_models[1].specs.iter().cloned());
            let bad: Vec<_> = temp_ts
                .specs
                .iter()
                .filter_map(|s| {
                    if !s.invariant().eval(state) {
                        Some(s)
                    } else {
                        None
                    }
                })
                .collect();

            if !bad.is_empty() {
                // try to find a way out of this situation by temporarily relaxing the specs
                // and instead planning to a new state where the specs holds.
                let mut temp_ts = self.transition_system_models[0].clone();
                temp_ts
                    .specs
                    .retain(|spec| !bad.iter().any(|b| b.path() == spec.path()));
                let goals = bad
                    .iter()
                    .map(|b| (b.invariant().clone(), None))
                    .collect::<Vec<_>>();
                let pr =
                    planning::plan_with_cache(&temp_ts, goals.as_slice(), state, LVL0_MAX_STEPS, &mut self.store);

                let plan = pr
                    .trace
                    .iter()
                    .map(|x| x.transition.to_string())
                    .collect::<Vec<_>>()
                    .join("\n");

                let spec_names = bad
                    .iter()
                    .map(|b| b.path().to_string())
                    .collect::<Vec<_>>()
                    .join(", ");
                log_error!("We are in a forbidden state! Spec(s) {} are violated. A way to get out could be \n{}", spec_names, plan);

                self.bad_state = true;
            }
        } else {
            self.bad_state =
                bad_state(state, &self.transition_system_models[0]);
        }

        if self.bad_state {
            return None;
        }

        println!("The State:\n{}", state);

        let ts_models = self.transition_system_models.clone();
        let goals = self.goal(state);
        for (i,g) in goals.iter().enumerate() {
            if !g.is_empty() {
                println!("Goals for {}", i);
                for g in g {
                    println!("{}", g.0);
                }
                println!("--");
            }
        }

        let disabled_operations: HashSet<SPPath> = self.operations.iter()
            .filter(|o|o.is_error(state))
            .map(|o|o.path().clone()).collect();

        let mut now = Instant::now();

        // TODO: Move to other function. This code handles error operations and
        // resets them if posssible. Will be checked now and then.
        if now.duration_since(self.disabled_operation_check).as_secs() > 5 {
            self.disabled_operation_check = std::time::Instant::now();

            let mut something_was_fixed = self.prev_disabled_operations != disabled_operations;

            let fixed_ops: Vec<SPPath> = disabled_operations.iter().filter(|p| {
                // check if the state if OK so we can remove any error states.
                println!(
                    "checking if we can remove error on low level operation: {}",
                    p
                );
                let op = self.operations.iter().find(|o|&o.path() == p)
                    .expect("no operation for disabled path");
                let goal = vec![(op.get_goal(None).clone(), None)];

                let mut ts = self.transition_system_models[0].clone();
                let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects"))
                    .state.into_iter().filter(|(_k,v)|
                                              v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                ts.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));

                let pr = planning::plan_with_cache(
                    &ts,
                    goal.as_slice(),
                    state,
                    LVL0_MAX_STEPS,
                    &mut self.store
                );

                if !pr.plan_found {
                    println!("operation still problematic...");
                } else {
                    // runner
                    //     .ticker
                    //     .state
                    //     .force_from_path(&p, &"i".to_spvalue())
                    //     .unwrap();
                    something_was_fixed = true;
                }

                !pr.plan_found
            }).cloned().collect();

            if something_was_fixed {
                // check all high level ops with error states. maybe we can move some of them
                // back to their executing state.
                // HACKS!

                for p in &self.intentions {
                    // TODO: add support in the domain instead of checking paths...
                    if state.sp_value_from_path(p).unwrap() == &"error".to_spvalue() {
                        println!(
                            "checking if we can remove error on high level operation: {}",
                            p
                        );
                        let goal_path = p.clone().add_child("goal");
                        println!("goal name {}", goal_path);

                        let g1 = &self.intention_goals;
                        let i: &IfThen = g1.iter().find(|g| g.path() == &goal_path).unwrap();
                        let goal = vec![(i.goal().clone(), None)];

                        let disabled_vec: Vec<_> = disabled_operations.iter().cloned().collect();
                        let pr = planning::plan_async_with_cache(
                            &self.transition_system_models[1],
                            &goal,
                            state,
                            &disabled_vec,
                            LVL1_MAX_STEPS,
                            LVL1_CUTOFF,
                            LVL1_LOOKOUT,
                            LVL1_MAX_TIME,
                            self.store_async.clone(),
                        );

                        if pr.plan_found {
                            log_info!("automatically restarting operation {}", p);
                            // runner
                            //     .ticker
                            //     .state
                            //     .force_from_path(&p, &"e".to_spvalue())
                            //     .unwrap();
                        }
                    }
                }
            }
        }

        for (i, (ts, goals)) in ts_models.iter().zip(goals.iter()).enumerate().rev() {
            //println!("TS {}", i);
            let mut ts = ts.clone();
            if i == 1 {
                ts.transitions
                    .retain(|t| !disabled_operations.contains(&t.path().parent()));
            }
            let ts = &ts;

            let planner = SPPath::from_slice(&["runner", "planner", &i.to_string()]);
            if state
                .sp_value_from_path(&planner)
                .unwrap_or(&false.to_spvalue())
                != &true.to_spvalue()
            {
                self.prev_goals.insert(i, Vec::new());
                let block_plan = SPPlan {
                    plan: crate::planning::block_all(ts),
                    included_trans: Vec::new(),
                    state_change: SPState::new(),
                };

                // update planning state plan
                self.plans[i] = block_plan.clone();

                // rename paths before sending plan to the runner.
                let plan_to_runner = if i == 1 {
                    self.preprocess_operation_plan(&block_plan)
                } else {
                    block_plan
                };

                return Some((i, plan_to_runner));
            }

            //println!("TS {} NOT SKIPPED", i);

            // for each namespace, check if we need to replan because
            // 1. got new goals from the runner or
            // 2. can no longer reach our goal (because the state has changed)

            // Because the plan is already encoded in the guards
            // of the runner transitions we can use the runner for
            // this purpose.

            // This is also true for the goals -> they are a function of the state.

            let prev_goal = self.prev_goals.get(&i);

            let gr: Vec<&Predicate> = goals.iter().map(|g| &g.0).collect(); // dont care about the invariants for now

            //println!("TS {} GOT GOALS", i);

            let replan = {
                let is_empty = prev_goal.is_none();
                if is_empty {
                    let pg = Predicate::AND(gr.iter().map(|&c| c.clone()).collect());
                    log_info!(
                        "replanning because previous goal was empty. {}: new goal: {}",
                        i,
                        pg
                    );
                }
                is_empty
            } || {
                let ok = &prev_goal.map(|g| g == goals).unwrap_or(false);
                // let all low-level effects complete before computing a new high level goal.
                if i == 1 && !ok {
                    let active_effects = self.replan_specs.iter().any(|t| {
                        let x = !t.invariant().eval(state);
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
                    let pg = Predicate::AND(gr.iter().map(|&c| c.clone()).collect());
                    log_info!("replanning because goal changed. {}: new goal: {}", i, pg);
                    prev_goal.map(|g| g.iter().for_each(|g| println!("prev goals {}", g.0)));
                }
                !ok
            } || {
                let now = Instant::now();

                println!("TS {} CHECKING GOALS", i);

                let ok = if i == 1 {
                    check_goals_op_model(
                        state,
                        goals,
                        &self.plans[i],
                        &self.transition_system_models[i],
                        disabled_paths,
                        &self.operations,
                    )
                } else {
                    let mut tsm = self.transition_system_models[i].clone();
                    let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects")).state.into_iter().filter(|(_k,v)|
                                                                                                                                             v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                    tsm.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));
                    check_goals_fast(
                        state,
                        &gr,
                        &self.plans[i],
                        &tsm,
                        disabled_paths,
                    )
                };

                //println!("TS {} CHECKING GOALS DONE", i);

                if now.elapsed().as_millis() > 100 {
                    println!(
                        "WARNINIG goal check for {}: {} (took {}ms)",
                        i,
                        ok,
                        now.elapsed().as_millis()
                    );
                }

                if !ok {
                    println!(
                        "goal check for {}: {} (took {}ms)",
                        i,
                        ok,
                        now.elapsed().as_millis()
                    );
                    let pg = Predicate::AND(gr.iter().map(|&c| c.clone()).collect());
                    log_info!("replanning because we cannot reach goal. {}: {}", i, pg);
                }
                !ok
            };

            if replan {
                //println!("TS {} REPLAN", i);

                println!("computing plan for namespace {}", i);

                let planner_result = if i == 0 {
                    // in the default case we should ensure monotonicity of the planning problem,
                    // because 1) sops and high level plans need it
                    // 2) operations (and errors!) are more intuitive this way.

                    let mono = SPPath::from_slice(&["runner", "planner", "monotonic"]);
                    let mut pr = if state
                        .sp_value_from_path(&mono)
                        .unwrap_or(&false.to_spvalue())
                        == &true.to_spvalue()
                    {
                        let modifies: HashSet<SPPath> =
                            goals.iter().map(|(a, _)| a.support()).flatten().collect();
                        let ts_1 = &self.transition_system_models[1];
                        let mut all: HashSet<SPPath> =
                            ts_1.vars.iter().map(|v| v.path().clone()).collect();
                        // HACK below!
                        all.retain(|p| p.path.contains(&"product_state".to_string()));
                        let no_change: HashSet<&SPPath> = all.difference(&modifies).collect();
                        let no_change_specs: Vec<Predicate> = no_change
                            .iter()
                            .flat_map(|p| {
                                state.sp_value_from_path(p).map(|val| {
                                    Predicate::EQ(
                                        PredicateValue::SPPath((*p).clone(), None),
                                        PredicateValue::SPValue(val.clone()),
                                    )
                                })
                            })
                            .collect();
                        // TODO: these specs should really be extended to include any auto trans which can change them.
                        // the extending should be computed whenever the model changes
                        // this is temporary.
                        let mut tts = ts.clone();
                        if !no_change_specs.is_empty() {
                            let no_change_pred = Predicate::AND(no_change_specs);
                            tts.specs
                                .push(Spec::new("monotonicity_constraints", no_change_pred));
                        }

                        // skip heuristic for the low level (cannot use cache if we dont serialize the extra invariants)

                        let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects")).state.into_iter().filter(|(_k,v)|
                                                                                                                                                 v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                        tts.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));

                        // planning::plan_with_cache(&tts, &goals, state, LVL0_MAX_STEPS, &mut self.store)
                        planning::plan(&tts, &goals, state, LVL1_MAX_STEPS)
                    } else {
                        // skip heuristic for the low level
                        // planning::plan_with_cache(ts, &goals, state, 2 * LVL0_MAX_STEPS, &mut self.store)
                        planning::plan(ts, &goals, state, 2 * LVL0_MAX_STEPS)
                    };

                    planning::bubble_up_delibs(ts, &gr, &mut pr);
                    pr
                } else {
                    let disabled_vec: Vec<_> = disabled_operations.iter().cloned().collect();
                    planning::plan_async_with_cache(
                        &ts,
                        &goals,
                        state,
                        &disabled_vec,
                        LVL1_MAX_STEPS,
                        LVL1_CUTOFF,
                        LVL1_LOOKOUT,
                        LVL1_MAX_TIME,
                        self.store_async.clone(),
                    )
                };

                let plan_p = SPPath::from_slice(&["runner", "plans", &i.to_string()]);

                let (tr, s) = if i == 1 {
                    planning::convert_planning_result_with_packing_heuristic(
                        &ts,
                        &planner_result,
                        &plan_p
                    )
                } else {
                    planning::convert_planning_result(&ts, &planner_result, &plan_p)
                };

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

                if no_plan && i == 0 {
                    // temp
                    std::fs::copy(
                        "./last_planning_request.bmc",
                        "./last_failed_planning_request.bmc",
                    )
                        .expect("file copy failed");
                    // no low level plan found, we are in trouble.

                    println!("no low level plan found!");

                    // look for the problematic goals
                    let executing_ops: Vec<&Operation> =
                        self.operations.iter()
                        .filter(|o| o.is_executing(state)).collect();
                    for o in executing_ops {
                        let op_path = o.path();
                        let goal = o.get_goal(Some(state));
                        println!("checking low level operation: {} -- {:?}", op_path, goal);
                        println!("disabled ops {:?}", disabled_operations);
                        if disabled_operations.contains(op_path) {
                            continue;
                        }
                        let goal = vec![(goal.clone(), None)];


                        let mut ts = ts.clone();
                        let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects"))
                            .state.into_iter().filter(|(_k,v)|
                                                      v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                        ts.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));


                        let pr = planning::plan_with_cache(
                            &ts,
                            goal.as_slice(),
                            state,
                            LVL0_MAX_STEPS,
                            &mut self.store
                        );
                        if !pr.plan_found {
                            println!("offending low level operation: {}", op_path);
                            log_warn!("offending low level operation: {}", op_path);
                            // TODO, verify that this works
                            plan.state_change.add_variable(op_path.clone(), "error".to_spvalue());
                            return Some((1, plan));
                        }
                    }
                    if disabled_operations.is_empty() {
                        // panic!("NO PLAN FOUND BUT ALSO NOW OFFENDING OPS");
                        log_error!("NO PLAN FOUND BUT ALSO NO OFFENDING OPS.\n\
                                    It could be that the planning horizon needs to be increased.");
                    }
                }

                if no_plan && i == 1 {
                    // no high level plan found, we are in trouble.

                    // look for the problematic goals
                    let g1 = &self.intention_goals;
                    let ifthens: Vec<&IfThen> = g1
                        .iter()
                        .filter(|g| g.condition.eval(state))
                        .collect();

                    for i in ifthens {
                        let goal = vec![(i.goal().clone(), i.invariant().clone())];
                        let disabled_vec: Vec<_> = disabled_operations.iter().cloned().collect();
                        let pr = planning::plan_async_with_cache(
                            &ts,
                            &goal,
                            state,
                            &disabled_vec,
                            LVL1_MAX_STEPS,
                            LVL1_CUTOFF,
                            LVL1_LOOKOUT,
                            LVL1_MAX_TIME,
                            self.store_async.clone(),
                        );

                        if !pr.plan_found {
                            let offending_op = i.path().parent();
                            log_warn!("offending high level operation: {}", offending_op);
                            // TODO, verify that this works
                            plan.state_change.add_variable(offending_op, "error".to_spvalue());
                        }
                    }
                }

                // temporary hack -- actually probably not so
                // temporary, this is something we need to deal with
                if i == 1 {
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
                }

                // update last set of goals
                if no_plan {
                    log_warn!(
                        "No plan was found for namespace {}! time to fail {}ms",
                        i,
                        planner_result.time_to_solve.as_millis()
                    );
                    self.prev_goals.remove(&i);
                } else {
                    log_info!(
                        "New plan was found for namespace {}! time to solve {}ms",
                        i,
                        planner_result.time_to_solve.as_millis()
                    );
                    self.prev_goals.insert(i, goals.clone());
                }

                // we save out own copy of the plan because we are
                // not necessarily in sync with the runner
                self.plans[i] = plan.clone();

                // rename paths before sending plan to the runner.
                let plan_to_runner = if i == 1 {
                    self.preprocess_operation_plan(&plan)
                } else {
                    plan
                };

                // return early as soon as we have a new plan
                return Some((i, plan_to_runner));
            }
        }
        println!("got to the end of planning task.");
        return None;
    }
}
