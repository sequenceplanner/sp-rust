#![allow(dead_code)]
use sp_domain::*;
use sp_formal::*;
use sp_ros::*;
use std::time::{Duration,Instant};
use super::sp_ticker::SPTicker;
use super::sp_runner::*;
use std::collections::HashSet;

// some planning constants
const LVL0_MAX_STEPS: u32 = 100;

#[derive(Debug, Clone)]
pub struct TransitionPlanner {
    pub plan: SPPlan, // current plan
    pub model: TransitionSystemModel, // planning model
    pub operations: Vec<Operation>, // our operations
    pub bad_state: bool, // todo.
    pub prev_state: SPState, // to check if something relevant for this planner has changed
    pub prev_goals: Vec<(Predicate, Option<Predicate>)>, // previous goals
    pub store: planning::PlanningStore, // cache
    pub disabled_operation_check: Instant,
}

fn bad_state(state: &SPState, ts_model: &TransitionSystemModel) -> bool {
    ts_model.invariants.iter().any(|s| !s.invariant().eval(state))
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
fn check_goals_fast(
    s: &SPState, goals: &[&Predicate], plan: &SPPlan, ts_model: &TransitionSystemModel
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

    let tm = SPTicker::create_transition_map(&trans, &plan.plan);

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
                    if !t.path().to_string().contains("Blocked") {
                        // println!("for {}: eval: {}, trans: {}", t.path(), x, t);
                    }
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
                        // println!("for {}: eval: {}, trans: {}", t.path(), x, t);
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

            // println!("NOT CHECKING GOALS COMPLETE, replan instead.");
            // return self.check_goals_complete(s, &goals, plan, ts_model);
            return false; // think about how we want to do it...
        }
    }
}

/// A slower, but more forgiving forward search to goal.
/// This handles the case described in check_goals_fast.
fn check_goals_complete(
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

    let tm = SPTicker::create_transition_map(&trans, &plan.plan);

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

impl TransitionPlanner {
    /// Get the current goals and invariants given a state.
    fn goals(&mut self, state: &SPState) -> Vec<(Predicate, Option<Predicate>)> {
        self.operations.iter().filter_map(|op| {
            if op.is_executing(state) {
                Some((op.get_goal(Some(state)), None))
            } else {
                None
            }
        }).collect()
    }

    pub fn block_all(&mut self) -> SPPlan {
        self.prev_goals.clear();
        let block_plan = SPPlan {
            plan: planning::block_all(&self.model),
            included_trans: Vec::new(),
            state_change: SPState::new(),
        };

        // update current plan
        self.plan = block_plan.clone();

        block_plan
    }

    /// Only keep parts of the state that are relevant to this planner.
    pub fn filter_state(&self, state: SPState) -> SPState {
        // Planning index
        let mut to_keep = vec![SPPath::from_slice(&["runner", "plans", "1"])];
        // We want to keep any paths contained in our formal model.
        to_keep.extend(self.model.get_state_paths());
        // As well as the state of any operations in the model.
        to_keep.extend(self.operations.iter().map(|o|o.path().clone()).collect::<Vec<_>>());

        // TODO. temporary effect flags
        self.model.transitions.iter().for_each(|t| {
            if t.type_ == TransitionType::Effect {
                to_keep.push(t.path().add_parent("effects"));
            }
        });

        state.filter_by_paths(&to_keep)
    }

    pub fn compute_new_plan(
        &mut self,
        mut state: SPState) -> Option<SPPlan> {
        let new_state = self.filter_state(state.clone());

        // nothing has changed, no need to do anything.
        if new_state == self.prev_state {
            // back off a bit
            std::thread::sleep(Duration::from_millis(1));
            return None;
        }

        // hack to add /auto_guards ...
        // this is used only in the planner
        let agp = SPPath::from_string("auto_guards");
        if let Some(ag) = self.model.state_predicates.iter().find(|v| v.path() == &agp) {
            if let VariableType::Predicate(p) = ag.variable_type() {
                let on = p.eval(&new_state);
                state.add_variable(agp, on.to_spvalue());
            }
        }

        self.prev_state = new_state.clone();

        if !self.bad_state {
            // previously we had the above
            // temp_ts.specs.extend(self.transition_system_models[1].specs.iter().cloned());
            let bad: Vec<_> = self.model
                .invariants
                .iter()
                .filter_map(|s| {
                    if !s.invariant().eval(&state) {
                        Some(s)
                    } else {
                        None
                    }
                })
                .collect();

            if !bad.is_empty() {
                let mut temp_ts = self.model.clone();
                // try to find a way out of this situation by temporarily relaxing the specs
                // and instead planning to a new state where the specs holds.
                temp_ts
                    .invariants
                    .retain(|spec| !bad.iter().any(|b| b.path() == spec.path()));
                let goals = bad
                    .iter()
                    .map(|b| (b.invariant().clone(), None))
                    .collect::<Vec<_>>();
                let pr = planning::plan(&temp_ts, goals.as_slice(), &state, LVL0_MAX_STEPS);

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

                self.bad_state = true; // TODO: should not be needed
            }
        } else {
            self.bad_state = bad_state(&state, &self.model);
        }

        if self.bad_state {
            return None;
        }

        let goals = self.goals(&state);
        println!("Transition planner goals:");
        for g in &goals {
            println!("{}", g.0);
        }
        println!("--");

        let disabled_operations: HashSet<SPPath> = self.operations.iter()
            .filter(|o|o.is_error(&state))
            .map(|o|o.path().clone()).collect();

        // TODO: Move to other function. This code handles error operations and
        // resets them if posssible. Will be checked now and then.
        if Instant::now().duration_since(self.disabled_operation_check).as_secs() > 5 {
            self.disabled_operation_check = Instant::now();

            let fixed_ops: Vec<SPPath> = disabled_operations.iter().filter_map(|p| {
                // check if the state if OK so we can remove any error states.
                println!(
                    "checking if we can remove error on low level operation: {}",
                    p
                );
                let op = self.operations.iter().find(|o|o.path() == p)
                    .expect("no operation for disabled path");
                let goal = vec![(op.get_goal(None).clone(), None)];

                let mut ts = self.model.clone();
                let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects"))
                    .state.into_iter().filter(|(_k,v)|
                                              v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                ts.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));

                let pr = planning::plan(
                    &ts,
                    goal.as_slice(),
                    &state,
                    LVL0_MAX_STEPS,
                );

                if !pr.plan_found {
                    println!("operation still problematic...");
                    None
                } else {
                    println!("operation fixed! {}", p);
                    Some(p)
                }
            }).cloned().collect();

            if !fixed_ops.is_empty() {
                fixed_ops.iter().for_each(|p| {
                    println!("fixed operation {} so we are returning plan that blocks everything.", p);
                    self.plan.state_change.add_variable(p.clone(), "i".to_spvalue());
                });
                return Some(self.plan.clone());
            }
        }

        // check if planner is enabled
        let planner = SPPath::from_slice(&["runner", "planner", "0"]);
        if state
            .sp_value_from_path(&planner)
            .unwrap_or(&false.to_spvalue())
            != &true.to_spvalue()
        {
            let block_plan = self.block_all();

            return Some(block_plan);
        }

        //println!("TS {} GOT GOALS", i);

        let gr: Vec<&Predicate> = goals.iter().map(|g| &g.0).collect();

        let replan = {
            let ok = goals == self.prev_goals;
            if !ok {
                let pg = Predicate::AND(goals.iter().map(|(c,_)| c.clone()).collect());
                log_info!("transition planner replanning because goal changed. new goal: {}", pg);
                self.prev_goals.iter().for_each(|g| println!("prev goals {}", g.0));
            }
            !ok
        } || {
            let now = Instant::now();

            let ok = {
                let mut tsm = self.model.clone();
                let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects")).state
                    .into_iter().filter(|(_k,v)| v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                tsm.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));
                check_goals_fast(
                    &state,
                    &gr,
                    &self.plan,
                    &tsm,
                )
            };

            if now.elapsed().as_millis() > 100 {
                println!(
                    "WARNINIG goal check for transition planner: {} (took {}ms)",
                    ok,
                    now.elapsed().as_millis()
                );
            }

            if !ok {
                println!(
                    "goal check for transition planner: {} (took {}ms)",
                    ok,
                    now.elapsed().as_millis()
                );
                let pg = Predicate::AND(goals.iter().map(|(c,_)| c.clone()).collect());
                log_info!("transition planner replanning because we cannot reach goal: {}", pg);
            }
            !ok
        };

        if !replan {
            // we are done!
            println!("did not have to replan transition planner.");
            return None;
        }


        println!("transition planner replanning ");

        let planner_result = {
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
                let product_state: HashSet<SPPath> =
                    self.model.vars.iter().filter_map(|v| {
                        if v.path().path.contains(&String::from("product_state")) {
                            Some(v.path().clone())
                        } else {
                            None
                        }
                    }).collect();
                // HACK below!
                let no_change: HashSet<&SPPath> = product_state.difference(&modifies).collect();
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
                let mut tts = self.model.clone();
                if !no_change_specs.is_empty() {
                    let no_change_pred = Predicate::AND(no_change_specs);
                    tts.invariants
                        .push(Specification::new_transition_invariant("monotonicity_constraints", no_change_pred));
                }

                // skip heuristic for the low level (cannot use cache if we dont serialize the extra invariants)

                let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects")).state
                    .into_iter().filter(|(_k,v)| v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                tts.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));

                // planning::plan_with_cache(&tts, &goals, state, LVL0_MAX_STEPS, &mut self.store)
                planning::plan(&tts, &goals, &state, LVL0_MAX_STEPS)
            } else {
                // skip heuristic for the low level
                // planning::plan_with_cache(ts, &goals, state, 2 * LVL0_MAX_STEPS, &mut self.store)
                planning::plan(&self.model, &goals, &state, 2 * LVL0_MAX_STEPS)
            };

            planning::bubble_up_delibs(&self.model, &gr, &mut pr);
            pr
        };

        let plan_p = SPPath::from_slice(&["runner", "plans", "0"]);

        let (tr, s) = planning::convert_planning_result(&self.model, &planner_result, &plan_p);

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
                .filter(|o| o.is_executing(&state)).collect();
            for o in executing_ops {
                let op_path = o.path();
                let goal = o.get_goal(Some(&state));
                println!("checking low level operation: {} -- {:?}", op_path, goal);
                println!("disabled ops {:?}", disabled_operations);
                if disabled_operations.contains(op_path) {
                    continue;
                }
                let goal = vec![(goal.clone(), None)];


                let mut ts = self.model.clone();
                let removed: Vec<SPPath> = state.sub_state_projection(&SPPath::from_string("effects"))
                    .state.into_iter().filter(|(_k,v)|
                                              v.value() == &false.to_spvalue()).map(|(k,_v)| k.drop_root()).collect();
                ts.transitions.retain(|t| !(t.type_ == TransitionType::Effect && removed.contains(t.path())));


                let pr = planning::plan(
                    &ts,
                    goal.as_slice(),
                    &state,
                    LVL0_MAX_STEPS,
                );
                if !pr.plan_found {
                    println!("offending low level operation: {}", op_path);
                    log_warn!("offending low level operation: {}", op_path);
                    // TODO, verify that this works
                    plan.state_change.add_variable(op_path.clone(), "error".to_spvalue());
                    return Some(plan);
                }
            }
            if disabled_operations.is_empty() {
                // panic!("NO PLAN FOUND BUT ALSO NOW OFFENDING OPS");
                log_error!("NO PLAN FOUND BUT ALSO NO OFFENDING OPS.\n\
                            It could be that the planning horizon needs to be increased.");
            }
        }

        // update last set of goals
        if no_plan {
            log_warn!(
                "No plan was found for transition planner! time to fail {}ms",
                planner_result.time_to_solve.as_millis()
            );
            self.prev_goals.clear();
        } else {
            log_info!(
                "New plan was found for transition planner! time to solve {}ms",
                planner_result.time_to_solve.as_millis()
            );
            self.prev_goals = goals.clone();
        }

        // we save out own copy of the plan because we are
        // not necessarily in sync with the runner
        self.plan = plan.clone();

        // rename paths before sending plan to the runner.
        // return early as soon as we have a new plan
        return Some(plan);
    }

    pub fn from(compiled: &CompiledModel) -> Self {
        let mut ts_model = TransitionSystemModel::from(&compiled.model);

        // TODO: do this in the model compilation step?
        ts_model.invariants.extend(compiled.computed_transition_planner_invariants.clone());

        let global_invariants: Vec<_> = compiled.model
            .global_specs
            .iter()
            .flat_map(|s|
                if let SpecificationType::OperationInvariant(_) = &s.type_ {
                    Some(s.clone())
                } else {
                    None
                })
            .collect();

        ts_model.invariants.extend(compiled.computed_transition_planner_invariants.clone());
        ts_model.invariants.extend(global_invariants);

        let operations = compiled.model.operations.clone();

        let tp = TransitionPlanner {
            plan: SPPlan::default(),
            model: ts_model,
            operations,
            bad_state: false,
            prev_state: SPState::new(),
            prev_goals: vec![],
            store: planning::PlanningStore::default(),
            disabled_operation_check: std::time::Instant::now(),
        };

        tp
    }
}
