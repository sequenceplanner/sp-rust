#![allow(dead_code)]

use super::sp_ticker::{RunnerPredicate, SPTicker};
use sp_domain::*;
use crate::formal_model::*;
use std::collections::HashMap;

/// The SPRunner that keep tracks of the state and the transition execution
/// When using the runner, call the input method on every state change and probably
/// also at a specific frequency
#[derive(Debug, PartialEq, Clone)]
pub struct SPRunner {
    pub name: String,
    pub ticker: SPTicker,
    pub variables: Vec<Variable>,
    pub predicates: Vec<Variable>,
    pub intention_goals: Vec<IfThen>,
    pub global_transition_specs: Vec<TransitionSpec>,
    pub plans: Vec<SPPlan>, // one plan per namespace
    pub last_fired_transitions: Vec<SPPath>,
    pub transition_system_models: Vec<TransitionSystemModel>,
    pub in_sync: bool,
    pub resources: Vec<SPPath>,
    pub intentions: Vec<SPPath>,
    pub replan_specs: Vec<Spec>,
    pub operations: Vec<Operation>,
    pub operation_goals: HashMap<SPPath,Predicate>, // todo: move to the state
}

/// The input to the runner.
///
/// Tick is used to trigger one evaluation and execution of the transitions. Use
/// it to run the runner at a frequency.
///
/// TickAsync is used when the runner is in sync mode but youdo not want to
/// do the planning before executing the transition.
///
/// StateChange(upd_variables) is used when the input variables have changed. If
/// they are the same as the internal state, the runner will not execute the transitions.
///
/// Settings will change the internal settings of the runner. Maybe send in goals and specs using this?
///
#[derive(Debug, PartialEq, Clone)]
pub enum SPRunnerInput {
    Tick,
    StateChange(SPState),
    NodeChange(SPState),
    Settings(sp_runner_api::RunnerCommand), // Will come later
    NewPlan(i32, SPPlan),
}
#[derive(Debug, PartialEq, Clone, Default)]
pub struct SPPlan {
    //id: usize, // probably use later. Or maybe we should include some kind of timestamp,
    pub plan: Vec<TransitionSpec>, // the plan in the form of transition specification
    pub included_trans: Vec<SPPath>, // list of all included transitions in the plan (in order).
    pub state_change: SPState,     // for setting variables use in the plans
                                   //sequence: Vec<(usize, SPPath),  // probably need this when handling unsync planning
}

impl SPRunner {
    /// Creates a new runner
    pub fn new(
        name: &str,
        transitions: Vec<Transition>,
        variables: Vec<Variable>,
        intention_goals: Vec<IfThen>,
        global_transition_specs: Vec<TransitionSpec>,
        forbidden: Vec<IfThen>,
        transition_system_models: Vec<TransitionSystemModel>,
        resources: Vec<SPPath>,
        intentions: Vec<SPPath>,
        replan_specs: Vec<Spec>,
        operations: Vec<Operation>,
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
        state.add_variable(
            SPPath::from_slice(&["runner", "plans", "0"]),
            0.to_spvalue(),
        );
        state.add_variable(
            SPPath::from_slice(&["runner", "plans", "1"]),
            0.to_spvalue(),
        );
        let runner_predicates: Vec<RunnerPredicate> = preds
            .iter()
            .flat_map(|p| RunnerPredicate::new(p, &state))
            .collect();
        let mut ticker = SPTicker::new();
        ticker.state = state;
        ticker.transitions = transitions;
        ticker.forbidden = forbidden;
        ticker.predicates = runner_predicates;
        ticker.reload_state_paths();
        ticker.disabled_paths = resources.clone();

        SPRunner {
            name: name.to_string(),
            ticker,
            variables: vars,
            predicates: preds,
            intention_goals,
            global_transition_specs,
            plans: vec![SPPlan::default(); 2],
            last_fired_transitions: vec![],
            transition_system_models,
            in_sync: false,
            resources,
            intentions,
            replan_specs,
            operations,
            operation_goals: HashMap::new(),
        }
    }

    /// The main function to use when running the runner. Connect this to
    /// a channel either using async or standard threads
    pub fn input(&mut self, input: SPRunnerInput) {
        use sp_runner_api::RunnerCommand;
        match input {
            SPRunnerInput::Tick => {
                self.take_a_tick(SPState::new(), true);
            }
            SPRunnerInput::StateChange(s) => {
                self.take_a_tick(s, false);
            }
            SPRunnerInput::NodeChange(s) => {
                self.take_a_tick(s, true);
            }
            SPRunnerInput::Settings(cmd) => {
                match cmd {
                    RunnerCommand::Mode(_x) => {},
                    RunnerCommand::ForceState(_s) => {},
                    RunnerCommand::SetState(s) => {
                        println!("WE GOT A SET STATE: {}", s);
                        self.take_a_tick(s, false);
                    },
                    RunnerCommand::ForceGoal(_) => {},
                }
            }
            SPRunnerInput::NewPlan(idx, plan) => {
                self.plans[idx as usize] = plan;
                self.update_state_variables(self.plans[idx as usize].state_change.clone());
                self.load_plans();
            }
        }
    }

    /// Get the current state from the runner
    pub fn state(&self) -> &SPState {
        &self.ticker.state
    }

    /// For each planning level, get the current goal and respective invariant
    /// that runner tries to reach.
    pub fn goal(&mut self) -> Vec<Vec<(Predicate, Option<Predicate>)>> {
        let low_level = self.operation_goals.iter().filter_map(|(op,g)| {
            // we need to check if the operation is running, because
            // the goal is still there in error mode.
            let op = self.operations.iter().find(|o| o.path() == op).unwrap();
            let sp = op.node().path();
            let is_running = self.ticker.state.sp_value_from_path(sp).map(|v| v == &"e".to_spvalue()).unwrap_or(false);
            if is_running {
                Some((g.clone(), None))
            } else {
                None
            }
        }).collect();
        let high_level = self.int_goal();

        vec![low_level, high_level]
    }

    /// Currently we don't perform any concretization on the high level
    pub fn int_goal(&self) -> Vec<(Predicate, Option<Predicate>)> {
        self.intention_goals.iter()
            .filter(|g| g.condition.eval(&self.ticker.state))
            .map(|x| (x.goal.clone(), x.invariant.clone()))
            .collect()
    }

    pub fn bad_state(state: &SPState, ts_model: &TransitionSystemModel) -> bool {
        ts_model.specs.iter().any(|s| !s.invariant().eval(state))
    }

    /// Checks wheter we can reach the current goal As it doesnt
    /// exactly follow the plan but instead simulated the runner, it
    /// can fail if the plan contains two "choices" involving the same
    /// transitions. Then the "wrong" one might be taken first.
    pub fn check_goals_fast(&self, s: &SPState, goals: &[&Predicate], plan: &SPPlan, ts_model: &TransitionSystemModel) -> bool {
        if goals.iter().all(|g| g.eval(s)) {
            return true;
        }

        let trans: Vec<Transition> = ts_model.transitions.iter()
            .filter(|t| plan.included_trans.contains(t.path()))
            .cloned().collect();

        let tm = SPTicker::create_transition_map(&trans, &plan.plan, &self.ticker.disabled_paths);

        // we need to make sure goals can be ticked off when reached.
        let mut goals = goals.to_vec();

        let mut state = s.clone();
        let mut counter = 0;
        loop {
            let state_changed = tm.iter().flat_map(|ts| {
                if ts.iter().all(|t| {
                    let x = t.eval(&state);
                    //println!("for {}: {}", t.path(), x);
                    x
                }) {
                    // transitions enabled.
                    counter+=1;
                    if counter > 1000 {
                        // there is a self loop in the model
                        let t_names = ts.iter().map(|t|t.path().to_string()).collect::<Vec<_>>().join(",");
                        panic!("self loop with transitions {}", t_names);
                    }

                    // take all actions
                    ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                        let _res = a.next(&mut state);
                    });

                    // update state predicates
                    SPTicker::upd_preds(&mut state, &self.ticker.predicates);

                    // next -> cur
                    let changed = state.take_transition();

                    //println!("THE STATE\n{}", state);

                    if SPRunner::bad_state(&state, ts_model) {
                        None
                    } else {
                        Some(changed)
                    }
                } else {
                    None
                }
            }).any(|x|x);

            // if SPRunner::bad_state(&state, ts_model) {
            //     println!("final state is bad!");
            //     return false;
            // }

            goals.retain(|g| !g.eval(&state));
            if goals.is_empty() {
                return true;
            }
            else if !state_changed {
                // double check with complete search in the normal
                // case where we fail early in the plan this is still
                // fast, but it saves us from the issue descibed
                // above. worst case we waste some time here but but
                // it should be faster than replanning anyway.

                return self.check_goals_complete(s, &goals, plan, ts_model);
                // return false; // think about how we want to do it...
            }
        }
    }

    /// Same as above but with special hacks for level 1...
    /// We need to branch whenever there are alternatives.
    /// We only care about reaching the end goal.
    pub fn check_goals_op_model(&self, s: &SPState, goals: &[&Predicate], plan: &SPPlan, ts_model: &TransitionSystemModel) -> bool {
        // filter out unrelated state
        let rp = SPPath::from_slice(&["runner", "plans", "1"]);
        let new_state = s.clone();
        let new_state: Vec<_> = new_state.extract().into_iter().filter(|(p,_)| {
            p == &rp || ts_model.vars.iter().any(|v|v.path() == p)
        }).map(|(p,v)| (p, v.extract())).collect();
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
        let running = s.projection().state.iter()
            .filter_map(|(k,v)| {
                if v.current_value() != &"e".to_spvalue() {
                    return None
                }

                self.operations.iter().find(|op| &op.node().path() == k)
            }).collect::<Vec<&Operation>>();
        let ok = running.iter().all(|op| {
            let tp = op.node().path().clone().add_child("start"); // hack
            let t: Vec<_> = ts_model.transitions.iter().filter(|t| t.path().parent() == tp.parent()).collect();
            if t.is_empty() {
                panic!("no such transition: {}", tp);
            } else if t.len() == 1 {
                // no alternative, just apply.
                let t = t.first().unwrap();

                let mut ok = false;
                states.iter_mut().for_each(|mut state| {
                    let x = t.guard.support();
                    // TODO: again, does not handle "mixed" operations
                    let iok = if x.iter().any(|p| !p.path.contains(&"product_state".to_string())) {
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
                        let iok = if x.iter().any(|p| !p.path.contains(&"product_state".to_string())) {
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

            let mut goals = goals.to_vec();
            goals.retain(|g| !g.eval(state));
            if goals.is_empty() {
                return true;
            }

            let trans: Vec<Transition> = ts_model.transitions.iter()
                .filter(|t| plan.included_trans.contains(t.path()))
                .cloned().collect();

            let tm = SPTicker::create_transition_map(&trans, &plan.plan, &self.ticker.disabled_paths);

            // create

            loop {
                let state_changed = tm.iter().flat_map(|ts| {
                    if ts.iter().all(|t| {
                        let x = t.eval(&state);
                        // println!("for {}: {}", t.path(), x);
                        x
                    }) {
                        // transitions enabled. clone the state to start a new search branch.

                        // take all actions
                        ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                            let _res = a.next(&mut state);
                        });

                        // next -> cur
                        let changed = state.take_transition();
                        Some(changed)
                    } else {
                        None
                    }
                }).any(|x|x);

                goals.retain(|g| !g.eval(&state));
                if goals.is_empty() {
                    return true;
                }
                else if !state_changed {
                    break;
                }
            }
        }
        return false; // all branches break:d
    }

    /// A slower, but more forgiving forward search to goal.
    /// This handles the case described in check_goals_fast.
    pub fn check_goals_complete(&self, state: &SPState, goals: &[&Predicate], plan: &SPPlan, ts_model: &TransitionSystemModel) -> bool {
        if goals.iter().all(|g| g.eval(state)) {
            return true;
        }

        let trans: Vec<Transition> = ts_model.transitions.iter()
            .filter(|t| plan.included_trans.contains(t.path()))
            .cloned().collect();

        let tm = SPTicker::create_transition_map(&trans, &plan.plan, &self.ticker.disabled_paths);

        fn rec<'a>(state: &SPState, goals: &[&Predicate],
                   tm: &Vec<Vec<&'a Transition>>, ticker: &SPTicker, visited: &mut Vec<SPState>,
                   ts_model: &TransitionSystemModel) -> bool {
            if goals.iter().all(|g| g.eval(&state)) {
                return true;
            }

            let new_states: Vec<SPState> = tm.iter().flat_map(|ts| {
                if ts.iter().all(|t| t.eval(&state) && !SPRunner::bad_state(&state, ts_model)) {
                    // transitions enabled. clone the state to start a new search branch.
                    let mut state = state.clone();

                    // take all actions
                    ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                        // if actions write to the same path, only the first will be used
                        let _res = a.next(&mut state);
                    });

                    // update state predicates
                    SPTicker::upd_preds(&mut state, &ticker.predicates);

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
            }).collect();

            new_states.iter().any(|s| rec(&s, goals, tm, ticker, visited, ts_model))
        }

        let mut visited = vec![state.clone()];
        rec(state, &goals, &tm, &self.ticker, &mut visited, ts_model)
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

    fn take_a_tick(&mut self, state: SPState, check_resources: bool) {
        if check_resources {
            self.check_resources();
        }
        self.update_state_variables(state);

        // do nothing if we are in a (globally) bad state.
        // these exprs can be pretty big. do some benchmark here.
        let bad: Vec<_> = self
            .transition_system_models
            .iter()
            .flat_map(|ts| {
                ts.specs
                    .iter()
                    .filter(|s| !s.invariant().eval(&self.ticker.state))
            })
            .collect();

        if !bad.is_empty() {
            println!("\nDOING NOTHING: WE ARE IN A BAD STATE:\n");
            println!("{}", self.ticker.state);
            println!(
                "because of the following invariant(s):\n{}",
                bad.iter().map(|s| s.path().to_string()).collect::<Vec<_>>().join("n")
            );
            return;
        }

        let res = self.ticker.tick_transitions();

        let mut new_state = vec![];
        // check if we started any operations.
        for taken in &res.1 {
            println!("taken... {}", taken);
            for o in &self.operations {
                let start = o.node().path().clone().add_child("start"); // hack
                if &start == taken {
                    // since there is no way to control the outcome of
                    // operations with multiple effects, we must be ok
                    // with any

                    let goal = if o.effects_goals.len() == 1 {
                        // we need to update the goal and postcond.
                        Predicate::AND(
                            o.effects_goals[0].0
                                .iter()
                                .map(|e| e.to_concrete_predicate(&res.0).expect("weird goal"))
                                .collect())
                    } else {
                        // we need to update the goal and postcond.
                        let inner = o.effects_goals.iter()
                            .map(|(e,_goal)| Predicate::AND(
                                e.iter()
                                    .map(|e| e.to_concrete_predicate(&res.0).expect("weird goal"))
                                    .collect())).collect();
                        Predicate::OR(inner)
                    };

                    let goal_sp_val = goal.to_string().to_spvalue();
                    let goal_path = o.path().clone().add_child("goal");
                    new_state.push((goal_path, goal_sp_val));

                    self.operation_goals.insert(o.path().clone(), goal);
                }
            }
        }


        let new_state = SPState::new_from_values(&new_state);

        // update operation specs and clean goals
        let mut new_specs = Vec::new();
        for op in &self.operations {
            let sp = op.node().path();
            let is_running = res.0.sp_value_from_path(sp).map(|v| v == &"e".to_spvalue()).unwrap_or(false);
            let is_error = res.0.sp_value_from_path(sp).map(|v| v == &"error".to_spvalue()).unwrap_or(false);
            let finish = op.node().path().clone().add_child("finish"); // hack
            if is_running {
                let goal = self.operation_goals.get(op.path()).unwrap_or(&Predicate::FALSE);
                let spec = TransitionSpec::new(
                    &format!("{}_post", op.path()),
                    Transition::new("post", goal.clone(), vec![], TransitionType::Controlled),
                    vec![finish],
                );
                new_specs.push(spec);
            } else {
                // clear goal
                if !is_error {
                    // when error, remember goal.
                    self.operation_goals.remove(op.path());
                }

                // block finish
                let spec = TransitionSpec::new(
                    &format!("{}_post", op.path()),
                    Transition::new("post", Predicate::FALSE, vec![], TransitionType::Controlled),
                    vec![finish],
                );
                new_specs.push(spec);
            }
        }
        // temp: using this field for now as its not used for anything else...
        // make one for operations later.
        self.global_transition_specs = new_specs;

        self.last_fired_transitions = res.1;

        self.load_plans(); // only here to update ticker specs. do it better later
        self.update_state_variables(new_state); // show goals in the state.
    }

    fn load_plans(&mut self) {
        self.reload_state_paths_plans();
        self.ticker.specs = self.plans[0].plan.clone();

        // big hack to fix plan path names.
        let ts1 = self.transition_system_models[1].clone();

        let a = self.plans[1].plan.iter().map(|p| {
            let mut np = p.clone();
            let sync = np.syncronized_with.clone();
            let new_sync = sync.iter().map(|p| {
                if ts1.transitions.iter().any(|t| t.path() == p) {
                    p.parent().add_child("start")
                } else {
                    p.clone()
                }
            }).collect();
            np.syncronized_with = new_sync;
            np
        });

        self.ticker.specs.extend(a);

        self.ticker
            .specs
            .extend(self.global_transition_specs.clone());
    }

    fn reload_state_paths(&mut self) {
        println!("RELOAD STATE PATHS");
        self.ticker.reload_state_paths();
        self.reload_state_paths_plans();
        for x in self.intention_goals.iter_mut() {
            x.upd_state_path(&self.ticker.state)
        }
        for x in self.global_transition_specs.iter_mut() {
            x.spec_transition.upd_state_path(&self.ticker.state)
        }
    }
    fn reload_state_paths_plans(&mut self) {
        let s = &self.ticker.state;
        self.plans.iter_mut().for_each(|p| {
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
                PredicateValue::value(SPValue::Bool(true))
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
                PredicateValue::path(registered.clone())
            );
            let toff = Predicate::TOFF(PredicateValue::path(r.clone().add_child("timestamp")), PredicateValue::SPValue(SPValue::Int32(5000))); // Hardcoded 5 seconds
            let enabled_pred = Predicate::AND(vec!(is_member.clone(), toff.clone()));
            let enabled = enabled_pred.eval(&self.ticker.state);
            self.ticker.state.add_variable(r.clone(), SPValue::Bool(enabled));
        });

    }
}

// #[cfg(test)]
// mod test_new_runner {
//     use super::*;
//     use crate::planning::*;

//     #[test]
//     fn testing_me() {
//         let mut runner = make_dummy_robot_runner();

//         println!{""};
//         println!("Initial goal: {:?}", runner.goal());
//         println!{""};
//         println!("Initial State: {}", runner.state());
//         println!{""};
//         println!("Initial plan");
//         runner.ability_plan.plan.iter().for_each(|x| println!("{:?}", x));
//         println!{""};
//         println!("Transitions plan");
//         runner.ticker.transitions.iter().for_each(|x| println!("{:?}", x));
//         println!{""};
//         runner.input(SPRunnerInput::Tick);
//         println!("fired:");
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
//         println!("fired:");
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));

//         let planner_result = crate::planning::plan(&runner.transition_system_model, &runner.goal(), &runner.state(), 20);
//         let (tr, s) = convert_planning_result(&runner.transition_system_model, planner_result);
//         let plan = SPPlan{plan: tr, state_change: s};
//         runner.input(SPRunnerInput::AbilityPlan(plan));

//         runner.input(SPRunnerInput::Tick);
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
//         println!("fired:");
//         runner.input(SPRunnerInput::Tick);
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
//         println!("fired:");
//         runner.input(SPRunnerInput::Tick);
//         println!("fired:");
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
//         println!{""};
//         println!("State: {}", runner.state());
//         println!{""};
//         println!("goal:{:?}", runner.goal());
//         runner.input(SPRunnerInput::Tick);
//         println!("fired:");
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
//         println!{""};
//         println!("State: {}", runner.state());
//         println!{""};
//         runner.input(SPRunnerInput::Tick);
//         println!("fired:");
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
//         println!{""};
//         println!("State: {}", runner.state());
//         println!{""};
//         runner.input(SPRunnerInput::Tick);
//         runner.input(SPRunnerInput::Tick);
//         runner.input(SPRunnerInput::Tick);
//         runner.input(SPRunnerInput::Tick);
//         println!("fired:");
//         runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
//         println!{""};
//         println!("State: {}", runner.state());
//         println!{""};
//         runner.input(SPRunnerInput::Tick);
//     }

//     fn make_dummy_robot_runner() -> SPRunner {
//         let (model, mut initial_state) = crate::testing::two_dummy_robots();
//         let ts_model = TransitionSystemModel::from(&model);
//         let m: RunnerModel = crate::helpers::make_runner_model(&model);
//         //println!("{:?}", m);
//         let mut trans = vec!();
//         let mut restrict_controllable = vec!();
//         let false_trans = Transition::new("empty", Predicate::FALSE, vec!(), vec!(), true);
//         m.op_transitions.ctrl.iter().for_each(|t| {
//             trans.push(t.clone());
//             // restrict_controllable.push(TransitionSpec::new(
//             //     &format!("s_{}_false", t.name()),
//             //     false_trans.clone(),
//             //     vec!(t.path().clone())
//             // ))
//         });
//         m.op_transitions.un_ctrl.iter().for_each(|t| {
//             trans.push(t.clone());
//         });
//         m.ab_transitions.ctrl.iter().for_each(|t| {
//             trans.push(t.clone());
//             restrict_controllable.push(TransitionSpec::new(
//                 &format!("s_{}_false", t.path()),
//                 false_trans.clone(),
//                 vec!(t.path().clone())
//             ))
//         });
//         m.ab_transitions.un_ctrl.iter().for_each(|t| {
//             trans.push(t.clone());
//         });
//         let mut runner = SPRunner::new(
//             "test",
//             trans,
//             m.state_predicates.clone(),
//             m.goals.clone(),
//             vec!(),
//             vec!(),
//             ts_model
//         );
//         runner.input(SPRunnerInput::AbilityPlan(SPPlan{
//             plan: restrict_controllable,
//             state_change: SPState::new(),
//         }));
//         let the_upd_state = state!(
//             ["dummy_robot_model", "r1", "State", "act_pos"] => "away",
//             ["dummy_robot_model", "r1", "Control", "ref_pos"] => "away",
//             ["dummy_robot_model", "r2", "State", "act_pos"] => "away",
//             ["dummy_robot_model", "r2", "Control", "ref_pos"] => "away"
//         );
//         initial_state.extend(the_upd_state);
//         runner.update_state_variables(initial_state);
//         runner
//     }
// }

// name: &str,
//         transitions: Vec<Transition>,
//         variables: Vec<Variable>,
//         goals: Vec<IfThen>,
//         global_transition_specs: Vec<TransitionSpec>,
//         forbidden: Vec<IfThen>,
//         old_model: RunnerModel,

// pub struct RunnerModel {
//     pub op_transitions: RunnerTransitions,
//     pub ab_transitions: RunnerTransitions,
//     pub plans: RunnerPlans,
//     pub state_predicates: Vec<Variable>,
//     pub goals: Vec<IfThen>,
//     pub invariants: Vec<IfThen>,
//     pub model: Model,
// }
