use sp_domain::*;

#[derive(Debug, PartialEq, Clone)]
pub struct RunnerPredicate(StatePath, Predicate);

impl RunnerPredicate {
    pub fn new(v: &Variable, state: &SPState) -> Option<Self> {
        if let VariableType::Predicate(p) = v.variable_type() {
            if let Some(sp) = state.state_path(v.path()) {
                return Some(RunnerPredicate(sp, p));
            }
        }
        None
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct SPTicker {
    pub state: SPState,
    pub transitions: Vec<Transition>,
    pub specs: Vec<TransitionSpec>,
    pub forbidden: Vec<IfThen>,
    pub predicates: Vec<RunnerPredicate>,
}

impl SPTicker {
    pub fn new() -> Self {
        SPTicker {
            state: SPState::new(),
            transitions: vec![],
            specs: vec![],
            forbidden: vec![],
            predicates: vec![],
        }
    }

    /// This function will execute the enabled transition and update the state.
    /// The functions returns the updated state and the fired transitions
    ///
    pub fn tick_transitions(&mut self, changed_variables: SPState) -> (&SPState, Vec<SPPath>) {
        let state_id = self.state.id();
        self.state.extend(changed_variables);
        if state_id != self.state.id() {
            // changed_variables have added a new variabel
            self.reload_state_paths();
        }

        let temp_transition_map = SPTicker::create_transition_map(&self.transitions, &self.specs);
        let fired = SPTicker::tick(
            &mut self.state,
            &temp_transition_map,
            &self.predicates,
            &self.forbidden,
        );
        self.state.take_transition();

        (&self.state, fired)
    }

    /// After changing anything in the Ticker, run this method to update the state variables.
    pub fn reload_state_paths(&mut self) {
        for x in self.transitions.iter_mut() {
            x.upd_state_path(&self.state)
        }
        for x in self.specs.iter_mut() {
            x.spec_transition.upd_state_path(&self.state)
        }
        for x in self.forbidden.iter_mut() {
            x.upd_state_path(&self.state)
        }
        for x in self.predicates.iter_mut() {
            let old_p: &StatePath = &x.0;
            let sp = self.state.state_path(&old_p.path).expect(&format!(
                "All predicates must have a path in the state!: P={:?}",
                old_p
            ));
            x.0 = sp;
        }
    }

    /// This function takes the specs and create a transition map so that the specs are syncronized
    /// with the correct transitions. This is used internally to simplify the runner before each tick. A spec is often generated
    /// by the planner or optimizer
    fn create_transition_map<'a>(
        ts: &'a [Transition],
        specs: &'a [TransitionSpec],
    ) -> Vec<Vec<&'a Transition>> {
        let mut temp_xs: Vec<(Vec<&SPPath>, Vec<&Transition>)> = specs
            .iter()
            .map(|s| {
                let x: Vec<&SPPath> = s.syncronized_with.iter().collect();
                (x, vec![&s.spec_transition])
            })
            .collect();
        for t in ts.iter() {
            let mut found = false;
            // let t: &Transition = t;  // Keeping these to show how to help the rust-analyzer find the types
            for (paths, ref mut transitions) in temp_xs.iter_mut() {
                if paths.contains(&t.path()) {
                    transitions.push(t);
                    found = true;
                }
            }
            if !found {
                temp_xs.push((vec![], vec![t]));
            }
        }
        temp_xs.into_iter().map(|(_, xs)| xs).collect()
    }

    /// The core tick function of the runner that tries to execute all transitions one step.
    ///
    /// state is the state that will be updated
    /// trans is a list of transition rows where each transition in a row is syncronized and will always fire together
    /// predicates is all state predicate variables that should update after each state change. A state predicate is a boolean state
    /// variable that is assigned its value based on a predicate on the current state.
    /// forbidden_states is a list of IfThen predicates that define forbidden states that the runner never should end up in. If a
    /// transition row takes the state into any of the forbidden states, the change is reverted and the transitions are not taken.
    ///
    /// A "transition row" can only execute if all guards of the transitions in the row are fullfilled and that no action tries to
    /// change an already updated variable (when a variable has a next value). When the transitions fires, the actions takes in order
    /// and if multiple actions tried to change the same variable, the first action will be used.
    ///
    /// Tick returns the paths of the transitions that fired
    ///
    fn tick(
        state: &mut SPState,
        trans: &[Vec<&Transition>],
        predicates: &[RunnerPredicate],
        forbidden_states: &[IfThen],
    ) -> Vec<SPPath> {
        SPTicker::upd_preds(state, predicates);
        trans
            .iter()
            .flat_map(|ts| {
                let xs = SPTicker::tick_ts(state, ts, forbidden_states);
                SPTicker::upd_preds(state, predicates);
                xs
            })
            .collect()
    }

    /// Update the state predicate variables
    fn upd_preds(state: &mut SPState, predicates: &[RunnerPredicate]) {
        predicates.iter().for_each(|pr| {
            let value = pr.1.eval(state).to_spvalue();
            if let Err(e) = state.force(&pr.0, value) {
                eprintln!(
                    "The predicate {:?} could not be updated in the runner. Got error: {}",
                    pr.0, e
                );
            }
        })
    }

    /// Check if the row of transitions is enabled and fire the actions. Also checks the forbidden state after
    /// the state change, and will revert if we are forbidden.
    fn tick_ts(
        state: &mut SPState,
        ts: &[&Transition],
        forbidden_states: &[IfThen],
    ) -> Vec<SPPath> {
        let enabled = ts.iter().all(|t| t.eval(state));
        if enabled {
            ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                a.next(state); // if actions write to the same path, only the first will be used
            });
            if SPTicker::check_forbidden(state, forbidden_states) {
                ts.iter().flat_map(|t| t.actions.iter()).for_each(|a| {
                    a.revert_action(state); // reverts the actions by removing next if we are in a forbidden state
                });
            } else {
                return ts.iter().map(|t| t.node().path().clone()).collect();
            }
        }
        vec![]
    }

    /// Are we in a forbidden state?
    fn check_forbidden(state: &mut SPState, forbidden_states: &[IfThen]) -> bool {
        forbidden_states
            .iter()
            .any(|f| f._if.eval(state) && f._then.eval(state))
    }
}

#[cfg(test)]
mod test_new_runner {
    use super::*;

    #[test]
    fn testing_tick() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let xy = SPPath::from_slice(&["x", "y"]);
        let pred = SPPath::from_slice(&["pred"]);

        let mut s = state!(ab => 2, ac => true, kl => 3, xy => false, pred => false);
        let p = pr! {{p!(!ac)} && {p!(!xy)}};

        let a = a!(ac = false);
        let b = a!(ab <- kl);
        let c = a!(xy ? p);

        let t1 = Transition::new("t1", p!(ac), vec![a], vec![], false);
        let t2 = Transition::new("t2", p!(!ac), vec![b], vec![], false);
        let t3 = Transition::new("t3", Predicate::TRUE, vec![c], vec![], false);

        let sp_pred = s.state_path(&pred).unwrap();
        let pred_ac = p!(ac);

        let rp = RunnerPredicate(sp_pred, pred_ac);

        let ts = vec![vec![&t1], vec![&t2], vec![&t3]];
        let ps = vec![rp];
        let fs = vec![];
        let res = SPTicker::tick(&mut s, &ts, &ps, &fs);
        println!("FIRED: {:?}", res);
    }

    #[test]
    fn creating_transition_map() {
        //create_transition_map<'a>(ts: &'a [Transition], specs: &'a [Spec]) -> Vec<Vec<&'a Transition>> {
        let t1 = Transition::new_empty("t1");
        let t2 = Transition::new_empty("t2");
        let t3 = Transition::new_empty("t3");
        let t4 = Transition::new_empty("t4");
        let t5 = Transition::new_empty("t5");
        let ts = vec![t1.clone(), t2.clone(), t3.clone(), t4.clone(), t5.clone()];

        let specs = vec![
            TransitionSpec::new("s1", Transition::new_empty("s1"), vec![t1.path().clone()]),
            TransitionSpec::new(
                "s2",
                Transition::new_empty("s2"),
                vec![t2.path().clone(), t3.path().clone()],
            ),
            TransitionSpec::new("s3", Transition::new_empty("s3"), vec![t1.path().clone()]),
            TransitionSpec::new("s4", Transition::new_empty("s4"), vec![]),
        ];

        let res = SPTicker::create_transition_map(&ts, &specs);
        println!("Creating ts map:");
        for xs in res.iter() {
            println!("{:?}", xs);
        }

        assert_eq!(
            res,
            vec!(
                vec!(&Transition::new_empty("s1"), &t1),
                vec!(&Transition::new_empty("s2"), &t2, &t3),
                vec!(&Transition::new_empty("s3"), &t1),
                vec!(&Transition::new_empty("s4")),
                vec!(&t4),
                vec!(&t5),
            )
        )
    }

    #[test]
    fn testing_large_model() {
        let plan = SPPath::from_string("plan");
        let mut s = state!(plan => 1);

        let ts: Vec<Transition> = (1..100)
            .map(|i| {
                let g = p! {plan == i};
                let a = a!(plan = (i + 1));
                Transition::new(&format!("t_{}", i), g, vec![a], vec![], false)
            })
            .collect();

        let upd_ts: Vec<Vec<&Transition>> = ts.iter().map(|t| (vec![t])).collect();
        let x = &upd_ts;

        let ps = vec![];
        let fs = vec![];
        for i in 1..100 {
            let res = SPTicker::tick(&mut s, &upd_ts, &ps, &fs);
            s.take_transition();
            println!("fired: {:?}, state: {:?}", res, s.sp_value_from_path(&plan));
        }
    }

    fn large_model() {
        let ps: Vec<SPPath> = (1..100)
            .map(|i| SPPath::from_string(&i.to_string()))
            .collect();

        let plan = SPPath::from_string("plan");

        let ts: Vec<Transition> = (1..100)
            .map(|i| {
                let g = p! {plan == i};
                let a = a!(plan = (i + 1));
                Transition::new(&format!("t_{}", i), g, vec![a], vec![], false)
            })
            .collect();
    }
}
