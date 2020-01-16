use super::*;
/// In this file both predicates and actions are defined
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Predicate {
    AND(Vec<Predicate>),
    OR(Vec<Predicate>),
    XOR(Vec<Predicate>),
    NOT(Box<Predicate>),
    TRUE,
    FALSE,
    EQ(PredicateValue, PredicateValue),
    NEQ(PredicateValue, PredicateValue),
    // GT(PredicateValue, PredicateValue),
    // LT(PredicateValue, PredicateValue),
    // INDOMAIN(PredicateValue, Vec<PredicateValue>),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Action {
    pub var: SPPath,
    pub value: Compute,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PredicateValue {
    SPValue(SPValue),
    SPPath(SPPath),
}

/// Used in actions to compute a new SPValue.
/// When using delay and fetching a value from another variable, the current value of that
/// variable will be taken and assigned to the action variable after the delay, and not the
/// value that the other has after the delay.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Compute {
    PredicateValue(PredicateValue),
    Predicate(Predicate), // used for boolean actions
    Delay(PredicateValue, u64),
    CancelDelay,
    // If we need more advanced functions we can add them here
    //TakeNext(SPValue, Vec<SPValue>), // to be impl when needed
    //TakeBefore(SPValue, Vec<SPValue>),
    // Add(Box<Compute>, Box<Compute>),
    // Sub(Box<Compute>, Box<Compute>),
    // Join(Box<Compute>, Box<Compute>),
}

impl<'a> PredicateValue {
    pub fn get_value(&'a self, state: &'a SPState) -> Option<&'a SPValue> {
        match self {
            PredicateValue::SPValue(x) => Some(x),
            PredicateValue::SPPath(name) => state.get_value(&name),
        }
    }

    pub fn clone_with_global_paths(&self, parent: &GlobalPath) -> PredicateValue {
        match self {
            PredicateValue::SPPath(p) => {
                let np = match p {
                    SPPath::GlobalPath(gp) => SPPath::GlobalPath(gp.clone()),
                    SPPath::LocalPath(lp) => lp.to_global(parent).to_sp(),
                    SPPath::TemporaryPath(_tp) => panic!("temporary path must be resolved"),
                };
                return PredicateValue::SPPath(np);
            },
            _ => self.clone()
        }
    }

    // ns_to can be Local when you are done...
    pub fn temp_add_parent(&mut self, ns_from: TemporaryPathNS, ns_to: TemporaryPathNS, parent: &str) {
        match self {
            PredicateValue::SPPath(p) => {
                match p {
                    SPPath::TemporaryPath(tp) if tp.namespace == ns_from => {
                        let mut np = vec![parent.to_string()];
                        np.extend(tp.path.iter().cloned());
                        if ns_to == TemporaryPathNS::Local {
                            let lp = LocalPath::from(np);
                            *p = lp.to_sp();
                        } else {
                            let lp = TemporaryPath::from(ns_to, np);
                            *p = lp.to_sp();
                        }
                    },
                    _ => {}
                };
            },
            _ => {}
        }
    }

    // pub fn replace_variable_path(&mut self, map: &HashMap<SPPath, SPPath>) {
    //     if let PredicateValue::SPPath(n) = self {
    //         if map.contains_key(n) {
    //             *n = map.get(n).unwrap().clone();
    //         }
    //     }
    // }
}

impl Default for PredicateValue {
    fn default() -> Self {
        PredicateValue::SPValue(false.to_spvalue())
    }
}

// fn replace_in_vec(xs: &mut Vec<Predicate>, map: &HashMap<SPPath, SPPath>) {
//     xs.iter_mut().for_each(|p| {
//         p.replace_variable_path(map);
//     })
// }

impl Predicate {
    pub fn clone_with_global_paths(&self, parent: &GlobalPath) -> Predicate {
        match self {
            Predicate::AND(x) => Predicate::AND(x.iter().map(|p|p.clone_with_global_paths(parent)).collect()),
            Predicate::OR(x) => Predicate::OR(x.iter().map(|p|p.clone_with_global_paths(parent)).collect()),
            Predicate::XOR(x) => Predicate::XOR(x.iter().map(|p|p.clone_with_global_paths(parent)).collect()),
            Predicate::NOT(x) => Predicate::NOT(Box::new(x.clone_with_global_paths(parent))),
            Predicate::TRUE => self.clone(),
            Predicate::FALSE => self.clone(),
            Predicate::EQ(x, y) => Predicate::EQ(x.clone_with_global_paths(parent),
                                                 y.clone_with_global_paths(parent)),
            Predicate::NEQ(x, y) => Predicate::NEQ(x.clone_with_global_paths(parent),
                                                   y.clone_with_global_paths(parent)),
        }
    }

    pub fn change_temps(&mut self, ns_from: TemporaryPathNS, ns_to: TemporaryPathNS, new_parent: &str) {
        let upd_vec = |xs: &mut Vec<Predicate>| {
            xs.iter_mut().for_each(|p| {
                p.change_temps(ns_from, ns_to, new_parent);
            })
        };

        match self {
            Predicate::AND(x) => { upd_vec(x); }
            Predicate::OR(x) => { upd_vec(x); },
            Predicate::XOR(x) => { upd_vec(x); },
            Predicate::NOT(x) => { x.change_temps(ns_from, ns_to, new_parent); },
            Predicate::TRUE => {},
            Predicate::FALSE => {},
            Predicate::EQ(x, y) => {
                x.temp_add_parent(ns_from, ns_to, new_parent);
                y.temp_add_parent(ns_from, ns_to, new_parent);
            },
            Predicate::NEQ(x, y) => {
                x.temp_add_parent(ns_from, ns_to, new_parent);
                y.temp_add_parent(ns_from, ns_to, new_parent);
            },
        }
    }

    // pub fn replace_variable_path(&mut self, map: &HashMap<SPPath, SPPath>) {
    //     match self {
    //         Predicate::AND(x) => { replace_in_vec(x, map) }
    //         Predicate::OR(x) => { replace_in_vec(x, map) },
    //         Predicate::XOR(x) => { replace_in_vec(x, map) },
    //         Predicate::NOT(x) => { x.replace_variable_path(map)},
    //         Predicate::TRUE => {},
    //         Predicate::FALSE => {},
    //         Predicate::EQ(x, y) => {
    //             x.replace_variable_path(map);
    //             y.replace_variable_path(map);
    //         },
    //         Predicate::NEQ(x, y) => {
    //             x.replace_variable_path(map);
    //             y.replace_variable_path(map);
    //         },
    //     }
    // }
}

impl Action {

    pub fn clone_with_global_paths(&self, parent: &GlobalPath) -> Action {
        let mut clone = self.clone();
        let np = match clone.var {
            SPPath::GlobalPath(gp) => SPPath::GlobalPath(gp),
            SPPath::LocalPath(lp) => lp.to_global(parent).to_sp(),
            SPPath::TemporaryPath(_tp) => panic!("temporary path not fixed"),
        };
        clone.var = np;
        clone.value = self.value.clone_with_global_paths(parent);

        return clone;
    }

    // pub fn replace_variable_path(&mut self, map: &HashMap<SPPath, SPPath>) {
    //     match &mut self.value {
    //         Compute::PredicateValue(x) => { x.replace_variable_path(map) }
    //         Compute::Predicate(x) => { x.replace_variable_path(map) }
    //         Compute::Delay(x, _) => { x.replace_variable_path(map) }
    //         _ => (),
    //     }
    // }
}

impl Default for Predicate {
    fn default() -> Self {
        Predicate::TRUE
    }
}

impl Default for Compute {
    fn default() -> Self {
        Compute::PredicateValue(PredicateValue::default())
    }
}

impl Compute {
    pub fn clone_with_global_paths(&self, parent: &GlobalPath) -> Compute {
        match self {
            Compute::PredicateValue(p) => Compute::PredicateValue(p.clone_with_global_paths(parent)),
            Compute::Predicate(p) => Compute::Predicate(p.clone_with_global_paths(parent)),
            Compute::Delay(p, u) => Compute::Delay(p.clone_with_global_paths(parent), *u),
            Compute::CancelDelay => Compute::CancelDelay
        }
    }
}


/// Eval is used to evaluate a predicate (or an operation ).
pub trait EvaluatePredicate {
    fn eval(&self, state: &SPState) -> bool;
}

pub trait NextAction {
    fn next(&self, state: &SPState) -> SPResult<AssignState>;
}

impl EvaluatePredicate for Predicate {
    fn eval(&self, state: &SPState) -> bool {
        match self {
            Predicate::AND(ps) => ps.iter().all(|p| p.eval(state)),
            Predicate::OR(ps) => ps.iter().any(|p| p.eval(state)),
            Predicate::XOR(ps) => {
                ps.iter()
                    .filter(|p| p.eval(state))
                    .count()
                    == 1
            }
            Predicate::NOT(p) => !p.eval(state),
            Predicate::TRUE => true,
            Predicate::FALSE => false,
            Predicate::EQ(lp, rp) => lp.get_value(state) == rp.get_value(state),
            Predicate::NEQ(lp, rp) => lp.get_value(state) != rp.get_value(state), // Predicate::GT(lp, rp) => {}
                                                                                  // Predicate::LT(lp, rp) => {}
                                                                                  // Predicate::INDOMAIN(value, domain) => {}
        }
    }
}

impl NextAction for Action {
    fn next(&self, state: &SPState) -> SPResult<AssignState> {
        let c = match &self.value {
            Compute::PredicateValue(pv) => {
                match pv
                    .get_value(state)
                    .map(|x| AssignStateValue::SPValue(x.clone()))
                {
                    Some(x) => x,
                    None => {
                        eprintln!(
                            "The action {:?}, next did not find a value for variable: {:?}",
                            self, pv
                        );
                        return Err(SPError::No(format!(
                            "The action {:?}, next did not find a value for variable: {:?}",
                            self, pv
                        )));
                    }
                }
            }
            Compute::Predicate(p) => {
                let res = p.eval(state);
                AssignStateValue::SPValue(res.to_spvalue())
            }
            Compute::Delay(pv, ms) => {
                match pv
                    .get_value(state)
                    .map(|x| AssignStateValue::Delay(x.clone(), *ms))
                {
                    Some(x) => x,
                    None => {
                        eprintln!(
                            "The action {:?}, next did not find a value for variable: {:?}",
                            self, pv
                        );
                        return Err(SPError::No(format!(
                            "The action {:?}, next did not find a value for variable: {:?}",
                            self, pv
                        )));
                    }
                }
            }
            Compute::CancelDelay => {
                match state
                    .get_value(&self.var)
                    .map(|x| AssignStateValue::SPValue(x.clone()))
                {
                    Some(x) => x,
                    None => {
                        eprintln!("The delay action {:?}, did not exist in the state", self);
                        return Err(SPError::No(format!(
                            "The delay action {:?}, did not exist in the state",
                            self
                        )));
                    }
                }
            }
        };
        let s: HashMap<SPPath, AssignStateValue> =
            vec![(self.var.clone(), c)].into_iter().collect();
        Ok(AssignState { s })
    }
}

impl EvaluatePredicate for Action {
    fn eval(&self, state: &SPState) -> bool {
        match state.get(&self.var) {
            Some(StateValue::SPValue(_)) => true,
            Some(StateValue::Unknown) => true,
            Some(StateValue::Next(_)) => false,
            Some(StateValue::Delay(_)) => self.value == Compute::CancelDelay,
            None => true, // We allow actions to add new state variables. But maybe this is not ok?
        }
    }
}

// TODO: Just an experimental impl to learn. Hard to make it general
// TODO: Fix this later

#[macro_export]
macro_rules! p {
    // EQ
    ($name:ident == $value:expr) => {
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath($name.to_sp()),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    };
    ($name:block == $value:expr) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(&xs)),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    ($name:block == $value:block) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(&xs)),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    // NEQ
    ($name:ident != $value:expr) => {
        Predicate::NEQ(
            $crate::predicates::PredicateValue::SPPath($name.clone()),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    };
    ($name:block != $value:expr) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::NEQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(&xs)),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    ($name:block != $value:block) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::NEQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(&xs)),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    // Boolean variable
    ($name:ident) => {
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath($name.to_sp()),
            $crate::predicates::PredicateValue::SPValue(true.to_spvalue()),
        )
    };
    (!$name:ident) => {
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath($name.to_sp()),
            $crate::predicates::PredicateValue::SPValue(false.to_spvalue()),
        )
    };
    ($name:block) => {
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(&xs)),
            $crate::predicates::PredicateValue::SPValue(true.to_spvalue()),
        )
    };
    (!$name:block) => {
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(&xs)),
            $crate::predicates::PredicateValue::SPValue(false.to_spvalue()),
        )
    };
}

/// helping making predicates with a macro
#[macro_export]
macro_rules! pr {
    // AND
    ($first:ident && $($and:ident) &&* ) => {{
        let mut s: Vec<Predicate> = Vec::new();
        s.push($first.clone());
        $(
            s.push($and.clone());
        )*
        Predicate::AND(s)
    }};
    ($first:block && $($and:block) &&* ) => {{
        let mut s: Vec<Predicate> = Vec::new();
        s.push($first.clone());
        $(
            s.push($and.clone());
        )*
        Predicate::AND(s)
    }};
    // OR
    ($first:ident || $($and:ident) ||* ) => {{
        let mut s: Vec<Predicate> = Vec::new();
        s.push((&$first).clone());
        $(
            s.push((&$and).clone());
        )*
        Predicate::OR(s)
    }};
    ($first:block || $($and:block) ||* ) => {{
        let mut s: Vec<Predicate> = Vec::new();
        s.push($first.clone());
        $(
            s.push($and.clone());
        )*
        Predicate::OR(s)
    }};
    // NOT
    (!$not:expr) => {{
        let s = Box(not);
        Predicate::NOT(s)
    }};
    // NOT
    (!$not:block) => {{
        let s = Box(not);
        Predicate::NOT(s)
    }};
}

#[macro_export]
macro_rules! a {
    ($var:ident) => {
        Action {
            var: $var.to_sp(),
            value: $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPValue(true.to_spvalue()),
            ),
        }
    };
    (!$var:ident) => {
        Action {
            var: $var.to_sp(),
            value: $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPValue(false.to_spvalue()),
            ),
        }
    };
    ($var:ident = $val:expr) => {
        Action {
            var: $var.to_sp(),
            value: $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPValue($val.to_spvalue()),
            ),
        }
    };
    ($var:ident <- $val:expr) => {
        Action {
            var: $var.clone(),
            value: $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPPath($val.clone()),
            ),
        }
    };
    ($var:ident ? $val:expr) => {
        Action {
            var: $var.clone(),
            value: $crate::predicates::Compute::Predicate($val.clone()),
        }
    };
}

/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    #![warn(unused_must_use)]
    #![warn(unused_variables)]

    use super::*;
    #[test]
    fn create_predicate() {
        let v = SPPath::from_array(&["a", "b"]);

        //let eq = Predicate::EQ(PredicateValue::SPValue(2.to_spvalue()), PredicateValue::SPPath(v.clone()));

        let p = p! {v == 2};
        println!("TEST: {:?}", &p);
        let p2 = p! {{["a", "b"]} == 2};
        println!("TEST: {:?}", &p2);

        let x = pr! {p2 && p && p && p};
        println!("TEST2: {:?}", x);

        // not doesnt work...
        // let nx = pr! { !{p2 && p && p && p}};
        // println!("TEST2: {:?}", nx);

        let y = pr! {{p!{{["a", "b"]} == 10}} && {p!{{["a", "b"]} == 20}}};
        println!("TEST3: {:?}", y);

        let z = pr! { {p!{{["a", "b"]}}} && {p!{v == 2}}};
        let k = pr! {{pr!{x || y}} && {p!{v != 5}}};

        let long = pr! {z || k};
        println!("TEST4: {:?}", long);
    }

    #[test]
    fn eval_pred() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let v = SPPath::from_array(&["a", "b"]);
        let eq = Predicate::EQ(
            PredicateValue::SPValue(2.to_spvalue()),
            PredicateValue::SPPath(v.clone()),
        );
        let eq2 = Predicate::EQ(
            PredicateValue::SPValue(3.to_spvalue()),
            PredicateValue::SPPath(v.clone()),
        );
        assert!(eq.eval(&s));
        assert!(!eq2.eval(&s));
    }
    #[test]
    fn next_pred() {
        let ab = SPPath::from_array(&["a", "b"]);
        let ac = SPPath::from_array(&["a", "c"]);
        let kl = SPPath::from_array(&["k", "l"]);
        let xy = SPPath::from_array(&["x", "y"]);
        let mut s = state!(ab => 2, ac => true, kl => true, xy => false);
        let p = pr! {{p!(ac)} && {p!(kl)}};

        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
        };
        let a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPPath(kl)),
        };
        let a3 = Action {
            var: xy.clone(),
            value: Compute::Predicate(p),
        };

        a3.next(&s).map(|x| s.insert_map(x).unwrap()).unwrap();
        let next = StateValue::Next(states::Next {
            current_value: false.to_spvalue(),
            next_value: true.to_spvalue(),
        });
        assert_eq!(s.get(&xy), Some(&next));

        a.next(&s).map(|x| s.insert_map(x).unwrap()).unwrap();
        let next = StateValue::Next(states::Next {
            current_value: true.to_spvalue(),
            next_value: false.to_spvalue(),
        });
        assert_eq!(s.get(&ac), Some(&next));

        a2.next(&s).map(|x| s.insert_map(x).unwrap()).unwrap();
        let next = StateValue::Next(states::Next {
            current_value: 2.to_spvalue(),
            next_value: true.to_spvalue(),
        });
        assert_eq!(s.get(&ab), Some(&next));

        s.take_all_next();

        a3.next(&s).map(|x| s.insert_map(x).unwrap()).unwrap();
        let next = StateValue::Next(states::Next {
            current_value: true.to_spvalue(),
            next_value: false.to_spvalue(),
        });
        assert_eq!(s.get(&xy), Some(&next));

        println!("res from action: {:?} is: {:?}", &a2, &s);
    }
    #[test]
    fn action_macros() {
        let ab = SPPath::from_array(&["a", "b"]);
        let ac = SPPath::from_array(&["a", "c"]);
        let kl = SPPath::from_array(&["k", "l"]);
        let xy = SPPath::from_array(&["x", "y"]);
        //let s = state!(ab => 2, ac => true, kl => true, xy => false);
        let p = pr! {{p!(ac)} && {p!(kl)}};

        let a_m = a!(ac = false);
        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
        };
        assert_eq!(a_m, a);

        let a_m = a!(!ac);
        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
        };
        assert_eq!(a_m, a);

        let a_m = a!(ac);
        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())),
        };
        assert_eq!(a_m, a);

        let a2_m = a!(ab <- kl);
        let a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPPath(kl)),
        };
        assert_eq!(a2_m, a2);

        let a3_m = a!(xy ? p);
        let a3 = Action {
            var: xy.clone(),
            value: Compute::Predicate(p),
        };
        assert_eq!(a3_m, a3);
    }

    #[test]
    fn action_eval() {
        let ab = SPPath::from_array(&["a", "b"]);
        let ac = SPPath::from_array(&["a", "c"]);
        let kl = SPPath::from_array(&["k", "l"]);
        let mut s = state!(ab => 2, ac => true, kl => true);

        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
        };
        let a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPPath(kl)),
        };

        a.next(&s).map(|x| s.insert_map(x).unwrap()).unwrap();

        assert!(a2.eval(&s));
        assert!(!a.eval(&s));
    }


    #[test]
    fn test_make_guard() {
        let ab = SPPath::from_array(&["a", "b"]);
        let mut s = SPState::default();

        let a = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPValue(SPValue::Bool(true)))
        };

        let a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPValue(SPValue::Bool(false)))
        };

        a.next(&s).map(|x| s.insert_map(x).unwrap()).unwrap();
        s.take_all_next();
        println!("STATE: {:?}", s);
        a2.next(&s).map(|x| s.insert_map(x).unwrap()).unwrap();
        s.take_all_next();
        println!("STATE: {:?}", s);

        let guard = Predicate::AND(s.s.iter().flat_map(|(k,v)| {
            if let StateValue::SPValue(spval) = v {
                Some(Predicate::EQ(PredicateValue::SPPath(k.clone()),
                                   PredicateValue::SPValue(spval.clone())))
            } else { None }
        }).collect());


        println!("GUARD: {:?}", guard);

        assert!(false);
    }

    //#[test]
    // fn replace_variable_path() {
    //     let ab = SPPath::from_array(&["a", "b"]);
    //     let ac = SPPath::from_array(&["a", "c"]);
    //     let kl = SPPath::from_array(&["k", "l"]);

    //     let p = p!{ab == 2};
    //     let p2 = p!{{["a", "b"]} == 2};
    //     let x = pr!{p2 && p && p && p};
    //     let y = pr!{{p!{{["a", "c"]} == 10}} && {p!{{["a", "b"]} == 20}}};
    //     let z = pr!{ {p!{{["a", "c"]}}} && {p!{ab == 2}}};
    //     let k = pr!{{pr!{x || y}} && {p!{ab != 5}}};
    //     let mut long = pr!{z || k};
    //     println!("TEST: {:?}", &long.clone());

    //     let mut m = HashMap::new();
    //     m.insert(ab, kl);
    //     long.replace_variable_path(&m);
    //     println!("After replace: {:?}", &long);

    // }
}
