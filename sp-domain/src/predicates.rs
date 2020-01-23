use super::*;
/// In this file both predicates and actions are defined
use std::collections::HashMap;
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
    state_path: Option<StatePath>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PredicateValue {
    SPValue(SPValue),
    SPPath(SPPath, Option<StatePath>),
}


/// Used in actions to compute a new SPValue.
/// When using delay and fetching a value from another variable, the current value of that
/// variable will be taken and assigned to the action variable after the delay, and not the
/// value that the other has after the delay.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Compute {
    PredicateValue(PredicateValue),
    Predicate(Predicate), // used for boolean actions
    // If we need more advanced functions we can add them here
    //TakeNext(SPValue, Vec<SPValue>), // to be impl when needed
    //TakeBefore(SPValue, Vec<SPValue>),
    // Add(Box<Compute>, Box<Compute>),
    // Sub(Box<Compute>, Box<Compute>),
    // Join(Box<Compute>, Box<Compute>),
}

impl<'a> PredicateValue {
    pub fn sp_value(&'a self, state: &'a SPState) -> Option<&'a SPValue> {
        match self {
            PredicateValue::SPValue(x) => Some(x),
            PredicateValue::SPPath(path, sp) => {
                if sp.is_none() {
                    if let Some(the_path) = state.state_path(&path) {
                        return state.sp_value(&the_path);
                    }
                } else {
                    if let Some(the_path) = sp {
                        return state.sp_value(&the_path);
                    }
                }
                return None;
            }
        }
    }

    pub fn upd_state_path(&mut self, state: &SPState) {
        match self {
            PredicateValue::SPPath(path, sp) => {
                if sp.is_none() {
                    *sp = state.state_path(path)
                } else if sp.clone().map(|x| x.state_id != state.id()).unwrap_or(false) {
                    *sp = state.state_path(path);
                }


            }
            _ => {}
        }
    }

    pub fn value(v: SPValue) -> Self {
        PredicateValue::SPValue(v)
    }
    pub fn path(p: SPPath) -> Self {
        PredicateValue::SPPath(p, None)
    }


    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match self {
            PredicateValue::SPValue(_) => {} ,
            PredicateValue::SPPath(op, _) => {
                if let Some(np) = mapping.get(op) {
                    *op = np.clone();
                }
            }
        }
    }
}


impl Default for PredicateValue {
    fn default() -> Self {
        PredicateValue::SPValue(false.to_spvalue())
    }
}

impl Predicate {
    pub fn upd_state_path(&mut self, state: &SPState) {
        match self {
                Predicate::AND(x) => { x.iter_mut().for_each(|p| p.upd_state_path(state)) },
                Predicate::OR(x) => { x.iter_mut().for_each(|p| p.upd_state_path(state)) },
                Predicate::XOR(x) => { x.iter_mut().for_each(|p| p.upd_state_path(state)) },
                Predicate::NOT(x) => { x.upd_state_path(state) },
                Predicate::TRUE => {},
                Predicate::FALSE => {},
                Predicate::EQ(x, y) => {
                    x.upd_state_path(state);
                    y.upd_state_path(state);
                },
                Predicate::NEQ(x, y) => {
                    x.upd_state_path(state);
                    y.upd_state_path(state);
                },
            }
    }


    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match self {
            Predicate::AND(v) => { v.iter_mut().for_each(|e| e.replace_variable_path(mapping)); },
            Predicate::OR(v) => { v.iter_mut().for_each(|e| e.replace_variable_path(mapping)); },
            Predicate::XOR(v) => { v.iter_mut().for_each(|e| e.replace_variable_path(mapping)); },
            Predicate::NOT(b) => { b.replace_variable_path(mapping);},
            Predicate::TRUE => {},
            Predicate::FALSE => {},
            Predicate::EQ(pv1, pv2) => {
                pv1.replace_variable_path(mapping);
                pv2.replace_variable_path(mapping);
            },
            Predicate::NEQ(pv1, pv2) => {
                pv1.replace_variable_path(mapping);
                pv2.replace_variable_path(mapping);
            },
        }
    }
}

impl Action {
    pub fn new(var: SPPath, value: Compute) -> Self {
        Action {
            var,
            value,
            state_path: None,
        }
    }

    pub fn upd_state_path(&mut self, state: &SPState) {
        match &self.state_path {
            Some(sp) if sp.state_id != state.id() =>
                self.state_path = state.state_path(&self.var),
            None =>
                self.state_path = state.state_path(&self.var),
            _ => {}
        }
    }


    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        if let Some(np) = mapping.get(&self.var) {
            self.var = np.clone();
        }
        match &mut self.value {
            Compute::PredicateValue(pv) => { pv.replace_variable_path(mapping); }
            Compute::Predicate(p) => { p.replace_variable_path(mapping); }
        }
    }
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


/// Eval is used to evaluate a predicate (or an operation ).
pub trait EvaluatePredicate {
    fn eval(&self, state: &SPState) -> bool;
}

pub trait NextAction {
    fn next(&self, state: &mut SPState) -> SPResult<()>;
}

impl EvaluatePredicate for Predicate {
    fn eval(&self, state: &SPState) -> bool {
        match self {
            Predicate::AND(ps) => ps.iter().all(|p| p.eval(state)),
            Predicate::OR(ps) => ps.iter().any(|p| p.eval(state)),
            Predicate::XOR(ps) => {
                let mut c = 0;
                for p in ps.iter() {
                    if p.eval(state) {
                        c += 1;
                    }
                }
                c == 1
                // ps.iter_mut()
                //     .filter(|p| p.eval(state))  // for some reason does not filter with &mut
                //     .count()
                //     == 1
            }
            Predicate::NOT(p) => !p.eval(state),
            Predicate::TRUE => true,
            Predicate::FALSE => false,
            Predicate::EQ(lp, rp) => lp.sp_value(state) == rp.sp_value(state),
            Predicate::NEQ(lp, rp) => lp.sp_value(state) != rp.sp_value(state),
            // Predicate::GT(lp, rp) => {}
            // Predicate::LT(lp, rp) => {}
            // Predicate::INDOMAIN(value, domain) => {}
        }
    }
}

impl NextAction for Action {
    fn next(&self, state: &mut SPState) -> SPResult<()> {
        let c = match &self.value {
            Compute::PredicateValue(pv) => {
                match pv
                    .sp_value(state)
                    .map(|x| x.clone())
                {
                    Some(x) => x,
                    None => {
                        eprintln!(
                            "The action PredicateValue, next did not find a value for variable: {:?}",
                             pv
                        );
                        return Err(SPError::No(format!(
                            "The action PredicateValue, next did not find a value for variable: {:?}",
                             pv
                        )));
                    }
                }
            },
            Compute::Predicate(p) => {
                let res = p.eval(state);
                res.to_spvalue()
            }
        };

        match &self.state_path {
            Some(sp) => {
                state.next(&sp, c)
            },
            None => {
                state.next_from_path(&self.var, c)
            }
        }
    }

}

impl EvaluatePredicate for Action {
    fn eval(&self, state: &SPState) -> bool {
        let sp = match &self.state_path {
            Some(x) => {
                state.state_value(x)
            },
            None => state.state_value_from_path(&self.var)
        };
        match sp
        {
            Some(x) => x.has_next(),
            None => false, // We do not allow actions to add new state variables. But maybe this should change?
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
            $crate::predicates::PredicateValue::SPPath($name.clone(), None),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    };
    ($name:block == $value:expr) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(xs), None),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    ($name:block == $value:block) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(xs), None),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    // NEQ
    ($name:ident != $value:expr) => {
        Predicate::NEQ(
            $crate::predicates::PredicateValue::SPPath($name.clone(), None),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    };
    ($name:block != $value:expr) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::NEQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(xs), None),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    ($name:block != $value:block) => {{
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::NEQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(xs), None),
            $crate::predicates::PredicateValue::SPValue($value.to_spvalue()),
        )
    }};
    // Boolean variable
    ($name:ident) => {
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath($name.clone(), None),
            $crate::predicates::PredicateValue::SPValue(true.to_spvalue()),
        )
    };
    (!$name:ident) => {
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath($name.clone(), None),
            $crate::predicates::PredicateValue::SPValue(false.to_spvalue()),
        )
    };
    ($name:block) => {
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(xs), None),
            $crate::predicates::PredicateValue::SPValue(true.to_spvalue()),
        )
    };
    (!$name:block) => {
        let xs: Vec<String> = $name.iter().map(|x| x.to_string()).collect();
        Predicate::EQ(
            $crate::predicates::PredicateValue::SPPath(SPPath::from(xs), None),
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
        Action::new(
            $var.clone(),
            $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPValue(true.to_spvalue()),
            ),
        )
    };
    (!$var:ident) => {
        Action::new(
            $var.clone(),
            $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPValue(false.to_spvalue()),
            ),
        )
    };
    ($var:ident = $val:expr) => {
        Action::new(
            $var.clone(),
            $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPValue($val.to_spvalue()),
            ),
        )
    };
    ($var:ident <- $val:expr) => {
        Action::new(
            $var.clone(),
            $crate::predicates::Compute::PredicateValue(
                $crate::predicates::PredicateValue::SPPath($val.clone(), None),
            ),
        )
    };
    ($var:ident ? $val:expr) => {
        Action::new(
            $var.clone(),
            $crate::predicates::Compute::Predicate($val.clone()),
        )
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
        let v = SPPath::from_slice(&["a", "b"]);

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
        let v = SPPath::from_slice(&["a", "b"]);
        let mut eq = Predicate::EQ(
            PredicateValue::SPValue(2.to_spvalue()),
            PredicateValue::SPPath(v.clone(), None),
        );
        let mut eq2 = Predicate::EQ(
            PredicateValue::SPValue(3.to_spvalue()),
            PredicateValue::SPPath(v.clone(), None),
        );
        assert!(eq.eval(&s));
        assert!(!eq2.eval(&s));
    }
    #[test]
    fn next_pred() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let xy = SPPath::from_slice(&["x", "y"]);
        let mut s = state!(ab => 2, ac => true, kl => true, xy => false);
        let p = pr! {{p!(ac)} && {p!(kl)}};

        let mut a = Action::new(
            ac.clone(),
            Compute::PredicateValue(PredicateValue::default()),
        );
        let mut a2 = Action::new(
            ab.clone(),
            Compute::PredicateValue(PredicateValue::SPPath(kl, None)),
        );
        let mut a3 = Action::new(xy.clone(), Compute::Predicate(p));

        a3.next(&mut s);
        // let next = StateValue::Next(states::Next {
        //     current_value: false.to_spvalue(),
        //     next_value: true.to_spvalue(),
        // });
        println!{"next pred: {:?}", a3};
        //assert_eq!(s.state_value_from_path(&xy), Some(&next));

        a.next(&mut s);
        // let next = StateValue::Next(states::Next {
        //     current_value: true.to_spvalue(),
        //     next_value: false.to_spvalue(),
        // });
        //assert_eq!(s.state_value_from_path(&ac), Some(&next));

        a2.next(&mut s);
        // let next = StateValue::Next(states::Next {
        //     current_value: 2.to_spvalue(),
        //     next_value: true.to_spvalue(),
        // });
        // assert_eq!(s.state_value_from_path(&ab), Some(&next));

        s.take_transition();

        a3.next(&mut s);
        // let next = StateValue::Next(states::Next {
        //     current_value: true.to_spvalue(),
        //     next_value: false.to_spvalue(),
        // });
        // assert_eq!(s.state_value_from_path(&xy), Some(&next));

        println!("res from action: {:?} is: {:?}", &a2, &s);
    }
    #[test]
    fn action_macros() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let xy = SPPath::from_slice(&["x", "y"]);
        //let s = state!(ab => 2, ac => true, kl => true, xy => false);
        let p = pr! {{p!(ac)} && {p!(kl)}};

        let a_m = a!(ac = false);
        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
            state_path: None,
        };
        assert_eq!(a_m, a);

        let a_m = a!(!ac);
        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
            state_path: None,
        };
        assert_eq!(a_m, a);

        let a_m = a!(ac);
        let a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())),
            state_path: None,
        };
        assert_eq!(a_m, a);

        let a2_m = a!(ab <- kl);
        let a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPPath(kl, None)),
            state_path: None,
        };
        assert_eq!(a2_m, a2);

        let a3_m = a!(xy ? p);
        let a3 = Action {
            var: xy.clone(),
            value: Compute::Predicate(p),
            state_path: None,
        };
        assert_eq!(a3_m, a3);
    }

    #[test]
    fn action_eval() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let mut s = state!(ab => 2, ac => true, kl => true);

        let mut a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
            state_path: None,
        };
        let mut a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPPath(kl, None)),
            state_path: None,
        };

        a.next(&mut s);

        assert!(a2.eval(&s));
        assert!(!a.eval(&s));
    }


    #[test]
    fn test_make_guard() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let mut s = SPState::default();

        let mut a = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPValue(SPValue::Bool(true))),
            state_path: None
        };

        let mut a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPValue(SPValue::Bool(false))),
            state_path: None
        };

        a.next(&mut s);
        s.take_transition();
        println!("STATE: {:?}", s);
        a2.next(&mut s);
        s.take_transition();
        println!("STATE: {:?}", s);

        assert!(false);
    }

    #[test]
    fn replace_variable_path() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);

        let p = p!{ab == 2};
        let p2 = p!{{["a", "b"]} == 2};
        let x = pr!{p2 && p && p && p};
        let y = pr!{{p!{{["a", "c"]} == 10}} && {p!{{["a", "b"]} == 20}}};
        let z = pr!{ {p!{{["a", "c"]}}} && {p!{ab == 2}}};
        let k = pr!{{pr!{x || y}} && {p!{ab != 5}}};
        let mut long = pr!{z || k};
        println!("before");
        println!("{:?}", &long.clone());

        let mapping = hashmap![ ab => kl ];
        long.replace_variable_path(&mapping);
        println!("after");
        println!("{:?}", &long);

        assert!(false);

    }
}
