/// In this file both predicates and actions are defined

use serde::{Deserialize, Serialize};
use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PredValue {
    SPValue(SPValue),
    SPName(SPName)
}

impl<'a> PredValue {
    pub fn get_value(&'a self, state: &'a State) -> Option<&'a SPValue> {
        match self {
            PredValue::SPValue(x) => Some(x),
            PredValue::SPName(name) => state.get_value(&name)
        }
    }
}

impl Default for PredValue {
    fn default() -> Self {
        PredValue::SPValue(false.to_spvalue())
    }
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Predicate {
    AND(Vec<Predicate>),
    OR(Vec<Predicate>),
    XOR(Vec<Predicate>),
    NOT(Box<Predicate>),
    TRUE,
    FALSE,
    EQ(PredValue, PredValue), 
    NEQ(PredValue, PredValue),
    // GT(PredValue, PredValue),
    // LT(PredValue, PredValue),
    // INDOMAIN(PredValue, Vec<PredValue>),
}

impl Default for Predicate {
    fn default() -> Self {
        Predicate::TRUE
    }
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Action {
    var: SPName,
    value: Compute,
}

/// Used in actions to compute a new SPValue.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Compute {
    PredValue(PredValue),
    Delay(SPValue, u64)
    //TakeNext(SPValue, Vec<SPValue>), // to be impl when needed
    //TakeBefore(SPValue, Vec<SPValue>),
    // Add(Box<Compute>, Box<Compute>),
    // Sub(Box<Compute>, Box<Compute>),
    // Join(Box<Compute>, Box<Compute>),
}

impl Default for Compute {
    fn default() -> Self {
        Compute::PredValue(PredValue::default())
    }
}

/// Eval is used to evaluate a predicate (or an operation ). 
pub trait Eval {
    fn eval(&self, state: &State) -> bool;
}

pub trait Next {
    fn next(&self, state: &mut State) -> Result<(), StateError>;
}


impl Eval for Predicate {
    fn eval(&self, state: &State) -> bool {
        match self {
            Predicate::AND(ps) => { ps.iter().all(|p| p.eval(state)) }
            Predicate::OR(ps) => { ps.iter().any(|p| p.eval(state)) }
            Predicate::XOR(ps) => {
                ps.iter().filter(|p| p.eval(state)).collect::<Vec<_>>().len() == 1
            }
            Predicate::NOT(p) => { !p.eval(state) }
            Predicate::TRUE => { true }
            Predicate::FALSE => { false }
            Predicate::EQ(lp, rp) => { lp.get_value(state) == rp.get_value(state) } 
            Predicate::NEQ(lp, rp) => { lp.get_value(state) != rp.get_value(state) }
            // Predicate::GT(lp, rp) => {}
            // Predicate::LT(lp, rp) => {}
            // Predicate::INDOMAIN(value, domain) => {}
        }
    }
}

impl Next for Action {
    fn next(&self, state: &mut State) -> Result<(), StateError> {
        let c = match &self.value {
            Compute::PredValue(pv) => {
                match pv {
                    PredValue::SPName(name) => {
                        match state.get_value(&name).map(|x| { AssignStateValue::SPValue(x.clone())}) {
                            Some(x) => x,
                            None => {
                                eprintln!("The action {:?}, next did not find a value for variable: {:?}", self, name);
                                return Err(StateError::Undefined)
                            },
                        }
                    },
                    PredValue::SPValue(x) => {AssignStateValue::SPValue(x.clone())},
                }
            },
            Compute::Delay(x, ms) => {
                AssignStateValue::Delay(x.clone(), *ms)
            }
        };

        state.insert(&self.var, c)     

    }
}

// TODO: Just an experimental impl to learn. Hard to make it general
// TODO: Fix this later

#[macro_export]
macro_rules! p{
    // EQ
    ($name:ident == $value:expr) => {
        Predicate::EQ(PredValue::SPName($name.clone()), PredValue::SPValue($value.to_spvalue()))
    };
    ($name:block == $value:expr) => {{
        let xs: Vec<String> = $name.iter().map(|x|x.to_string()).collect();
        Predicate::EQ(PredValue::SPName(SPName::from(&xs)), PredValue::SPValue($value.to_spvalue()))
    }};
    ($name:block == $value:block) => {{
        let xs: Vec<String> = $name.iter().map(|x|x.to_string()).collect();
        Predicate::EQ(PredValue::SPName(SPName::from(&xs)), PredValue::SPValue($value.to_spvalue()))
    }};
    // NEQ
    ($name:ident != $value:expr) => {
        Predicate::NEQ(PredValue::SPName($name.clone()), PredValue::SPValue($value.to_spvalue()))
    };
    ($name:block != $value:expr) => {{
        let xs: Vec<String> = $name.iter().map(|x|x.to_string()).collect();
        Predicate::NEQ(PredValue::SPName(SPName::from(&xs)), PredValue::SPValue($value.to_spvalue()))
    }};
    ($name:block != $value:block) => {{
        let xs: Vec<String> = $name.iter().map(|x|x.to_string()).collect();
        Predicate::NEQ(PredValue::SPName(SPName::from(&xs)), PredValue::SPValue($value.to_spvalue()))
    }};
    // Boolean variable
    ($name:ident) => {
        Predicate::EQ(PredValue::SPName($name.clone()), PredValue::SPValue(true.to_spvalue()))
    };
    (!$name:ident) => {
        Predicate::EQ(PredValue::SPName($name.clone()), PredValue::SPValue(false.to_spvalue()))
    };
    ($name:block) => {
        let xs: Vec<String> = $name.iter().map(|x|x.to_string()).collect();
        Predicate::EQ(PredValue::SPName(SPName::from(&xs)), PredValue::SPValue(true.to_spvalue()))
    };
    (!$name:block) => {
        let xs: Vec<String> = $name.iter().map(|x|x.to_string()).collect();
        Predicate::EQ(PredValue::SPName(SPName::from(&xs)), PredValue::SPValue(false.to_spvalue()))
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



/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    use super::*;
    #[test]
    fn create_predicate() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let v = SPName::from_str(&["a", "b"]);

        let eq = Predicate::EQ(PredValue::SPValue(2.to_spvalue()), PredValue::SPName(v.clone()));

        let p = p!{v == 2};
        println!("TEST: {:?}", &p);
        let p2 = p!{{["a", "b"]} == 2};
        println!("TEST: {:?}", &p2);

        let x = pr!{p2 && p && p && p};
        println!("TEST2: {:?}", x);

        let y = pr!{{p!{{["a", "b"]} == 10}} && {p!{{["a", "b"]} == 20}}};
        println!("TEST3: {:?}", y);

        let z = pr!{ {p!{{["a", "b"]}}} && {p!{v == 2}}};
        let k = pr!{{pr!{x || y}} && {p!{v != 5}}};

        let long = pr!{z || k};
        println!("TEST4: {:?}", long);



    }

    #[test]
    fn eval_pred() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let v = SPName::from_str(&["a", "b"]);
        let eq = Predicate::EQ(PredValue::SPValue(2.to_spvalue()), PredValue::SPName(v.clone()));
        let eq2 = Predicate::EQ(PredValue::SPValue(3.to_spvalue()), PredValue::SPName(v.clone()));
        assert!(eq.eval(&s));
        assert!(!eq2.eval(&s));

    }
}