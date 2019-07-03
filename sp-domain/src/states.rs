//! State represents a state in SP
//! 

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::*;

/// Representing a State in SP with variables and their values.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct State{
    pub s: HashMap<SPPath, StateValue>
}

/// StateValue wrapps the value of a variable in a state. SPValue are the normal type
/// and Delay and next are mainly used in the runner.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum StateValue {
    SPValue(SPValue),
    Delay(Delay),
    Next(Next),
    Unknown
}

/// AssignStateValue is used when assigning a new value to the state
/// It will either result in a Next or a delay
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum AssignStateValue {
    SPValue(SPValue),
    Delay(SPValue, u64),
    Force(SPValue) // used to overwrite Next and Delay StateValues
}

pub trait ToStateValue {
    fn to_state(&self) -> StateValue;
}

/// Delaying a next value change in actions. This will be included in the state
/// and after the delay, the Delay will be replaced by the new_value in the state.
/// 
/// Use the Delay action in the action to tell the runner to delay the change. The
/// Runner will create a future that can be canceled
/// 
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Delay{
    pub current_value: SPValue,
    pub next_value: SPValue,
    pub millis: u64,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Next{
    pub current_value: SPValue,
    pub next_value: SPValue
}

use std::error;
use std::fmt;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum StateError{
    OverwriteDelay(Delay, AssignStateValue),
    OverwriteNext(Next, AssignStateValue),
    Undefined,
}



impl State {
    pub fn filter(&self, partial_name: &[String]) -> State {
        let s = self.s.iter().filter(|(k, _)| {
            partial_name.iter().all(|x|{ 
                k.name.contains(x) 
            })
        })
        .map(|(k, v)|{ (k.clone(), v.clone())})
        .collect();
        State{s}
    }

    pub fn get_value(&self, var: &SPPath) -> Option<&SPValue> {
        self.s.get(var).and_then(|x| {
            match x {
                StateValue::SPValue(v) => Some(v),
                StateValue::Delay(d) => Some(&d.current_value),
                StateValue::Next(n) => Some(&n.next_value),
                StateValue::Unknown => None
            }
        })
    }
    pub fn get(&self, var: &SPPath) -> Option<&StateValue> {
        self.s.get(var)
    }

    fn make_insert(next: AssignStateValue, prev: StateValue) -> Result<StateValue, StateError> {
        match (next, prev) {
            (AssignStateValue::Force(n), _) => {
                return Ok(StateValue::SPValue(n))
            }
            (x, StateValue::Next(p)) => {
                eprintln!("Your are trying to overwrite a next: current: {:?}, new: {:?} ", x, p);
                return Err(StateError::OverwriteNext(p, x));
            }
            (x, StateValue::Delay(p)) => {
                eprintln!("Your are trying to overwrite a delay: current: {:?}, new: {:?} ", x, p);
                return Err(StateError::OverwriteDelay(p, x));
            }
            (AssignStateValue::SPValue(n), StateValue::SPValue(p)) => {
                return Ok(StateValue::Next(Next{current_value: p, next_value: n}))
            }
            (AssignStateValue::Delay(n, ms), StateValue::SPValue(p)) => {
                return Ok(StateValue::Delay(Delay{current_value: p, next_value: n, millis: ms}))
            }
            (AssignStateValue::SPValue(n), StateValue::Unknown) => {
                return Ok(StateValue::SPValue(n))
            }
            (AssignStateValue::Delay(n, _), StateValue::Unknown) => {  /// Can not delay if current is unknown
                return Ok(StateValue::SPValue(n))
            }
        }
    }

    pub fn insert(&mut self, key: &SPPath, value: AssignStateValue) -> Result<(), StateError> {
        let x = self.s.entry(key.clone()).or_insert(StateValue::Unknown);
        match State::make_insert(value, x.clone()) {
            Ok(v) => {
                *x = v;
                return Ok(());
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    pub fn take_all_next(&mut self) -> () {
        self.s.iter_mut().for_each(|(k, v)| {
            if let StateValue::Next(n) = v {
                *v = StateValue::SPValue(n.next_value.clone());
            } 
        });
    }
}


impl fmt::Display for StateError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            StateError::OverwriteDelay(prev, next) => {
                write!(f, "You are trying to overwrite a Delay in the State. current: {:?}, new: {:?} ", prev, next)
            }
            StateError::OverwriteNext(prev, next) => {
                write!(f, "You are trying to overwrite a Next in the State. current: {:?}, new: {:?} ", prev, next)
            }
            StateError::Undefined  => {
                write!(f, "An undefined State error!")
            }
        }
    }
}

impl error::Error for StateError {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        None
    }
}

/// helping making states with a macro
#[macro_export]
macro_rules! state {
    ($( $key: ident => $val: expr ),*) => {{
        let mut s = std::collections::HashMap::<SPPath, StateValue>::new();
        $(
            s.insert($key.clone(), StateValue::SPValue($val.to_spvalue())); 
        )*
        State{s}
    }};
    ($( $key: expr => $val: expr ),*) => {{
        let mut s = std::collections::HashMap::<SPPath, StateValue>::new();
        $(
            let xs: Vec<String> = $key.iter().map(|x|x.to_string()).collect();
            s.insert(SPPath::from(&xs), StateValue::SPValue($val.to_spvalue())); 
        )*
        State{s}
    }}
}



impl ToStateValue for bool {
    fn to_state(&self) -> StateValue { StateValue::SPValue(self.to_spvalue())}
}
impl ToStateValue for i32 {
    fn to_state(&self) -> StateValue { StateValue::SPValue(self.to_spvalue())}
}
impl ToStateValue for String {
    fn to_state(&self) -> StateValue { StateValue::SPValue(self.to_spvalue())}
}
impl ToStateValue for &str {
    fn to_state(&self) -> StateValue { StateValue::SPValue(self.to_spvalue())}
}
impl<T> ToStateValue for Vec<T> where T: ToSPValue {
    fn to_state(&self) -> StateValue { StateValue::SPValue(self.to_spvalue())}
}



/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    use super::*;
    #[test]
    fn create_state() {
        let ab = SPPath::from_str(&["a", "b"]);
        let ac = SPPath::from_str(&["a", "c"]);
        let kl = SPPath::from_str(&["k", "l"]);
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let s2 = state!(ab => 2, ac => true, kl => true);
        println!("{:?}", s);

        assert_eq!(s, s2);

        let var_a = SPPath{ name: vec!("a".to_string(), "b".to_string())};
        let var_b = SPPath{ name: vec!("a".to_string(), "c".to_string())};
        let var_c = SPPath{ name: vec!("k".to_string(), "l".to_string())};
        let mut m = HashMap::new();
        m.insert(var_a, 2.to_state());
        m.insert(var_b, true.to_state());
        m.insert(var_c, true.to_state());


        assert_eq!(s, State{s:m});

    }


    #[test]
    fn get_value() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

        let v = SPPath::from_str(&["a", "b"]);
        let res = s.get_value(&v);

        assert_eq!(res, Some(&2.to_spvalue()));
    }

    #[test]
    fn get_substate() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

        let a = &vec!("a".to_string());
        let sub = s.filter(a);

        assert_eq!(sub, state!(["a", "b"] => 2, ["a", "c"] => true));
    }

    #[test]
    fn insert() {
        let mut s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let v = &SPPath::from_str(&["a", "b"]);
        
        let x = s.insert(v, AssignStateValue::SPValue(5.to_spvalue()));
        assert_eq!(Ok(()), x);
        assert_eq!(s.get_value(v), Some(&5.to_spvalue()));

        let e = s.insert(v, AssignStateValue::Delay(6.to_spvalue(), 1000));
        assert!(e.is_err());
        assert_eq!(s.get_value(v), Some(&5.to_spvalue()));

        let x = s.insert(v, AssignStateValue::Force(10.to_spvalue()));
        assert_eq!(Ok(()), x);
        assert_eq!(s.get_value(v), Some(&10.to_spvalue()));

        let x = s.insert(v, AssignStateValue::Delay(0.to_spvalue(), 1000));
        assert_eq!(Ok(()), x);
        assert_eq!(s.get_value(v), Some(&10.to_spvalue()));
        assert_eq!(s.get(v), Some(&StateValue::Delay(Delay{current_value: 10.to_spvalue(), next_value: 0.to_spvalue(), millis: 1000})));

        println!("{:?}", e);
        println!("{:?}", s);


    }
    #[test]
    fn take_all_next() {
        let mut s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let ab = &SPPath::from_str(&["a", "b"]);
        let ac = &SPPath::from_str(&["a", "c"]);
        
        s.insert(ab, AssignStateValue::SPValue(5.to_spvalue())).expect("Oh no");
        s.insert(ac, AssignStateValue::Delay(false.to_spvalue(), 1000)).expect("oh no");
        s.take_all_next();

        assert_eq!(s.get(ac), Some(&StateValue::Delay(Delay{current_value: true.to_spvalue(), next_value: false.to_spvalue(), millis: 1000})));
        assert_eq!(s.get(ab), Some(&StateValue::SPValue(5.to_spvalue())));

        println!("The state: {:?}", s);


    }
}