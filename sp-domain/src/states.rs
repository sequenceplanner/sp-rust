//! SPState represents a state in SP
//!

use super::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;

/// Representing a State in SP with variables and their values. This is used by the runner
/// and predicates and actions. This should not be sent out, instead use State
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct SPState {
    pub s: HashMap<SPPath, StateValue>,
}

/// Representing a State in SP that is shared to others and is used for sending state
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct StateExternal {
    pub s: HashMap<SPPath, SPValue>,
}

/// Representing variables that should be assigned to a SPState. Just a wrapper to simplify life
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct AssignState {
    pub s: HashMap<SPPath, AssignStateValue>,
}

/// StateValue wrapps the value of a variable in a state. SPValue are the normal type
/// and Delay and next are mainly used in the runner.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum StateValue {
    SPValue(SPValue),
    Delay(Delay),
    Next(Next),
    Unknown,
}

/// AssignStateValue is used when assigning a new value to the state
/// It will either result in a Next or a delay
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum AssignStateValue {
    SPValue(SPValue),
    Delay(SPValue, u64),
    CancelDelay,    // to cancel a Delay
    Force(SPValue), // used to overwrite Next and Delay StateValues
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
pub struct Delay {
    pub current_value: SPValue,
    pub next_value: SPValue,
    pub millis: u64,
    pub has_been_spawned: bool,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Next {
    pub current_value: SPValue,
    pub next_value: SPValue,
}

impl SPState {
    // pub fn filter(&self, partial_name: &[String]) -> SPState {
    //     let s = self.s.iter().filter(|(k, _)| {
    //         partial_name.iter().all(|x|{
    //             k.path.contains(x)
    //         })
    //     })
    //     .map(|(k, v)|{ (k.clone(), v.clone())})
    //     .collect();
    //     SPState{s}
    // }

    pub fn get_value(&self, var: &SPPath) -> Option<&SPValue> {
        self.s.get(var).map(|x| x.get_value())
    }
    pub fn get(&self, var: &SPPath) -> Option<&StateValue> {
        self.s.get(var)
    }

    /// Extract a clone of the sub part of the state where the variables are children to the path
    ///
    /// ["a", "b"] is a child of ["a"]
    ///
    pub fn sub_state(&self, path: &SPPath) -> SPState {
        let s = self
            .s
            .iter()
            .filter(|(key, _)| key.is_child_of(path))
            .map(|(key, value)| (key.clone(), value.clone()))
            .collect();

        SPState { s }
    }

    /// Checks if a sub state is the same as another states sub state given the same path
    /// This is used to check this so we do not need to create a clone with sub_state.
    pub fn is_sub_state_the_same(&self, state: &SPState, path: &SPPath) -> bool {
        self.s
            .iter()
            .filter(|(key, _)| key.is_child_of(path))
            .all(|(key, value)| state.get(key).map(|x| x == value).unwrap_or(false))
    }

    pub fn external(&self) -> StateExternal {
        let res = self
            .s
            .iter()
            .map(|(key, value)| (key.clone(), value.get_value().clone()))
            .collect();

        StateExternal { s: res }
    }

    fn make_insert(next: AssignStateValue, prev: StateValue) -> SPResult<StateValue> {
        match (next, prev) {
            (AssignStateValue::Force(n), _) => Ok(StateValue::SPValue(n)),
            (AssignStateValue::CancelDelay, StateValue::Delay(p)) => {
                Ok(StateValue::SPValue(p.current_value))
            }
            (x, StateValue::Next(p)) => {
                eprintln!(
                    "Your are trying to overwrite a next: current: {:?}, new: {:?} ",
                    x, p
                );
                Err(SPError::OverwriteNext(p, x))
            }
            (x, StateValue::Delay(p)) => {
                eprintln!(
                    "Your are trying to overwrite a delay: current: {:?}, new: {:?} ",
                    x, p
                );
                Err(SPError::OverwriteDelay(p, x))
            }
            (AssignStateValue::SPValue(n), StateValue::SPValue(p)) => Ok(StateValue::Next(Next {
                current_value: p,
                next_value: n,
            })),
            (AssignStateValue::Delay(n, ms), StateValue::SPValue(p)) => {
                Ok(StateValue::Delay(Delay {
                    current_value: p,
                    next_value: n,
                    millis: ms,
                    has_been_spawned: false,
                }))
            }
            (AssignStateValue::CancelDelay, StateValue::SPValue(p)) => Ok(StateValue::SPValue(p)),
            (AssignStateValue::SPValue(n), StateValue::Unknown) => Ok(StateValue::SPValue(n)),
            (AssignStateValue::Delay(n, _), StateValue::Unknown) => {
                // Can not delay if current is unknown
                Ok(StateValue::SPValue(n))
            }
            (AssignStateValue::CancelDelay, StateValue::Unknown) => {
                eprintln!("Your are trying to cancel an unknown");
                Err(SPError::No(
                    "Your are trying to cancel an unknown".to_string(),
                ))
            }
        }
    }

    pub fn insert(&mut self, key: &SPPath, value: AssignStateValue) -> SPResult<()> {
        let x = self.s.entry(key.clone()).or_insert(StateValue::Unknown);
        match SPState::make_insert(value, x.clone()) {
            Ok(v) => {
                *x = v;
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    pub fn insert_map(&mut self, map: AssignState) -> SPResult<()> {
        for (var, value) in map.s.into_iter() {
            self.insert(&var, value)?
        }
        Ok(())
    }

    pub fn take_all_next(&mut self) -> () {
        self.s.iter_mut().for_each(|(_, v)| {
            if let StateValue::Next(n) = v {
                *v = StateValue::SPValue(n.next_value.clone());
            } else if let StateValue::Delay(d) = v {
                d.has_been_spawned = true;
            }
        });
    }

    pub fn is_allowed(&self, key: &SPPath, value: AssignStateValue) -> bool {
        self.s
            .get(key)
            .map(|x| SPState::make_insert(value, x.clone()).is_ok())
            .unwrap_or(false)
    }

    pub fn extend(&mut self, other_state: SPState) {
        self.s.extend(other_state.s)
    }
}

impl StateExternal {
    pub fn new() -> StateExternal {
        StateExternal { s: HashMap::new() }
    }

    pub fn to_spstate(&self) -> SPState {
        let res = self
            .s
            .iter()
            .map(|(key, value)| (key.clone(), StateValue::SPValue(value.clone())))
            .collect();

        SPState { s: res }
    }

    pub fn to_assignstate(&self) -> AssignState {
        let res = self
            .s
            .iter()
            .map(|(key, value)| (key.clone(), AssignStateValue::SPValue(value.clone())))
            .collect();

        AssignState { s: res }
    }
}

impl fmt::Display for StateExternal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut buf = Vec::new();
        for (p, val) in &self.s {
            buf.push(format!("{}: {:?}", p, val));
        }
        write!(f, "{}", buf.join("\n"))
    }
}

impl StateValue {
    fn get_value(&self) -> &SPValue {
        match self {
            StateValue::SPValue(v) => v,
            StateValue::Delay(d) => &d.current_value,
            StateValue::Next(n) => &n.next_value,
            StateValue::Unknown => &SPValue::Unknown,
        }
    }
}

impl AssignState {
    /// Consumes and merges the AssignState into this. Return true if a key was overwritten
    pub fn merge(&mut self, x: AssignState) {
        self.s.extend(x.s);
    }

    /// Checks if the two AssignStates overlap. Not very good if they do. Check before merge
    pub fn will_overwrite(&self, x: &AssignState) -> bool {
        self.s.keys().all(|k| x.s.contains_key(k))
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
        SPState{s}
    }};
    ($( $key: expr => $val: expr ),*) => {{
        let mut s = std::collections::HashMap::<SPPath, StateValue>::new();
        $(
            let xs: Vec<String> = $key.iter().map(|x|x.to_string()).collect();
            s.insert(SPPath::from(&xs), StateValue::SPValue($val.to_spvalue()));
        )*
        SPState{s}
    }}
}

impl ToStateValue for bool {
    fn to_state(&self) -> StateValue {
        StateValue::SPValue(self.to_spvalue())
    }
}
impl ToStateValue for i32 {
    fn to_state(&self) -> StateValue {
        StateValue::SPValue(self.to_spvalue())
    }
}
impl ToStateValue for String {
    fn to_state(&self) -> StateValue {
        StateValue::SPValue(self.to_spvalue())
    }
}
impl ToStateValue for &str {
    fn to_state(&self) -> StateValue {
        StateValue::SPValue(self.to_spvalue())
    }
}
impl<T> ToStateValue for Vec<T>
where
    T: ToSPValue,
{
    fn to_state(&self) -> StateValue {
        StateValue::SPValue(self.to_spvalue())
    }
}

/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    use super::*;
    #[test]
    fn create_state() {
        let ab = SPPath::from_array_to_global(&["a", "b"]);
        let ac = SPPath::from_array_to_global(&["a", "c"]);
        let kl = SPPath::from_array_to_global(&["k", "l"]);
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let s2 = state!(ab => 2, ac => true, kl => true);
        println!("{:?}", s);

        assert_eq!(s, s2);

        let var_a = SPPath::GlobalPath(GlobalPath::from(vec!["a".to_string(), "b".to_string()]));
        let var_b = SPPath::GlobalPath(GlobalPath::from(vec!["a".to_string(), "c".to_string()]));
        let var_c = SPPath::GlobalPath(GlobalPath::from(vec!["k".to_string(), "l".to_string()]));
        let mut m = HashMap::new();
        m.insert(var_a, 2.to_state());
        m.insert(var_b, true.to_state());
        m.insert(var_c, true.to_state());

        assert_eq!(s, SPState { s: m });
    }

    #[test]
    fn get_value() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

        let v = SPPath::from_array(&["a", "b"]);
        let res = s.get_value(&v);

        assert_eq!(res, Some(&2.to_spvalue()));
    }

    // #[test]
    // fn get_substate() {
    //     let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

    //     let a = &vec!("a".to_string());
    //     let sub = s.filter(a);

    //     assert_eq!(sub, state!(["a", "b"] => 2, ["a", "c"] => true));
    // }

    #[test]
    fn insert() {
        let mut s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let v = &SPPath::from_array(&["a", "b"]);

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
        assert_eq!(
            s.get(v),
            Some(&StateValue::Delay(Delay {
                current_value: 10.to_spvalue(),
                next_value: 0.to_spvalue(),
                millis: 1000,
                has_been_spawned: false
            }))
        );

        println!("{:?}", e);
        println!("{:?}", s);
    }
    #[test]
    fn take_all_next() {
        let mut s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let ab = &SPPath::from_array(&["a", "b"]);
        let ac = &SPPath::from_array(&["a", "c"]);

        s.insert(ab, AssignStateValue::SPValue(5.to_spvalue()))
            .expect("Oh no");
        s.insert(ac, AssignStateValue::Delay(false.to_spvalue(), 1000))
            .expect("oh no");
        println!("The state before: {:?}", s);
        s.take_all_next();

        assert_eq!(
            s.get(ac),
            Some(&StateValue::Delay(Delay {
                current_value: true.to_spvalue(),
                next_value: false.to_spvalue(),
                millis: 1000,
                has_been_spawned: true
            }))
        );
        assert_eq!(s.get(ab), Some(&StateValue::SPValue(5.to_spvalue())));

        println!("The state: {:?}", s);
    }

    #[test]
    fn sub_state_testing() {
        let a = SPPath::from_array(&["a"]);
        let ab = SPPath::from_array(&["a", "b"]);
        let ax = SPPath::from_array(&["a", "x"]);
        let abc = SPPath::from_array(&["a", "b", "c"]);
        let abx = SPPath::from_array(&["a", "b", "x"]);
        let b = SPPath::from_array(&["b"]);
        let s = state!(abc => false, abx => false, ax => true);

        assert_eq!(s.sub_state(&ab), state!(abc => false, abx => false));
        assert_eq!(s.sub_state(&abc), state!(abc => false));
        assert_eq!(s.sub_state(&b), state!());
        assert_eq!(s.sub_state(&a), s.clone());

        let mut s2 = s.clone();
        assert!(s.is_sub_state_the_same(&s2, &ab));
        s2.insert(&ax, AssignStateValue::SPValue(false.to_spvalue()))
            .unwrap();
        assert!(s.is_sub_state_the_same(&s2, &ab));
        s2.insert(&abc, AssignStateValue::SPValue(true.to_spvalue()))
            .unwrap();
        assert!(!s.is_sub_state_the_same(&s2, &ab));
    }
}
