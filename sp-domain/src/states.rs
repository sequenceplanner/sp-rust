//! SPState represents a state in SP
//!

use super::*;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap};
use std::fmt;
use uuid::Uuid;

/// Representing a State in SP with variables and their values. The values are 
/// stored in a vec to speed up reading and writing. The position of a value in the
/// vec is stored in the Hashmap. This should be cashed by user of the state for 
/// index access.
#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct SPState {
    index: HashMap<SPPath, usize>,
    values: Vec<StateValue>,
    id: Uuid,
}

// TODO: Maybe also check the id and if we want to have eq variable values, the we can use
// the projection
impl PartialEq for SPState {
    fn eq(&self, other: &Self) -> bool {
        self.projection() == other.projection()
    }
}

#[derive(Debug, Clone)]
pub struct StateProjection<'a> {
    pub projection: Vec<(&'a SPPath, &'a StateValue)>,
    pub id: Uuid,
}

impl PartialEq for StateProjection<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.projection == other.projection
    }
}

impl<'a> StateProjection<'a> {
    fn clone_state(&self) -> SPState {
        SPState::new_from_state_values(&self.clone_vec())
    }
    fn clone_vec(&self) -> Vec<(SPPath, StateValue)> {
        self.projection.iter().map(|(p, v)| {((*p).clone(), (*v).clone())}).collect()
    }
    fn clone_vec_value(&self) -> Vec<(SPPath, SPValue)> {
        self.projection.iter().map(|(p, v)| {((*p).clone(), (*v).get_value().clone())}).collect()
    }
    fn sort(&mut self) {
        self.projection.sort_by(|a, b| a.0.to_string().cmp(&b.0.to_string()));
    }
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

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct StatePath {
    path: SPPath,
    index: usize,
    state_id: Uuid,
}

/// AssignStateValue is used by actions when assigning a new value to the state
/// It will either result in a Next or a delay
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum AssignStateValue {
    SPValue(SPValue),
    Delay(SPValue, u64),
    CompleteDelay,
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

/// The next value that an action has written to the state.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Next {
    pub current_value: SPValue,
    pub next_value: SPValue,
}

impl SPState {
    /// Creates a new empty state.
    pub fn new() -> SPState {
        SPState{
            index: HashMap::new(),
            values: Vec::new(),
            id: Uuid::new_v4(),
        }
    }

    /// Creates a new State that includes the state defined in the hashmap. Prefered method
    pub fn new_from_values(hm: &[(SPPath, SPValue)]) -> SPState {
        let mut xs = HashMap::new();
        let mut v = Vec::with_capacity(hm.len());
        for (key, value) in hm.iter() {
            v.push(StateValue::SPValue(value.clone()));
            xs.insert(key.clone(), v.len()-1);
        }
        SPState{
            index: xs,
            values: v,
            id: Uuid::new_v4(),
        }
    }

    /// Creates a new State that includes the state defined in the hashmap. Prefered method
    pub fn new_from_state_values(hm: &[(SPPath, StateValue)]) -> SPState {
        let mut xs = HashMap::new();
        let mut v = Vec::with_capacity(hm.len());
        for (key, value) in hm.iter() {
            v.push(value.clone());
            xs.insert(key.clone(), v.len()-1);
        }
        SPState{
            index: xs,
            values: v,
            id: Uuid::new_v4(),
        }
    }

    /// Add a new state variable to the state. If the path already is included, the value is updated.
    /// Maybe we should change this and not update the state? This will also change the id of the state
    pub fn add_state_variable(&mut self, path: SPPath, value: SPValue) {
        let new_v = StateValue::SPValue(value);
        if self.index.contains_key(&path) {
            self.values[*self.index.get(&path).unwrap()] = new_v;
        } else {
            self.values.push(new_v);
            self.index.insert(path, self.values.len()-1);
            self.id = Uuid::new_v4();  // the index has changed and it is probably better to reload
        }
    }

    /// Add new variables to the state. If a path is already is included, the value is updated.
    /// Maybe we should change this and not update the state?
    pub fn add_state_variables(&mut self, map: &[(SPPath, SPValue)]) {
        for (path, value) in map.into_iter() {
            if self.index.contains_key(&path) {
                self.values[*self.index.get(&path).unwrap()] = StateValue::SPValue(value.clone());
            } else {
                self.values.push(StateValue::SPValue(value.clone()));
                self.index.insert(path.clone(), self.values.len()-1);
            }
            self.id = Uuid::new_v4();  // the index has changed and it is probably better to reload
        }        
    }

    pub fn id(&self) -> Uuid {
        self.id
    }

    /// Returns the index of a variable to be used for faster access. Cash this and compare with
    /// the id of the state if it is still valid
    pub fn state_path(&self, path: &SPPath) -> Option<StatePath> {
        self.index.get(path).map(|index| StatePath{
            path: path.clone(),
            index: *index,
            state_id: self.id
        })
    }

    pub fn check_state_path(&self, state_path: &StatePath) -> bool {
        state_path.state_id == self.id 
    }

    /// The standard way of getting values from the state. Get the state_path first
    pub fn state_value(&self, state_path: &StatePath) -> Option<&StateValue> {
        if self.check_state_path(state_path) {
            Some(&self.values[state_path.index])
        } else {
            self.state_value_from_path(&state_path.path)
        }
    }

    pub fn sp_value(&self, state_path: &StatePath) -> Option<&SPValue> {
        self.state_value(state_path).map(|x| x.get_value())
    }




    /// Get a StateValue from the state based on a path. Only use this when you need do not
    /// need to get the same variable multiple times. Else use state_value_with_index
    pub fn state_value_from_path(&self, path: &SPPath) -> Option<&StateValue> {
        self.index.get(path).map(|i| &self.values[*i])
    }

    /// Get a StateValue based on its index when you need to access the same variable multiple
    /// times. First get and cash the index from fn state_path(&path), and then use that index. 
    /// If you do not know if the state can change, check the state id.
    /// This fn will panic if i is larger than no of values
    pub fn state_value_from_index(&self, i: usize) -> &StateValue {
        &self.values[i]
    }


    /// Get a SPValue from the state based on a path. Only use this when you need do not
    /// need to get the same variable multiple times. Else use sp_value
    pub fn sp_value_from_path(&self, path: &SPPath) -> Option<&SPValue> {
        self.state_value_from_path(path).map(|x| x.get_value())
    }

    /// Get a SPValue based on its index when you need to access the same variable multiple
    /// times. First get and cash the index from fn state_path(&path), and then use that index. 
    /// If you do not know if the state can change, check the state id.
    /// This fn will panic if i is larger than no of values
    pub fn sp_value_from_index(&self, i: usize) -> &SPValue {
        self.state_value_from_index(i).get_value()
    }

    /// Get a projection of the state
    pub fn projection(&self) -> StateProjection {
        let s: Vec<(&SPPath, &StateValue)> = self
            .index
            .iter()
            .map(|(key, i)| (key, &self.values[*i]))
            .collect();

        let mut p = StateProjection{
            projection: s,
            id: self.id
        };
        p.sort();
        p

    }

    /// Returns a projection of the sub part of the state where the variables are children to the path
    /// TODO: Maybe move this to struct StateProjection
    ///
    /// ["a", "b"] is a child of ["a"]
    ///
    pub fn sub_state_projection(&self, path: &SPPath) -> StateProjection {
        let s: Vec<(&SPPath, &StateValue)> = self
            .index
            .iter()
            .filter(|(key, _)| key.is_child_of(path))
            .map(|(key, i)| (key, &self.values[*i]))
            .collect();

        StateProjection{
            projection: s,
            id: self.id,
        }
    }

    /// Checks if a sub state is the same as another states sub state given the same path
    /// This is used to check this so we do not need to create a clone with sub_state.
    pub fn is_sub_state_the_same(&self, state: &SPState, path: &SPPath) -> bool {
        self.index
            .iter()
            .filter(|(key, _)| key.is_child_of(path))
            .all(|(key, i)| state.sp_value_from_path(key).map(|x| x == self.values[*i].get_value()).unwrap_or(false))
    }


    pub fn prefix_paths(&mut self, parent: &GlobalPath) {
        let mut xs = HashMap::new();
        for (path, i) in self.index.iter() {
            if let SPPath::LocalPath(lp) = path {
                xs.insert(lp.to_global(parent).to_sp(), *i);
            } else {
                xs.insert(path.clone(), *i);
            }
        }
        self.index = xs;
        self.id = Uuid::new_v4(); // Changing id since the paths changes
    }
    pub fn unprefix_paths(&mut self, parent: &GlobalPath) {
        for mut x in self.index.iter_mut() {
            if let SPPath::GlobalPath(gp) = x.0 {
                x.0 = &gp.to_local(parent).to_sp();
            }
        }
        self.id = Uuid::new_v4(); // Changing id since the paths changes
    }

    pub fn next(&mut self, state_path: &StatePath, value: AssignStateValue) -> SPResult<()> {
        if !self.check_state_path(state_path) {
            Err(SPError::No("The state path is wrong".to_string()))
        } else {
            match SPState::create_next_state_value(value, &self.values[state_path.index]) {
                Ok(v) => {
                    self.values[state_path.index] = v;
                    Ok(())
                }
                Err(e) => Err(e),
            }
        }
        
    }
    pub fn next_from_path(&mut self, path: &SPPath, value: AssignStateValue) -> SPResult<()> {
        if let Some(sp) = self.state_path(&path) {
            self.next(&sp, value)
        } else {
            Err(SPError::No(format!{"Can not find the path: {:?}", path}))
        }
    }


    fn create_next_state_value(next: AssignStateValue, prev: &StateValue) -> SPResult<StateValue> {
        match (next, prev.clone()) {
            (AssignStateValue::Force(n), _) => Ok(StateValue::SPValue(n)),
            (AssignStateValue::CancelDelay, StateValue::Delay(p)) => {
                Ok(StateValue::SPValue(p.current_value))
            }
            (AssignStateValue::CompleteDelay, StateValue::Delay(d)) => {
                let n = Next{
                    current_value: d.current_value,
                    next_value: d.next_value
                };
                Ok(StateValue::Next(n))
            }
            (x, StateValue::Next(p)) => {
                std::eprintln!(
                    "Your are trying to overwrite a next: current: {:?}, new: {:?} ",
                    x, p
                );
                Err(SPError::OverwriteNext(p, x))
            }
            (x, StateValue::Delay(p)) => {
                std::eprintln!(
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
            (AssignStateValue::CompleteDelay, StateValue::SPValue(p)) => Ok(StateValue::SPValue(p)),
            (AssignStateValue::SPValue(n), StateValue::Unknown) => Ok(StateValue::SPValue(n)),
            (AssignStateValue::Delay(n, _), StateValue::Unknown) => {
                // Can not delay if current is unknown
                Ok(StateValue::SPValue(n))
            }
            (AssignStateValue::CancelDelay, StateValue::Unknown) => {
                std::eprintln!("Your are trying to cancel an unknown");
                Err(SPError::No(
                    "Your are trying to cancel an unknown".to_string(),
                ))
            }
            (AssignStateValue::CompleteDelay, StateValue::Unknown) => {
                std::eprintln!("Your are trying to cancel an unknown");
                Err(SPError::No(
                    "Your are trying to cancel an unknown".to_string(),
                ))
            }
        }
    }
    pub fn insert_map(&mut self, map: Vec<(StatePath, AssignStateValue)>) -> SPResult<()> {
        for (var, value) in map.into_iter() {
            self.next(&var, value)?
        }
        Ok(())
    }

    pub fn take_transition(&mut self) {
        self.values.iter_mut().for_each(|v| {
            if let StateValue::Next(n) = v {
                *v = StateValue::SPValue(n.next_value.clone());
            } else if let StateValue::Delay(d) = v {
                d.has_been_spawned = true;
            }
        });
    }

    pub fn is_allowed(&self, state_path: &StatePath, value: AssignStateValue) -> bool {
        self.check_state_path(state_path) && SPState::create_next_state_value(value, &self.values[state_path.index]).is_ok()
    }

    pub fn extend(&mut self, other_state: SPState) {
        let p = other_state.projection().clone_vec_value();
        self.add_state_variables(&p);
    }
}


impl fmt::Display for SPState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Sort keys by name.
        let proj = self.projection();
        let mut buf = Vec::new();
        for (p, val) in proj.projection {
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


/// helping making states with a macro
#[macro_export]
macro_rules! state {

    ($( $key: ident => $val: expr ),*) => {{
        let mut s: Vec<(SPPath, StateValue)> = Vec::new();
        $(
            s.push(($key.clone(), StateValue::SPValue($val.to_spvalue())));
        )*
        SPState::new_from_state_values(&s)
    }};
    ($( $key: expr => $val: expr ),*) => {{
        let mut s: Vec<(SPPath, StateValue)> = Vec::new();
        $(
            let xs: Vec<String> = $key.iter().map(|x|x.to_string()).collect();
            s.push((SPPath::from_as_global(&xs), StateValue::SPValue($val.to_spvalue())));
        )*
        SPState::new_from_state_values(&s)
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
        let ab = SPPath::from_array_as_global(&["a", "b"]);
        let ac = SPPath::from_array_as_global(&["a", "c"]);
        let kl = SPPath::from_array_as_global(&["k", "l"]);
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let s2 = state!(ab => 2, ac => true, kl => true);
        println!("s proj {:?}", s.projection());
        println!("s2 proj {:?}", s2.projection());

        assert_eq!(s, s2);

        let var_a = SPPath::GlobalPath(GlobalPath::from(vec!["a".to_string(), "b".to_string()]));
        let var_b = SPPath::GlobalPath(GlobalPath::from(vec!["a".to_string(), "c".to_string()]));
        let var_c = SPPath::GlobalPath(GlobalPath::from(vec!["k".to_string(), "l".to_string()]));
        let mut m = Vec::new();
        m.push((var_a, 2.to_state()));
        m.push((var_b, true.to_state()));
        m.push((var_c, true.to_state()));

        assert_eq!(s, SPState::new_from_state_values(&m));
    }

    #[test]
    fn get_value() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

        let v = SPPath::from_array_as_global(&["a", "b"]);
        let res = s.sp_value_from_path(&v);

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
    fn next() {
        let mut s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let v = &SPPath::from_array_as_global(&["a", "b"]);

        let state_path = s.state_path(v).unwrap();

        let x = s.next(&state_path, AssignStateValue::SPValue(5.to_spvalue()));
        assert_eq!(Ok(()), x);
        assert_eq!(s.sp_value_from_path(v), Some(&5.to_spvalue()));

        let e = s.next(&state_path, AssignStateValue::Delay(6.to_spvalue(), 1000));
        assert!(e.is_err());
        assert_eq!(s.sp_value_from_path(v), Some(&5.to_spvalue()));

        let x = s.next(&state_path, AssignStateValue::Force(10.to_spvalue()));
        assert_eq!(Ok(()), x);
        assert_eq!(s.sp_value_from_path(v), Some(&10.to_spvalue()));

        let x = s.next(&state_path, AssignStateValue::Delay(0.to_spvalue(), 1000));
        assert_eq!(Ok(()), x);
        assert_eq!(s.sp_value_from_path(v), Some(&10.to_spvalue()));
        assert_eq!(
            s.state_value_from_path(v),
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
        let ab = &SPPath::from_array_as_global(&["a", "b"]);
        let ac = &SPPath::from_array_as_global(&["a", "c"]);
        let p_ab = s.state_path(ab).unwrap();
        let p_ac = s.state_path(ac).unwrap();

        s.next(&p_ab, AssignStateValue::SPValue(5.to_spvalue()))
            .expect("Oh no");
        s.next(&p_ac, AssignStateValue::Delay(false.to_spvalue(), 1000))
            .expect("oh no");
        println!("The state before: {:?}", s);
        s.take_transition();

        assert_eq!(
            s.state_value(&p_ac),
            Some(&StateValue::Delay(Delay {
                current_value: true.to_spvalue(),
                next_value: false.to_spvalue(),
                millis: 1000,
                has_been_spawned: true
            }))
        );
        assert_eq!(s.state_value(&p_ab), Some(&StateValue::SPValue(5.to_spvalue())));

        println!("The state: {:?}", s);
    }

    // #[test]
    // fn sub_state_testing() {
    //     let a = SPPath::from_array_as_global(&["a"]);
    //     let ab = SPPath::from_array(&["a", "b"]);
    //     let ax = SPPath::from_array(&["a", "x"]);
    //     let abc = SPPath::from_array(&["a", "b", "c"]);
    //     let abx = SPPath::from_array(&["a", "b", "x"]);
    //     let b = SPPath::from_array(&["b"]);
    //     let s = state!(abc => false, abx => false, ax => true);

    //     assert_eq!(s.sub_state(&ab), state!(abc => false, abx => false));
    //     assert_eq!(s.sub_state(&abc), state!(abc => false));
    //     assert_eq!(s.sub_state(&b), state!());
    //     assert_eq!(s.sub_state(&a), s.clone());

    //     let mut s2 = s.clone();
    //     assert!(s.is_sub_state_the_same(&s2, &ab));
    //     s2.insert(&ax, AssignStateValue::SPValue(false.to_spvalue()))
    //         .unwrap();
    //     assert!(s.is_sub_state_the_same(&s2, &ab));
    //     s2.insert(&abc, AssignStateValue::SPValue(true.to_spvalue()))
    //         .unwrap();
    //     assert!(!s.is_sub_state_the_same(&s2, &ab));
    // }
}
