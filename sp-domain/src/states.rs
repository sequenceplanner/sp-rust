//! SPState represents a state in SP
//!

use super::*;
use serde::de::{MapAccess, Visitor};
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use std::collections::HashMap;
use std::fmt;
use uuid::Uuid;

/// Representing a State in SP with variables and their values. The values are
/// stored in a vec to speed up reading and writing. The position of a value in the
/// vec is stored in the Hashmap. This should be cashed by user of the state for
/// index access.
#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct SPState {
    #[serde(serialize_with = "serialize_state_map")]
    #[serde(deserialize_with = "deserialize_state_map")]
    index: HashMap<SPPath, usize>,
    values: Vec<StateValue>,
    id: Uuid,
}

fn serialize_state_map<S>(x: &HashMap<SPPath, usize>, s: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let mut map = s.serialize_map(Some(x.len()))?;
    for (k, v) in x {
        map.serialize_entry(&k.to_string(), v)?;
    }
    map.end()
}

fn deserialize_state_map<'de, D>(deserializer: D) -> Result<HashMap<SPPath, usize>, D::Error>
where
    D: Deserializer<'de>,
{
    struct StateMapVisitor {}

    impl<'de> Visitor<'de> for StateMapVisitor {
        type Value = HashMap<SPPath, usize>;

        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_str("statemap")
        }

        fn visit_map<M>(self, mut access: M) -> Result<Self::Value, M::Error>
        where
            M: MapAccess<'de>,
        {
            let mut map = HashMap::with_capacity(access.size_hint().unwrap_or(0));

            // While there are entries remaining in the input, add them
            // into our map.
            while let Some((key, value)) = access.next_entry::<String, usize>()? {
                let sppath = SPPath::from_string(&key);
                map.insert(sppath, value);
            }

            Ok(map)
        }
    }
    deserializer.deserialize_map(StateMapVisitor {})
}

// TODO: Maybe check the id?
impl PartialEq for SPState {
    fn eq(&self, other: &Self) -> bool {
        let mut x = self.projection();
        x.sort();
        let mut y = other.projection();
        y.sort();
        x == y
    }
}

#[derive(Debug, Clone)]
pub struct StateProjection<'a> {
    pub state: Vec<(&'a SPPath, &'a StateValue)>,
    pub id: Uuid,
}

impl PartialEq for StateProjection<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.state == other.state
    }
}

impl<'a> StateProjection<'a> {
    pub fn clone_state(&self) -> SPState {
        SPState::new_from_state_values(&self.clone_vec())
    }
    pub fn clone_vec(&self) -> Vec<(SPPath, StateValue)> {
        self.state
            .iter()
            .map(|(p, v)| ((*p).clone(), (*v).clone()))
            .collect()
    }
    pub fn clone_vec_value(&self) -> Vec<(SPPath, SPValue)> {
        self.state
            .iter()
            .map(|(p, v)| ((*p).clone(), (*v).value().clone()))
            .collect()
    }
    pub fn sort(&mut self) {
        self.state
            .sort_by(|a, b| a.0.to_string().cmp(&b.0.to_string()));
    }
    pub fn sorted(mut self) -> Self {
        self.sort();
        self
    }
    pub fn value(&self, path: &SPPath) -> Option<&StateValue> {
        for (p, v) in self.state.iter() {
            if *p == path {
                return Some(*v);
            }
        }
        None
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Default, Clone)]
pub struct SPStateJson(serde_json::Map<String, serde_json::Value>);

impl SPStateJson {
    pub fn new(state: HashMap<String, serde_json::Value>) -> Self {
        let map: serde_json::Map<String, serde_json::Value> =
            state.into_iter().map(|(k, v)| (k, v)).collect();
        SPStateJson(map)
    }

    pub fn to_state(&self) -> SPState {
        fn dig(
            obj: &serde_json::Map<String, serde_json::Value>, path: &SPPath,
        ) -> Vec<(SPPath, SPValue)> {
            obj.iter()
                .flat_map(|(k, v)| {
                    let mut p = if k != &"0".to_string() {
                        SPPath::from_string(&k)
                    } else {
                        SPPath::new()
                    };
                    p.add_parent_path_mut(path);
                    let spv = SPValue::from_json(v);
                    match spv {
                        SPValue::Unknown => {
                            if let serde_json::Value::Object(map) = v {
                                dig(map, &p)
                            } else {
                                vec![(p, SPValue::Unknown)]
                            }
                        }
                        x => vec![(p, x)],
                    }
                })
                .collect()
        }
        let res = dig(&self.0, &SPPath::new());
        SPState::new_from_values(&res)
    }

    pub fn from_state_flat(state: &SPState) -> SPStateJson {
        let state = state
            .projection()
            .state
            .iter()
            .map(|(k, v)| (k.to_string(), v.value().to_json()))
            .collect();
        SPStateJson(state)
    }

    pub fn from_state_recursive(state: &SPState) -> SPStateJson {
        fn insert(xs: &mut serde_json::Map<String, serde_json::Value>, p: &SPPath, v: &SPValue) {
            if p.is_empty() {
                return;
            }
            let root = p.root();
            let x = xs.get(&root).cloned();
            match x {
                None => {
                    if p.path.len() == 1 {
                        xs.insert(root, v.to_json());
                    } else {
                        let mut map = serde_json::Map::new();
                        insert(&mut map, &p.drop_root(), v);
                        xs.insert(root, serde_json::Value::Object(map));
                    }
                }
                Some(serde_json::Value::Object(_)) => {
                    let elm_path = if p.path.len() == 1 {
                        SPPath::from_string("0")
                    } else {
                        p.drop_root()
                    };
                    if let serde_json::Value::Object(map) = xs
                        .entry(&root)
                        .or_insert(serde_json::Value::Object(serde_json::Map::new()))
                    {
                        insert(map, &elm_path, v);
                    }
                }
                Some(x) => {
                    if p.path.len() == 1 {
                        panic!("THIS SHOULD NEVER HAPPEN IN SPJSONSTATE {}, {}", p, &x);
                    }

                    let mut map = serde_json::Map::new();
                    map.insert("0".to_string(), x.clone());
                    insert(&mut map, &p.drop_root(), v);
                    let mut ch = xs
                        .entry(&root)
                        .or_insert(serde_json::Value::Object(serde_json::Map::new()));
                    *ch = serde_json::Value::Object(map);
                }
            }
        }
        let mut map = serde_json::Map::new();
        let mut proj = state.projection();
        proj.sort();
        proj.state.into_iter().for_each(|(k, v)| {
            insert(&mut map, k, v.value());
        });
        SPStateJson(map)
    }

    pub fn to_json(&self) -> serde_json::Value {
        serde_json::to_value(self).unwrap()
    }

    pub fn from_json(json: serde_json::Value) -> Result<SPStateJson, serde_json::Error> {
        serde_json::from_value(json)
    }
}

/// StateValue includes the current and an optional next and prev value.
#[derive(Debug, PartialEq, Serialize, Deserialize, Default, Clone)]
pub struct StateValue {
    current: SPValue,
    next: Option<SPValue>,
    prev: Option<SPValue>,
}

impl StateValue {
    pub fn new(value: SPValue) -> StateValue {
        StateValue {
            current: value,
            next: None,
            prev: None,
        }
    }
    pub fn next(&mut self, value: SPValue) -> bool {
        if self.next.is_none() {
            self.next = Some(value);
            true
        } else {
            false
        }
    }
    pub fn take(&mut self) -> bool {
        self.prev = None;
        if self.next.is_none() {
            false
        } else {
            let n = self.next.take().unwrap();
            let p = std::mem::replace(&mut self.current, n);
            self.prev = Some(p);
            true
        }
    }
    pub fn force(&mut self, value: SPValue) {
        let p = std::mem::replace(&mut self.current, value);
        self.prev = Some(p);
    }
    pub fn revert_next(&mut self) {
        self.next = None;
    }
    pub fn revert_to_prev(&mut self) {
        if self.prev.is_some() {
            self.current = self.prev.take().unwrap();
            self.next = None; // Maybe we should allow a next, but i think it is better like this
        }
    }
    pub fn has_next(&self) -> bool {
        self.next.is_some()
    }
    pub fn value(&self) -> &SPValue {
        match self.next {
            Some(ref x) => x,
            None => &self.current,
        }
    }
    pub fn current_value(&self) -> &SPValue {
        &self.current
    }
    pub fn next_value(&self) -> &Option<SPValue> {
        &self.next
    }

    pub fn previous_value(&self) -> &Option<SPValue> {
        &self.prev
    }
    pub fn extract(self) -> SPValue {
        match self.next {
            Some(x) => x,
            None => self.current,
        }
    }
}

/// The StatePath is used to speed up the access of variables in the state by instead using the index of the variable
/// in the state instead of the path. You may have to check that the state has the same id as the state path if
/// you do not have control over the state. However, the methods that take a path will also check it.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct StatePath {
    pub path: SPPath,
    pub index: usize,
    pub state_id: Uuid,
}

impl SPState {
    /// Creates a new empty state.
    pub fn new() -> SPState {
        SPState {
            index: HashMap::new(),
            values: Vec::new(),
            id: Uuid::new_v4(),
        }
    }

    /// Creates a new State that includes the state defined in the array tuple. Preferred method
    pub fn new_from_values(hm: &[(SPPath, SPValue)]) -> SPState {
        let mut xs = HashMap::new();
        let mut v = Vec::with_capacity(hm.len());
        for (key, value) in hm.iter() {
            v.push(StateValue::new(value.clone()));
            xs.insert(key.clone(), v.len() - 1);
        }
        SPState {
            index: xs,
            values: v,
            id: Uuid::new_v4(),
        }
    }

    /// Creates a new State that includes the state defined in the hashmap.
    pub fn new_from_state_values(hm: &[(SPPath, StateValue)]) -> SPState {
        let mut xs = HashMap::new();
        let mut v = Vec::with_capacity(hm.len());
        for (key, value) in hm.iter() {
            v.push(value.clone());
            xs.insert(key.clone(), v.len() - 1);
        }
        SPState {
            index: xs,
            values: v,
            id: Uuid::new_v4(),
        }
    }

    /// Useful to avoid cloning (e.g. on extracted states).
    pub fn new_from_owned_values(hm: Vec<(SPPath, StateValue)>) -> SPState {
        let mut xs = HashMap::new();
        let mut v = Vec::with_capacity(hm.len());
        for (key, value) in hm.into_iter() {
            v.push(value);
            xs.insert(key, v.len() - 1);
        }
        SPState {
            index: xs,
            values: v,
            id: Uuid::new_v4(),
        }
    }

    pub fn add_state_variable(&mut self, path: SPPath, value: StateValue) {
        if self.index.contains_key(&path) {
            self.values[*self.index.get(&path).unwrap()] = value;
        } else {
            self.values.push(value);
            self.index.insert(path, self.values.len() - 1);
            self.id = Uuid::new_v4(); // the index has changed and it is probably better to reload
        }
    }
    pub fn add_state_variables(&mut self, map: Vec<(SPPath, StateValue)>) {
        map.into_iter().for_each(|(path, value)| {
            self.add_state_variable(path, value);
        });
    }

    /// Add a new state variable to the state. If the path already is included, the value is updated.
    /// Maybe we should change this and not update the state? This will also change the id of the state
    pub fn add_variable(&mut self, path: SPPath, value: SPValue) {
        let new_v = StateValue::new(value);
        self.add_state_variable(path, new_v);
    }

    /// Add new variables to the state. If a path is already included, the value is updated.
    /// Maybe we should change this and not update the state?
    pub fn add_variables(&mut self, map: Vec<(SPPath, SPValue)>) {
        map.into_iter().for_each(|(path, value)| {
            self.add_variable(path, value);
        });
    }

    pub fn id(&self) -> Uuid {
        self.id
    }

    /// Returns the index of a variable to be used for faster access. Cash this and compare with
    /// the id of the state if it is still valid
    pub fn state_path(&self, path: &SPPath) -> Option<StatePath> {
        self.index.get(path).map(|index| StatePath {
            path: path.clone(),
            index: *index,
            state_id: self.id,
        })
    }

    pub fn check_state_path(&self, state_path: &StatePath) -> bool {
        state_path.state_id == self.id && self.values.len() > state_path.index
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
        self.state_value(state_path).map(|x| x.value())
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
        self.state_value_from_path(path).map(|x| x.value())
    }

    /// Get a SPValue based on its index when you need to access the same variable multiple
    /// times. First get and cash the index from fn state_path(&path), and then use that index.
    /// If you do not know if the state can change, check the state id.
    /// This fn will panic if i is larger than no of values
    pub fn sp_value_from_index(&self, i: usize) -> &SPValue {
        self.state_value_from_index(i).value()
    }

    /// Get a projection of the state
    pub fn projection(&self) -> StateProjection {
        let s: Vec<(&SPPath, &StateValue)> = self
            .index
            .iter()
            .map(|(key, i)| (key, &self.values[*i]))
            .collect();

        StateProjection {
            state: s,
            id: self.id,
        }
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

        StateProjection {
            state: s,
            id: self.id,
        }
    }

    /// Checks if a sub state is the same as another states sub state given the same path
    /// This is used to check this so we do not need to create a clone with sub_state.
    pub fn is_sub_state_the_same(&self, state: &SPState, path: &SPPath) -> bool {
        self.index
            .iter()
            .filter(|(key, _)| key.is_child_of(path))
            .all(|(key, i)| {
                state
                    .sp_value_from_path(key)
                    .map(|x| x == self.values[*i].value())
                    .unwrap_or(false)
            })
    }

    pub fn are_new_values_the_same(&self, new_values: &SPState) -> bool {
        new_values.index.iter().all(|(key, i)| {
            let new_value = new_values.values[*i].value();
            new_value.is_type(SPValueType::Time)
                || self
                    .sp_value_from_path(key)
                    .map(|x| x == new_value)
                    .unwrap_or(false)
        })
    }

    pub fn difference(&self, new_state: &SPState) -> SPState {
        let self_p = self.projection();
        let new_p = new_state.projection();
        let res: Vec<(SPPath, StateValue)> = self_p
            .state
            .iter()
            .flat_map(|(p, v)| {
                new_p.value(p).map(|new_v| {
                    if new_v.value() != v.value() {
                        Some(((*p).clone(), new_v.clone()))
                    } else {
                        None
                    }
                })
            })
            .flatten()
            .collect();
        SPState::new_from_state_values(&res)
    }

    pub fn prefix_paths(&mut self, parent: &SPPath) {
        let mut xs = HashMap::new();
        for (path, i) in self.index.iter() {
            let mut new_p = path.clone();
            new_p.add_parent_path_mut(parent);
            xs.insert(new_p, *i);
        }
        self.index = xs;
        self.id = Uuid::new_v4(); // Changing id since the paths changes
    }

    pub fn unprefix_paths(&mut self, parent: &SPPath) {
        // return a new state without parent for all variables
        let mut new_index = HashMap::new();
        for (path, i) in &self.index {
            let mut new_key = path.clone();
            let _e = new_key.drop_parent(parent);
            new_index.insert(new_key, *i);
        }
        self.index = new_index;
        self.id = Uuid::new_v4(); // Changing id since the paths changes
    }

    pub fn next_is_allowed(&self, state_path: &StatePath) -> bool {
        self.check_state_path(state_path) && !self.values[state_path.index].has_next()
    }

    pub fn next(&mut self, state_path: &StatePath, value: SPValue) -> SPResult<()> {
        if !self.check_state_path(state_path) {
            panic!("The state path is wrong: {:?}", state_path);
        // TODO: Probably try with the path. But for now, we panic to find bugs in the runner.
        //Err(SPError::No(format!("The state path is not ok: sp_id:{}, s_id:{}, s_len:{}, index:{}", state_path.state_id,self.id, self.values.len(),state_path.index)))
        } else if !self.values[state_path.index].next(value) {
            Err(SPError::No(
                "The state already have a next value".to_string(),
            ))
        } else {
            Ok(())
        }
    }
    pub fn next_from_path(&mut self, path: &SPPath, value: SPValue) -> SPResult<()> {
        if let Some(sp) = self.state_path(&path) {
            self.next(&sp, value)
        } else {
            Err(SPError::No(format! {"Can not find the path: {:?}", path}))
        }
    }
    pub fn force(&mut self, state_path: &StatePath, value: &SPValue) -> SPResult<()> {
        if !self.check_state_path(state_path) {
            Err(SPError::No(format!(
                "The state path is not ok: sp_id:{}, s_id:{}, s_len:{}, index:{}",
                state_path.state_id,
                self.id,
                self.values.len(),
                state_path.index
            )))
        } else {
            self.values[state_path.index].force(value.clone());
            Ok(())
        }
    }
    pub fn force_from_path(&mut self, path: &SPPath, value: &SPValue) -> SPResult<()> {
        if let Some(sp) = self.state_path(&path) {
            self.force(&sp, &value)
        } else {
            Err(SPError::No(format! {"Can not find the path: {:?}", path}))
        }
    }
    pub fn revert_next(&mut self, state_path: &StatePath) -> SPResult<()> {
        if !self.check_state_path(state_path) {
            Err(SPError::No(format!(
                "The state path is not ok: sp_id:{}, s_id:{}, s_len:{}, index:{}",
                state_path.state_id,
                self.id,
                self.values.len(),
                state_path.index
            )))
        } else {
            self.values[state_path.index].revert_next();
            Ok(())
        }
    }
    pub fn revert_next_from_path(&mut self, path: &SPPath) -> SPResult<()> {
        if let Some(sp) = self.state_path(&path) {
            self.revert_next(&sp)
        } else {
            Err(SPError::No(format! {"Can not find the path: {:?}", path}))
        }
    }

    pub fn next_map(&mut self, map: Vec<(StatePath, SPValue)>) -> bool {
        let ok = map.iter().all(|(p, _)| self.next_is_allowed(p));
        ok && {
            map.into_iter().for_each(|(p, v)| {
                self.values[p.index].next(v);
            });
            true
        }
    }

    pub fn take_transition(&mut self) -> bool {
        let mut changed = false;
        self.values.iter_mut().for_each(|v: &mut StateValue| {
            changed |= v.take();
        });
        changed
    }

    pub fn extract(self) -> Vec<(SPPath, StateValue)> {
        let index = self.index;
        let mut values = self.values;
        index
            .into_iter()
            .map(|(key, i)| {
                (
                    key,
                    std::mem::replace(&mut values[i], StateValue::new(SPValue::Unknown)),
                )
            })
            .collect()
    }

    pub fn extend(&mut self, other_state: SPState) {
        let p = other_state.extract();
        self.add_state_variables(p);
    }

    /// Given a list of paths, consume this state and return a new
    /// state that only contain the exact paths given.
    pub fn filter_by_paths(self, paths: &[SPPath]) -> SPState {
        let mut kv = self.extract();
        let filtered = kv
            .into_iter()
            .filter(|(k, _)| paths.iter().any(|p| k == p))
            .collect();
        SPState::new_from_owned_values(filtered)
    }
}

impl fmt::Display for SPState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Sort keys by name.
        let mut proj = self.projection();
        proj.sort();
        let mut buf = Vec::new();
        for (p, val) in proj.state {
            let current = val.current.to_string();
            let next = val
                .next
                .as_ref()
                .map_or("".to_string(), |x| format!("- n: {}", x));
            let prev = val
                .prev
                .as_ref()
                .map_or("".to_string(), |x| format!("- p: {}", x));

            buf.push(format!("{}: {} {} {}", p, current, next, prev));
        }
        write!(f, "{}", buf.join("\n"))
    }
}

impl<'a> fmt::Display for StateProjection<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Sort keys by name.
        let mut proj = self.clone();
        proj.sort();
        let mut buf = Vec::new();
        for (p, val) in proj.state {
            let current = val.current.to_string();
            buf.push(format!("{}: {}", p, current));
        }
        write!(f, "\n{}", buf.join("\n"))
    }
}

/// helping making states with a macro
#[macro_export]
macro_rules! state {

    ($( $key: ident => $val: expr ),*) => {{
        let mut s: Vec<(SPPath, StateValue)> = Vec::new();
        $(
            s.push(($key.clone(), StateValue::new($val.to_spvalue())));
        )*
        SPState::new_from_state_values(&s)
    }};
    ($( $key: expr => $val: expr ),*) => {{
        let mut s: Vec<(SPPath, StateValue)> = Vec::new();
        $(
            let xs: Vec<String> = $key.iter().map(|x|x.to_string()).collect();
            s.push((SPPath::from(xs), StateValue::new($val.to_spvalue())));
        )*
        SPState::new_from_state_values(&s)
    }}
}

/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    use super::*;
    #[test]
    fn create_state() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let mut s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let s2 = state!(ab => 2, ac => true, kl => true);
        println!("s proj {:?}", s.projection());
        println!("s2 proj {:?}", s2.projection());

        assert_eq!(s, s2);

        s.add_variable(
            SPPath::from_string("timer/test"),
            SPValue::Time(std::time::SystemTime::now()),
        );
        s.add_variable(SPPath::from_string("path/test"), SPValue::Path(ac.clone()));

        let x = SPStateJson::from_state_recursive(&s).to_json();
        println!("{}", x);
    }

    #[test]
    fn get_value() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

        let v = SPPath::from_slice(&["a", "b"]);
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
        let v = &SPPath::from_slice(&["a", "b"]);
        let state_path = s.state_path(v).unwrap();
        let x = s.next(&state_path, 5.to_spvalue());
        assert_eq!(Ok(()), x);

        let x = s.next(&state_path, 5.to_spvalue());
        assert!(x.is_err());

        println!("{:?}", x);
        println!("{:?}", s);
    }

    #[test]
    fn sub_state_testing() {
        let a = SPPath::from_slice(&["a"]);
        let ab = SPPath::from_slice(&["a", "b"]);
        let ax = SPPath::from_slice(&["a", "x"]);
        let abc = SPPath::from_slice(&["a", "b", "c"]);
        let abx = SPPath::from_slice(&["a", "b", "x"]);
        let b = SPPath::from_slice(&["b"]);
        let s = state!(abc => false, abx => false, ax => true);

        // sub_state_projection
        assert_eq!(
            s.sub_state_projection(&ab).clone_state(),
            state!(abc => false, abx => false)
        );
        assert_eq!(
            s.sub_state_projection(&abc).clone_state(),
            state!(abc => false)
        );
        assert_eq!(s.sub_state_projection(&b).clone_state(), state!());
        assert_eq!(s.sub_state_projection(&a).clone_state(), s.clone());

        let mut s2 = s.clone();
        assert!(s.is_sub_state_the_same(&s2, &ab));
        s2.add_variable(ax, false.to_spvalue());
        assert!(s.is_sub_state_the_same(&s2, &ab));
        s2.add_variable(abc, true.to_spvalue());
        assert!(!s.is_sub_state_the_same(&s2, &ab));
    }

    #[test]
    fn test_difference() {
        let mut s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let ab = s.state_path(&SPPath::from_slice(&["a", "b"])).unwrap();
        let ac = s.state_path(&SPPath::from_slice(&["a", "c"])).unwrap();

        let mut new_s = s.clone();
        new_s.force(&ab, &3.to_spvalue()).unwrap();

        println!("{}", s.difference(&new_s));
        println!();

        new_s.force(&ab, &2.to_spvalue()).unwrap();

        println!("{}", s.difference(&new_s));
        println!();

        new_s.force(&ab, &4.to_spvalue()).unwrap();
        new_s.force(&ac, &2.to_spvalue()).unwrap();

        println!("{}", s.difference(&new_s));
        println!();
        println!("{}", new_s.difference(&s));
        println!();
    }

    #[test]
    fn state_json() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let abc = SPPath::from_slice(&["a", "b", "c"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let mut s = state!(ab => 2, ac => true, kl => true, abc => false);
        s.add_variable(
            SPPath::from_string("timer/test"),
            SPValue::Time(std::time::SystemTime::now()),
        );
        s.add_variable(
            SPPath::from_string("a/b/c/d/e"),
            SPValue::Time(std::time::SystemTime::now()),
        );

        let json_flat = SPStateJson::from_state_flat(&s);
        let json_rec = SPStateJson::from_state_recursive(&s);

        let from_flat = json_flat.clone().to_state();
        let from_rec_flat = json_rec.clone().to_state();

        println!(
            "flat: {}",
            serde_json::to_string_pretty(&json_flat).unwrap()
        );
        println!("rec: {}", serde_json::to_string_pretty(&json_rec).unwrap());

        println!("from_flat: {}", &from_flat);
        println!("from_rec_flat: {}", &from_rec_flat);

        assert_eq!(from_flat, from_rec_flat);
        assert_eq!(from_flat, s);
        assert_eq!(s, from_rec_flat);
    }
}
