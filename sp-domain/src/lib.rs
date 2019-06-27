use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

pub type SPJson = serde_json::Value;
//pub type SPJsonError = serde_json::Error;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct SPAttributes {
    attr: SPJson,
}

impl SPAttributes {
    pub fn empty() -> SPAttributes {
        let m = serde_json::Value::Object(serde_json::Map::default());
        SPAttributes { attr: m }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SPValue {
    Bool(bool),
    //byte(u8), deprecated
    //char(char), deprecated
    Float32(f32),
    Float64(f64),
    Int8(i8),
    Uint8(u8),
    Int16(i16),
    Uint16(u16),
    Int32(i32),
    Uint32(u32),
    Int64(i64),
    Uint64(u64),
    String(String),
    Time(u32),
    Duration(u32),
    ID(Uuid), // use to also find the value in a state of variable with id
    Array(Vec<SPValue>),
    Map(HashMap<String, SPValue>),
}

impl Default for SPValue {
    fn default() -> Self {
        SPValue::Bool(false)
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Predicate {
    AND(Vec<Predicate>),
    OR(Vec<Predicate>),
    NOT(Box<Predicate>),
    TRUE,
    FALSE,
    EQ(SPValue, SPValue), // use SPValue::ID to fetch the value from the state
    NEQ(SPValue, SPValue),
    INDOMAIN(SPValue, Vec<SPValue>),
}

impl Default for Predicate {
    fn default() -> Self {
        Predicate::TRUE
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Action {
    var: Uuid,
    value: Compute,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
/// Used in actions to compute a new SPValue.
pub enum Compute {
    Get(SPValue),
    //TakeNext(SPValue, Vec<SPValue>), // to be impl when needed
    //TakeBefore(SPValue, Vec<SPValue>),
    // Add(Box<Compute>, Box<Compute>),
    // Sub(Box<Compute>, Box<Compute>),
    // Join(Box<Compute>, Box<Compute>),
}

impl Default for Compute {
    fn default() -> Self {
        Compute::Get(SPValue::default())
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct SPID {
    pub id: Uuid,
    pub name: String,
    pub attributes: SPAttributes,
}

impl SPID {
    pub fn new(name: &str) -> SPID {
        SPID {
            id: Uuid::new_v4(),
            name: String::from(name),
            attributes: SPAttributes::empty(),
        }
    }
}
impl Default for SPID {
    fn default() -> Self {
        SPID::new("default_name")
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Variable {
    pub spid: SPID,
    pub domain: Vec<SPValue>,
    pub initial_value: SPValue,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Transition {
    pub spid: SPID,
    pub guard: Predicate,
    pub action: Vec<Action>,
    pub effects: Vec<Action>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct StatePredicate {
    pub spid: SPID,
    pub predicate: Predicate,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct AbilityOperation {
    pub spid: SPID,
    pub controlled: Vec<Transition>,
    pub uncontrolled: Vec<Transition>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct PlanningOperation {
    pub spid: SPID,
    pub precondition: Vec<Transition>,
    pub postcondition: Vec<Transition>,
    pub uncontrolled: Vec<Transition>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct State {
    pub state: HashMap<Uuid, SPValue>,
}

impl State {
    pub fn get_value<'a>(&'a self, v: &'a SPValue) -> &'a SPValue {
        match v {
            SPValue::ID(id) => self.state.get(id).unwrap_or(v),
            _ => v,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Resource {
    pub spid: SPID,
    pub input_state: Vec<Variable>,
    pub command_state: Vec<Variable>,
    pub estimated_state: Vec<Variable>,
    pub initial_state: State,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SOP {
    Sequence { spid: SPID, sop: Vec<SOP> },
    Parallel { spid: SPID, sop: Vec<SOP> },
    Alternative { spid: SPID, sop: Vec<SOP> },
    Arbitrary { spid: SPID, sop: Vec<SOP> },
    Other { spid: SPID, sop: Vec<SOP> },
    Operation { op: Uuid, spid: SPID, sop: Vec<SOP> },
}

impl Default for SOP {
    fn default() -> Self {
        SOP::Parallel {
            spid: SPID::default(),
            sop: Vec::new(),
        }
    }
}

trait IDAble {
    fn spid(&self) -> &SPID;
    fn to_json(&self) -> String;

    fn id(&self) -> &Uuid {
        &self.spid().id
    }
    fn name(&self) -> &String {
        &self.spid().name
    }
    fn attributes(&self) -> &SPAttributes {
        &self.spid().attributes
    }
}

enum SPModel {
    Variable(Variable),
    Transition(Transition),
    Ability(AbilityOperation),
    Operation(PlanningOperation),
    Resource(Resource),
    State(State),
    SOP(SOP),
}

#[cfg(test)]
mod idable_tests {
    use super::*;
    #[test]
    fn create() {
        let spid = SPID::new("hej");
        let mut op = PlanningOperation::default();

        op.spid.name = "hej".to_owned();
        println!("{:?}", op);

        // let attr = SPAttributes::empty();
        // println!("{:?}", attr);

        // let mut r = SPID{
        //     id: Uuid::new_v4(),
        //     name: "test".to_owned(),
        //     attributes: SPAttributes::empty()
        // };

        // r.name = "no".to_owned();
        // println!("{:?}", r);
    }
}
