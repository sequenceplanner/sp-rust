use serde::{Deserialize, Serialize};
use uuid::Uuid;


pub mod sp_value;
pub use crate::sp_value::*;

pub mod predicates;
pub use crate::predicates::*;

pub mod state;
pub use crate::state::*;



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
pub struct SPID {
    pub id: Uuid,
    pub name: String,
    //pub attributes: SPAttributes,
}

impl SPID {
    pub fn new(name: &str) -> SPID {
        SPID {
            id: Uuid::new_v4(),
            name: String::from(name),
            //attributes: SPAttributes::empty(),
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

    /// Testing show we know that SPValues and support can be used outside
    #[test]
    fn sp_value_testing_external() {
        assert_eq!(true.to_spvalue(), SPValue::Bool(true));
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
    }
}
