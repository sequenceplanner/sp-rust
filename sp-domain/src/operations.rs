//! The operations used in SP
//! 
//! 


use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Ability {
    pub spid: SPID,
    pub controlled: Vec<Transition>,
    pub uncontrolled: Vec<Transition>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Operation {
    pub spid: SPID,
    pub precondition: Vec<Transition>,
    pub postcondition: Vec<Transition>,
    pub uncontrolled: Vec<Transition>,
    pub invariant: Predicate,  // Must hold during the execution of this operation (for the planner)
}

