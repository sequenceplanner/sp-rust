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
    pub functions: Vec<OperationFunction>, 
}

/// The operation functions are used by the operation to define the goal and the invariant. 
/// The first parameter is the predicate that defined when this goal or invariant should hold.
/// Usually this predicate is true when the operation is executing
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum OperationFunction {
    Goal(Predicate, Predicate),
    Invariant(Predicate, Predicate)
}

