//! Variables in SP


use serde::{Deserialize, Serialize};
use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Variable {
    Measured(VariableData),
    Estimated(VariableData), 
    Command(VariableData), 
    StatePredicate(VariableData, Predicate),  // Maybe have these here? 
    StateFunction(VariableData, Action),       // Maybe have these here? 
}


/// Var is the attributes in all types of variables, but should not be used by itself. Use Variable
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct VariableData {
    pub spid: SPID,
    pub inital_value: SPValue, 
    pub domain: Vec<SPValue>,
}


/// The possible variable types used by operations to define parameters
/// Must be the same as Variable
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Copy)]
pub enum VariableType {
    Measured,
    Estimated,
    Command,
    StatePredicate,
    StateFunction,
}

impl Variable {
    pub fn is_type(&self, t: VariableType) -> bool {
        match self {
            Variable::Measured(_) => { VariableType::Measured == t },
            Variable::Estimated(_) => { VariableType::Estimated == t },
            Variable::Command(_) => { VariableType::Command == t },
            Variable::StatePredicate(_, _) => { VariableType::StatePredicate == t },
            Variable::StateFunction(_, _) => { VariableType::StateFunction == t },
        }
    }
    pub fn has_type(&self) -> VariableType {
        match self {
            Variable::Measured(_) => { VariableType::Measured },
            Variable::Estimated(_) => { VariableType::Estimated },
            Variable::Command(_) => { VariableType::Command },
            Variable::StatePredicate(_, _) => { VariableType::StatePredicate },
            Variable::StateFunction(_, _) => { VariableType::StateFunction },
        }
    }
}

impl VariableType {
    pub fn is_type(&self, v: &Variable) -> bool {
        v.is_type(*self)
    }
}

impl Default for VariableType {
    fn default() -> Self {
        VariableType::Estimated
    }
}