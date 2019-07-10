//! # Sequence Planner domain
//! 
//! This crate represents a modeling domain used by SP


pub mod ids;
pub use crate::ids::{SPID, SPPath};

pub mod values;
pub use crate::values::{SPValue, SPValueType, ToSPValue};

pub mod predicates;
pub use crate::predicates::{Predicate, Action, EvaluatePredicate, NextAction};

pub mod states;
pub use crate::states::{State, StateValue, AssignStateValue, ToStateValue};

pub mod variables;
pub use crate::variables::{Variable, VariableData, VariableType};

pub mod transitions;
pub use crate::transitions::{Transition};

pub mod operations;
pub use crate::operations::{Operation, Ability, OperationFunction};

pub mod resources;
pub use crate::resources::{Resource};

pub mod structures;
pub use crate::structures::{SPStruct};

pub mod sops;
pub use crate::sops::{SOP};



use serde::{Deserialize, Serialize};
use uuid::Uuid;
use std::collections::HashMap;




#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SPItem {
    Resource(Resource),
    Operation(Operation),
    Ability(Ability),
    SOP(SOP)
}


use std::error;
use std::fmt;

type Result<T> = std::result::Result<T, SPError>;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SPError{
    OverwriteDelay(states::Delay, AssignStateValue),
    OverwriteNext(states::Next, AssignStateValue),
    No(String),
    Undefined,
}

impl fmt::Display for SPError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            SPError::OverwriteDelay(prev, next) => {
                write!(f, "You are trying to overwrite a Delay in the State. current: {:?}, new: {:?} ", prev, next)
            },
            SPError::OverwriteNext(prev, next) => {
                write!(f, "You are trying to overwrite a Next in the State. current: {:?}, new: {:?} ", prev, next)
            },
            SPError::Undefined  => {
                write!(f, "An undefined SP error!")
            },
            SPError::No(s)  => {
                write!(f, "Oh No: {}", s)
            },
        }
    }
}

impl error::Error for SPError {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        None
    }
}