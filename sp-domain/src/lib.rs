//! # Sequence Planner domain
//!
//! This crate represents a modeling domain used by SP


pub mod ids;
pub use crate::ids::{SPID, SPPath};

pub mod values;
pub use crate::values::{SPValue, SPValueType, ToSPValue};

pub mod predicates;
pub use crate::predicates::{Predicate, Action, PredicateValue, Compute, EvaluatePredicate, NextAction};

pub mod states;
pub use crate::states::{SPState, StateValue, StateExternal, AssignState, AssignStateValue, ToStateValue};

pub mod variables;
pub use crate::variables::{Variable, VariableData, VariableType};

pub mod transitions;
pub use crate::transitions::{Transition};

pub mod operations;
pub use crate::operations::{Operation, Ability, OperationFunction};

pub mod resources;
pub use crate::resources::*;

pub mod structures;
pub use crate::structures::{SPStruct, Structure};

pub mod sops;
pub use crate::sops::{SOP};

pub mod z3config;
pub use crate::z3config::{ConfigZ3};

pub mod z3context;
pub use crate::z3context::{ContextZ3};

pub mod z3solver;
pub use crate::z3solver::{SolverZ3};

pub mod z3sorts;
pub use crate::z3sorts::{IntSortZ3, BoolSortZ3, RealSortZ3, StringSortZ3};

pub mod z3values;
pub use crate::z3values::{BoolZ3, IntZ3, RealZ3};

pub mod z3variables;
pub use crate::z3variables::{BoolVarZ3, IntVarZ3, RealVarZ3};

pub mod z3relations;
pub use crate::z3relations::{EQZ3};

pub mod z3operations;
pub use crate::z3operations::{MULZ3, DIVZ3, MODZ3, ADDZ3, SUBZ3};

mod utils;
use utils::*;

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use std::collections::HashMap;


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SPItem {
    Topic{topic: String},
    MessageType{msg_type: String},
    Variable{field_name: String, var: Variable},
}

impl SPItem {
    pub fn name(&self) -> &str {
        match self {
            SPItem::Topic{topic} => &topic,
            SPItem::MessageType{msg_type} => &msg_type,
            SPItem::Variable{field_name, var: _} => &field_name,
        }
    }
}

// #[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
// pub enum SPItem {
//     Resource(Resource),
//     Operation(Operation),
//     Ability(Ability),
//     SOP(SOP)
// }


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
            SPError::OverwriteDelay(next, prev) => {
                write!(f, "You are trying to overwrite a Delay in the State. current: {:?}, new: {:?} ", prev, next)
            },
            SPError::OverwriteNext(next, prev) => {
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
