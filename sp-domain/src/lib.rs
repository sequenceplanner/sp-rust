//! # Sequence Planner domain
//! 
//! This crate represents a modeling domain used by SP


pub mod ids;
pub use crate::ids::{SPID, IDAble, SPPath};

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
pub use crate::operations::{Operation, Ability};

pub mod resources;
pub use crate::resources::{Resource};

pub mod instantiations;
pub use crate::instantiations::{Parameter, Instantiable};



use serde::{Deserialize, Serialize};
use uuid::Uuid;
use std::collections::HashMap;