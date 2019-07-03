//! The resources used in 

use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Resource {
    pub spid: SPID,
    pub variables: Vec<Variable>,
    pub abilities: Vec<Ability>,
    pub parameters: Vec<Parameter>
}
