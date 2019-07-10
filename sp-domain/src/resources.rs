//! # Resource
//! 
//! A resources is a core item when defining a system.
//! 
//! It includes a list of variables that defines it state and 
//! a list of abilties that defines what it can do. 
//! 
//! The variables are connected to ROS-messages that is defined by the
//! messages SPStruct. 
//! 
//! The resource can also depend on external variables, which is defined by 
//! the parameters. Each parameter must be instantiated before the resource
//! can be used. The paths of all variables must also be updated based on 
//! the overall structure of the system
//!  

use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Resource {
    pub spid: SPID,
    pub variables: Vec<Variable>,
    pub abilities: Vec<Ability>,
    pub parameters: Vec<Variable>,
    pub messages: SPStruct,  // defines the variable structure
}

impl Resource {
    // The new path should incl the name of the resource
    pub fn change_path(&mut self, new_path: SPPath) {

    }
}
