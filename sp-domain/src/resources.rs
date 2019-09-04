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
//! Paths includes the structure of the messages, the local variables and the parameters
//! The local variables are named under the local/... path branch and the paramteters
//! under the param/... branch. Before running the resource, all abilities must be instanciated
//! where the local and parameters get a global path.
//!  

use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Resource {
    pub spid: SPID,
    pub variables: Vec<Variable>,
    pub abilities: Vec<Ability>,
    pub parameters: Vec<Variable>,
    pub paths: SPStruct,  // defines the variable structure. Also include local paths for parameters and local variables. 
}

impl Resource {
    // The new path should incl the name of the resource
    pub fn change_path(&mut self, new_path: SPPath) {
        // TODO
    }
}
