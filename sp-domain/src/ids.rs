//! Support to handle identification of items in SP and in the states

use super::*;

/// SPID is used by things in the model that needs to be identified 
#[derive(Debug, PartialEq, Eq, Hash, Serialize, Deserialize, Clone)]
pub struct SPID {
    pub id: Uuid,
    pub name: String,
}

impl SPID {
    pub fn new(name: &str) -> SPID {
        SPID {
            id: Uuid::new_v4(),
            name: String::from(name),
        }
    }
}
impl Default for SPID {
    fn default() -> Self {
        SPID::new("default_name")
    }
}

/// Representing a variable in a hiearchy
#[derive(Debug, Hash, Eq, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct SPPath{
    pub path: Vec<String>,
}

impl SPPath {
    pub fn new() -> SPPath { SPPath{path: Vec::new()} }
    pub fn from(n: &[String]) -> SPPath {
        let v: Vec<String> = n.iter().map(|s| s.to_string()).collect();
        SPPath{path: v}
    }
    pub fn from_str(n: &[&str]) -> SPPath {
        let v: Vec<String> = n.iter().map(|s| s.to_string()).collect();
        SPPath{path: v}
    }
}