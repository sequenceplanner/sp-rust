//! Support to handle identification of items in SP and in the states

use super::*;

/// SPID is used by things in the model that needs to be identified 
#[derive(Debug, PartialEq, Eq, Hash, Serialize, Deserialize, Clone)]
pub struct SPID {
    pub id: Uuid,
    pub name: String,
    //pub attributes: SPAttributes,
}

impl SPID {
    pub fn new(name: &str) -> SPID {
        SPID {
            id: Uuid::new_v4(),
            name: String::from(name),
            //attributes: SPAttributes::empty(),
        }
    }
}
impl Default for SPID {
    fn default() -> Self {
        SPID::new("default_name")
    }
}

pub trait IDAble {
    fn spid(&self) -> &SPID;
}

/// Representing a variable in a hiearchy
#[derive(Debug, Hash, Eq, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct SPPath{
    pub name: Vec<String>,
}

impl SPPath {
    pub fn new() -> SPPath { SPPath{name: Vec::new()} }
    pub fn from(n: &[String]) -> SPPath {
        let v: Vec<String> = n.iter().map(|s| s.to_string()).collect();
        SPPath{name: v}
    }
    pub fn from_str(n: &[&str]) -> SPPath {
        let v: Vec<String> = n.iter().map(|s| s.to_string()).collect();
        SPPath{name: v}
    }
}