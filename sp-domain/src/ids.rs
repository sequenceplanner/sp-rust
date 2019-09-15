//! Support to handle identification of items in SP and in the states

use super::*;
use std::fmt;

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

impl fmt::Display for SPPath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f,"{}",self.path.join("/"))
    }
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
    pub fn is_child_of(&self, path: &SPPath) -> bool {
        (self.path.len() >= path.path.len()) && path.path.iter().zip(self.path.iter()).all(|(a, b)| a == b)
    }
}

/// ********** TESTS ***************

#[cfg(test)]
mod sppath_test {
    use super::*;
    #[test]
    fn is_child_of() {
        let a = SPPath::from_str(&["a"]);
        let ab = SPPath::from_str(&["a", "b"]);
        let abc = SPPath::from_str(&["a", "b", "c"]);
        let abcd = SPPath::from_str(&["a", "b", "c", "d"]);
        let abcx = SPPath::from_str(&["a", "b", "c", "x"]);

        assert!(ab.is_child_of(&a));
        assert!(abcd.is_child_of(&a));
        assert!(!a.is_child_of(&ab));
        assert!(!a.is_child_of(&abcd));
        assert!(!abcd.is_child_of(&abcx));
        assert!(!abcx.is_child_of(&abcd));
        assert!(abcd.is_child_of(&abc));
        assert!(abcx.is_child_of(&abc));
    }
}
