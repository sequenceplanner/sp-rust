//! The SPPath is used for identifying items in the model

use super::{SPError, SPResult};
use serde::{Deserialize, Serialize};

/// The SPPath is used for identifying all objects in a model. The path will be defined
/// based on where the item is in the model hierarchy
#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Serialize, Deserialize, Clone, Default, Debug)]
pub struct SPPath {
    pub path: Vec<String>,
}

impl std::fmt::Display for SPPath {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "/{}", self.path.join("/"))
    }
}

impl SPPath {
    pub fn new() -> SPPath {
        SPPath { path: vec![] }
    }
    pub fn from(path: Vec<String>) -> SPPath {
        SPPath { path }
    }
    pub fn from_slice<T: AsRef<str>>(path: &[T]) -> SPPath {
        let xs: Vec<String> = path.iter().map(|s| s.as_ref().to_string()).collect();
        SPPath { path: xs }
    }
    pub fn from_string(s: &str) -> SPPath {
        let res: Vec<&str> = s.trim_start_matches('/').split('/').collect();
        SPPath::from_slice(&res)
    }
    pub fn add_child(&self, sub: &str) -> Self {
        let mut p = self.path.clone();
        p.push(sub.to_string());
        SPPath::from(p)
    }
    pub fn add_child_mut(&mut self, sub: &str) {
        self.path.push(sub.to_string());
    }
    pub fn add_parent(&self, root: &str) -> Self {
        let mut p = self.path.clone();
        p.insert(0, root.to_string());
        SPPath::from(p)
    }
    pub fn add_parent_mut(&mut self, root: &str) {
        self.path.insert(0, root.to_string());
    }
    pub fn add_child_path_mut(&mut self, sub: &SPPath) {
        self.path.append(&mut sub.path.clone())
    }
    pub fn add_parent_path_mut(&mut self, root: &SPPath) -> SPPath {
        let mut new_path = root.path.clone();
        new_path.append(&mut self.path);
        self.path = new_path;
        self.clone()
    }
    pub fn add_child_path(&self, sub: &SPPath) -> SPPath {
        let mut p = self.path.clone();
        p.append(&mut sub.path.clone());
        SPPath::from(p)
    }
    pub fn add_parent_path(&self, root: &SPPath) -> SPPath {
        let mut new_path = root.path.clone();
        new_path.append(&mut self.path.clone());
        SPPath::from(new_path)
    }

    pub fn drop_parent(&mut self, parent: &SPPath) -> SPResult<()> {
        let zipped = self.path.iter().zip(parent.path.iter());
        let match_len = zipped.filter(|(a, b)| a == b).count();
        if match_len == parent.path.len() {
            self.path.drain(0..match_len);
            Ok(())
        } else {
            Err(SPError::No(format!(
                "cannot drop parent as it does not exist: {} - {}",
                self, parent
            )))
        }
    }

    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    pub fn is_child_of(&self, other: &SPPath) -> bool {
        (self.path.len() >= other.path.len())
            && other.path.iter().zip(self.path.iter()).all(|(a, b)| a == b)
    }

    pub fn is_child_of_any(&self, others: &[SPPath]) -> bool {
        others.iter().any(|o| self.is_child_of(o))
    }

    pub fn root(&self) -> String {
        self.path.first().unwrap_or(&String::default()).clone()
    }

    pub fn parent(&self) -> SPPath {
        if self.path.len() <= 1 {
            SPPath::new()
        } else {
            SPPath::from_slice(&self.path[..self.path.len() - 1])
        }
    }

    pub fn drop_root(&self) -> SPPath {
        if self.path.is_empty() {
            SPPath::new()
        } else {
            SPPath::from_slice(&self.path[1..])
        }
    }

    pub fn leaf(&self) -> String {
        if self.path.is_empty() {
            "".to_string()
        } else {
            self.path[self.path.len() - 1].clone()
        }
    }

    pub fn leaf_as_path(&self) -> SPPath {
        let leaf = if self.path.is_empty() {
            "".to_string()
        } else {
            self.path[self.path.len() - 1].clone()
        };
        SPPath { path: vec![leaf] }
    }

    pub fn drop_leaf(&mut self) -> String {
        if !self.path.is_empty() {
            self.path.remove(self.path.len() - 1)
        } else {
            String::new()
        }
    }

    /// returns the next name in the path of this SPPath based on a path
    /// that is the current parent to this path
    pub fn next_node_in_path(&self, parent_path: &SPPath) -> Option<String> {
        if self.is_child_of(parent_path) && self.path.len() > parent_path.path.len() {
            Some(self.path[parent_path.path.len()].clone())
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests_paths {
    use super::*;
    #[test]
    fn making() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ab_v2 = SPPath::new();
        let ab_v2 = ab_v2.add_child("a").add_child("b");
        let ab_v3 = SPPath::from_string("a/b");

        assert_eq!(ab.to_string(), "/a/b".to_string());
        assert_eq!(ab_v2, ab);
        assert_eq!(ab_v3, ab);
        assert_ne!(SPPath::from_string("b/a"), ab);
        assert_ne!(SPPath::from_slice(&["b", "a"]), ab);
        assert_ne!(SPPath::from_slice(&["a", "b", "c"]), ab);
    }

    #[test]
    fn drop_parent() {
        let mut ab = SPPath::from_slice(&["a", "b", "c"]);
        let parent = SPPath::from_slice(&["a", "b"]);
        ab.drop_parent(&parent).unwrap();
        assert_eq!(ab, SPPath::from_slice(&["c"]));

        let mut ab = SPPath::from_slice(&["a", "b", "c"]);
        let parent = SPPath::from_slice(&["a", "b", "c"]);
        ab.drop_parent(&parent).unwrap();
        assert_eq!(ab, SPPath::new());

        let mut ab = SPPath::from_slice(&["a", "b", "c"]);
        let parent = SPPath::from_slice(&["a"]);
        ab.drop_parent(&parent).unwrap();
        assert_eq!(ab, SPPath::from_slice(&["b", "c"]));
    }

    #[test]
    fn drop_parent_fail() {
        let mut ab = SPPath::from_slice(&["a", "b", "c"]);
        let parent = SPPath::from_slice(&["a", "c"]);
        let res = ab.drop_parent(&parent);
        assert!(res.is_err())
    }

    #[test]
    fn get_next_name() {
        let p = SPPath::from_string("a/b/c/d");
        println! {"{}", serde_json::to_string(&p).unwrap()};
    }
}
