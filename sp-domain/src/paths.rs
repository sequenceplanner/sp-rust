//! The SPPath is used for identifying items in the model

use serde::{Deserialize, Serialize};

/// The SPPath is used for identifying all objects in a model. The path will be defined
/// based on where the item is in the model hierarchy
#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Serialize, Deserialize, Clone, Default, Debug)]
pub struct SPPath {
    pub path: Vec<String>,
}

impl std::fmt::Display for SPPath {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self.path.join("/"))
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
        let res: Vec<&str> = s.split("/").collect();
        SPPath::from_slice(&res)
    }
    pub fn add_child(mut self, sub: &str) -> Self {
        self.path.push(sub.to_string());
        self
    }
    pub fn add_parent(mut self, root: &str) -> Self {
        self.path.insert(0, root.to_string());
        self
    }
    pub fn add_child_path(&mut self, sub: &SPPath) {
        self.path.append(&mut sub.path.clone())
    }
    pub fn add_parent_path(&mut self, root: &SPPath) {
        let mut new_path = root.path.clone();
        new_path.append(&mut self.path);
        self.path = new_path;
    }
    // maybe should return result instead of panicing. use with caution.
    pub fn drop_parent(&mut self, parent: &SPPath) {
        let zipped = self.path.iter().zip(parent.path.iter());
        let match_len = zipped.filter(|(a, b)| a == b).count();
        if match_len == parent.path.len() {
            // all parent paths matched, remove
            self.path.drain(0..match_len);
        } else {
            panic!(
                "cannot drop parent as it does not exist: {} - {}",
                self, parent
            );
        }
    }

    pub fn is_child_of(&self, other: &SPPath) -> bool {
        (self.path.len() >= other.path.len())
            && other.path.iter().zip(self.path.iter()).all(|(a, b)| a == b)
    }

    pub fn is_child_of_any(&self, others: &[SPPath]) -> bool {
        others.iter().find(|o| self.is_child_of(o)).is_some()
    }

    pub fn parent(&self) -> SPPath {
        if self.path.len() <= 1 {
            SPPath::new()
        } else {
            SPPath::from_slice(&self.path[..self.path.len() - 1])
        }
    }

    pub fn leaf(&self) -> String {
        if self.path.len() == 0 {
            "".to_string()
        } else {
            self.path[self.path.len() - 1].clone()
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

        assert_eq!(ab.to_string(), "a/b".to_string());
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
        ab.drop_parent(&parent);
        assert_eq!(ab, SPPath::from_slice(&["c"]));

        let mut ab = SPPath::from_slice(&["a", "b", "c"]);
        let parent = SPPath::from_slice(&["a", "b", "c"]);
        ab.drop_parent(&parent);
        assert_eq!(ab, SPPath::new());

        let mut ab = SPPath::from_slice(&["a", "b", "c"]);
        let parent = SPPath::from_slice(&["a"]);
        ab.drop_parent(&parent);
        assert_eq!(ab, SPPath::from_slice(&["b", "c"]));
    }

    #[test]
    #[should_panic]
    fn drop_parent_fail() {
        let mut ab = SPPath::from_slice(&["a", "b", "c"]);
        let parent = SPPath::from_slice(&["a", "c"]);
        ab.drop_parent(&parent);
    }

    #[test]
    fn get_next_name() {}
}
