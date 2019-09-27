//!
//! 


use super::*;



/// The SPNode is tracking the name and the local and global path of an item
/// The SPNode should be wrapped inside the item struct and the item should 
/// also impl the Noder trait.
#[derive(PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct SPNode {
    name: String,
    paths: SPPaths,
}

impl SPNode {
    pub fn new(name: &str) -> SPNode {
        SPNode {
            name: name.to_string(),
            paths: SPPaths::empty(),
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn local_path(&self) -> &Option<LocalPath> {
        &self.paths.local_path()
    }
    pub fn global_path(&self) -> &Option<GlobalPath> {
        &self.paths.global_path()
    }
    pub fn paths(&self) -> &SPPaths {
        &self.paths
    }
    pub fn paths_mut(&mut self) -> &mut SPPaths {
        &mut self.paths
    }

    pub fn update_path(&mut self, paths: &SPPaths) -> SPPaths {
        self.paths.upd(paths);
        self.paths.add(self.name.clone());
        self.paths().clone()
    }

    pub fn is_eq(&self, path: &SPPath) -> bool {
        self.paths().is_eq(path)
    }

    pub fn next_node_in_path(&self, path: &SPPath) -> Option<String> {
        self.paths().next_node_in_path(path)
    }

    fn write_nice(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}{}", self.name, self.paths())
    }
}



impl std::fmt::Display for SPNode {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        self.write_nice(f)
    }
}

impl std::fmt::Debug for SPNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.write_nice(f)
    }
}

pub trait Noder {
    fn node(&self) -> &SPNode;
    fn node_mut(&mut self) -> &mut SPNode;
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>>;
    fn update_path_children(&mut self, paths: &SPPaths);
    fn as_ref<'a>(&'a self) -> SPItemRef<'a>;
    fn paths(&self) -> &SPPaths {
        self.node().paths()
    }
    fn is_eq(&self, path: &SPPath) -> bool {
        self.paths().is_eq(path)
    }
    fn has_global(&self) -> bool {
        self.paths().global_path().is_some()
    }
    fn get_path(&self) -> SPPath {
        if let Some(g) = self.paths().global_path() {
            return g.to_sp().clone()
        }
        if let Some(l) = self.paths().local_path() {
            return l.to_sp().clone()
        }
        panic!("We do not have a path in {} and get_path", self.node());
    }

    fn name(&self) -> &str {
        &self.node().name
    }

    /// Finds the item with a specific SPPath. Will only find locals if asked from the
    /// correct resource (or below)
    fn find<'a>(&'a self, path: &SPPath) -> Option<SPItemRef<'a>>  {
        if self.node().is_eq(path) {
            return Some(self.as_ref());
        }
        let next = self.node().next_node_in_path(path);
        if next.is_none() {
            return None
        }
        self.find_child(&next.unwrap(), path)
    }

    /// updates the path of this item and its children
    fn update_path(&mut self, paths: &SPPaths) -> SPPaths {
        let paths = self.node_mut().update_path(paths);
        self.update_path_children(&paths);
        paths
    }
    
}


/// A method used by the items when impl the Noder trait
/// Tries to find an item with the path in a list that incl
/// items that impl Noder
pub fn find_in_list<'a, T>(
    xs: &'a [T], 
    next: &str, 
    path: &SPPath) 
-> Option<SPItemRef<'a>> where T: Noder {
    for i in xs.iter() {
        if i.node().name() == next {
            if let Some(x) = i.find(path) {
                return Some(x)
            }
        }
    }
    return None
}

/// A method used by the items when impl the Noder trait
/// Updates the path in items in the list of items impl Noder
pub fn update_path_in_list<'a, T>(
    xs: &'a mut [T], 
    paths: &SPPaths) where T: Noder {
    for i in xs.iter_mut() {
        i.update_path(paths);
    }
}

