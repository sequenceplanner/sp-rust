//!
//!

use super::*;
use std::collections::HashMap;

/// The SPNode is tracking the name and global path of an item. The name of the
/// node is also an SPPath if an extra level in the state is needed, but
/// is in most cases just a String.

/// The SPNode should be wrapped inside the item struct and the item should
/// also impl the Noder trait.
#[derive(PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct SPNode {
    name: SPPath,
    path: SPPath,
}

impl SPNode {
    /// Creates a new SPNode with a name. It is possible to give it a path as a
    /// name on the notation "/my/node/has/a/path_name"
    pub fn new(name: &str) -> SPNode {
        let p = SPPath::from_string(name);
        SPNode {
            name: p.clone(),
            path: p,
        }
    }

    /// Create a new node with both a name and a parent path. They are
    /// concatenated to form the node path
    pub fn new_with_path(name: &SPPath, parent_path: &SPPath) -> SPNode {
        let mut path = parent_path.clone();
        path.add_child_path_mut(name);
        SPNode {
            name: name.clone(),
            path,
        }
    }

    /// The name of the node, which is the leaf of its path
    pub fn name(&self) -> String {
        self.name.leaf()
    }

    /// The name path of the node
    pub fn name_path(&self) -> &SPPath {
        &self.name
    }

    /// The complete path of the node
    pub fn path(&self) -> &SPPath {
        &self.path
    }

    // returns the node's new and old path
    pub fn update_path(&mut self, path: &SPPath) -> (SPPath, SPPath) {
        let mut p = path.clone();
        let old = self.path.clone();
        p.add_child_path_mut(&self.name);
        self.path = p;
        (self.path().clone(), old)
    }

    pub fn update_name(&mut self, name: &str) {
        self.name.drop_leaf();
        self.name.add_child_mut(name);
        self.path.drop_leaf();
        self.path.add_child_mut(name);
    }

    pub fn is_eq(&self, path: &SPPath) -> bool {
        self.path() == path
    }

    fn write_nice(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}:{}", self.name, self.path)
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
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>>;
    fn get_child_mut<'a>(&'a mut self, next: &str, path: &SPPath) -> Option<SPMutItemRef<'a>>;
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>>;
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>>;
    fn update_path_children(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>);
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>);
    fn as_ref(&self) -> SPItemRef<'_>;
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_>;
    fn path(&self) -> &SPPath {
        self.node().path()
    }
    fn is_eq(&self, path: &SPPath) -> bool {
        self.path() == path
    }
    fn name(&self) -> String {
        self.node().name()
    }

    /// Finds the item with a specific SPPath. Will only find locals if asked from the
    /// correct resource (or below)
    fn get<'a>(&'a self, path: &SPPath) -> Option<SPItemRef<'a>> {
        if self.node().is_eq(path) {
            return Some(self.as_ref());
        }
        let next = path.next_node_in_path(self.path());
        next.as_ref().and_then(|n| self.get_child(n, path))
    }

    /// Finds the item with a specific SPPath. Will only find locals if asked from the
    /// correct resource (or below)
    fn get_mut<'a>(&'a mut self, path: &SPPath) -> Option<SPMutItemRef<'a>> {
        if self.node().is_eq(path) {
            return Some(self.as_mut_ref());
        }
        if let Some(n) = path.next_node_in_path(self.path()) {
            return self.get_child_mut(&n, path);
        }
        return None;
    }

    /// Finds the first item with a specific name and that includes all path sections (in any order).
    fn find_item<'a>(&'a self, name: &str, path_sections: &[&str]) -> Option<SPItemRef<'a>> {
        if self.node().name() == name {
            let found_it = path_sections
                .iter()
                .all(|x| self.node().path().path.contains(&(*x).to_string()));
            if found_it {
                return Some(self.as_ref());
            }
        }
        self.find_item_among_children(name, path_sections)
    }
    /// Finds the first item with a specific name and that includes all path sections (in any order).
    fn find_item_mut<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        if self.node().name() == name {
            let found_it = path_sections
                .iter()
                .all(|x| self.node().path().path.contains(&(*x).to_string()));
            if found_it {
                return Some(self.as_mut_ref());
            }
        }
        self.find_item_mut_among_children(name, path_sections)
    }

    /// updates the path of this item and its children
    fn update_path(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>) -> SPPath {
        let (p, old) = self.node_mut().update_path(path);
        changes.insert(old, p.clone());
        self.update_path_children(&p, changes);
        p
    }
}

/// A method used by the items when impl the Noder trait
/// Tries to find an item with the path in a list that incl
/// items that impl Noder
pub fn get_from_list<'a, T>(xs: &'a [T], next: &str, path: &SPPath) -> Option<SPItemRef<'a>>
where
    T: Noder,
{
    for i in xs.iter() {
        if i.node().name() == next {
            if let Some(x) = i.get(path) {
                return Some(x);
            }
        }
    }
    None
}

/// A method used by the items when impl the Noder trait
/// Tries to find an item with the path in a list that incl
/// items that impl Noder
pub fn get_from_list_mut<'a, T>(
    xs: &'a mut [T], next: &str, path: &SPPath,
) -> Option<SPMutItemRef<'a>>
where
    T: Noder,
{
    for i in xs.iter_mut() {
        if i.node_mut().name() == next {
            if let Some(x) = i.get_mut(path) {
                return Some(x);
            }
        }
    }
    None
}

/// A method used by the items when impl the Noder trait
/// Tries to find an item with the path in a list that incl
/// items that impl Noder
pub fn find_item_in_list<'a, T>(
    xs: &'a [T], name: &str, path_sections: &[&str],
) -> Option<SPItemRef<'a>>
where
    T: Noder,
{
    for i in xs.iter() {
        let res = i.find_item(name, path_sections);
        if res.is_some() {
            return res;
        }
    }
    None
}

pub fn find_item_mut_in_list<'a, T>(
    xs: &'a mut [T], name: &str, path_sections: &[&str],
) -> Option<SPMutItemRef<'a>>
where
    T: Noder,
{
    for i in xs.iter_mut() {
        let res = i.find_item_mut(name, path_sections);
        if res.is_some() {
            return res;
        }
    }
    None
}

/// A method used by the items when impl the Noder trait
/// Updates the path in items in the list of items impl Noder
pub fn update_path_in_list<'a, T>(
    xs: &'a mut [T], path: &SPPath, changes: &mut HashMap<SPPath, SPPath>,
) where
    T: Noder,
{
    for i in xs.iter_mut() {
        i.update_path(path, changes);
    }
}

#[cfg(test)]
mod node_tesing {
    use super::*;
    #[test]
    fn get() {
        let mut m = Model::new(
            "m",
            vec![
                SPItem::Model(Model::new(
                    "a",
                    vec![
                        SPItem::Model(Model::new("b", vec![])),
                        SPItem::Model(Model::new(
                            "c",
                            vec![SPItem::Model(Model::new("d", vec![]))],
                        )),
                    ],
                )),
                SPItem::Model(Model::new(
                    "k",
                    vec![SPItem::Model(Model::new("l", vec![]))],
                )),
            ],
        );

        let mut mapping = HashMap::new();
        m.update_path(&SPPath::new(), &mut mapping);

        let g_ab = SPPath::from_slice(&["m", "a", "b"]);
        let g_acd = SPPath::from_slice(&["m", "a", "c", "d"]);
        let g_k = SPPath::from_slice(&["m", "k"]);
        let ab = m.get(&g_ab);
        let acd = m.get(&g_acd);
        let k = m.get(&g_k);

        println!("{:?}", &ab);
        println!("{:?}", &acd);
        println!("{:?}", &k);

        assert!(ab.unwrap().name() == "b");
        assert!(acd.unwrap().name() == "d");
        assert!(k.unwrap().name() == "k");
    }

    #[test]
    fn find_single_name() {
        let mut m = Model::new(
            "m",
            vec![
                SPItem::Model(Model::new(
                    "a",
                    vec![
                        SPItem::Model(Model::new("b", vec![])),
                        SPItem::Model(Model::new(
                            "c",
                            vec![SPItem::Model(Model::new("d", vec![]))],
                        )),
                    ],
                )),
                SPItem::Model(Model::new(
                    "k",
                    vec![SPItem::Model(Model::new("l", vec![]))],
                )),
            ],
        );

        let mut mapping = HashMap::new();
        m.update_path(&SPPath::new(), &mut mapping);

        let g_ab = SPPath::from_slice(&["m", "a", "b"]);
        let g_acd = SPPath::from_slice(&["m", "a", "c", "d"]);
        let g_k = SPPath::from_slice(&["m", "k"]);
        let ab = m.find_item("b", &[]);
        let acd = m.find_item("d", &[]);
        let k = m.find_item("k", &[]);

        println!("{:?}", &ab);
        println!("{:?}", &acd);
        println!("{:?}", &k);

        assert!(*ab.unwrap().node().path() == g_ab);
        assert!(*acd.unwrap().node().path() == g_acd);
        assert!(*k.unwrap().node().path() == g_k);
    }

    #[test]
    fn find_with_path_sections() {
        let mut m = Model::new(
            "m",
            vec![
                SPItem::Model(Model::new(
                    "a",
                    vec![
                        SPItem::Model(Model::new("b", vec![])),
                        SPItem::Model(Model::new(
                            "c",
                            vec![SPItem::Model(Model::new("d", vec![]))],
                        )),
                    ],
                )),
                SPItem::Model(Model::new(
                    "k",
                    vec![SPItem::Model(Model::new("d", vec![]))],
                )),
            ],
        );

        let mut mapping = HashMap::new();
        m.update_path(&SPPath::new(), &mut mapping);

        let g_acd = SPPath::from_slice(&["m", "a", "c", "d"]);
        let g_k = SPPath::from_slice(&["m", "k", "d"]);
        let t1 = m.find_item("d", &["a"]);
        let t2 = m.find_item("d", &["c"]);
        let t3 = m.find_item("d", &["k", "m"]);
        let t4 = m.find_item("d", &["k"]);
        let t5 = m.find_item("d", &["k", "a"]);
        let t6 = m.find_item("d", &[]); // returns the first

        println!("{:?}", &t1);
        println!("{:?}", &t2);
        println!("{:?}", &t3);
        println!("{:?}", &t4);
        println!("{:?}", &t5);
        println!("{:?}", &t6);

        assert!(*t1.unwrap().node().path() == g_acd);
        assert!(*t2.unwrap().node().path() == g_acd);
        assert!(*t3.unwrap().node().path() == g_k);
        assert!(*t4.unwrap().node().path() == g_k);
        assert!(t5.is_none());
        assert!(t6.is_some());
    }
}
