//!
//!

use super::*;

/// The SPNode is tracking the name and the local and global path of an item
/// The SPNode should be wrapped inside the item struct and the item should
/// also impl the Noder trait.
#[derive(PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct SPNode {
    name: String,
    path: SPPath,
}

impl SPNode {
    pub fn new(name: &str) -> SPNode {
        SPNode {
            name: name.to_string(),
            path: SPPath::new(),
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }
    pub fn path_mut(&mut self) -> &mut SPPath {
        &mut self.path
    }

    pub fn update_path(&mut self, path: &SPPath) -> SPPath {
        let mut p = path.clone();
        self.path = p.add_child(&self.name);
        self.path().clone()
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
    fn find_item_among_children<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>>;
    fn update_path_children(&mut self, path: &SPPath);
    fn as_ref(&self) -> SPItemRef<'_>;
    fn path(&self) -> &SPPath {
        self.node().path()
    }
    fn is_eq(&self, path: &SPPath) -> bool {
        self.path() == path
    }
    fn name(&self) -> &str {
        &self.node().name
    }

    /// Finds the item with a specific SPPath. Will only find locals if asked from the
    /// correct resource (or below)
    fn get<'a>(&'a self, path: &SPPath) -> Option<SPItemRef<'a>> {
        if self.node().is_eq(path) {
            return Some(self.as_ref());
        }
        let next = path.next_node_in_path(self.path());
        next.as_ref().and_then(|n| self.get_child(n, path) )
    }
    /// Finds the first item with a specific name and that includes all path sections (in any order).
    fn find_item<'a>(&'a self, name: &str, path_sections: &[&str]) -> Option<SPItemRef<'a>> {
        if self.node().name() == name {
            let found_it = path_sections.iter().all(|x| {
                self.node()
                    .path()
                    .path
                    .contains(&x.to_string())
                    
            });
            if found_it {
                return Some(self.as_ref());
            }
        }
        self.find_item_among_children(name, path_sections)
    }

    /// updates the path of this item and its children
    fn update_path(&mut self, path: &SPPath) -> SPPath {
        let p = self.node_mut().update_path(path);
        self.update_path_children(&p);
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
pub fn find_item_in_list<'a, T>(
    xs: &'a [T],
    name: &str,
    path_sections: &[&str],
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

/// A method used by the items when impl the Noder trait
/// Updates the path in items in the list of items impl Noder
pub fn update_path_in_list<'a, T>(xs: &'a mut [T], path: &SPPath)
where
    T: Noder,
{
    for i in xs.iter_mut() {
        i.update_path(path);
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

        m.update_path(&SPPath::new());

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

        m.update_path(&SPPath::new());

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

        m.update_path(&SPPath::new());

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
