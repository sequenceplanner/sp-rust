//! The structures used by SP to organize objects and to define an items path
//!

use super::*;

/// SPStruct is used for structuring items in hierarchies. Only allows that a ID exists at one place.
/// If the same id needs to be in multiple positions, divide into multiple SPStructs
///
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct SPStruct {
    pub id: Uuid,
    nodes: HashMap<Uuid, SPNode>
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Structure {
    ids: Vec<(Uuid,SPItem)>,
    last: Option<Uuid>,
    children: HashMap<Uuid, Box<Structure>>,
}


impl SPStruct {

    /// Creates a new Struct if the Structure does not contains any loops and does not have any duplicates
    pub fn new(id: Uuid) -> SPStruct {
        SPStruct {
            id,
            nodes: HashMap::new(),
        }
    }

    /// Creates a new Struct if the Structure does not contains any loops and does not have any duplicates
    pub fn from(structure: Structure) -> Result<SPStruct> {
        let nodes = SPStruct::make_nodes(structure, None)?;

        Ok(SPStruct {
            id: Uuid::new_v4(),
            nodes
        })
    }

    /// Returns the path to the Uuid
    pub fn path(&self, id: Uuid) -> SPPath {
        let xs: Vec<String> = self.req_find_name(id, Vec::new()).into_iter().rev().collect();
        SPPath::from(&xs)
    }

    /// returns all root Uuids
    pub fn roots(&self) -> Vec<Uuid> {
        self.nodes.iter()
            .filter(|(_, n)| n.parent_id.is_none())
            .map(|(_, n)| n.id.clone())
            .collect()
    }

    pub fn root_items(&self) -> impl Iterator<Item = &SPItem> {
        self.nodes.iter()
            .filter(|(_, n)| n.parent_id.is_none())
            .map(|(_, n)| &n.item)
    }

    /// The parent of the given id
    pub fn parent(&self, id: Uuid) -> Option<Uuid> {
        self.nodes
            .get(&id)
            .and_then(|n| n.parent_id
                .and_then(|p_id| self.nodes.get(&p_id)
                    .map(|x| x.id.clone())))
    }

    pub fn children(&self, id: &Uuid) -> Vec<Uuid> {
        self.nodes.iter().filter(|(_, n)|{
            n.parent_id.map(|p_id| &p_id == id).unwrap_or(false)
        })
        .map(|(_, n)| n.id).collect()
    }

    // /// Returns the children of this Uuid, empty if no children
    // pub fn children_items(&self, id: &Uuid) -> impl Iterator<Item = &SPItem> {
    //     self.nodes.iter().filter(|(_, n)|{
    //         n.parent_id.map(|p_id| &&p_id == &id).unwrap_or(false)
    //     })
    //     .map(|(_, n)| &n.item)
    // }

    /// Returns all children of this Uuid, recursively
    pub fn children_all(&self, id: &Uuid) -> Vec<Uuid> {
        let mut childs_map = HashMap::new();
        for (_, n) in self.nodes.iter() {
            if let Some(p_id) = n.parent_id {
                let ch = childs_map.entry(p_id).or_insert(Vec::new());
                ch.push(n.id);
            }
        }

        SPStruct::get_the_children(id, &childs_map).into_iter().map(|id| {
            self.nodes.get(&id).map(|n| n.id)
        })
        .flatten()
        .collect()
    }

    /// Returns an Uuid and all its children in the structure. Returns all Uuids that
    /// was removed
    pub fn remove(&mut self, id: &Uuid) -> Vec<Uuid> {
        match self.nodes.get(id) {
            None => Vec::new(),
            Some(n) => {
                let mut ch = self.children_all(id);
                ch.push(n.id.clone());
                for id in ch.iter() {
                    self.nodes.remove(&id);
                };
                ch
            }
        }
    }

    /// Adds the structure as children to id. If id does not exist in the struct,
    /// it is added as a root node. If the struct includes the same
    pub fn add(&mut self, id: &Uuid, structure: Structure) -> Result<()> {
        let xs = SPStruct::make_nodes(structure, Some(id.clone()))?;

        let exists: Vec<Uuid> = xs.iter().filter(|(id, _)|{
            self.nodes.contains_key(&id)
        })
        .map(|(_, n)| n.id.clone())
        .collect();

        if !exists.is_empty() {
            return Err(SPError::No(format!("The following Uuids already exists in the SPStruct: {:?}", exists)))
        }

        self.nodes.extend(xs);
        // TODO: unsure what this did
        // self.nodes.entry(id.clone()).or_insert(SPNode::from(id));

        Ok(())



    }



    fn get_the_children(id: &Uuid, map: &HashMap<Uuid, Vec<Uuid>>) -> Vec<Uuid> {
        match map.get(&id) {
            None => Vec::new(),
            Some(xs) => {
                xs.iter().map(|ch_id| {
                    let mut ch = SPStruct::get_the_children(ch_id, map);
                    ch.push(*ch_id);
                    ch
                })
                .flatten()
                .collect()
            }
        }
    }


    fn make_nodes(structure: Structure, parent: Option<Uuid>) -> Result<HashMap<Uuid, SPNode>> {
        let mut xs: Vec<SPNode> = structure.ids.iter().map(|(id,item)| {
            let mut node = SPNode::from(id,item);
            parent.map(|p_id|{
                node.add_parent(p_id);
            });
            node
        }).collect();


        let ch = structure.children.into_iter().fold((Vec::new(), Vec::new()), |mut acc, (p, s)| {
            match SPStruct::make_nodes(*s, Some(p)) {
                Ok(n) => {
                    let nodes: Vec<SPNode> = n.into_iter().map(|(_, v)| v).collect();
                    acc.0.push(nodes);
                    acc
                },
                Err(e) => {
                    acc.1.push(e);
                    acc
                },
            }
        });

        if !ch.1.is_empty() {
            return Err(ch.1[0].clone())
        };

        xs.extend(ch.0.into_iter().flatten().collect::<Vec<SPNode>>());
        let len_of_xs = xs.len();

        let res: HashMap<Uuid, SPNode> = xs.into_iter().map(|n|{
            (n.id.clone(), n)
        }).collect();
        if res.len() < len_of_xs {
            Err(SPError::No("The same Uuid is used multiple times".to_string()))
        } else {
            Ok(res)
        }

    }




    fn req_find_name(&self, node_id: Uuid, mut aggr: Vec<String>) -> Vec<String> {
        match self.nodes.get(&node_id) {
            Some(n) => {
                aggr.push(n.item.name().to_owned());
                if let Some(p) = n.parent_id {
                    self.req_find_name(p, aggr)
                } else {
                    aggr
                }
            },
            None => aggr
        }
    }
}


impl Structure {
    pub fn new() -> Structure {
        Structure{
            ids: vec!(),
            last: None,
            children: HashMap::new(),
        }
    }

    pub fn n(mut self, item: SPItem) -> Self {
        let id = Uuid::new_v4();
        self.ids.push((id.clone(), item));
        self.last = Some(id.clone());
        self
    }

    pub fn ch(mut self, structure: Structure) -> Self {
        self.last.map(|id|{
            self.children.insert(id, Box::new(structure));
        });
        self
    }
}

/// Internal representation.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
struct SPNode {
    id: Uuid,
    pub item: SPItem,
    parent_id: Option<Uuid>
}

impl SPNode {
    fn from(id: &Uuid, item: &SPItem) -> SPNode {
        SPNode{
            id: id.clone(),
            item: item.clone(),
            parent_id: None,
        }
    }

    pub fn parent(&self) -> Option<Uuid> {
        self.parent_id.clone()
    }

    /// Adds a parent and returns the old parent id if any
    fn add_parent(&mut self, parent_id: Uuid) -> Option<Uuid> {
        let old = self.parent_id;
        self.parent_id = Some(parent_id);
        old
    }
}


/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    use super::*;

    // #[test]
    // fn create_struct() {
    //     let x = SPItem::Topic;
    //     let y = SPItem::Topic;
    //     let z = SPItem::Topic;

    //     let k = x.clone();
    //     assert_eq!(x, k);

    //     let mut xn = SPNode::from("x", &x);
    //     let mut yn = SPNode::from("y", &y);
    //     let mut zn = SPNode::from("z", &z);

    //     assert_eq!(xn.name, "x");
    //     assert_eq!(xn.item, x);
    //     assert_eq!(None, xn.parent_id);

    //     xn.add_parent(yn.id);
    //     assert_eq!(Some(yn.id), xn.parent_id);

    //     let mut str = SPStruct{
    //         id: Uuid::new("test"),
    //         nodes: [
    //             (xn.id, xn.clone()),
    //             (yn.id, yn.clone()),
    //             (zn.id, zn.clone()),
    //             ].iter().cloned().collect()
    //     };

    //     assert_eq!(SPPath::from(&["y".to_string()]), str.path(yn.id));




    // }

    // #[test]
    // fn make_structer() {
    //     let x = Uuid::new("x");
    //     let y = Uuid::new("y");
    //     let z = Uuid::new("z");
    //     let s = Structure::new()
    //         .n(&x)
    //         .n(&y)
    //         .ch(Structure::new()
    //             .n(&z)
    //         );

    //     println!("Structure: {:?}", s);

    // }

    // #[test]
    // fn make_SPStruct() {
    //     let x = Uuid::new("x");
    //     let y = Uuid::new("y");
    //     let z = Uuid::new("z");
    //     let s = Structure::new().n(&x)
    //         .n(&y)
    //         .ch(Structure::new()
    //             .n(&z)
    //         );

    //     let res = SPStruct::from(Uuid::new("hej"), s).unwrap();
    //     assert_eq!(res.nodes.get(&z.id).unwrap().parent_id, Some(y.id));
    //     println!("make SPStruct: {:?}", res);

    // }

    // #[test]
    // fn get_stuff_from_SPStruct() {
    //     let x = Uuid::new("x");
    //     let y = Uuid::new("y");
    //     let z = Uuid::new("z");
    //     let k = Uuid::new("k");
    //     let l = Uuid::new("l");
    //     let m = Uuid::new("m");

    //     let s = SPStruct::from(Uuid::new("hej"),
    //         Structure::new()
    //             .n(&x)
    //             .ch(Structure::new()
    //                 .n(&z)
    //                 .ch(Structure::new()
    //                     .n(&k)
    //                 )
    //                 .n(&l)
    //             )
    //             .n(&y)
    //             .ch(Structure::new()
    //                 .n(&m)
    //             )
    //     ).unwrap();

    //     use std::collections::HashSet;
    //     let test: HashSet<Uuid> = s.roots().into_iter().collect();
    //     let expect: HashSet<Uuid> = [&x, &y].iter().cloned().cloned().collect();
    //     assert_eq!(test, expect);

    //     let test: HashSet<Uuid> = s.children(x.id).into_iter().collect();
    //     let expect: HashSet<Uuid> = [&z, &l].iter().cloned().cloned().collect();
    //     assert_eq!(test, expect);

    //     let test: HashSet<Uuid> = s.children(k.id).into_iter().collect();
    //     let expect: HashSet<Uuid> = HashSet::new();
    //     assert_eq!(test, expect);

    //     let test: HashSet<Uuid> = s.children_all(x.id).into_iter().collect();
    //     let expect: HashSet<Uuid> = [&z, &k, &l].iter().cloned().cloned().collect();
    //     assert_eq!(test, expect);

    //     let test = s.path(k.id);
    //     let expect = SPPath::from(&vec!(x.name, z.name, k.name));
    //     assert_eq!(test, expect);

    // }

    // #[test]
    // fn add_and_remove_SPStruct() {
    //     let x = Uuid::new("x");
    //     let y = Uuid::new("y");
    //     let z = Uuid::new("z");
    //     let k = Uuid::new("k");
    //     let l = Uuid::new("l");
    //     let m = Uuid::new("m");

    //     let mut s = SPStruct::from(Uuid::new("hej"),
    //         Structure::new()
    //             .n(&x)
    //             .ch(Structure::new()
    //                 .n(&z)
    //                 .ch(Structure::new()
    //                     .n(&k)
    //                 )
    //                 .n(&l)
    //             )
    //             .n(&y)
    //             .ch(Structure::new()
    //                 .n(&m)
    //             )
    //     ).unwrap();

    //     use std::collections::HashSet;
    //     let test: HashSet<Uuid> = s.remove(z.id).into_iter().collect();
    //     let expect: HashSet<Uuid> = [&z, &k].iter().cloned().cloned().collect();
    //     assert_eq!(test, expect);

    //     let s2 = SPStruct::from(Uuid::new("hej"),
    //         Structure::new()
    //             .n(&x)
    //             .ch(Structure::new()
    //                 .n(&l)
    //             )
    //             .n(&y)
    //             .ch(Structure::new()
    //                 .n(&m)
    //             )
    //     ).unwrap();
    //     assert_eq!(s.nodes, s2.nodes);

    //     s.add(&y, Structure::new()
    //         .n(&z)
    //         .ch(Structure::new()
    //             .n(&k)
    //         )
    //     ).unwrap();

    //     let s2 = SPStruct::from(Uuid::new("hej"),
    //         Structure::new()
    //             .n(&x)
    //             .ch(Structure::new()
    //                 .n(&l)
    //             )
    //             .n(&y)
    //             .ch(Structure::new()
    //                 .n(&m)
    //                 .n(&z)
    //                 .ch(Structure::new()
    //                     .n(&k)
    //                 )
    //             )
    //     ).unwrap();
    //     assert_eq!(s.nodes, s2.nodes);

    //     let no = s.add(&y, Structure::new()
    //         .n(&z)
    //         .ch(Structure::new()
    //             .n(&k)
    //         )
    //     );
    //     assert!(no.is_err());
    //     assert_eq!(s.nodes, s2.nodes);


    // }


}
