//! The structures used by SP to organize objects and to define an items path
//! 

use super::*;

/// SPStruct is used for structuring items in hierarchies. Only allows that a SPID exists at one place.
/// If the same spid needs to be in multiple positions, divide into multiple SPStructs
/// 
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct SPStruct {
    pub spid: SPID,
    nodes: HashMap<Uuid, SPNode>
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Structure {
    ids: Vec<SPID>,
    last: Option<Uuid>,
    children: HashMap<Uuid, Box<Structure>>,
}


impl SPStruct {

    /// Creates a new Struct if the Structure does not contains any loops and does not have any duplicates
    pub fn new(spid: SPID) -> SPStruct {
        SPStruct {
            spid,
            nodes: HashMap::new(),
        }
    }    

    /// Creates a new Struct if the Structure does not contains any loops and does not have any duplicates
    pub fn from(spid: SPID, structure: Structure) -> Result<SPStruct> {
        let nodes = SPStruct::make_nodes(structure, None)?;

        Ok(SPStruct {
            spid,
            nodes
        })
    }

    /// Returns the path to the SPID
    pub fn path(&self, id: Uuid) -> SPPath {
        let xs: Vec<String> = self.req_find_name(id, Vec::new()).into_iter().rev().collect();
        SPPath::from(&xs)
    }

    /// returns all root SPIDs
    pub fn roots(&self) -> Vec<SPID> {
        self.nodes.iter()
            .filter(|(_, n)| n.parent_id.is_none())
            .map(|(_, n)| n.item.clone())
            .collect()
    }

    /// The parent of the given id
    pub fn parent(&self, id: Uuid) -> Option<SPID> {
        self.nodes
            .get(&id)
            .and_then(|n| n.parent_id
                .and_then(|p_id| self.nodes.get(&p_id)
                    .map(|x| x.item.clone())))
    }

    /// Returns the children of this SPID, empty if no children
    pub fn children(&self, id: Uuid) -> Vec<SPID> {
        self.nodes.iter().filter(|(_, n)|{
            n.parent_id.map(|p_id| p_id == id).unwrap_or(false)
        })
        .map(|(_, n)| n.item.clone())
        .collect()
    }

    /// Returns all children of this SPID, recursively
    pub fn children_all(&self, id: Uuid) -> Vec<SPID> {
        let mut childs_map = HashMap::new();
        for (_, n) in self.nodes.iter() {
            if let Some(p_id) = n.parent_id {
                let ch = childs_map.entry(p_id).or_insert(Vec::new());
                ch.push(n.item.id);
            }
        }

        SPStruct::get_the_children(id, &childs_map).into_iter().map(|id| {
            self.nodes.get(&id).map(|n| n.item.clone())
        })
        .flatten()
        .collect()


    }

    /// Returns an SPID and all its children in the structure. Returns all SPIDs that 
    /// was removed
    pub fn remove(&mut self, id: Uuid) -> Vec<SPID> {
        match self.nodes.get(&id) {
            None => Vec::new(),
            Some(n) => {
                let mut ch = self.children_all(id);
                ch.push(n.item.clone());
                for spid in ch.iter() {
                    self.nodes.remove(&spid.id);
                };
                ch
            }
        }
    }

    /// Adds the structure as children to spid. If spid does not exist in the struct,
    /// it is added as a root node. If the struct includes the same 
    pub fn add(&mut self, spid: &SPID, structure: Structure) -> Result<()> {
        let xs = SPStruct::make_nodes(structure, Some(spid.id))?;

        let exists: Vec<SPID> = xs.iter().filter(|(id, _)|{
            self.nodes.contains_key(&id)
        })
        .map(|(_, n)| n.item.clone())
        .collect();

        if !exists.is_empty() { 
            return Err(SPError::No(format!("The following SPIDs already exists in the SPStruct: {:?}", exists)))
        }

        self.nodes.extend(xs);
        self.nodes.entry(spid.id).or_insert(SPNode::from(&spid));

        Ok(())



    }



    fn get_the_children(id: Uuid, map: &HashMap<Uuid, Vec<Uuid>>) -> Vec<Uuid> {
        match map.get(&id) {
            None => Vec::new(),
            Some(xs) => {
                xs.iter().map(|ch_id| {
                    let mut ch = SPStruct::get_the_children(*ch_id, map);
                    ch.push(*ch_id);
                    ch
                })
                .flatten()
                .collect()
            }
        }
    }


    fn make_nodes(structure: Structure, parent: Option<Uuid>) -> Result<HashMap<Uuid, SPNode>> {
        let mut xs: Vec<SPNode> = structure.ids.iter().map(|spid| {
            let mut node = SPNode::from(spid);
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
            (n.item.id, n)
        }).collect();
        if res.len() < len_of_xs {
            Err(SPError::No("The same SPID is used multiple times".to_string()))
        } else {
            Ok(res)
        }

    }




    fn req_find_name(&self, node_id: Uuid, mut aggr: Vec<String>) -> Vec<String> {
        match self.nodes.get(&node_id) {
            Some(n) => {
                aggr.push(n.item.name.clone());
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

    pub fn n(mut self, spid: &SPID) -> Self {
        self.ids.push(spid.clone());
        self.last = Some(spid.id);
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
    item: SPID,
    parent_id: Option<Uuid>
}

impl SPNode {
    fn from(spid: &SPID) -> SPNode {
        SPNode{
            item: spid.clone(),
            parent_id: None,
        }
    }


    pub fn item(&self) -> &SPID {
        &self.item
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

    #[test]
    fn create_struct() {
        let x = SPID::new("x");
        let y = SPID::new("y");
        let z = SPID::new("z");

        let k = x.clone();
        assert_eq!(x, k);
        assert_eq!(x.id, k.id);
        assert_eq!(x.name, k.name);


        let mut xn = SPNode::from(&x);
        let mut yn = SPNode::from(&y);
        let mut zn = SPNode::from(&z);

        assert_eq!(x.id, xn.item.id);
        assert_eq!(None, xn.parent_id);

        xn.add_parent(yn.item.id);
        assert_eq!(Some(yn.item.id), xn.parent_id);

        let mut str = SPStruct{
            spid: SPID::new("test"),
            nodes: [
                (xn.item.id, xn.clone()),
                (yn.item.id, yn.clone()),
                (zn.item.id, zn.clone()),
                ].into_iter().cloned().collect()
        };
        
        assert_eq!(SPPath::from(&["y".to_string()]), str.path(yn.item.id));

        


    }

    #[test]
    fn make_structer() {
        let x = SPID::new("x");
        let y = SPID::new("y");
        let z = SPID::new("z");
        let s = Structure::new()
            .n(&x)
            .n(&y)
            .ch(Structure::new()
                .n(&z)
            );

        println!("Structure: {:?}", s);

    }

    #[test]
    fn make_SPStruct() {
        let x = SPID::new("x");
        let y = SPID::new("y");
        let z = SPID::new("z");
        let s = Structure::new().n(&x)
            .n(&y)
            .ch(Structure::new()
                .n(&z)
            );

        let res = SPStruct::from(SPID::new("hej"), s).unwrap();
        assert_eq!(res.nodes.get(&z.id).unwrap().parent_id, Some(y.id));
        println!("make SPStruct: {:?}", res);

    }

    #[test]
    fn get_stuff_from_SPStruct() {
        let x = SPID::new("x");
        let y = SPID::new("y");
        let z = SPID::new("z");
        let k = SPID::new("k");
        let l = SPID::new("l");
        let m = SPID::new("m");

        let s = SPStruct::from(SPID::new("hej"), 
            Structure::new()
                .n(&x)
                .ch(Structure::new()
                    .n(&z)
                    .ch(Structure::new()
                        .n(&k)
                    )
                    .n(&l)
                )
                .n(&y)
                .ch(Structure::new()
                    .n(&m)
                )
        ).unwrap();

        use std::collections::HashSet;
        let test: HashSet<SPID> = s.roots().into_iter().collect();
        let expect: HashSet<SPID> = [&x, &y].into_iter().cloned().cloned().collect();
        assert_eq!(test, expect);

        let test: HashSet<SPID> = s.children(x.id).into_iter().collect();
        let expect: HashSet<SPID> = [&z, &l].iter().cloned().cloned().collect();
        assert_eq!(test, expect);

        let test: HashSet<SPID> = s.children(k.id).into_iter().collect();
        let expect: HashSet<SPID> = HashSet::new();
        assert_eq!(test, expect);

        let test: HashSet<SPID> = s.children_all(x.id).into_iter().collect();
        let expect: HashSet<SPID> = [&z, &k, &l].iter().cloned().cloned().collect();
        assert_eq!(test, expect);

        let test = s.path(k.id);
        let expect = SPPath::from(&vec!(x.name, z.name, k.name));
        assert_eq!(test, expect);

    }

    #[test]
    fn add_and_remove_SPStruct() {
        let x = SPID::new("x");
        let y = SPID::new("y");
        let z = SPID::new("z");
        let k = SPID::new("k");
        let l = SPID::new("l");
        let m = SPID::new("m");

        let mut s = SPStruct::from(SPID::new("hej"), 
            Structure::new()
                .n(&x)
                .ch(Structure::new()
                    .n(&z)
                    .ch(Structure::new()
                        .n(&k)
                    )
                    .n(&l)
                )
                .n(&y)
                .ch(Structure::new()
                    .n(&m)
                )
        ).unwrap();

        use std::collections::HashSet;
        let test: HashSet<SPID> = s.remove(z.id).into_iter().collect();
        let expect: HashSet<SPID> = [&z, &k].into_iter().cloned().cloned().collect();
        assert_eq!(test, expect);

        let s2 = SPStruct::from(SPID::new("hej"), 
            Structure::new()
                .n(&x)
                .ch(Structure::new()
                    .n(&l)
                )
                .n(&y)
                .ch(Structure::new()
                    .n(&m)
                )
        ).unwrap();
        assert_eq!(s.nodes, s2.nodes);

        s.add(&y, Structure::new()
            .n(&z)
            .ch(Structure::new()
                .n(&k)
            )
        ).unwrap();

        let s2 = SPStruct::from(SPID::new("hej"), 
            Structure::new()
                .n(&x)
                .ch(Structure::new()
                    .n(&l)
                )
                .n(&y)
                .ch(Structure::new()
                    .n(&m)
                    .n(&z)
                    .ch(Structure::new()
                        .n(&k)
                    )
                )
        ).unwrap();
        assert_eq!(s.nodes, s2.nodes);

        let no = s.add(&y, Structure::new()
            .n(&z)
            .ch(Structure::new()
                .n(&k)
            )
        );
        assert!(no.is_err());
        assert_eq!(s.nodes, s2.nodes);


    }


}