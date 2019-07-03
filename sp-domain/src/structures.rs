//! The structures used by SP to organize objects. We can use this to define the names used in the state
//! 

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
struct SPNode {
    id: Uuid,
    item: SPIDAble,
    parent: Option<Node>
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
struct SPStruct {
    spid: SPID,
    nodes: Vec<SPNode>
}