use uuid::Uuid;
use serde::{Serialize, Deserialize, };
use std::collections::HashMap;

pub type SPJson = serde_json::Value;
pub type SPJsonError = serde_json::Error;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct SPAttributes {
    attr: SPJson
}

impl SPAttributes {
    pub fn empty() -> SPAttributes {
        let m = serde_json::Value::Object(serde_json::Map::default());
        SPAttributes{attr: m}
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct SPID {
    pub id: Uuid,
    pub name: String,
    pub attributes: SPAttributes
}

trait IDAble {
    fn get_id(&self) -> &SPID;
    fn to_json(&self) -> String;

    fn id(&self) -> &Uuid {
        &self.get_id().id
    }
    fn name(&self) -> &String {
        &self.get_id().name
    }
    fn attributes(&self) -> &SPAttributes {
        &self.get_id().attributes
    }

    
}


#[cfg(test)]
mod idable_tests {
    use super::*;
    #[test]
    fn create() {
        let attr = SPAttributes::empty();
        println!("{:?}", attr);

        let mut r = SPID{
            id: Uuid::new_v4(),
            name: "test".to_owned(),
            attributes: SPAttributes::empty()
        };

        r.name = "no".to_owned();
        println!("{:?}", r);
    }
}