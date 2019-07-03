//! Maybe we will use attributes later

use serde::{Deserialize, Serialize};
use uuid::Uuid;




pub type SPJson = serde_json::Value;
//pub type SPJsonError = serde_json::Error;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct SPAttributes {
    attr: SPJson,
}

impl SPAttributes {
    pub fn empty() -> SPAttributes {
        let m = serde_json::Value::Object(serde_json::Map::default());
        SPAttributes { attr: m }
    }
}