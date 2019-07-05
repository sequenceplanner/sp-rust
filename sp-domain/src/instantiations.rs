//! Maybe we will use this for resources

use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Parameter {
    pub path: SPPath,
    pub var_type: VariableType,
    pub value_type: SPValueType,
}


pub trait Instantiable {
    type Item;

    fn instantiate(&self, map: &HashMap<SPPath, SPPath>) -> Self::Item;
    fn parameters(&self) -> &Vec<Parameter>;

    fn is_instantiated(&self) -> bool {
        self.parameters().is_empty()
    }
}
