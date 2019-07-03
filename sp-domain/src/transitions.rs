//! The transitions used in SP

use super::*;



#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Transition {
    pub spid: SPID,
    pub guard: Predicate,
    pub action: Vec<Action>,
    pub effects: Vec<Action>,
}

impl Transition {
    pub fn replace_variable_path(&mut self, map: &HashMap<SPPath, SPPath>) {
        self.guard.replace_variable_path(map);
        self.action.iter_mut().for_each(|a|{
            a.replace_variable_path(map)
        });
        self.effects.iter_mut().for_each(|a|{
            a.replace_variable_path(map)
        });
    }
}