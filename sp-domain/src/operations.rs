//! The operations used in SP
//! 
//! 


use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Ability {
    pub spid: SPID,
    pub controlled: Vec<Transition>,
    pub uncontrolled: Vec<Transition>,
    pub parameters: Vec<Parameter>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Operation {
    pub spid: SPID,
    pub precondition: Vec<Transition>,
    pub postcondition: Vec<Transition>,
    pub uncontrolled: Vec<Transition>,
    pub parameters: Vec<Parameter>,
    pub invariant: Predicate,  // Must hold during the execution of this operation (for the planner)
}


impl Instantiable for Ability {
    type Item = Ability;

    fn instantiate(&self, map: &HashMap<SPPath, SPPath>) -> Self::Item {
        let mut new_ability = self.clone();

        new_ability.controlled.iter_mut().for_each(|t| {
            t.replace_variable_path(map);
        });

        new_ability
    }

    fn parameters(&self) -> &Vec<Parameter> {
        &self.parameters
    }
}
