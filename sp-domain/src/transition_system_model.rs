/// This module contain a simple model type that we can use for the
/// formal verification stuff.

use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct TransitionSystemModel {
    pub name: String,
    pub vars: Vec<Variable>,
    pub state_predicates: Vec<Variable>,
    pub transitions: Vec<Transition>,
    pub specs: Vec<Spec>,
}

impl TransitionSystemModel {
    pub fn from(model: &Model) -> Self {
        let mut vars: Vec<Variable> = model.resources()
            .iter()
            .flat_map(|r| r.get_variables())
            .collect();

        let sub_items: Vec<_> = model.resources()
            .iter()
            .flat_map(|r| r.sub_items())
            .collect();

        let sub_item_variables: Vec<Variable> = sub_items
            .iter()
            .flat_map(|i| match i {
                SPItem::Variable(v) => Some(v.clone()),
                _ => None,
            }).collect();

        let transitions: Vec<Transition> = model.resources()
            .iter().flat_map(|r| r.get_transitions()).collect();
        let state_predicates: Vec<Variable> = model.resources()
            .iter().flat_map(|r| r.get_state_predicates()).collect();

        vars.extend(sub_item_variables.iter().cloned());

        let mut specs: Vec<Spec> = model.items().iter().flat_map(|i| match i {
            SPItem::Spec(s) => Some(s),
            _ => None,
        }).cloned().collect();
        let sub_item_specs: Vec<Spec> = sub_items.iter().flat_map(|i| match i {
            SPItem::Spec(s) => Some(s.clone()),
            _ => None,
        }).collect();
        specs.extend(sub_item_specs.iter().cloned());

        TransitionSystemModel {
            name: model.name().into(),
            vars,
            state_predicates,
            transitions,
            specs
        }
    }
}
