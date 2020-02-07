use serde::{Deserialize, Serialize};
use sp_domain::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct PlanningResult {
    pub plan_found: bool,
    pub trace: Vec<PlanningFrame>,
    pub time_to_solve: std::time::Duration,
    pub raw_output: String,
    pub raw_error_output: String,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct PlanningFrame {
    // The state (for debug)
    pub state: SPState,
    // The transition taken this frame
    pub transition: SPPath,
}



pub struct PlanningModel {
    pub name: String,
    pub vars: Vec<Variable>,
    pub state_predicates: Vec<Variable>,
    pub transitions: Vec<Transition>,
    pub specs: Vec<Spec>,
}

impl PlanningModel {
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

        PlanningModel {
            name: model.name().into(),
            vars,
            state_predicates,
            transitions,
            specs
        }
    }
}

mod nuxmv;
pub use nuxmv::*;
