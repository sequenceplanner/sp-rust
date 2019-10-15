// Public APIs for the runner
use sp_domain::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerModel {
    pub op_transitions: RunnerTransitions,
    pub ab_transitions: RunnerTransitions,
    pub plans: RunnerPlans,
    pub state_predicates: Vec<Variable>,
    pub goals: Vec<IfThen>,
    pub invariants: Vec<IfThen>,
    pub model: Model,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerPlans {
    pub op_plan: Vec<SPPath>,         // maybe have spids here?
    pub ab_plan: Vec<SPPath>,
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerTransitions {
    pub ctrl: Vec<Transition>,
    pub un_ctrl: Vec<Transition>
}

impl RunnerTransitions {
    pub fn extend(&mut self, other: RunnerTransitions) {
        self.ctrl.extend(other.ctrl);
        self.un_ctrl.extend(other.un_ctrl)
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct RunnerCommand {
    pub pause: bool,
    pub override_ability_transitions: Vec<SPPath>,
    pub override_operation_transitions: Vec<SPPath>,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PlannerResult {
    OpPlan(Vec<SPPath>),
    AbPlan(Vec<SPPath>)
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct RunnerInfo {
    pub state: StateExternal,
    pub ability_plan: Vec<SPPath>,
    pub enabled_ability_transitions: Vec<SPPath>,
    pub operation_plan: Vec<SPPath>,
    pub enabled_operation_transitions: Vec<SPPath>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PlannerCommand {
    ToDO,
}