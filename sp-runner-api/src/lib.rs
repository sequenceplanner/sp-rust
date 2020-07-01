// Public APIs for the runner
use serde::{Deserialize, Serialize};
use sp_domain::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerModel {
    // temporary!
    pub hl_op_transitions: RunnerTransitions,

    pub op_transitions: RunnerTransitions,
    pub ab_transitions: RunnerTransitions,
    pub plans: RunnerPlans,
    pub state_predicates: Vec<Variable>,

    // temporary!
    pub goals: Vec<IfThen>,
    pub hl_goals: Vec<IfThen>,

    // temporary!
    pub model: TransitionSystemModel,
    pub op_model: TransitionSystemModel,

    // temporary!
    pub op_states: Vec<Variable>,
    pub hl_op_states: Vec<Variable>,
}

impl RunnerModel {
    pub fn upd_state_paths(&mut self, state: &SPState) {
        // TODO. Add all other actions, preds etc...
        self.op_transitions
            .ctrl
            .iter_mut()
            .for_each(|t| t.upd_state_path(state));
        self.op_transitions
            .un_ctrl
            .iter_mut()
            .for_each(|t| t.upd_state_path(state));
        self.ab_transitions
            .ctrl
            .iter_mut()
            .for_each(|t| t.upd_state_path(state));
        self.ab_transitions
            .un_ctrl
            .iter_mut()
            .for_each(|t| t.upd_state_path(state));
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct AbPlanItem {
    pub transition: SPPath,
    pub guard: Predicate,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerPlans {
    pub op_plan: Vec<SPPath>,
    pub ab_plan: Vec<AbPlanItem>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerTransitions {
    pub ctrl: Vec<Transition>,
    pub un_ctrl: Vec<Transition>,
}

impl RunnerTransitions {
    pub fn extend(&mut self, other: RunnerTransitions) {
        self.ctrl.extend(other.ctrl);
        self.un_ctrl.extend(other.un_ctrl)
    }
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum PlannerResult {
    OpPlan(Vec<SPPath>),
    AbPlan(Vec<SPPath>),
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerInfo {
    pub state: SPState,
    pub plans: Vec<PlanningInfo>,
    pub mode: String,
    pub forced_state: SPState,
    pub forced_goal: Vec<ForcedGoal>,
    pub model: ModelInfo,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct PlanningInfo {
    pub level: String,
    pub plan: Vec<SPPath>,
    pub goal: String,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct ModelInfo {
    operations: Vec<SPPath>,
    resources: Vec<ResourceInfo>,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct ResourceInfo {
    path: SPPath,
    variables: Vec<VariableInfo>,
    abilities: Vec<AbilityInfo>
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct VariableInfo {
    path: SPPath,
    type_: String,
    value_type: String,
    domain: Vec<SPValue>,
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct AbilityInfo {
    path: SPPath,
    predicates: Vec<SPPath>,
    transitions: Vec<TransitionInfo>
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct TransitionInfo {
    path: SPPath,
    guard: String,
    actions: Vec<String>,
    effects: Vec<String>,
    controllable: bool,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum RunnerCommand {
    Mode(String),
    ForceState(SPState),
    SetState(SPState),
    ForceGoal(ForcedGoal),
}
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct ForcedGoal {
    level: String,
    goal: Predicate,
}
