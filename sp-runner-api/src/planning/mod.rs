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
    pub state: StateExternal,
    // The controllable transition taken this frame, if one was taken.
    pub ctrl: Option<SPPath>,
}

mod nuxmv;
pub use nuxmv::*;
