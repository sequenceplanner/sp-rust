use sp_domain::*;
use std::time::{Duration, Instant};
use std::collections::HashSet;
use crate::planning::*;
use sp_smt::*;

pub struct Z3Planner {}

impl Planner for Z3Planner {
    fn plan(model: &TransitionSystemModel,
        goals: &[(Predicate, Option<Predicate>)],
        state: &SPState,
        max_steps: u32) -> PlanningResult {
        
        let result = SeqComputePlanSPModelZ3::plan(&model, goals, &state, max_steps);
        let trace_new: Vec<PlanningFrame> = result.trace.iter().map(|x| PlanningFrame {
            state: x.state.clone(),
            transition: x.transition.clone()
        }).collect();

        PlanningResult {
            plan_found: result.plan_found,
            // plan_length: nr_steps,
            plan_length: result.plan_length,
            // trace: result.trace as Vec<PlanningFrame>,
            trace: trace_new,
            time_to_solve: result.time_to_solve,
            raw_output: "".to_string(),
            raw_error_output: "".to_string(),
        }
    }
}