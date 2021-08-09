use sp_domain::*;
use super::sp_runner::*;
use super::operation_planner::*;
use super::transition_planner::*;

#[derive(Debug, Clone)]
pub struct PlanningState {
    pub operation_planner: OperationPlanner,
    pub transition_planner: TransitionPlanner,
}

impl PlanningState {
    pub fn compute_new_plan(&mut self, state: &SPState, disabled_paths: &[SPPath]) -> Option<(usize,SPPlan)> {
        println!("The State:\n{}", state);

        let plan = self.operation_planner.compute_new_plan(state, disabled_paths);
        if plan != None {
            return plan;
        }

        // transition planner
        let plan = self.transition_planner.compute_new_plan(state, disabled_paths);
        return plan;
    }
}
