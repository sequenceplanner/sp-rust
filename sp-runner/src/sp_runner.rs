#![allow(dead_code)]

use super::sp_ticker::{RunnerPredicate, SPTicker};
use sp_domain::*;
use sp_runner_api::*;

/// The SPRunner that keep tracks of the state and the transition execution
/// When using the runner, call the input method on every state change and probably
/// also at a specific frequency
#[derive(Debug, PartialEq, Clone)]
pub struct SPRunner {
    pub name: String,
    ticker: SPTicker,
    pub variables: Vec<Variable>,
    pub predicates: Vec<Variable>,
    pub goals: Vec<IfThen>,
    pub global_transition_specs: Vec<TransitionSpec>,
    pub ability_plan: SPPlan,
    pub operation_plan: SPPlan,
    pub last_fired_transitions: Vec<SPPath>,
    pub transition_system_model: TransitionSystemModel,
    pub in_sync: bool,
}

/// The input to the runner.
///
/// Tick is used to trigger one evaluation and execution of the transitions. Use
/// it to run the runner at a frequency.
///
/// TickAsync is used when the runner is in sync mode but youdo not want to
/// do the planning before executing the transition.
///
/// StateChange(upd_variables) is used when the input variables have changed. If
/// they are the same as the internal state, the runner will not execute the transitions.
///
/// Settings will change the internal settings of the runner. Maybe send in goals and specs using this?
///
/// AbilityPlan and OperationPlan will update the plans in the runner. If the runner
/// is in sync-mode, the AbilityPlan will be computed at each tick and the plan written
/// via this input will be overwritten.
#[derive(Debug, PartialEq, Clone)]
pub enum SPRunnerInput {
    Tick,
    StateChange(SPState),
    Settings, // Will come later
    AbilityPlan(SPPlan),
    OperationPlan(SPPlan),
}
#[derive(Debug, PartialEq, Clone, Default)]
pub struct SPPlan {
    //id: usize, // probably use later. Or maybe we should include some kind of timestamp,
    pub plan: Vec<TransitionSpec>, // the plan in the form of transition specification
    pub state_change: SPState,  // for setting variables use in the plans
    //sequence: Vec<(usize, SPPath),  // probably need this when handling unsync planning
}

impl SPRunner {
    /// Creates a new runner
    pub fn new(
        name: &str,
        transitions: Vec<Transition>,
        variables: Vec<Variable>,
        goals: Vec<IfThen>,
        global_transition_specs: Vec<TransitionSpec>,
        forbidden: Vec<IfThen>,
        transition_system_model: TransitionSystemModel,
    ) -> Self {
        let mut vars = vec!();
        let mut preds = vec!();
        let mut initial_state_map = vec!();

        variables.into_iter().for_each(|v| {
            let v: Variable = v;
            initial_state_map.push((v.path().clone(), v.initial_value().clone()));
            match v.variable_type() {
                VariableType::Predicate(_) => preds.push(v),
                _ => vars.push(v),
            }

        });

        let mut state = SPState::new_from_values(&initial_state_map);
        state.add_variable(SPPath::from_slice(&["runner","ability_plan"]), 0.to_spvalue());
        state.add_variable(SPPath::from_slice(&["runner", "operation_plan"]), 0.to_spvalue());
        let runner_predicates: Vec<RunnerPredicate> = preds.iter().flat_map(|p| RunnerPredicate::new(p, &state)).collect();
        let mut ticker = SPTicker::new();
        ticker.state = state;
        ticker.transitions = transitions;
        ticker.forbidden = forbidden;
        ticker.predicates = runner_predicates;
        ticker.reload_state_paths();

        SPRunner {
            name: name.to_string(),
            ticker: ticker,
            variables: vars,
            predicates: preds,
            goals,
            global_transition_specs,
            ability_plan: SPPlan::default(),
            operation_plan: SPPlan::default(),
            last_fired_transitions: vec!(),
            transition_system_model,
            in_sync: false,
        }
    }

    /// The main function to use when running the runner. Connect this to
    /// a channel either using async or standard threads
    pub fn input(&mut self, input: SPRunnerInput) {
        match input {
            SPRunnerInput::Tick => {
                self.take_a_tick(SPState::new());
            },
            SPRunnerInput::StateChange(s) => {
                self.take_a_tick(s);
            },
            SPRunnerInput::Settings => {}, // Will come later
            SPRunnerInput::AbilityPlan(plan) => {
                self.ability_plan = plan;
                self.update_state_variables(self.ability_plan.state_change.clone());
                self.load_plans();
            },
            SPRunnerInput::OperationPlan(plan) => {
                self.operation_plan = plan;
                self.update_state_variables(self.operation_plan.state_change.clone());
                self.load_plans();
            },
        }
    }

    /// Get the current state from the runner
    pub fn state(&self) -> &SPState {
        &self.ticker.state
    }

    /// Get the current goal and respective invariant that runner
    /// tries to reach.
    pub fn goal(&self) -> Vec<(Predicate, Option<Predicate>)> {
        self.goals.iter().filter(|g| {
            g.condition.eval(&self.ticker.state)
        }).map(|x| (x.goal.clone(), x.invariant.clone())).collect()
    }

    /// A special function that the owner of the runner can use to
    /// add new state variables
    pub fn update_state_variables(&mut self, update: SPState) {
        let state_id = self.ticker.state.id();
        self.ticker.state.extend(update);
        if state_id != self.ticker.state.id() {
            // changed_variables have added a new variabel
            self.reload_state_paths();
        }
    }

    /// A special function that the owner of the runner can use to
    /// force a new state. Must include all variables used
    /// by the runner!
    pub fn force_new_state(&mut self, update: SPState) {
        self.ticker.state = update;
        self.reload_state_paths();
    }


    fn take_a_tick(&mut self, state: SPState) {
        self.update_state_variables(state);

        // do nothing if we are in a (globally) bad state.
        // these exprs can be pretty big. do some benchmark here.
        let good = self.transition_system_model.specs.iter()
            .fold(true, |x, s| x && s.invariant().eval(&self.ticker.state));

        if !good {
            println!("\nDOING NOTHING: WE ARE IN A BAD STATE.\n");
            return;
        }

        let res = self.ticker.tick_transitions();
        self.last_fired_transitions = res.1;
    }

    fn load_plans(&mut self) {
        self.reload_state_paths_plans();
        self.ticker.specs = self.ability_plan.plan.clone();
        self.ticker.specs.extend(self.ability_plan.plan.clone());
        self.ticker.specs.extend(self.global_transition_specs.clone());
    }

    fn reload_state_paths(&mut self) {
        println!("RELOAD STATE PATHS");
        self.ticker.reload_state_paths();
        self.reload_state_paths_plans();
        for x in self.goals.iter_mut() {
            x.upd_state_path(&self.ticker.state)
        }
        for x in self.global_transition_specs.iter_mut() {
            x.spec_transition.upd_state_path(&self.ticker.state)
        }
    }
    fn reload_state_paths_plans(&mut self) {
        for x in self.ability_plan.plan.iter_mut() {
            x.spec_transition.upd_state_path(&self.ticker.state)
        }
        for x in self.operation_plan.plan.iter_mut() {
            x.spec_transition.upd_state_path(&self.ticker.state)
        }
    }

}


#[cfg(test)]
mod test_new_runner {
    use super::*;
    use crate::planning::*;

    #[test]
    fn testing_me() {
        let mut runner = make_dummy_robot_runner();


        println!{""};
        println!("Initial goal: {:?}", runner.goal());
        println!{""};
        println!("Initial State: {}", runner.state());
        println!{""};
        println!("Initial plan");
        runner.ability_plan.plan.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("Transitions plan");
        runner.ticker.transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        runner.input(SPRunnerInput::Tick);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));


        let planner_result = crate::planning::plan(&runner.transition_system_model, &runner.goal(), &runner.state());
        let (tr, s) = convert_planning_result(&runner.transition_system_model, planner_result);
        let plan = SPPlan{plan: tr, state_change: s};
        runner.input(SPRunnerInput::AbilityPlan(plan));



        runner.input(SPRunnerInput::Tick);
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!("fired:");
        runner.input(SPRunnerInput::Tick);
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!("fired:");
        runner.input(SPRunnerInput::Tick);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        println!("goal:{:?}", runner.goal());
        runner.input(SPRunnerInput::Tick);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        runner.input(SPRunnerInput::Tick);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        runner.input(SPRunnerInput::Tick);
        runner.input(SPRunnerInput::Tick);
        runner.input(SPRunnerInput::Tick);
        runner.input(SPRunnerInput::Tick);
        println!("fired:");
        runner.last_fired_transitions.iter().for_each(|x| println!("{:?}", x));
        println!{""};
        println!("State: {}", runner.state());
        println!{""};
        runner.input(SPRunnerInput::Tick);
    }

    fn make_dummy_robot_runner() -> SPRunner {
        let (model, mut initial_state) = crate::testing::two_dummy_robots();
        let ts_model = TransitionSystemModel::from(&model);
        let m: RunnerModel = crate::helpers::make_runner_model(&model);
        //println!("{:?}", m);
        let mut trans = vec!();
        let mut restrict_controllable = vec!();
        let false_trans = Transition::new("empty", Predicate::FALSE, vec!(), vec!(), true);
        m.op_transitions.ctrl.iter().for_each(|t| {
            trans.push(t.clone());
            // restrict_controllable.push(TransitionSpec::new(
            //     &format!("s_{}_false", t.name()),
            //     false_trans.clone(),
            //     vec!(t.path().clone())
            // ))
        });
        m.op_transitions.un_ctrl.iter().for_each(|t| {
            trans.push(t.clone());
        });
        m.ab_transitions.ctrl.iter().for_each(|t| {
            trans.push(t.clone());
            restrict_controllable.push(TransitionSpec::new(
                &format!("s_{}_false", t.path()),
                false_trans.clone(),
                vec!(t.path().clone())
            ))
        });
        m.ab_transitions.un_ctrl.iter().for_each(|t| {
            trans.push(t.clone());
        });
        let mut runner = SPRunner::new(
            "test",
            trans,
            m.state_predicates.clone(),
            m.goals.clone(),
            vec!(),
            vec!(),
            ts_model
        );
        runner.input(SPRunnerInput::AbilityPlan(SPPlan{
            plan: restrict_controllable,
            state_change: SPState::new(),
        }));
        let the_upd_state = state!(
            ["dummy_robot_model", "r1", "State", "act_pos"] => "away",
            ["dummy_robot_model", "r1", "Control", "ref_pos"] => "away",
            ["dummy_robot_model", "r2", "State", "act_pos"] => "away",
            ["dummy_robot_model", "r2", "Control", "ref_pos"] => "away"
        );
        initial_state.extend(the_upd_state);
        runner.update_state_variables(initial_state);
        runner
    }
}

// name: &str,
//         transitions: Vec<Transition>,
//         variables: Vec<Variable>,
//         goals: Vec<IfThen>,
//         global_transition_specs: Vec<TransitionSpec>,
//         forbidden: Vec<IfThen>,
//         old_model: RunnerModel,

// pub struct RunnerModel {
//     pub op_transitions: RunnerTransitions,
//     pub ab_transitions: RunnerTransitions,
//     pub plans: RunnerPlans,
//     pub state_predicates: Vec<Variable>,
//     pub goals: Vec<IfThen>,
//     pub invariants: Vec<IfThen>,
//     pub model: Model,
// }
