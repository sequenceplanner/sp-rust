use serde::{Deserialize, Serialize};
use sp_domain::*;

pub fn plan(
    model: &TransitionSystemModel, goals: &[(Predicate, Option<Predicate>)], state: &SPState,
    max_steps: u32,
) -> PlanningResult {
    // if we have an invariant for our goal, express it as inv U (inv
    // & goal) e.g. we make sure that the invariant also holds in the
    // post state. consider for example the two robots that cannot be
    // at the table at the same time. reaching a goal that they should
    // both at the table shouldn't make us ignore the
    // invariants. instead we don't want a plan to be found.
    let goals: Vec<_> = goals
        .iter()
        .map(|(g, i)| {
            if let Some(invar) = i {
                (Predicate::AND(vec![g.clone(), invar.clone()]), i.clone())
            } else {
                (g.clone(), i.clone())
            }
        })
        .collect();

    let result = NuXmvPlanner::plan(model, &goals, state, max_steps);
    // let result2 = SatPlanner::plan(model, &goals, state, max_steps);


    // for f in &result.trace {
    //     println!("==========================");
    //     println!("{}", f.transition);
    //     println!("==========================");
    //     println!("{}", f.state);

    // }

    // for f in &result2.trace {
    //     println!("==========================");
    //     println!("{}", f.transition);
    //     println!("==========================");
    //     println!("{}", f.state);

    // }

    // assert_eq!(result.plan_found, result2.plan_found);

    if result.plan_found {
        // assert_eq!(result.plan_length, result2.plan_length);
        println!("we have a plan of length {}", result.plan_length);
        println!("nuxmv time: {}ms", result.time_to_solve.as_millis());
        // println!("satplanner time: {}ms", result2.time_to_solve.as_millis());
    }

    result
}

/// Return a plan that blocks all deliberation transitions.
pub fn block_all(model: &TransitionSystemModel) -> Vec<TransitionSpec> {
    model
        .transitions
        .iter()
        .filter_map(|t: &Transition| {
            if t.controlled() {
                Some(t.path().clone())
            } else {
                None
            }
        })
        .map(|p| {
            let t = Transition::new(
                &format!("Blocked {}", p),
                Predicate::FALSE,
                vec![],
                vec![],
                true,
            );
            TransitionSpec::new(&format!("Blocked {}", p), t, vec![p.clone()])
        })
        .collect()
}

pub fn convert_planning_result(
    model: &TransitionSystemModel, res: &PlanningResult, plan_counter: &SPPath,
) -> (Vec<TransitionSpec>, SPState) {
    if !res.plan_found {
        return (block_all(model), SPState::new());
    }
    let ctrl: Vec<SPPath> = model
        .transitions
        .iter()
        .filter_map(|t: &Transition| {
            if t.controlled() {
                Some(t.path().clone())
            } else {
                None
            }
        })
        .collect();
    let in_plan: Vec<SPPath> = res.trace.iter().map(|x| x.transition.clone()).collect();
    let mut tr = vec![];
    let mut i = 0;

    // trace[0] is always the initial state.
    let mut cur_state: &SPState = &res.trace[0].state;
    let mut last_ctrl_state: Option<&SPState> = None;
    for pf in res.trace.iter().skip(1) {
        if ctrl.contains(&pf.transition) {
            let mut pred = Vec::new();
            if let Some(last_ctrl_state) = &last_ctrl_state {
                let diff = last_ctrl_state.difference(cur_state);
                let preds: Vec<Predicate> = diff
                    .projection()
                    .sorted()
                    .state
                    .iter()
                    .map(|(p, v)| {
                        Predicate::EQ(
                            PredicateValue::path((*p).clone()),
                            PredicateValue::value(v.value().clone()),
                        )
                    })
                    .collect();
                pred.extend(preds);
            }
            pred.push(p!(p: plan_counter == i));
            println!("Transition: {:?} {}", i, pf.transition);
            let guard = Predicate::AND(pred);
            println!("A new Guard: {}", guard);
            println!("");

            let t = Transition::new(
                &format!("step{:?}", i),
                guard,
                vec![a!(p: plan_counter = { i + 1 })],
                vec![],
                true,
            );

            tr.push(TransitionSpec::new(
                &format!("spec{:?}", i),
                t,
                vec![pf.transition.clone()],
            ));

            last_ctrl_state = Some(cur_state);
            i += 1;
        }
        cur_state = &pf.state;
    }

    let blocked: Vec<TransitionSpec> = ctrl
        .iter()
        .filter(|x| !in_plan.contains(x))
        .map(|p| {
            let t = Transition::new(
                &format!("Blocked {}", p),
                Predicate::FALSE,
                vec![],
                vec![],
                true,
            );
            TransitionSpec::new(&format!("Blocked {}", p), t, vec![p.clone()])
        })
        .collect();

    tr.extend(blocked);

    println!("THE PLAN");
    in_plan.iter().for_each(|x| println!("{}", x));
    println!();

    (tr, SPState::new_from_values(&[(plan_counter.clone(), 0.to_spvalue())]))
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct PlanningResult {
    pub plan_found: bool,
    pub plan_length: u32,
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

pub trait Planner {
    fn plan(
        model: &TransitionSystemModel, goal: &[(Predicate, Option<Predicate>)], state: &SPState,
        max_steps: u32,
    ) -> PlanningResult;
}

mod nuxmv;
pub use nuxmv::*;

mod sat_planner;
pub use sat_planner::*;
