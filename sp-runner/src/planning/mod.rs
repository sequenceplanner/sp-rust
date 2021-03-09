use serde::{Deserialize, Serialize};
use sp_domain::*;
use std::collections::HashMap;

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

#[cfg(feature = "sat_planner")]
mod sat_planner;
#[cfg(feature = "sat_planner")]
pub use sat_planner::*;

mod algorithms;
pub use algorithms::*;

use crate::formal_model::*;

#[derive(Clone, Debug, Hash, Eq, PartialEq, Serialize, Deserialize)]
struct PlannerRequestKey {
    goal: String,
    state: String,
}

#[derive(Clone, Debug, Default)]
pub struct PlanningStore {
    cache: HashMap<PlannerRequestKey, PlanningResult>,
    hits: i64,
    lookups: i64,
}

pub fn plan_with_cache(
    model: &TransitionSystemModel, goals: &[(Predicate, Option<Predicate>)], state: &SPState,
    max_steps: u32, store: &mut PlanningStore,
) -> PlanningResult {
    let now = std::time::Instant::now();
    // filter the state based on the ts model and serialize it to make it hashable
    let state_str = state
        .projection()
        .sorted()
        .state
        .iter()
        .filter(|(k, _)| model.vars.iter().any(|v| &v.path() == k))
        .map(|(k, v)| {
            let s = format!("{}{}", k, serde_json::to_string(v.value()).unwrap());
            s
        })
        .fold("".to_string(), |acum, s| format!("{}{}", acum, s));

    // serialize goals
    let goal_str = goals
        .iter()
        .map(|(g, i)| {
            let i = if let Some(i) = i {
                i.to_string()
            } else {
                "".to_string()
            };
            format!("{}+{}", g, i)
        })
        .collect::<Vec<_>>()
        .join("");
    let key = PlannerRequestKey {
        goal: goal_str,
        state: state_str,
    };

    store.lookups += 1;
    if let Some(plan) = store.cache.get(&key) {
        store.hits += 1;
        println!(
            "Used cached plan! Current plan count {}, hit% {}, lookup time {} ms",
            store.cache.len(),
            ((100 * store.hits) / store.lookups),
            now.elapsed().as_millis()
        );
        return plan.clone();
    } else {
        println!(
            "Did not use cached plan! Current plan count {}, hit% {}, lookup time {} ms",
            store.cache.len(),
            ((100 * store.hits) / store.lookups),
            now.elapsed().as_millis()
        );
    }

    let result = NuXmvPlanner::plan(model, &goals, state, max_steps);
    if result.plan_found {
        // assert_eq!(result.plan_length, result2.plan_length);
        println!("plan_result: {} {}", result.plan_length, result.time_to_solve.as_millis());
        println!("we have a plan of length {}", result.plan_length);
        println!("nuxmv time: {}ms", result.time_to_solve.as_millis());
        // println!("satplanner time: {}ms", result2.time_to_solve.as_millis());
    }
    store.cache.insert(key, result.clone());
    println!(
        "Added new state/goal pair to plan store. Current plan count {}",
        store.cache.len()
    );

    result
}

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
        println!("plan_result: {} {}", result.plan_length, result.time_to_solve.as_millis());
        println!("we have a plan of length {}", result.plan_length);
        println!("nuxmv time: {}ms", result.time_to_solve.as_millis());
        // println!("satplanner time: {}ms", result2.time_to_solve.as_millis());
    }

    result
}
