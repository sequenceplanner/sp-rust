use serde::{Deserialize, Serialize};
use sp_domain::*;
use std::collections::{HashMap,HashSet};

#[derive(Debug, Hash, Eq, PartialEq)]
struct PlannerRequestKey {
    goal: String,
    state: String,
}

#[derive(Debug, Default)]
pub struct PlanningStore {
    cache: HashMap<PlannerRequestKey, PlanningResult>,
    hits: i64,
    lookups: i64,
}

pub fn plan_with_cache(model: &TransitionSystemModel, goals: &[(Predicate, Option<Predicate>)],
                       state: &SPState, max_steps: u32, store: &mut PlanningStore) -> PlanningResult {
    let now = std::time::Instant::now();
    // filter the state based on the ts model and serialize it to make it hashable
    let state_str = state.projection().sorted().state.iter()
        .filter(|(k,_)| model.vars.iter().any(|v|&v.path() == k))
        .map(|(k,v)| {
            let s = format!("{}{}", k,serde_json::to_string(v.value()).unwrap());
            s
        })
        .fold("".to_string(), |acum,s| format!("{}{}", acum,s));

    // serialize goals
    let goal_str = serde_json::to_string(goals).unwrap();
    let key = PlannerRequestKey { goal: goal_str, state: state_str };

    store.lookups += 1;
    if let Some(plan) = store.cache.get(&key) {
        store.hits += 1;
        println!("Used cached plan! Current plan count {}, hit% {}, lookup time {} ms",
                 store.cache.len(), ((100 * store.hits) / store.lookups), now.elapsed().as_millis());
        return plan.clone();
    }

    let result = plan(model, goals, state, max_steps);
    store.cache.insert(key, result.clone());
    println!("Added new state/goal pair to plan store. Current plan count {}", store.cache.len());

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


pub fn convert_planning_result_with_packing_heuristic(
    model: &TransitionSystemModel, res: &PlanningResult, plan_counter: &SPPath,
) -> (Vec<TransitionSpec>, SPState) {
    if !res.plan_found {
        return (block_all(model), SPState::new());
    }
    let in_plan: Vec<SPPath> = res.trace.iter().map(|x| x.transition.clone()).collect();
    let mut tr = vec![];
    let mut i = 0;

    let mut index_touches = HashSet::new();
    let mut last_intersect_state: Option<&SPState> = None;
    let mut cur_state: &SPState = &res.trace[0].state;
    for x in 1..res.trace.len() {
        let current = &res.trace[x];
        let independent = if x < res.trace.len()-1 {
            let next_path = &res.trace[x+1].transition;
            let next_transition = model.transitions.iter().find(|t| t.path() == next_path).unwrap();
            let cur_path = &current.transition;
            let cur_transition = model.transitions.iter().find(|t| t.path() == cur_path).unwrap();

            let mut next_touches = HashSet::new();
            next_touches.extend(next_transition.modifies());
            next_touches.extend(next_transition.guard().support().iter().cloned());

            let mut cur_touches: HashSet<SPPath> = HashSet::new();
            cur_touches.extend(cur_transition.modifies());
            cur_touches.extend(cur_transition.guard().support().iter().cloned());

            index_touches.extend(cur_touches);

            let intersection: Vec<_> = next_touches.intersection(&index_touches).collect();
            if intersection.len() > 0 {
                intersection.iter().for_each(|p| println!(" intersect: {}", p));
            }
            intersection.is_empty()
        } else {
            false
        };

        let mut pred = Vec::new();
        if let Some(last_intersect_state) = &last_intersect_state {
            let diff = last_intersect_state.difference(cur_state);
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

        println!("Transition: {:?} {}", i, current.transition);

        pred.push(p!(p: plan_counter == i));
        let guard = Predicate::AND(pred);
        println!("A new Guard: {}", guard);
        println!("");

        let action = if !independent {
            vec![a!(p: plan_counter = { i + 1 })]
        } else { Vec::new() };
        let t = Transition::new(
            &format!("step{:?}", i),
            guard,
            action,
            vec![],
            true,
        );

        tr.push(TransitionSpec::new(
            &format!("spec{:?}", i),
            t,
            vec![current.transition.clone()],
        ));

        if !independent {
            i += 1;
            index_touches.clear();
            last_intersect_state = Some(cur_state);
            cur_state = &current.state;
        }
    }

    let blocked: Vec<TransitionSpec> = model.transitions
        .iter()
        .filter(|t| !in_plan.contains(t.path()))
        .map(|x| {
            let t = Transition::new(
                &format!("Blocked {}", x.path()),
                Predicate::FALSE,
                vec![],
                vec![],
                true,
            );
            TransitionSpec::new(&format!("Blocked {}", x.path()), t, vec![x.path().clone()])
        })
        .collect();

    tr.extend(blocked);

    println!("THE PLAN");
    in_plan.iter().for_each(|x| println!("{}", x));
    println!();

    (tr, SPState::new_from_values(&[(plan_counter.clone(), 0.to_spvalue())]))
}


// create a planning result based on an initial state and a sequence
// of transitions this is to make it easy to try to rearrange plans.
pub fn make_planning_trace(model: &TransitionSystemModel, plan: &[SPPath], initial: &SPState) -> Vec<PlanningFrame> {
    let mut state = initial.clone();
    let mut result = Vec::new();

    let frame = PlanningFrame {
        state: state.clone(),
        transition: SPPath::new(),
    };
    result.push(frame);

    for pt in plan {
        let t = model.transitions.iter().find(|c| c.path() == pt).expect("Model does not match plan");
        t.actions.iter().for_each(|a| {
            a.next(&mut state).unwrap();
        });
        t.effects.iter().for_each(|a| {
            a.next(&mut state).unwrap();
        });
        state.take_transition();

        let frame = PlanningFrame {
            state: state.clone(),
            transition: pt.clone(),
        };

        result.push(frame);
    }
    result
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
