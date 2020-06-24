use sp_domain::*;
use std::collections::HashSet;
use super::*;

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
    let mut last_ctrl_state: Option<&SPState> = Some(cur_state);
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
    model: &TransitionSystemModel, res: &PlanningResult, plan_counter: &SPPath
) -> (Vec<TransitionSpec>, SPState) {
    if !res.plan_found {
        return (block_all(model), SPState::new());
    }
    let in_plan: Vec<SPPath> = res.trace.iter().map(|x| x.transition.clone()).collect();
    let mut tr = vec![];

    // which variables are touched?
    let mut touches = Vec::new();
    touches.push(HashSet::new());
    for x in 1..res.trace.len() {
        let current = &res.trace[x];
        let cur_path = &current.transition;
        let cur_transition = model.transitions.iter().find(|t| t.path() == cur_path).unwrap();
        let mut cur_touches: HashSet<SPPath> = HashSet::new();
        cur_touches.extend(cur_transition.modifies());
        cur_touches.extend(cur_transition.guard().support().iter().cloned());
        touches.push(cur_touches);
    }

    let mut highest = 0;
    let mut lowest = 0; // for independents...
    for x in 1..res.trace.len() {
        let current = &res.trace[x];
        let state = &res.trace[x-1].state;

        println!("Transition {}: {}", x, current.transition);

        let mut last_intersection_point = 0;
        let mut last_intersection = Vec::new();
        for y in 0..x {
            let cur_path = &current.transition;
            let cur_transition = model.transitions.iter().find(|t| t.path() == cur_path).unwrap();
            let mut cur_touches: HashSet<SPPath> = HashSet::new();
            cur_touches.extend(cur_transition.modifies());
            cur_touches.extend(cur_transition.guard().support().iter().cloned());

            let intersection: Vec<_> = cur_touches.intersection(&touches[y]).cloned().collect();
            if intersection.len() > 0 {
                intersection.iter().for_each(|p| println!("{} intersects with {} at index {}", cur_path, p, y));
                last_intersection_point = y;
                last_intersection = intersection;
            }
        }

        let mut pred: Vec<Predicate> = Vec::new();
        let (guard, action) = if last_intersection_point > 0 {
            let diff = &res.trace[last_intersection_point-1].state.difference(state);
            let guards: Vec<Predicate> = diff
                .projection()
                .sorted()
                .state
                .iter()
                .filter(|(p, _)| last_intersection.contains(p))
                .map(|(p, v)| {
                    Predicate::EQ(
                        PredicateValue::path((*p).clone()),
                        PredicateValue::value(v.value().clone()),
                    )
                })
                .collect();
            pred.extend(guards);
            pred.push(p!(p: plan_counter == highest));
            let action = if last_intersection_point > highest { // previous state
                highest = last_intersection_point;
                vec![a!(p: plan_counter = highest)]
            } else { vec![] };

            let guard = Predicate::AND(pred);

            (guard, action)
        } else {
            lowest -= 1;
            let action = vec![a!(p: plan_counter = (lowest + 1))];
            let guard = p!(p: plan_counter == lowest);
            (guard, action)
        };

        let t = Transition::new(
            &format!("step{:?}", x),
            guard,
            action,
            vec![],
            true,
        );

        println!("A new spec: {}", t);

        tr.push(TransitionSpec::new(
            &format!("spec{:?}", x),
            t,
            vec![current.transition.clone()],
        ));
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

    (tr, SPState::new_from_values(&[(plan_counter.clone(), lowest.to_spvalue())]))
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

/// Checks wheter we can reach a goal exactly applying
/// a list of transitions in their given order
fn check_goals_exact(s: &SPState, goals: &[&Predicate], plan: &[SPPath], ts_model: &TransitionSystemModel) -> bool {
    if goals.iter().all(|g| g.eval(s)) {
        return true;
    }

    let trans: Vec<&Transition> = ts_model.transitions.iter()
        .filter(|t| plan.contains(t.path()))
        .collect();

    let mut state = s.clone();
    for p in plan {
        let t = trans.iter().find(|t| t.path() == p).unwrap();
        if !t.eval(&state) {
            return false;
        }

        // take all actions
        t.actions.iter().for_each(|a| {
            let _res = a.next(&mut state);
        });
        // and effects
        t.effects.iter().for_each(|e| {
            let _res = e.next(&mut state);
        });

        // update state predicates
        ts_model.state_predicates.iter().for_each(|sp| {
            if let VariableType::Predicate(p) = sp.variable_type() {
                let value = p.eval(&state).to_spvalue();
                if let Err(e) = state.force_from_path(sp.path(), value) {
                    eprintln!(
                        "The predicate {:?} could not be updated in the runner. Got error: {}",
                        sp.path(), e
                    );
                }
            }
        });

        // next -> cur
        let _changed = state.take_transition();

        if ts_model.bad_state(&state) {
            return false;
        }

        if goals.iter().all(|g| g.eval(&state)) {
            return true;
        }
    }

    return false;
}


pub fn bubble_up_delibs(ts: &TransitionSystemModel,
                        goals: &[&Predicate],
                        pr: &mut PlanningResult) {
    if pr.plan_length <= 2 {
        // check length > 2 because for the heuristic
        // to improve the situation sense we need at
        // least two deliberation trans + 1 other
        return;
    }

    let state = &pr.trace[0].state;

    let plan = pr.trace.iter()
        .filter(|f| f.transition != SPPath::new())
        .flat_map(|f| ts.transitions.iter().find(|t| t.path() == &f.transition))
        .collect::<Vec<_>>();

    let delib = plan.iter().filter(|t| t.controlled()).collect::<Vec<_>>();

    println!("original plan");
    for t in &plan {
        println!("{}", t.path());
    }

    //let mut new_plans = Vec::new();
    let mut new_plan = plan.clone();
    let mut idx = 1; // start at 2
    let mut done_paths = HashSet::new();
    if delib.contains(&&new_plan[0]) {
        done_paths.insert(new_plan[0].path().clone());
    }
    loop {
        if idx >= new_plan.len() {
            println!("done...");
            break;
        }

        if !delib.contains(&&new_plan[idx]) {
            println!("not moving {} to {}, this is not delib", new_plan[idx].path(), idx - 1);
            idx += 1;
            continue;
        }

        if done_paths.contains(new_plan[idx].path()) {
            println!("not moving {} to {}, it is already been moved enough",
                     new_plan[idx].path(), idx - 1);
            idx += 1;
            continue;
        }

        // else try to bubble it up.
        let mut p = new_plan.clone();
        let t = p.remove(idx);
        let path = t.path().clone();

        println!("moving {} to {}", path, idx - 1);
        p.insert(idx - 1, t);

        // check if the new plan successfully reaches the goals
        let plan: Vec<SPPath> = p.iter().map(|t|t.path().clone()).collect();

        let result = check_goals_exact(state, goals, &plan, &ts);
        println!("plan takes us to goal? {}", result);

        // transition is done if we could not move it, or we moved it
        // to the top
        if !result || (result && idx == 1) {
            done_paths.insert(path);
        }

        if result {
            new_plan = p;
            // can only move up to zero
            if idx > 1 {
                idx-=1;
            }
        } else {
            idx+=1;
        }
    }

    println!("Final plan after heuristic");
    for t in &new_plan {
        println!("{}", t.path());
    }
    println!("--------");


    // change original planner result
    let plan: Vec<SPPath> = new_plan.iter().map(|t|t.path().clone()).collect();
    let trace = make_planning_trace(&ts, &plan, &pr.trace[0].state);
    pr.trace = trace;
}
