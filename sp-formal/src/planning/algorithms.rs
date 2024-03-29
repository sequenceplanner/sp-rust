use super::*;
use std::collections::HashSet;

/// Return a plan that blocks all deliberation transitions.
pub fn block_all(model: &TransitionSystemModel) -> Vec<TransitionSpec> {
    model
        .transitions
        .iter()
        .filter_map(|t: &Transition| {
            if t.type_ == TransitionType::Controlled {
                Some(t.path().clone())
            } else {
                None
            }
        })
        .map(|p| {
            let t = Transition::new(
                &format!("Blocked {}", p),
                Predicate::FALSE,
                Predicate::FALSE,
                vec![],
                vec![],
                TransitionType::Controlled,
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
            if t.type_ == TransitionType::Controlled {
                Some(t.path().clone())
            } else {
                None
            }
        })
        .collect();
    let in_plan: Vec<SPPath> = res.trace.iter().map(|x| x.transition.clone()).collect();
    let mut tr = vec![];
    let mut i = 0;

    let mut plan_visualization = vec![];

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

            // for transitions that change the value of their parameters, we
            // need to also include setting that value as an action.
            let taken_t = model
                .transitions
                .iter()
                .find(|c| c.path() == &pf.transition)
                .expect("Model does not match plan");
            let anys: Vec<_> = taken_t
                .actions
                .iter()
                .filter_map(|a| {
                    if a.value == Compute::Any {
                        // look up what the planner set the value to
                        let val = pf
                            .state
                            .sp_value_from_path(&a.var)
                            .expect("Missing state in plan");
                        let pv = PredicateValue::SPValue(val.clone());
                        Some(Action::new(a.var.clone(), Compute::PredicateValue(pv)))
                    } else {
                        None
                    }
                })
                .collect();
            let mut actions = vec![a!(p: plan_counter = { i + 1 })];
            actions.extend(anys.clone());

            let t = Transition::new(
                &format!("step{:?}", i),
                guard,
                Predicate::TRUE,
                actions,
                vec![],
                TransitionType::Controlled,
            );

            tr.push(TransitionSpec::new(
                &format!("spec{:?}", i),
                t,
                vec![pf.transition.clone()],
            ));

            last_ctrl_state = Some(cur_state);
            i += 1;

            // add counter+transition for ui purposes. we do it after incrementing
            // to make it easier to see where in the plan we are as we increment
            // it as soon as an operation is started.
            let np = plan_counter.clone().add_child(&format!("{:0>2}", i));
            let val = if anys.is_empty() {
                pf.transition.clone().to_string().to_spvalue()
            } else {
                let any_str = anys
                    .iter()
                    .map(|a| a.to_string_short())
                    .collect::<Vec<_>>()
                    .join(",");
                format!("{} with {}", pf.transition, any_str).to_spvalue()
            };
            plan_visualization.push((np, val));
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
                Predicate::FALSE,
                vec![],
                vec![],
                TransitionType::Controlled,
            );
            TransitionSpec::new(&format!("Blocked {}", p), t, vec![p.clone()])
        })
        .collect();

    tr.extend(blocked);

    println!("THE PLAN");
    in_plan.iter().for_each(|x| println!("{}", x));
    println!();

    let mut new_state = plan_visualization;
    new_state.push((plan_counter.clone(), 0.to_spvalue()));

    (tr, SPState::new_from_values(new_state.as_slice()))
}

pub fn convert_planning_result_with_packing_heuristic(
    model: &TransitionSystemModel, res: &PlanningResult, plan_counter: &SPPath,
) -> (Vec<TransitionSpec>, SPState) {
    if !res.plan_found {
        return (block_all(model), SPState::new());
    }
    let in_plan: Vec<SPPath> = res.trace.iter().map(|x| x.transition.clone()).collect();
    let ctrl: Vec<SPPath> = model
        .transitions
        .iter()
        .filter_map(|t: &Transition| {
            if t.type_ == TransitionType::Controlled {
                Some(t.path().clone())
            } else {
                None
            }
        })
        .collect();

    let mut tr = vec![];

    // which variables are touched by which transitions?
    let mut touches = Vec::new();
    touches.push(HashSet::new());
    for x in 1..res.trace.len() {
        let current = &res.trace[x];
        let cur_path = &current.transition;
        if let Some(cur_transition) = model
            .transitions
            .iter()
            .find(|t| t.path() == cur_path)
            .as_ref()
        {
            let mut cur_touches: HashSet<SPPath> = HashSet::new();
            cur_touches.extend(cur_transition.modifies());
            cur_touches.extend(cur_transition.guard().support().iter().cloned());
            touches.push(cur_touches);
        } else {
            touches.push(HashSet::new());
        }
    }

    let mut starts = Vec::new();

    for x in 1..res.trace.len() {
        let current = &res.trace[x];
        let cur_touches = &touches[x];
        println!("Transition {}: {}", x, current.transition);

        // find all intersections between current transition and previous transitions in the plan
        let mut highest = 0;
        let mut all_intersections = HashSet::new();
        for y in 0..x {
            let intersection = cur_touches
                .intersection(&touches[y])
                .collect::<HashSet<_>>();
            if intersection.len() > 0 {
                highest = y;
                println!("intersects with {} {}", y, &res.trace[y].transition);
            }
            all_intersections.extend(intersection);
        }
        all_intersections.iter().for_each(|i| println!("  {}", i));
        starts.push((x, highest, all_intersections));
    }

    starts.sort_by(|&(x0, y0, _), &(x1, y1, _)| y0.cmp(&y1).then(x0.cmp(&x1)));

    let mut plan_visualization = vec![];

    let mut counter = 0;
    for (idx, high_dep, intersections) in starts {
        let current = &res.trace[idx];
        if !ctrl.contains(&current.transition) {
            continue;
        }
        let gc = p!(p: plan_counter == counter);
        let action = vec![a!(p: plan_counter = (counter + 1))];

        let guard = if high_dep > 0 {
            // make guard
            let cur_state = &res.trace[idx - 1].state;

            let mut guards: Vec<Predicate> = cur_state
                .projection()
                .sorted()
                .state
                .iter()
                .filter(|(p, _)| intersections.contains(p))
                .filter(|(p, _)| p.path.contains(&"product_state".to_string())) // hack...
                .map(|(p, v)| {
                    Predicate::EQ(
                        PredicateValue::path((*p).clone()),
                        PredicateValue::value(v.value().clone()),
                    )
                })
                .collect();
            guards.push(gc.clone());
            Predicate::AND(guards)
        } else {
            gc
        };

        println!();
        println!(
            "{} got a new spec: {}/{}",
            res.trace[idx].transition,
            guard,
            action
                .iter()
                .map(|a| a.to_string())
                .collect::<Vec<_>>()
                .join(",")
        );

        let st = Transition::new(
            &format!("hlstep{:?}", idx),
            guard,
            Predicate::TRUE,
            action,
            vec![],
            TransitionType::Controlled,
        );

        tr.push(TransitionSpec::new(
            &format!("spec{:?}", counter),
            st,
            vec![res.trace[idx].transition.clone()],
        ));

        counter += 1;

        // add counter+transition for ui purposes. we do it after incrementing
        // to make it easier to see where in the plan we are as we increment
        // it as soon as an operation is started.
        let np = plan_counter.clone().add_child(&format!("{:0>2}", counter));
        let val = res.trace[idx].transition.clone().to_string().to_spvalue();
        plan_visualization.push((np, val));
    }

    let blocked: Vec<TransitionSpec> = model
        .transitions
        .iter()
        .filter(|t| t.type_ == TransitionType::Controlled && !in_plan.contains(t.path()))
        .map(|x| {
            let t = Transition::new(
                &format!("Blocked {}", x.path()),
                Predicate::FALSE,
                Predicate::FALSE,
                vec![],
                vec![],
                TransitionType::Controlled,
            );
            TransitionSpec::new(&format!("Blocked {}", x.path()), t, vec![x.path().clone()])
        })
        .collect();

    tr.extend(blocked);

    println!("THE PLAN");
    in_plan.iter().for_each(|x| println!("{}", x));
    println!();

    let mut new_state = plan_visualization;
    new_state.push((plan_counter.clone(), 0.to_spvalue()));

    (tr, SPState::new_from_values(new_state.as_slice()))
}

// create a planning result based on an initial state and a sequence
// of transitions. this is to make it easy to try to rearrange plans.
fn rebuild_planning_trace(plan: &[Transition], initial: &SPState) -> Vec<PlanningFrame> {
    let mut state = initial.clone();
    let mut result = Vec::new();

    let frame = PlanningFrame {
        state: state.clone(),
        transition: SPPath::new(),
    };
    result.push(frame);

    for t in plan {
        t.actions.iter().for_each(|a| {
            let result = a.next(&mut state);
            if result.is_err() {
                // when you have a double assign in the actions
                println!("WARNING: could not take transition for {} ({:?})", t.path(), result);
            }
        });
        state.take_transition();

        let frame = PlanningFrame {
            state: state.clone(),
            transition: t.path().clone(),
        };

        result.push(frame);
    }
    result
}

/// Checks wheter we can reach a goal exactly applying
/// a list of transitions in their given order
fn check_goals_exact(
    s: &SPState, goals: &[&Predicate], plan: &[Transition], ts_model: &TransitionSystemModel,
) -> bool {
    if goals.iter().all(|g| g.eval(s)) {
        return true;
    }

    let mut state = s.clone();
    for t in plan {
        if !t.guard.eval(&state) {
            return false;
        }

        // take all actions
        t.actions.iter().for_each(|a| {
            let _res = a.next(&mut state);
        });

        // update state predicates
        ts_model.state_predicates.iter().for_each(|sp| {
            if let VariableType::Predicate(p) = sp.variable_type() {
                let value = p.eval(&state).to_spvalue();
                if let Err(e) = state.force_from_path(sp.path(), &value) {
                    eprintln!(
                        "The predicate {:?} could not be updated in the check_goals_exact. Got error: {}",
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

pub fn bubble_up_delibs(ts: &TransitionSystemModel, goals: &[&Predicate], pr: &mut PlanningResult) {
    if pr.plan_length <= 2 {
        // check length > 2 because for the heuristic
        // to improve the situation sense we need at
        // least two deliberation trans + 1 other
        return;
    }

    let state = &pr.trace[0].state;

    let plan = pr
        .trace
        .iter()
        .filter(|f| f.transition != SPPath::new())
        .map(|f| {
            let mut t = ts
                .transitions
                .iter()
                .find(|t| t.path() == &f.transition)
                .expect("Model does not match plan")
                .clone();
            // now we also need to set any free variables, add them as actions to the transition
            let anys = t
                .actions
                .iter()
                .filter_map(|a| {
                    if a.value == Compute::Any {
                        // look up what the planner set the value to
                        let val = f
                            .state
                            .sp_value_from_path(&a.var)
                            .expect("Missing state in plan");
                        let pv = PredicateValue::SPValue(val.clone());
                        let a = Action::new(a.var.clone(), Compute::PredicateValue(pv));
                        println!("UPDATED TRANS {} WITH PARAMETER SELECTION {}", t.path(), a);
                        Some(a)
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>();
            t.actions.extend(anys);
            t
        })
        .collect::<Vec<_>>();

    let delib = plan
        .iter()
        .filter(|t| t.type_ != TransitionType::Effect)
        .collect::<Vec<_>>();

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
            println!(
                "not moving {} to {}, this is not delib",
                new_plan[idx].path(),
                idx - 1
            );
            idx += 1;
            continue;
        }

        if done_paths.contains(new_plan[idx].path()) {
            println!(
                "not moving {} to {}, it is already been moved enough",
                new_plan[idx].path(),
                idx - 1
            );
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
        let result = check_goals_exact(state, goals, &p, &ts);
        // if the previous transition is auto, the check will fail,
        // but it makes sense to keep searching. see below.
        println!("plan takes us to goal? {}", result);

        // transition is done if we could not move it, or we moved it
        // to the top
        if (!result) || (result && idx == 1) {
            done_paths.insert(path);
        }

        if result {
            new_plan = p;
            // can only move up to zero
            if idx > 1 {
                idx -= 1;
            }
        } else {
            idx += 1;
        }
    }

    println!("Final plan after heuristic");
    for t in &new_plan {
        println!("{}", t.path());
    }
    println!("--------");

    // change original planner result
    let trace = rebuild_planning_trace(&new_plan, &pr.trace[0].state);
    pr.trace = trace;
}
