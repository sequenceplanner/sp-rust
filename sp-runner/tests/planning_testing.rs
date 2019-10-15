use sp_domain::*;
use sp_runner::*;

#[test]
fn plan_fail_1_step() {
    let (model, state) = one_robot();
    let state = state.external();

    let activated = model
        .model
        .find_item("data", &["activated"])
        .unwrap_global_path();
    let goal = p!(activated);

    // requires at least step = 2 to find a plan
    let result = compute_plan(&goal, &state, &model, 1);

    assert!(!result.plan_found);

    assert_ne!(
        result
            .trace
            .last()
            .and_then(|f| f.state.s.get(&SPPath::GlobalPath(activated))),
        Some(&true.to_spvalue())
    );
}

#[test]
fn plan_success_2_steps() {
    let (model, state) = one_robot();
    let state = state.external();

    let activated = model
        .model
        .find_item("data", &["activated"])
        .unwrap_global_path();
    let goal = p!(activated);

    // requires at least step = 2 to find a plan
    let result = compute_plan(&goal, &state, &model, 2);

    assert!(result.plan_found);

    assert_eq!(
        result
            .trace
            .last()
            .and_then(|f| f.state.s.get(&SPPath::GlobalPath(activated))),
        Some(&true.to_spvalue())
    );
}

#[test]
fn planner_debug_printouts() {
    let (model, state) = one_robot();
    let state = state.external();

    let activated = model
        .model
        .find_item("data", &["activated"])
        .unwrap_global_path();
    let goal = p!(activated);

    let result = compute_plan(&goal, &state, &model, 20);

    println!("INITIAL STATE");
    println!("{}", state);
    println!("GOAL PREDICATE: {:?}", goal);
    println!("-------\n");

    println!("TIME TO SOLVE: {}ms", result.time_to_solve.as_millis());

    if !result.plan_found {
        println!("STDOUT\n---------\n{}\n", result.raw_output);
        println!("STDERR\n---------\n{}\n", result.raw_error_output);
        return;
    }
    println!("RESULTING PLAN");

    for (i, f) in result.trace.iter().enumerate() {
        println!("FRAME ID: {}\n{}", i, f.state);
        println!("ACTION: {:?}", f.ctrl);
        println!("-------");
    }
}
