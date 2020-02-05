use sp_domain::*;
use sp_runner::*;
use sp_runner_api::*;


#[test]
fn planner_fail_1_step() {
    let (model, state) = one_dummy_robot();

    let activated = model
        .model
        .find_item("active", &[])
        .expect("check spelling").path();
    let goal = p!(p:activated);

    // requires at least step = 2 to find a plan
    let result = compute_plan(&vec![(goal,None)], &state, &model, 1);

    assert!(!result.plan_found);

    assert_ne!(
        result
            .trace
            .last()
            .and_then(|f| f.state.sp_value_from_path(&activated)),
        Some(&true.to_spvalue())
    );
}


#[test]
fn planner_success_2_steps() {
    let (model, state) = one_dummy_robot();

    let activated = model
        .model
        .find_item("active", &[])
        .expect("check spelling").path();
    let goal = p!(p:activated);

    // requires at least step = 2 to find a plan
    let result = compute_plan(&vec![(goal,None)], &state, &model, 2);

    assert!(result.plan_found);

    println!("{:#?}", result);

    assert_eq!(
        result
            .trace
            .last()
            .and_then(|f| f.state.sp_value_from_path(&activated)),
        Some(&true.to_spvalue())
    );
}

// ie guard extraction to satisfy global spec.
fn model_with_global_spec() -> (RunnerModel, SPState) {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    // (global offline) Specifications
    let table_zone = p!(!([p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
    m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);
    (rm, s)
}

// ie without guard extraction to satisfy global spec. needs the spec as invariant during planning
fn model_without_spec() -> (RunnerModel, SPState) {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // !
    // No specifications
    // !

    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);
    (rm, s)
}

#[test]
fn planner_fail_due_to_conflicting_online_spec_and_goal() {
    let (model, state) = model_without_spec();

    let r1_p_a = model.model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = model.model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    let goal = p!([p:r1_p_a == "at"] && [p:r2_p_a == "at"]);
    let invar = p!(!([p:r1_p_a == "at"] && [p:r2_p_a == "at"]));

    let result = compute_plan(&vec![(goal,Some(invar))], &state, &model, 20);
    assert!(!result.plan_found);
}

#[test]
fn planner_fail_due_to_conflicting_offline_spec_and_goal() {
    let (model, state) = model_with_global_spec();

    let r1_p_a = model.model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = model.model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    let goal = p!([p:r1_p_a == "at"] && [p:r2_p_a == "at"]);

    let result = compute_plan(&vec![(goal,None)], &state, &model, 20);
    assert!(!result.plan_found);
}

#[test]
fn planner_succeed_when_no_conflicting_spec_and_goal() {
    let (model, state) = model_without_spec();

    let r1_p_a = model.model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = model.model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    let goal = p!([p:r1_p_a == "at"] && [p:r2_p_a == "at"]);

    let result = compute_plan(&vec![(goal, None)], &state, &model, 20);
    assert!(result.plan_found);
}

/*
#[test]
fn planner_debug_printouts() {
    let (model, state) = one_robot();
    let state = state.external();

    let activated = model
        .model
        .find_item("data", &["activated"])
        .unwrap_global_path();
    let goal = p!(activated);

    println!("INITIAL STATE");
    println!("{}", state);
    println!("GOAL PREDICATE: {:?}", goal);
    println!("-------\n");

    let result = compute_plan(&vec![goal], &state, &model, 20);

    println!("TIME TO SOLVE: {}ms", result.time_to_solve.as_millis());

    if !result.plan_found {
        println!("STDOUT\n---------\n{}\n", result.raw_output);
        println!("STDERR\n---------\n{}\n", result.raw_error_output);
        return;
    }
    println!("RESULTING PLAN");

    for (i, f) in result.trace.iter().enumerate() {
        println!("FRAME ID: {}\n{}", i, f.state);
        println!("TRANSITION: {:?}", f.transition);
        println!("-------");
    }

    assert!(false);
}
 */
