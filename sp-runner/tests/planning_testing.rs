use sp_domain::*;
use sp_runner::*;

// ie guard extraction to satisfy global spec.
fn basic_model() -> Model {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m
}

fn basic_planning_request(model: &Model, n: u32) -> PlanningResult {
    let active = model.find_item("active", &["State"]).expect("check spelling").path();
    let activate = model.find_item("activate", &["Control"]).expect("check spelling").path();
    let goal = p!(p:active);

    let state = SPState::new_from_values(&[
        (active.clone(), false.to_spvalue()),
        (activate.clone(), false.to_spvalue()),
    ]);

    // requires at least step = 2 to find a plan
    compute_plan(&model, &vec![(goal,None)], &state, n)
}

#[test]
fn planner_fail_1_step() {
    let model = basic_model();

    let result = basic_planning_request(&model, 1);
    assert!(!result.plan_found);

    let active = model.find_item("active", &[]).expect("check spelling").path();

    assert_ne!(
        result
            .trace
            .last()
            .and_then(|f| f.state.sp_value_from_path(&active)),
        Some(&true.to_spvalue())
    );
}


#[test]
fn planner_success_2_steps() {
    let model = basic_model();

    let result = basic_planning_request(&model, 2);
    assert!(result.plan_found);

    let active = model.find_item("active", &[]).expect("check spelling").path();

    assert_eq!(
        result
            .trace
            .last()
            .and_then(|f| f.state.sp_value_from_path(&active)),
        Some(&true.to_spvalue())
    );
}

// ie guard extraction to satisfy global spec.
fn model_with_global_spec() -> Model {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    // (global offline) Specifications
    let table_zone = p!(!([p:r1a == "at"] && [p:r2a == "at"]));
    m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

    m
}

// ie without guard extraction to satisfy global spec. needs the spec as invariant during planning
fn model_without_spec() -> Model {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // !
    // No specifications
    // !

    m
}

#[test]
fn planner_fail_due_to_conflicting_online_spec_and_goal() {
    let model = model_without_spec();

    let r1a = model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2a = model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    let r1r = model.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
    let r2r = model.find_item("ref_pos", &["r2"]).expect("check spelling2").path();


    let goal = p!([p:r1a == "at"] && [p:r2a == "at"]);
    let invar = p!(!([p:r1a == "at"] && [p:r2a == "at"]));

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let result = compute_plan(&model, &[(goal,Some(invar))], &state, 20);
    assert!(!result.plan_found);
}

#[test]
fn planner_fail_due_to_conflicting_offline_spec_and_goal() {
    let model = model_with_global_spec();

    let r1a = model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2a = model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    let r1r = model.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
    let r2r = model.find_item("ref_pos", &["r2"]).expect("check spelling2").path();

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let goal = p!([p:r1a == "at"] && [p:r2a == "at"]);

    let result = compute_plan(&model, &[(goal,None)], &state, 20);
    assert!(!result.plan_found);
}

#[test]
fn planner_succeed_when_no_conflicting_spec_and_goal() {
    let model = model_without_spec();

    let r1a = model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2a = model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    let r1r = model.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
    let r2r = model.find_item("ref_pos", &["r2"]).expect("check spelling2").path();

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let goal = p!([p:r1a == "at"] && [p:r2a == "at"]);

    let result = compute_plan(&model, &[(goal, None)], &state, 20);
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
