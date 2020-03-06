use serial_test::serial;
use sp_domain::*;
use sp_runner::*;

mod test_models;
use test_models::*;

// ie guard extraction to satisfy global spec.
fn basic_model() -> Model {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m
}

fn basic_planning_request(model: &Model, n: u32) -> PlanningResult {
    let active = model
        .find_item("active", &["State"])
        .expect("check spelling")
        .path();
    let activate = model
        .find_item("activate", &["Control"])
        .expect("check spelling")
        .path();
    let goal = p!(p: active);

    let state = SPState::new_from_values(&[
        (active.clone(), false.to_spvalue()),
        (activate.clone(), false.to_spvalue()),
    ]);

    // requires at least step = 2 to find a plan
    let ts_model = TransitionSystemModel::from(&model);
    plan(&ts_model, &vec![(goal, None)], &state, n)
}

#[test]
#[serial]
fn planning_fail_1_step() {
    let model = basic_model();

    let result = basic_planning_request(&model, 1);
    assert!(!result.plan_found);

    let active = model
        .find_item("active", &[])
        .expect("check spelling")
        .path();

    assert_ne!(
        result
            .trace
            .last()
            .and_then(|f| f.state.sp_value_from_path(&active)),
        Some(&true.to_spvalue())
    );
}

#[test]
#[serial]
fn planning_success_2_steps() {
    let model = basic_model();

    let result = basic_planning_request(&model, 2);
    assert!(result.plan_found);

    let active = model
        .find_item("active", &[])
        .expect("check spelling")
        .path();

    println!("{:?}", result);

    assert_eq!(result.plan_length, 2);
    assert_eq!(result.trace.len(), 3);

    for f in &result.trace {
        println!("==========================");
        println!("{}", f.transition);
        println!("==========================");
        println!("{}", f.state);
    }

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
    let r1a = m
        .find_item("act_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2a = m
        .find_item("act_pos", &["r2"])
        .expect("check spelling2")
        .path();

    // (global offline) Specifications
    let table_zone = p!(!([p: r1a == "at"] && [p: r2a == "at"]));
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
#[serial]
fn planning_fail_due_to_conflicting_online_spec_and_goal() {
    let model = model_without_spec();

    let r1a = model
        .find_item("act_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2a = model
        .find_item("act_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let r1r = model
        .find_item("ref_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2r = model
        .find_item("ref_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let goal = p!([p: r1a == "at"] && [p: r2a == "at"]);
    let invar = p!(!([p: r1a == "at"] && [p: r2a == "at"]));

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let ts_model = TransitionSystemModel::from(&model);
    let result = plan(&ts_model, &[(goal, Some(invar))], &state, 20);
    assert!(!result.plan_found);
}

#[test]
#[serial]
fn planning_fail_when_goal_is_false() {
    let model = model_without_spec();

    let r1a = model
        .find_item("act_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2a = model
        .find_item("act_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let r1r = model
        .find_item("ref_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2r = model
        .find_item("ref_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let goal = Predicate::FALSE;

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let ts_model = TransitionSystemModel::from(&model);
    let result = plan(&ts_model, &[(goal, None)], &state, 20);
    assert!(!result.plan_found);
}

#[test]
#[serial]
fn planning_fail_when_invar_is_false() {
    let model = model_without_spec();

    let r1a = model
        .find_item("act_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2a = model
        .find_item("act_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let r1r = model
        .find_item("ref_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2r = model
        .find_item("ref_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let goal = Predicate::TRUE;
    let invar = Predicate::FALSE;

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let ts_model = TransitionSystemModel::from(&model);
    let result = plan(&ts_model, &[(goal, Some(invar))], &state, 20);
    assert!(!result.plan_found);
}

#[test]
#[serial]
fn planning_fail_due_to_conflicting_offline_spec_and_goal() {
    let model = model_with_global_spec();

    let r1a = model
        .find_item("act_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2a = model
        .find_item("act_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let r1r = model
        .find_item("ref_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2r = model
        .find_item("ref_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let goal = p!([p: r1a == "at"] && [p: r2a == "at"]);

    let model = TransitionSystemModel::from(&model);
    let result = plan(&model, &[(goal, None)], &state, 20);
    assert!(!result.plan_found);
}

#[test]
#[serial]
fn planning_succeed_when_no_conflicting_spec_and_goal() {
    let model = model_without_spec();

    let r1a = model
        .find_item("act_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2a = model
        .find_item("act_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let r1r = model
        .find_item("ref_pos", &["r1"])
        .expect("check spelling1")
        .path();
    let r2r = model
        .find_item("ref_pos", &["r2"])
        .expect("check spelling2")
        .path();

    let state = SPState::new_from_values(&[
        (r1a.clone(), "away".to_spvalue()),
        (r2a.clone(), "away".to_spvalue()),
        (r1r.clone(), "away".to_spvalue()),
        (r2r.clone(), "away".to_spvalue()),
    ]);

    let goal = p!([p: r1a == "at"] && [p: r2a == "at"]);

    let ts_model = TransitionSystemModel::from(&model);
    let result = plan(&ts_model, &[(goal, None)], &state, 20);
    assert!(result.plan_found);
}

#[test]
#[serial]
fn planning_ge_len11() {
    // Make model
    let mut m = Model::new_root("test_sat_planning", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    let r1a = m
        .find_item("act_pos", &["r1"])
        .expect("check spelling")
        .path();
    let r1r = m
        .find_item("ref_pos", &["r1"])
        .expect("check spelling")
        .path();
    let r1active = m
        .find_item("active", &["r1"])
        .expect("check spelling")
        .path();
    let r1activate = m
        .find_item("activate", &["r1", "Control"])
        .expect("check spelling")
        .path();

    let r2a = m
        .find_item("act_pos", &["r2"])
        .expect("check spelling")
        .path();
    let r2r = m
        .find_item("ref_pos", &["r2"])
        .expect("check spelling")
        .path();
    let r2active = m
        .find_item("active", &["r2"])
        .expect("check spelling")
        .path();
    let r2activate = m
        .find_item("activate", &["r2", "Control"])
        .expect("check spelling")
        .path();

    // spec to make it take more steps
    let table_zone = p!(!([p: r1a == "at"] && [p: r2a == "at"]));
    m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

    let mut ts_model = TransitionSystemModel::from(&m);
    let (new_guards, _new_initial) = extract_guards(&ts_model, &Predicate::TRUE);
    update_guards(&mut ts_model, &new_guards);

    let state = state! {
        r1a => "away",
        r1r => "away",
        r1active => false,
        r1activate => false,
        r2a => "away",
        r2r => "away",
        r2active => false,
        r2activate => false
    };

    // start planning test
    let g1 = p!(p: r1a == "at");
    let g2 = p!(p: r2a == "at");

    let goals = vec![(g1, None), (g2, None)];

    let result = plan(&ts_model, goals.as_slice(), &state, 20);
    assert!(result.plan_found);
    assert_eq!(result.trace.len(), 11);
    assert_eq!(result.plan_length, 10);
}

#[test]
#[serial]
fn planning_invar_len11() {
    // Make model
    let mut m = Model::new_root("tsi", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    let r1a = m
        .find_item("act_pos", &["r1"])
        .expect("check spelling")
        .path();
    let r1r = m
        .find_item("ref_pos", &["r1"])
        .expect("check spelling")
        .path();
    let r1active = m
        .find_item("active", &["r1"])
        .expect("check spelling")
        .path();
    let r1activate = m
        .find_item("activate", &["r1", "Control"])
        .expect("check spelling")
        .path();

    let r2a = m
        .find_item("act_pos", &["r2"])
        .expect("check spelling")
        .path();
    let r2r = m
        .find_item("ref_pos", &["r2"])
        .expect("check spelling")
        .path();
    let r2active = m
        .find_item("active", &["r2"])
        .expect("check spelling")
        .path();
    let r2activate = m
        .find_item("activate", &["r2", "Control"])
        .expect("check spelling")
        .path();

    let table_zone = p!(!([p: r1a == "at"] && [p: r2a == "at"]));
    let new_table_zone = refine_invariant(&m, &table_zone);

    // no guard extr.
    let ts_model = TransitionSystemModel::from(&m);

    let state = state! {
        r1a => "away",
        r1r => "away",
        r1active => false,
        r1activate => false,
        r2a => "away",
        r2r => "away",
        r2active => false,
        r2activate => false
    };

    // start planning test

    let i1 = new_table_zone.clone();
    let g1 = p!(p: r1a == "at");

    let i2 = new_table_zone.clone();
    let g2 = p!(p: r2a == "at");

    let goals = vec![(g1, Some(i1)), (g2, Some(i2))];

    let result = plan(&ts_model, goals.as_slice(), &state, 20);
    assert!(result.plan_found);
    assert_eq!(result.trace.len(), 11);
    assert_eq!(result.plan_length, 10);
}

#[test]
#[serial]
fn planning_invar_len9() {
    // Make model
    let mut m = Model::new_root("tsi", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    let r1a = m
        .find_item("act_pos", &["r1"])
        .expect("check spelling")
        .path();
    let r1r = m
        .find_item("ref_pos", &["r1"])
        .expect("check spelling")
        .path();
    let r1active = m
        .find_item("active", &["r1"])
        .expect("check spelling")
        .path();
    let r1activate = m
        .find_item("activate", &["r1", "Control"])
        .expect("check spelling")
        .path();

    let r2a = m
        .find_item("act_pos", &["r2"])
        .expect("check spelling")
        .path();
    let r2r = m
        .find_item("ref_pos", &["r2"])
        .expect("check spelling")
        .path();
    let r2active = m
        .find_item("active", &["r2"])
        .expect("check spelling")
        .path();
    let r2activate = m
        .find_item("activate", &["r2", "Control"])
        .expect("check spelling")
        .path();

    // no guard extr.
    let ts_model = TransitionSystemModel::from(&m);

    let state = state! {
        r1a => "away",
        r1r => "away",
        r1active => false,
        r1activate => false,
        r2a => "away",
        r2r => "away",
        r2active => false,
        r2activate => false
    };

    let i1 = Predicate::TRUE;
    let g1 = p!(p: r1a == "at");

    let i2 = Predicate::TRUE;
    let g2 = p!(p: r2a == "at");

    let goals = vec![(g1, Some(i1)), (g2, Some(i2))];
    let result = plan(&ts_model, goals.as_slice(), &state, 20);
    assert!(result.plan_found);
    assert_eq!(result.trace.len(), 9);
    assert_eq!(result.plan_length, 8);
}
