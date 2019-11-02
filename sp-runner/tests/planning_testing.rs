use sp_domain::*;
use sp_runner::*;
use sp_runner_api::*;

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

fn model_with_spec() -> (RunnerModel, SPState) {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    m.add_item(SPItem::Resource(make_dummy_robot("r1")));
    m.add_item(SPItem::Resource(make_dummy_robot("r2")));

    let r1_p_a = m.find_item("act_pos", &["r1"]).unwrap_global_path().to_sp();
    let r2_p_a = m.find_item("act_pos", &["r2"]).unwrap_global_path().to_sp();

    let table_zone = pr!{ {p!(r1_p_a == "at")} && {p!(r2_p_a == "at")} };
    let table_zone = Predicate::NOT(Box::new(table_zone));
    m.add_item(SPItem::Spec(Spec::new("table_zone", vec![table_zone])));

    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);
    (rm, s)
}

fn model_without_spec() -> (RunnerModel, SPState) {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    m.add_item(SPItem::Resource(make_dummy_robot("r1")));
    m.add_item(SPItem::Resource(make_dummy_robot("r2")));

    let r1_p_a = m.find_item("act_pos", &["r1"]).unwrap_global_path().to_sp();
    let r2_p_a = m.find_item("act_pos", &["r2"]).unwrap_global_path().to_sp();

    let table_zone = pr!{ {p!(r1_p_a == "at")} && {p!(r2_p_a == "at")} };
    let table_zone = Predicate::NOT(Box::new(table_zone));
    // m.add_item(SPItem::Spec(Spec::new("table_zone", vec![table_zone])));

    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);
    (rm, s)
}

#[test]
fn planner_fail_due_to_conflicting_specs_and_goal() {
    let (model, state) = model_with_spec();
    let state = state.external();

    let r1_p_a = model.model.find_item("act_pos", &["r1"]).unwrap_global_path().to_sp();
    let r2_p_a = model.model.find_item("act_pos", &["r2"]).unwrap_global_path().to_sp();

    let goal = pr!{{p!(r1_p_a == "at")} && {p!(r2_p_a == "at")}};

    let result = compute_plan(&goal, &state, &model, 20);
    assert!(!result.plan_found);
}

#[test]
fn planner_succeed_when_no_conflicting_spec_and_goal() {
    let (model, state) = model_without_spec();
    let state = state.external();

    let r1_p_a = model.model.find_item("act_pos", &["r1"]).unwrap_global_path().to_sp();
    let r2_p_a = model.model.find_item("act_pos", &["r2"]).unwrap_global_path().to_sp();

    let goal = pr!{{p!(r1_p_a == "at")} && {p!(r2_p_a == "at")}};

    let result = compute_plan(&goal, &state, &model, 20);
    assert!(result.plan_found);
}
