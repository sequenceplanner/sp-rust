use sp_smt::*;
use sp_domain::*;
use sp_runner::*;
use serial_test::serial;
use std::collections::HashMap;

pub fn make_dummy_robot(name: &str, poses: &[&str]) -> Resource {
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(poses.iter());
    resource!{
        name: name,
        command!{
            topic: "Control",
            msg_type: "dummy_robot_messages/msg/Control",

            ref_pos : domain,
            activate : bool,
        },
        measured!{
            topic: "State",
            msg_type: "dummy_robot_messages/msg/State",

            act_pos : domain,
            active : bool,

            echo / ref_pos : domain,   // these are used for handshaking.
            echo / activate : bool,    // "echo" has special meaning for us.
        },
        estimated!{
            prev_pos: domain,
        },

        ability!{
            name: move_to,

            enabled : p!([active] && [ref_pos <-> act_pos]),
            executing : p!([active] && [ref_pos <!> act_pos]),
            finished : p!([active] && [ref_pos <-> act_pos]),

            *start : p!(enabled) => [ a!(ref_pos?) ] / [a!(act_pos = "unknown")],
            finish : p!(executing) => [] / [a!(act_pos <- ref_pos)],

            sync_prev: p!([prev_pos <!> ref_pos] && [ref_pos <-> act_pos]) => [ a!(prev_pos <- ref_pos) ] / []
        },

        never!{
            name: cannot_go_to_unknown,
            prop: p!(ref_pos == "unknown")
        },

        ability!{
            name: activate,

            enabled: p!([!activate] && [!active]),
            executing: p!([activate] && [!active]),
            finished: p!([activate] && [active]),

            *start: p!(enabled) => [ a!(activate) ] / [],
            finish: p!(executing) => [] / [ a!(active) ],
        },

        ability!{
            name: deactivate,

            enabled: p!([activate] && [active]),
            executing: p!([!activate] && [active]),
            finished: p!([!activate] && [!active]),

            *start: p!(enabled) => [ a!(!activate) ] / [],
            finish: p!(executing) => [] / [ a!(!active) ],
        },
    }
}

fn basic_model() -> Model {
    let mut m = Model::new_root("dummy_robot_model", Vec::new());
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m
}

fn basic_planning_request(model: &Model, n: u32) -> PlanningResultZ3 {
    let active = model.find_item("active", &["State"]).expect("check spelling").path();
    let activate = model.find_item("activate", &["Control"]).expect("check spelling").path();
    let goal = p!(p:active);
    // println!("goal: {:?}", goal);

    let state = SPState::new_from_values(&[
        (active.clone(), false.to_spvalue()),
        (activate.clone(), false.to_spvalue()),
    ]);

    // requires at least step = 2 to find a plan
    let ts_model = TransitionSystemModel::from(&model);
    ComputePlanSPModelZ3::plan(&ts_model, &vec![(goal,None)], &state, n)
}

#[test]
#[serial]
fn planning_fail_1_step() {
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
#[serial]
fn planning_success_2_steps() {
    let model = basic_model();

    let result = basic_planning_request(&model, 2);
    assert!(result.plan_found);

    let active = model.find_item("active", &[]).expect("check spelling").path();

    // println!("{:?}", result);

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

// fn model_without_spec() -> Model {
//     let mut m = Model::new_root("dummy_robot_model", Vec::new());

//     // Make resoureces
//     m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
//     m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

//     // !
//     // No specifications
//     // !

//     m
// }

// // ie guard extraction to satisfy global spec.
// fn model_with_global_spec() -> Model {
//     let mut m = Model::new_root("dummy_robot_model", Vec::new());

//     // Make resoureces
//     m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
//     m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

//     // Make some global stuff
//     let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
//     let r2a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

//     // (global offline) Specifications
//     let table_zone = p!(!([p:r1a == "at"] && [p:r2a == "at"]));
//     m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

//     m
// }

// // #[test]
// // #[serial]
// // fn planning_fail_due_to_conflicting_online_spec_and_goal() {
// //     let model = model_without_spec();

// //     let r1a = model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
// //     let r2a = model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

// //     let r1r = model.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
// //     let r2r = model.find_item("ref_pos", &["r2"]).expect("check spelling2").path();

// //     let goal = p!([p:r1a == "at"] && [p:r2a == "at"]);
// //     let invar = p!(!([p:r1a == "at"] && [p:r2a == "at"]));

// //     let state = SPState::new_from_values(&[
// //         (r1a.clone(), "away".to_spvalue()),
// //         (r2a.clone(), "away".to_spvalue()),
// //         (r1r.clone(), "away".to_spvalue()),
// //         (r2r.clone(), "away".to_spvalue()),
// //     ]);

// //     let ts_model = TransitionSystemModel::from(&model);
// //     let result = ComputePlanSPModelZ3::plan(&ts_model, &[(goal,Some(invar))], &state, 20);
// //     assert!(!result.plan_found);
// // }

// #[test]
// #[serial]
// fn planning_fail_when_goal_is_false() {
//     let model = model_without_spec();

//     let r1a = model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
//     let r2a = model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

//     let r1r = model.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
//     let r2r = model.find_item("ref_pos", &["r2"]).expect("check spelling2").path();

//     let goal = Predicate::FALSE;

//     let state = SPState::new_from_values(&[
//         (r1a.clone(), "away".to_spvalue()),
//         (r2a.clone(), "away".to_spvalue()),
//         (r1r.clone(), "away".to_spvalue()),
//         (r2r.clone(), "away".to_spvalue()),
//     ]);

//     let ts_model = TransitionSystemModel::from(&model);
//     let result = ComputePlanSPModelZ3::plan(&ts_model, &[(goal,None)], &state, 20);
//     assert!(!result.plan_found);
// }

// // #[test]
// // #[serial]
// // fn planning_fail_when_invar_is_false() {
// //     let model = model_without_spec();

// //     let r1a = model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
// //     let r2a = model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

// //     let r1r = model.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
// //     let r2r = model.find_item("ref_pos", &["r2"]).expect("check spelling2").path();

// //     let goal = Predicate::TRUE;
// //     let invar = Predicate::FALSE;

// //     let state = SPState::new_from_values(&[
// //         (r1a.clone(), "away".to_spvalue()),
// //         (r2a.clone(), "away".to_spvalue()),
// //         (r1r.clone(), "away".to_spvalue()),
// //         (r2r.clone(), "away".to_spvalue()),
// //     ]);

// //     let ts_model = TransitionSystemModel::from(&model);
// //     let result = plan(&ts_model, &[(goal,Some(invar))], &state, 20);
// //     assert!(!result.plan_found);
// // }

// #[test]
// #[serial]
// fn planning_fail_due_to_conflicting_offline_spec_and_goal() {
//     let model = model_with_global_spec();

//     let r1a = model.find_item("act_pos", &["r1"]).expect("check spelling1").path();
//     let r2a = model.find_item("act_pos", &["r2"]).expect("check spelling2").path();

//     let r1r = model.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
//     let r2r = model.find_item("ref_pos", &["r2"]).expect("check spelling2").path();

//     let state = SPState::new_from_values(&[
//         (r1a.clone(), "away".to_spvalue()),
//         (r2a.clone(), "away".to_spvalue()),
//         (r1r.clone(), "away".to_spvalue()),
//         (r2r.clone(), "away".to_spvalue()),
//     ]);

//     let goal = p!([p:r1a == "at"] && [p:r2a == "at"]);

//     let model = TransitionSystemModel::from(&model);
//     let result = ComputePlanSPModelZ3::plan(&model, &[(goal,None)], &state, 20);
//     assert!(!result.plan_found);
// }