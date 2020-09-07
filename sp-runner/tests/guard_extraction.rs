use serial_test::serial;
use sp_domain::*;
use sp_runner::*;

mod test_models;
use test_models::*;

// This test is commented out since we no longer create one transition
// per possible assignment of the variables in the domain. This means
// we cannot use GE like we did before.

// #[test]
// #[serial]
// fn test_guard_extraction() {
//     // Make model
//     let mut m = Model::new_root("test_guard_extraction", Vec::new());

//     // Make resoureces
//     m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
//     m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

//     let inits: Vec<Predicate> = m
//         .resources()
//         .iter()
//         .flat_map(|r| r.specs.clone())
//         .filter_map(|s| if s.name() == "supervisor" {
//             Some(s.invariant.clone())
//         } else { None })
//         .collect();

//     // we need to assume that we are in a state that adheres to the resources
//     let initial = Predicate::AND(inits);

//     // Make some global stuff
//     let r1_p_a = m
//         .find_item("act_pos", &["r1"])
//         .expect("check spelling")
//         .path();
//     let r2_p_a = m
//         .find_item("act_pos", &["r2"])
//         .expect("check spelling")
//         .path();

//     // (offline) Specifications
//     let table_zone = p!(!([p: r1_p_a == "at"] && [p: r2_p_a == "at"]));
//     m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

//     let mut ts_model = TransitionSystemModel::from(&m);
//     let (new_guards, new_initial) = extract_guards(&ts_model, &initial);
//     update_guards(&mut ts_model, &new_guards);

//     ts_model.specs.clear();
//     generate_offline_nuxvm(&ts_model, &new_initial);

//     assert_ne!(new_initial, Predicate::TRUE);
//     assert_eq!(new_guards.len(), 4);
// }

#[test]
#[serial]
fn test_invariant_refinement() {
    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1_p_a = m
        .find_item("act_pos", &["r1"])
        .expect("check spelling")
        .path();
    let r2_p_a = m
        .find_item("act_pos", &["r2"])
        .expect("check spelling")
        .path();

    // (offline) Specifications
    let table_zone = p!(!([p: r1_p_a == "at"] && [p: r2_p_a == "at"]));

    let mut ts_model = TransitionSystemModel::from(&m);
    let new_table_zone = refine_invariant(&ts_model, &table_zone);
    assert_ne!(new_table_zone, table_zone);
    // println!("new spec: {}", new_table_zone);

    ts_model.specs.push(Spec::new("table_zone", new_table_zone));

    let ts_model = TransitionSystemModel::from(&m);
    generate_offline_nuxvm(&ts_model, &Predicate::TRUE);
}
