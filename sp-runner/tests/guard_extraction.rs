use sp_domain::*;
use sp_runner::*;


#[test]
fn test_invariant_refinement() {
    // Make model
    // let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // // Make resoureces
    // m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    // m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // // Make some global stuff
    // let r1_p_a = m
    //     .find_item("act_pos", &["r1"])
    //     .expect("check spelling")
    //     .path();
    // let r2_p_a = m
    //     .find_item("act_pos", &["r2"])
    //     .expect("check spelling")
    //     .path();

    // // (offline) Specifications
    // let table_zone = p!(!([p: r1_p_a == "at"] && [p: r2_p_a == "at"]));

    // let mut ts_model = TransitionSystemModel::from(&m);
    // let new_table_zone = refine_invariant(ts_model.clone(), table_zone.clone()).unwrap();
    // assert_ne!(new_table_zone, table_zone);
    // println!("new spec: {}", new_table_zone);

    // ts_model.specs.push(Spec::new("table_zone", new_table_zone));

    // let ts_model = TransitionSystemModel::from(&m);
    // generate_offline_nuxvm(&ts_model, &Predicate::TRUE);
}
