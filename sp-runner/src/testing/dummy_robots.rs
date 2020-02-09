use sp_domain::*;
use crate::modeling::*;
use crate::runners::*;
use sp_runner_api::*;
use crate::testing::*;

pub fn one_dummy_robot() -> (Model, SPState) {
    let r1 = make_dummy_robot("r1", &["at", "away"]);
    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let s = make_initial_state(&m);

    (m, s)
}

pub fn two_dummy_robots() -> (Model, SPState) {
    // two_dummy_robots_global_but_no_guard_extraction()
    two_dummy_robots_guard_extraction()
}

fn set_dummy_robots_initial_state(m: &Model, state: &mut SPState) {
    // fixing some things so it runs again...
    let r1r = m.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
    let r2r = m.find_item("ref_pos", &["r2"]).expect("check spelling2").path();

    state.force_from_path(&r1r, "away".to_spvalue());
    state.force_from_path(&r2r, "away".to_spvalue());
}

pub fn two_dummy_robots_online_specs_only() -> (Model, SPState) {
    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    // Specifications
    let table_zone = p!(!( [p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
    let table_zone = refine_invariant(&m, &table_zone);

    // Operations
    let r1_to_at = add_op(&mut m, "r1_to_at", false, p!(p:r1_p_a != "at"), p!(p:r1_p_a == "at"), vec![], Some(table_zone.clone()));

    let r2_to_at = add_op(&mut m, "r2_to_at", false,
                          // pr!{{p!(r2_p_a != "at")} && {p!(r1_p_a == "at")}}, // can start when r1 is at "at". what will happen?
                          p!(p:r2_p_a != "at"),
                          p!(p:r2_p_a == "at"), vec![], Some(table_zone.clone()));

    // reset previous ops so we can start over
    let both_to_away = add_op(&mut m, "both_to_away", true,
                              p!([p:r1_to_at == "f"] && [p:r2_to_at == "f"]),
                              p!([p:r1_p_a == "away"] && [p:r2_p_a == "away"]),
                              vec![a!(p:r1_to_at = "i"), a!(p:r2_to_at = "i")], None);

    // Make it runnable
    let mut s = make_initial_state(&m);
    set_dummy_robots_initial_state(&m, &mut s);

    (m, s)
}

pub fn two_dummy_robots_guard_extraction() -> (Model, SPState) {
    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    let r1_r_p = m.find_item("ref_pos", &["r1"]).expect("check spelling1").path();
    let r2_r_p = m.find_item("ref_pos", &["r2"]).expect("check spelling2").path();


    // Specifications
    let table_zone = p!(!([p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
    m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

    // Operations
    let r1_to_at = add_op(&mut m, "r1_to_at", false, p!(p:r1_p_a != "at"), p!(p:r1_p_a == "at"), vec![], None);

    let r2_to_at = add_op(&mut m, "r2_to_at", false,
                          // pr!{{p!(r2_p_a != "at")} && {p!(r1_p_a == "at")}}, // can start when r1 is at "at". what will happen?
                          p!(p:r2_p_a != "at"),
                          p!(p:r2_p_a == "at"), vec![], None);

    // reset previous ops so we can start over
    let both_to_away = add_op(&mut m, "both_to_away", true,
                              p!([p:r1_to_at == "f"] && [p:r2_to_at == "f"]),
                              p!([p:r1_p_a == "away"] && [p:r2_p_a == "away"]),
                              vec![a!(p:r1_to_at = "i"), a!(p:r2_to_at = "i")], None);

    // Make it runnable
    let mut s = make_initial_state(&m);
    set_dummy_robots_initial_state(&m, &mut s);

    (m, s)
}

pub fn two_dummy_robots_global_but_no_guard_extraction() -> (Model, SPState) {
    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    // Specifications
    let table_zone = p!(!( [p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
    let table_zone = refine_invariant(&m, &table_zone);
    m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

    // Operations
    let r1_to_at = add_op(&mut m, "r1_to_at", false, p!(p:r1_p_a != "at"), p!(p:r1_p_a == "at"), vec![], None);

    let r2_to_at = add_op(&mut m, "r2_to_at", false,
                          // pr!{{p!(r2_p_a != "at")} && {p!(r1_p_a == "at")}}, // can start when r1 is at "at". what will happen?
                          p!(p:r2_p_a != "at"),
                          p!(p:r2_p_a == "at"), vec![], None);

    // reset previous ops so we can start over
    let both_to_away = add_op(&mut m, "both_to_away", true,
                              p!([p:r1_to_at == "f"] && [p:r2_to_at == "f"]),
                              p!([p:r1_p_a == "away"] && [p:r2_p_a == "away"]),
                              vec![a!(p:r1_to_at = "i"), a!(p:r2_to_at = "i")], None);

    let mut s = make_initial_state(&m);
    set_dummy_robots_initial_state(&m, &mut s);

    (m, s)
}

#[test]
fn test_two_dummy_robots() {
    let (m, s) = two_dummy_robots();

//    println!("{:#?}", rm);
    println!("=================================");
//    println!("{:#?}", s);
    assert!(false);
}
