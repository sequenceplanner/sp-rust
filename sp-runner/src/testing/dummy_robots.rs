use sp_domain::*;
use crate::modeling::*;
use crate::runners::*;
use sp_runner_api::*;
use crate::testing::*;

pub fn one_dummy_robot() -> (RunnerModel, SPState) {
    let r1 = make_dummy_robot("r1");
    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}

pub fn two_dummy_robots() -> (RunnerModel, SPState) {
    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1")));
    m.add_item(SPItem::Resource(make_dummy_robot("r2")));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

    // Specifications
    let table_zone = p!(!([p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
    m.add_item(SPItem::Spec(Spec::new("table_zone", vec![table_zone])));

    // Operations
    let r1_to_at = add_op(&mut m, "r1_to_at", false, p!(p:r1_p_a != "at"), p!(p:r1_p_a == "at"), vec![]);

    let r2_to_at = add_op(&mut m, "r2_to_at", false,
                          // pr!{{p!(r2_p_a != "at")} && {p!(r1_p_a == "at")}}, // can start when r1 is at "at". what will happen?
                          p!(p:r2_p_a != "at"),
                          p!(p:r2_p_a == "at"), vec![]);

    // reset previous ops so we can start over
    let both_to_away = add_op(&mut m, "both_to_away", true,
                              p!([p:r1_to_at == "f"] && [p:r2_to_at == "f"]),
                              p!([p:r1_p_a == "away"] && [p:r2_p_a == "away"]),
                              vec![a!(p:r1_to_at = "i"), a!(p:r2_to_at = "i")]);

    // Make it runnable
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}

#[test]
fn test_two_dummy_robots() {
    let (rm, s) = two_dummy_robots();

    println!("{:#?}", rm);
    println!("=================================");
    println!("{:#?}", s);
    assert!(false);
}
