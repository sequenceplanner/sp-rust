use crate::runners::*;
use sp_domain::*;
use sp_runner_api::*;
use std::collections::HashMap;

use crate::testing::*;

pub fn two_mecademic_robots() -> (RunnerModel, SPState) {
    // Make model
    let mut m = Model::new_root("mecademic_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_mecademic_robot("robot1")));
    m.add_item(SPItem::Resource(make_mecademic_robot("robot2")));

    // Make some global stuff
    let r1_p_a = m.find_item("actual_pose", &["robot1"]).unwrap_global_path().to_sp();
    let r2_p_a = m.find_item("actual_pose", &["robot2"]).unwrap_global_path().to_sp();

    // Specifications
    let table_zone = pr!{ {p!(r1_p_a == "at")} && {p!(r2_p_a == "at")} };
    let table_zone = Predicate::NOT(Box::new(table_zone)); // NOT macro broken
    m.add_item(SPItem::Spec(Spec::new("table_zone", vec![table_zone])));

    // Operations
    let r1_to_at = add_op(&mut m, "r1_to_at", false, p!(r1_p_a != "at"), p!(r1_p_a == "at"), vec![]);

    let r2_to_at = add_op(&mut m, "r2_to_at", false,
                          // pr!{{p!(r2_p_a != "at")} && {p!(r1_p_a == "at")}}, // can start when r1 is at "at". what will happen?
                          p!(r2_p_a != "at"),
                          p!(r2_p_a == "at"), vec![]);

    // reset previous ops so we can start over
    let both_to_away = add_op(&mut m, "both_to_away", true,
                              pr!{{p!(r1_to_at == "f")} && {p!(r2_to_at == "f")}},
                              pr!{{p!(r1_p_a == "away")} && {p!(r2_p_a == "away")}},
                              vec![a!(r1_to_at = "i"), a!(r2_to_at = "i")]);

    // Make it runnable
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}

#[test]
fn test_two_mecademic_robots() {
    let (rm, s) = two_mecademic_robots();

    println!("{:#?}", rm);
    println!("=================================");
    println!("{:#?}", s);
    assert!(false);
}
