use crate::runners::*;
use sp_domain::*;
use sp_runner_api::*;
use std::collections::HashMap;

use guard_extraction::*;

pub fn make_mecademic_robot(name: &str) -> Resource {
    let mut r = Resource::new(name);

    let command = command_topic("mecademic_sp_to_esd", "dr_c", "ros2_mecademic_msgs/msg/MecademicSPToEsd",
                                &[
                                    ( "reference_pose",
                                       Some(&["UNKNOWN".to_spvalue(), "at".to_spvalue(), "away".to_spvalue()])),
                                    ( "reference_joint_speed",
                                       Some(&[ 0.to_spvalue(), 50.to_spvalue() ]))
                                ]);
    r.add_message(command);

    let measured = measured_topic("mecademic_esd_to_sp", "dr_m", "ros2_mecademic_msgs/msg/MecademicEsdToSP",
                                  &[
                                      ( "actual_pose", Some(&["UNKNOWN".to_spvalue(), "at".to_spvalue(), "away".to_spvalue()])),
                                  ]);
    r.add_message(measured);

    let rp_c = r.find_item("reference_pose", &[]).unwrap_local_path().to_sp();
    let ap_m = r.find_item("actual_pose", &[]).unwrap_local_path().to_sp();

    let speed_c = r.find_item("reference_joint_speed", &[]).unwrap_local_path().to_sp();

    let to_table =
        ability("to_table",
                &[
                    ("enabled", pr! {{p!(rp_c != "at")} && {p!(ap_m != "at")}}),
                    ("executing", pr! {{p!(rp_c == "at")} && {p!(ap_m != "at")}}),
                    ("finished", pr! {{p!(rp_c == "at")} && {p!(ap_m == "at")}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(rp_c = "at"), a!(speed_c = 50)], &[a!(ap_m = "UNKNOWN")]),

                    ("finish", false, pred_true("executing"), &[], &[a!(ap_m = "at")] ),
                ]);


    let to_away =
        ability("to_away",
                &[
                    ("enabled", pr! {{p!(rp_c != "away")} && {p!(ap_m != "away")}}),
                    ("executing", pr! {{p!(rp_c == "away")} && {p!(ap_m != "away")}}),
                    ("finished", pr! {{p!(rp_c == "away")} && {p!(ap_m == "away")}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(rp_c = "away"), a!(speed_c = 50)], &[a!(ap_m = "UNKNOWN")]),

                    ("finish", false, pred_true("executing"), &[], &[a!(ap_m = "away")] ),
                ]);

    r.add_ability(to_table);
    r.add_ability(to_away);

    return r;
}

#[test]
fn test_mecademic() {
    let r1 = make_mecademic_robot("robot1");
    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);
    let rm = make_runner_model(&m);

    println!("{:#?}", rm);
    assert!(false);
}
