use crate::runners::*;
use sp_domain::*;
use sp_runner_api::*;
use std::collections::HashMap;

use guard_extraction::*;

pub fn make_dummy_robot(name: &str) -> Resource {
    let mut r = Resource::new(name);

    let command = command_topic("Control", "dr_c", "dummy_robot_messages/msg/Control",
                                &[
                                    ( "ref_pos", Some(&["unknown", "at", "away"])),
                                    ( "activate", None )  // none is boolean
                                ]);
    r.add_message(command);

    let measured = measured_topic("State", "dr_m", "dummy_robot_messages/msg/State",
                                  &[
                                      ( "act_pos", Some(&["unknown", "at", "away"])),
                                      ( "active", None )
                                  ]);
    r.add_message(measured);

    let rp_c = r.find_item("ref_pos", &[]).expect("check spelling").path();
    let ap_m = r.find_item("act_pos", &[]).expect("check spelling").path();
    let activate_c = r.find_item("activate", &[]).expect("check spelling").path();
    let active_m = r.find_item("active", &[]).expect("check spelling").path();

    let to_table =
        ability("to_table",
                &[
                    ("enabled", pr! {{p!(active_m)} && {p!(rp_c != "at")} && {p!(ap_m != "at")}}),
                    ("executing", pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m != "at")}}),
                    ("finished", pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m == "at")}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(rp_c = "at")], &[a!(ap_m = "unknown")]),

                    ("finish", false, pred_true("executing"), &[], &[a!(ap_m = "at")] ),
                ]);


    let to_away =
        ability("to_away",
                &[
                    ("enabled", pr! {{p!(active_m)} && {p!(rp_c != "away")} && {p!(ap_m != "away")}}),
                    ("executing", pr! {{p!(active_m)} && {p!(rp_c == "away")} && {p!(ap_m != "away")}}),
                    ("finished", pr! {{p!(active_m)} && {p!(rp_c == "away")} && {p!(ap_m == "away")}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(rp_c = "away")], &[a!(ap_m = "unknown")]),

                    ("finish", false, pred_true("executing"), &[], &[a!(ap_m = "away")] ),
                ]);

    let activate =
        ability("activate",
                &[
                    ("enabled", pr! {{p!(activate_c == false)} && {p!(active_m == false)}} ),
                    ("executing", pr! {{p!(activate_c == true)} && {p!(active_m == false)}} ),
                    ("finished", pr! {{p!(activate_c == true)} && {p!(active_m == true)}} ),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(activate_c = true)], &[] ),
                    ("finish", false, pred_true("executing"), &[], &[a!(active_m = true)]),
                ]);

    let deactivate =
        ability("deactivate",
                &[
                    ("enabled", pr! {{p!(activate_c == true)} && {p!(active_m == true)}}),
                    ("executing", pr! {{p!(activate_c == false)} && {p!(active_m == true)}}),
                    ("finished", pr! {{p!(activate_c == false)} && {p!(active_m == false)}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(activate_c = false)], &[]),
                    ("finish", false, pred_true("executing"), &[], &[a!(active_m = false)]),
                ]);

    r.add_ability(to_table);
    r.add_ability(to_away);
    r.add_ability(activate);
    r.add_ability(deactivate);

    return r;
}

#[test]
fn test_dummy() {
    let r1 = make_dummy_robot("r1");
    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);
    let rm = make_runner_model(&m);

    println!("{:#?}", rm);
    assert!(false);
}
