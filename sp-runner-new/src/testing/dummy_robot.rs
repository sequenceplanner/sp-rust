use sp_domain::*;
use super::super::*;
use crate::modeling::*;
use crate::planning::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_dummy_robot(name: &str) -> Resource {
    resource!{
        name: name,
        command!{
            topic: "Control",
            msg_type: "dummy_robot_messages/msg/Control",

            ref_pos : ["unknown", "at", "away"],
            activate : bool,
        },
        measured!{
            topic: "State",
            msg_type: "dummy_robot_messages/msg/State",

            act_pos : ["unknown", "at", "away"],
            active : bool,
        },

        ability!{
            name: to_at,

            enabled : p!([active] && [ref_pos != "at"] && [act_pos != "at"]),
            executing : p!([active] && [ref_pos == "at"] && [act_pos != "at"]),
            finished : p!([active] && [ref_pos == "at"] && [act_pos == "at"]),

            *start : p!(enabled) => [ a!(ref_pos = "at") ] / [a!(act_pos = "unknown")],
            finish : p!(executing) => [] / [a!(act_pos = "at")]
        },

        ability!{
            name: to_away,

            enabled : p!([active] && [ref_pos != "away"] && [act_pos != "away"]),
            executing : p!([active] && [ref_pos == "away"] && [act_pos != "away"]),
            finished : p!([active] && [ref_pos == "away"] && [act_pos == "away"]),

            *start : p!(enabled) => [ a!(ref_pos = "away") ] / [a!(act_pos = "unknown")],
            finish : p!(executing) => [] / [a!(act_pos = "away")]
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

#[test]
fn test_dummy() {
    let r1 = make_dummy_robot("r1");
    println!("{:#?}", r1);

    let r1_p_a = r1.find_item("act_pos", &["r1"]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let r1_p_a = m.find_item("act_pos", &[]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    let rm = make_runner_model(&m);

    println!("{:#?}", rm);
    assert!(false);
}
