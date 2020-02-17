use sp_domain::*;
use crate::modeling::*;
use std::collections::HashMap; // todo: macro depends on this...

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

#[test]
fn xx_test_dummy() {
    let r1 = make_dummy_robot("r1", &["at", "away"]);
//    println!("{:#?}", r1);

    let r1_p_a = r1.find_item("act_pos", &["r1"]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let r1_p_a = m.find_item("act_pos", &[]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    // println!("{:#?}", rm);
    assert!(false);
}
