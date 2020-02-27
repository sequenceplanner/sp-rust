use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_mecademic(name: &str, poses: &[&str]) -> Resource {
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(poses.iter());
    resource!{
        name: name,
        command!{
            topic: "command",
            msg_type: "cubes_1_msgs/msg/RCommand",

            ref_pos : domain,
        },
        measured!{
            topic: "state",
            msg_type: "cubes_1_msgs/msg/RState",

            act_pos : domain,

            echo / ref_pos : domain,   // these are used for handshaking.
        },
        estimated!{
            prev_pos: domain,
        },

        ability!{
            name: move_to,

            enabled : p!(ref_pos <-> act_pos),
            executing : p!(ref_pos <!> act_pos),
            finished : p!(ref_pos <-> act_pos),

            *start : p!(enabled) => [ a!(ref_pos?) ] / [a!(act_pos = "unknown")],
            finish : p!(executing) => [] / [a!(act_pos <- ref_pos)],

            sync_prev: p!([prev_pos <!> ref_pos] && [ref_pos <-> act_pos]) => [ a!(prev_pos <- ref_pos) ] / []
        },

        never!{
            name: cannot_go_to_unknown,
            prop: p!(ref_pos == "unknown")
        },

    }
}

#[test]
fn test_mecademic() {
    let r1 = make_dummy_mecademic("r1", &["at", "away"]);
    println!("{:#?}", r1);

    let r1_p_a = r1.find_item("act_pos", &["r1"]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    let m = Model::new_root("one_mecademic_model", vec![SPItem::Resource(r1)]);

    let r1_p_a = m.find_item("act_pos", &[]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    // println!("{:#?}", rm);
    assert!(false);
}
