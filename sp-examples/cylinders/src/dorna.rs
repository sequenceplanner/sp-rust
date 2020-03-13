use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_dorna(name: &str, poses: &[&str]) -> Resource {
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(poses.iter());
    resource! {
        name: name,
        command!{
            topic: "command",
            msg_type: "ros2_dorna_msgs/msg/Goal",

            ref_pos : domain,
        },
        measured!{
            topic: "state",
            msg_type: "ros2_dorna_msgs/msg/State",

            act_pos : domain,
        },
        estimated!{
            prev_pos: domain,
        },

        ability!{
            name: move_to,

            enabled : p!(ref_pos <-> act_pos),
            executing : p!(ref_pos <!> act_pos),
            finished : p!(ref_pos <-> act_pos),

            *start : p!(enabled) => [ a!(prev_pos <- act_pos), a!(ref_pos?) ] / [a!(act_pos = "unknown")],
            finish : p!(executing) => [] / [a!(act_pos <- ref_pos)],
        },

        never!{
            name: cannot_go_to_unknown,
            prop: p!(ref_pos == "unknown")
        },

    }
}

#[test]
fn test_dorna() {
    let r1 = make_dorna("r1", &["at", "away"]);
    println!("{:#?}", r1);

    let r1_p_a = r1
        .find_item("act_pos", &["r1"])
        .expect("check spelling")
        .path();
    println!("path: {}", r1_p_a);

    let m = Model::new_root("one_dorna_model", vec![SPItem::Resource(r1)]);

    let r1_p_a = m.find_item("act_pos", &[]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    // println!("{:#?}", rm);
    assert!(false);
}
