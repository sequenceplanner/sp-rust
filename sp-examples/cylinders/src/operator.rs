use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_operator(name: &str, goals: &[&str]) -> Resource {
    // TODO: Handle multiple operators on the ROS_side
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(goals.iter());
    resource! {
        name: "sp_operator",
        command!{
            topic: "goal",
            msg_type: "sp_operator_msgs/msg/Goal",

            to_do : domain,
        },
        measured!{
            topic: "state",
            msg_type: "sp_operator_msgs/msg/State",

            doing : domain,
            e_stop: bool,
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
    let dorna = make_dorna("dorna", &["at", "away"]);
    println!("{:#?}", dorna);

    let dorna_act = dorna.find_item("act_pos", &["dorna"]).expect("check spelling").path();
    println!("act path: {}", dorna_act);

    let dorna_ref = dorna.find_item("ref_pos", &["dorna"]).expect("check spelling").path();
    println!("ref path: {}", dorna_ref);


    let m = Model::new_no_root("one_dorna_model", vec![SPItem::Resource(dorna)]);

    let dorna_act = m.find_item("act_pos", &[]).expect("check spelling").path();
    println!("path: {}", dorna_act);

    let dorna_ref = m.find_item("ref_pos", &[]).expect("check spelling").path();
    println!("path: {}", dorna_ref);



    // println!("{:#?}", rm);
    assert!(false);
}