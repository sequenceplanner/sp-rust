use sp_domain::*;
use sp_runner::*;

pub fn make_dummy_robot(name: &str, poses: &[&str]) -> Resource {
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(poses.iter());
    resource! {
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
        },
        estimated!{
            prev_pos: domain,
        },

        predicates!{
            move_enabled : p!([active] && [ref_pos <-> act_pos]),
            move_executing : p!([active] && [ref_pos <!> act_pos]),
            move_finished : p!([active] && [ref_pos <-> act_pos]),
        },

        transitions!{
            c_move_start : p!(move_enabled), vec![a!(ref_pos?)],
            e_move_finish : p!(move_executing), vec![a!(act_pos <- ref_pos)],

            a_sync_prev: p!([prev_pos <!> ref_pos] && [ref_pos <-> act_pos]), vec![ a!(prev_pos <- ref_pos) ],

            c_activate_start: p!([!activate] && [!active]), vec![ a!(activate) ],
            e_activate_finish: p!([activate] && [!active]), vec![ a!(active) ],

            c_deactivate_start: p!([activate] && [active]), vec![ a!(!activate) ],
            e_deactivate_finish: p!([!activate] && [active]), vec![ a!(!active) ],
        },

        never!{
            name: cannot_go_to_unknown,
            prop: p!(ref_pos == "unknown")
        },
    }
}

#[test]
fn test_dummy() {
    let r1 = make_dummy_robot("r1", &["at", "away"]);
    //    println!("{:#?}", r1);

    let r1_p_a = r1
        .find_item("act_pos", &["r1"])
        .expect("check spelling")
        .path();
    println!("path: {}", r1_p_a);

    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let r1_p_a = m.find_item("act_pos", &[]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    // println!("{:#?}", rm);
    // assert!(false);
}
