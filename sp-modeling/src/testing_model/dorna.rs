use sp_domain::*;
use crate::modeling::*;

pub fn create_instance(name: &str, poses: &[&str]) -> Resource {
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(poses.iter());
    resource! {
        name: name,
        command!{
            topic: "goal",
            msg_type: "robot_msgs/msg/RobotGoal",

            ref_pos : domain,
        },
        measured!{
            topic: "measured",
            msg_type: "robot_msgs/msg/RobotState",

            act_pos : domain,
        },
        estimated!{
            prev_pos: domain,
        },

        predicates!{
            moving: p!(ref_pos <!> act_pos),
        },

        transitions!{
            c_move_start : p!(!moving), vec![ a!(prev_pos <- act_pos), a!(ref_pos?) ],
            e_move_finish : p!(moving), vec![a!(act_pos <- ref_pos)],
        },

        never!{
            name: cannot_go_to_unknown,
            prop: p!(ref_pos == "unknown")
        },

    }
}

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
    fn test_dorna() {
        let dorna = create_instance("dorna", &["at", "away"]);
        println!("{:#?}", dorna);

        let dorna_act = dorna
            .find_item("act_pos", &["dorna"])
            .expect("check spelling")
            .path();
        println!("act path: {}", dorna_act);

        let dorna_ref = dorna
            .find_item("ref_pos", &["dorna"])
            .expect("check spelling")
            .path();
        println!("ref path: {}", dorna_ref);

        let m = Model::new_no_root("one_dorna_model", vec![SPItem::Resource(dorna)]);

        let dorna_act = m.find_item("act_pos", &[]).expect("check spelling").path();
        println!("path: {}", dorna_act);

        let dorna_ref = m.find_item("ref_pos", &[]).expect("check spelling").path();
        println!("path: {}", dorna_ref);
    }
}
