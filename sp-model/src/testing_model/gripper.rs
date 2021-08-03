use sp_domain::*;
use crate::modeling::*;

pub fn create_instance(name: &str) -> Resource {
    resource! {
        name: name,
        command!{
            topic: "goal",
            msg_type: "gripper_msgs/msg/Goal",

            close : bool,
        },
        measured!{
            topic: "state",
            msg_type: "gripper_msgs/msg/State",

            closed : bool,
            part_sensor : bool,
        },
        estimated!{

        },

        predicates!{
            opening: p!([!close] && [closed]),
            closing: p!([close] && [!closed]),
        },

        transitions!{
            // close
            c_close_start : p!([!closed]), vec![ a!(close) ],
            e_close_finish_part : p!(closing), vec![a!(closed), a!(part_sensor)],
            e_close_finish_no_part : p!(closing), vec![a!(closed), a!(!part_sensor)],

            // open
            c_open_start : p!(closed), vec![ a!(!close) ],
            e_open_finish : p!(opening), vec![a!(!closed), a!(!part_sensor)],
        },

        never!{
            name: state_does_not_exist,
            prop: p!([!closed] && [part_sensor])
        },

    }
}