use sp_domain::*;
use crate::modeling::*;

pub fn create_instance(name: &str) -> Resource {
    resource! {
        name: name,
        command!{
            topic: "goal",
            msg_type: "camera_msgs/msg/Goal",

            do_scan : bool,
        },
        measured!{
            topic: "measured",
            msg_type: "camera_msgs/msg/Measured",

            scanning : bool,
            done : bool,
            result : vec![0,1,2,3],
        },

        predicates!{
            enabled : p!([!do_scan] && [!scanning]),
            started: p!([do_scan] && [!scanning]),
            executing : p!([do_scan] && [scanning] && [!done]),
            finished : p!([do_scan] && [scanning] && [done]),
            resetting : p!([!do_scan] && [scanning] && [done]),
        },

        transitions!{
            c_start : p!(enabled), vec![ a!(do_scan) ],
            e_starting : p!(started), vec![a!(scanning), a!(!done)],
            e_finish_0 : p!(executing), vec![a!(done), a!(scanning), a!(result = 0)],
            e_finish_1 : p!(executing), vec![a!(done), a!(scanning), a!(result = 1)],
            e_finish_2 : p!(executing), vec![a!(done), a!(scanning), a!(result = 2)],
            e_finish_3 : p!(executing), vec![a!(done), a!(scanning), a!(result = 3)],
            c_reset : p!(finished), vec![a!(!do_scan)],
            e_reset_effect : p!(resetting), vec![a!(!done), a!(!scanning), a!(result = 0)],
        },

        never!{
            name: state_does_not_exist,
            prop: p!([!do_scan] && [scanning] && [!done])
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
    fn test_camera() {
        let camera = create_instance("camera");
        println!("{:#?}", camera);
    }
}
