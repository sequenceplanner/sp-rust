use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_camera(name: &str) -> Resource {
    resource! {
        name: name,
        command!{
            topic: "goal",
            msg_type: "camera_msgs/msg/Goal",

            do_scan : bool,
        },
        measured!{
            topic: "state",
            msg_type: "camera_msgs/msg/State",

            scanning : bool,
            done : bool,
            result : vec![0,1,2,3],
        },

        ability!{
            name: scan,

            enabled : p!([!do_scan] && [!scanning]),
            started: p!([do_scan] && [!scanning]),
            executing : p!([do_scan] && [scanning] && [!done]),
            finished : p!([do_scan] && [scanning] && [done]),

            *start : p!(enabled) => [ a!(do_scan) ] / [],
            starting : p!(started) => [] / [a!(scanning), a!(!done)],
            finish_0 : p!(executing) => [] / [a!(done), a!(scanning), a!(result = 0)],
            finish_1 : p!(executing) => [] / [a!(done), a!(scanning), a!(result = 1)],
            finish_2 : p!(executing) => [] / [a!(done), a!(scanning), a!(result = 2)],
            finish_3 : p!(executing) => [] / [a!(done), a!(scanning), a!(result = 3)],
            *reset : p!(finished) => [a!(!do_scan)] / [a!(!done), a!(!scanning), a!(result = 0)],
        },
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
    fn test_camera() {
        let camera = make_camera("camera");
        println!("{:#?}", camera);
    }
}
