use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_control_box(name: &str) -> Resource {
    resource! {
        name: name,
        command!{
            topic: "goal",
            msg_type: "control_box_msgs/msg/Goal",

            blue_light : bool,
        },
        measured!{
            topic: "state",
            msg_type: "control_box_msgs/msg/State",

            blue_light_on : bool,
        },

        ability!{
            name: blue_on,

            enabled : p!(!blue_light_on),
            executing : p!([blue_light] && [!blue_light_on]),
            finished : p!(blue_light_on),

            *start : p!(enabled) => [ a!(blue_light) ] / [],
            finish : p!(executing) => [] / [a!(blue_light_on)],
        },

        ability!{
            name: blue_off,

            enabled : p!(blue_light_on),
            executing : p!([!blue_light] && [blue_light_on]),
            finished : p!(!blue_light_on),

            *start : p!(enabled) => [ a!(!blue_light) ] / [],
            finish : p!(executing) => [] / [a!(!blue_light_on)],
        },
    }
}

#[test]
fn test_control_box() {
    let control_box = make_control_box("cb");
    println!("{:#?}", control_box);

    let bl = control_box.find_item("blue_light", &["control_box"]).expect("check spelling").path();
    println!("bl: {}", bl);

    let bl_on = control_box.find_item("blue_light_on", &["control_box"]).expect("check spelling").path();
    println!("bl on: {}", bl_on);

    assert!(false);
}
