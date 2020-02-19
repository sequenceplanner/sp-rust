use sp_domain::*;
use crate::modeling::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_dummy_sm(name: &str) -> Resource {
    resource!{
        name: name,
        command!{
            topic: "Control",
            msg_type: "dummy_sm_messages/msg/Control",

            attach_r1_box: bool,
            attach_r2_box: bool,
        },
        measured!{
            topic: "State",
            msg_type: "dummy_sm_messages/msg/State",

            attached_r1_box: bool,
            attached_r2_box: bool,

            echo / attach_r1_box: bool,
            echo / attach_r2_box: bool,
        },

        ability!{
            name: attach_r1,

            enabled : p!([!attached_r1_box] && [!attach_r1_box]),
            executing : p!([!attached_r1_box] && [attach_r1_box]),
            finished : p!([attached_r1_box] && [attach_r1_box]),

            *start : p!(enabled) => [ a!(attach_r1_box) ] / [],
            finish : p!(executing) => [] / [a!(attached_r1_box)],
        },

        ability!{
            name: detach_r1,

            enabled : p!([attached_r1_box] && [attach_r1_box]),
            executing : p!([attached_r1_box] && [!attach_r1_box]),
            finished : p!([!attached_r1_box] && [!attach_r1_box]),

            *start : p!(enabled) => [ a!(!attach_r1_box) ] / [],
            finish : p!(executing) => [] / [a!(!attached_r1_box)],
        },

        ability!{
            name: attach_r2,

            enabled : p!([!attached_r2_box] && [!attach_r2_box]),
            executing : p!([!attached_r2_box] && [attach_r2_box]),
            finished : p!([attached_r2_box] && [attach_r2_box]),

            *start : p!(enabled) => [ a!(attach_r2_box) ] / [],
            finish : p!(executing) => [] / [a!(attached_r2_box)],
        },

        ability!{
            name: detach_r2,

            enabled : p!([attached_r2_box] && [attach_r2_box]),
            executing : p!([attached_r2_box] && [!attach_r2_box]),
            finished : p!([!attached_r2_box] && [!attach_r2_box]),

            *start : p!(enabled) => [ a!(!attach_r2_box) ] / [],
            finish : p!(executing) => [] / [a!(!attached_r2_box)],
        },

        never!{
            name: cannot_attach_at_the_same_time,
            prop: p!(["attach_r1/executing"] && ["attach_r2/executing"])
        },

    }
}

#[test]
fn test_sm() {
    let r1 = make_dummy_sm("r1");
    println!("{:#?}", r1);

    let m = Model::new_root("one_sm_model", vec![SPItem::Resource(r1)]);

    // println!("{:#?}", rm);
    assert!(false);
}
