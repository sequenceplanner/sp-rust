use sp_domain::*;
use sp_runner::*;
use serial_test::serial;
use sp_smt::*;
use std::collections::HashMap;

mod test_models;
use test_models::*;

// use std::collections::HashMap; // todo: macro depends on this...

pub fn make_dummy_robot_1(name: &str, poses: &[&str]) -> Resource {
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(poses.iter());
    resource!{
        name: name,
        command!{
            topic: "Control",
            msg_type: "dummy_robot_messages/msg/Control",

            ref_pos : domain,
        },
        measured!{
            topic: "State",
            msg_type: "dummy_robot_messages/msg/State",

            act_pos : domain,
        },
        estimated!{
            prev_pos: domain,
        },

        ability!{
            name: move_to,

            enabled : p!([ref_pos <-> act_pos]),
            executing : p!([ref_pos <!> act_pos]),
            finished : p!([ref_pos <-> act_pos]),

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
#[serial]
fn test_monolithic_planning_1() {
    let mut m = Model::new_root("htn_test_robot", Vec::new());

    m.add_item(SPItem::Resource(make_dummy_robot_1("r1", &["home", "table", "buffer"])));

    let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
    let r1r = m.find_item("ref_pos", &["r1"]).expect("check spelling").path();
    let r1p = m.find_item("prev_pos", &["r1"]).expect("check spelling").path();

    let table_through_home = &p!([p:r1a == "table"] => [[p:r1p == "table"] || [p:r1p == "home"] || [p:r1p == "unknown"]]);
    let buffer_through_home = &p!([p:r1a == "buffer"] => [[p:r1p == "buffer"] || [p:r1p == "home"] || [p:r1p == "unknown"]]);
    let new_table_through_home = refine_invariant(&m, table_through_home);
    let new_buffer_through_home = refine_invariant(&m, buffer_through_home);

    m.add_item(SPItem::Spec(Spec::new("table_through_home", new_table_through_home)));
    m.add_item(SPItem::Spec(Spec::new("buffer_through_home", new_buffer_through_home)));

    let mut ts_model = TransitionSystemModel::from(&m);;

    let state = state! {
        r1a => "home",
        r1r => "home",
        r1p => "home"
    };

    let g1 = p!(p:r1a == "table");

    let goals = vec![(g1, None)];

    let result = plan(&ts_model, goals.as_slice(), &state, 20);
}

#[test]
#[serial]
fn test_monolithic_planning_2() {
    let mut m = GModel::new("htn_test_robot");

    let r1 = m.use_resource(make_dummy_robot_1("r1", &["home", "table", "buffer"]));

    let products = &["cube".to_spvalue(), "none".to_spvalue()];

    let r1a = &r1["act_pos"];
    let r1p = &r1["prev_pos"];
    let r1r = &r1["ref_pos"];

    let robot_holding = m.add_estimated_domain("r1_holding", products);
    let table_holding = m.add_estimated_domain("table1_holding", products);
    let buffer_holding = m.add_estimated_domain("buffer1_holding", products);

    // m.add_invar()
    m.add_invar("table_through_home", &p!([p:r1a == "table"] => [[p:r1p == "table"] || [p:r1p == "home"] || [p:r1p == "unknown"]]));
    m.add_invar("buffer_through_home", &p!([p:r1a == "buffer"] => [[p:r1p == "buffer"] || [p:r1p == "home"] || [p:r1p == "unknown"]]));

    m.add_delib("r_take_table", &p!([p:r1a == "table"] && [p:table_holding != "none"] && [p:robot_holding == "none"]),
                &[a!(p:robot_holding <- p:table_holding), a!(p:table_holding = "none")]);

    m.add_delib("r_take_buffer", &p!([p:r1a == "buffer"] && [p:buffer_holding != "none"] && [p:robot_holding == "none"]),
               &[a!(p:robot_holding <- p:buffer_holding), a!(p:buffer_holding = "none")]);

    m.add_delib("r_leave_table", &p!([p:r1a == "table"] && [p:robot_holding != "none"] && [p:table_holding == "none"]),
                &[a!(p:table_holding <- p:robot_holding), a!(p:robot_holding = "none")]);

    m.add_delib("r_leave_buffer", &p!([p:r1a == "buffer"] && [p:robot_holding != "none"] && [p:buffer_holding == "none"]),
               &[a!(p:buffer_holding <- p:robot_holding), a!(p:robot_holding = "none")]);

    m.initial_state(&[
        (r1a, "home".to_spvalue()),
        (r1r, "home".to_spvalue()),
        (r1p, "home".to_spvalue()),
        (&robot_holding, "none".to_spvalue()),
        (&table_holding, "none".to_spvalue()),
        (&buffer_holding, "cube".to_spvalue())
    ]);

    let (m, s) = m.make_model();

    let mut ts_model = TransitionSystemModel::from(&m);;

    let goal = p!([p:r1a == "buffer"] && [p:table_holding == "cube"]);

    let goals = vec![(goal, None)];

    let result = plan(&ts_model, goals.as_slice(), &s, 20);
}