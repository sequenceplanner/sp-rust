use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

use sp_smt::*;
use sp_domain::*;
// use sp_runner::*;
use serial_test::serial;
use std::collections::HashMap;

// pub fn make_dummy_robot(name: &str, poses: &[&str]) -> Resource {
//     // domain is a list of saved poses. add "unknown" to this list.
//     let mut domain = vec!["unknown"];
//     domain.extend(poses.iter());
//     resource!{
//         name: name,
//         command!{
//             topic: "Control",
//             msg_type: "dummy_robot_messages/msg/Control",

//             ref_pos : domain,
//             activate : bool,
//         },
//         measured!{
//             topic: "State",
//             msg_type: "dummy_robot_messages/msg/State",

//             act_pos : domain,
//             active : bool,

//             echo / ref_pos : domain,   // these are used for handshaking.
//             echo / activate : bool,    // "echo" has special meaning for us.
//         },
//         estimated!{
//             prev_pos: domain,
//         },

//         ability!{
//             name: move_to,

//             enabled : p!([active] && [ref_pos <-> act_pos]),
//             executing : p!([active] && [ref_pos <!> act_pos]),
//             finished : p!([active] && [ref_pos <-> act_pos]),

//             *start : p!(enabled) => [ a!(ref_pos?) ] / [a!(act_pos = "unknown")],
//             finish : p!(executing) => [] / [a!(act_pos <- ref_pos)],

//             sync_prev: p!([prev_pos <!> ref_pos] && [ref_pos <-> act_pos]) => [ a!(prev_pos <- ref_pos) ] / []
//         },

//         never!{
//             name: cannot_go_to_unknown,
//             prop: p!(ref_pos == "unknown")
//         },

//         ability!{
//             name: activate,

//             enabled: p!([!activate] && [!active]),
//             executing: p!([activate] && [!active]),
//             finished: p!([activate] && [active]),

//             *start: p!(enabled) => [ a!(activate) ] / [],
//             finish: p!(executing) => [] / [ a!(active) ],
//         },

//         ability!{
//             name: deactivate,

//             enabled: p!([activate] && [active]),
//             executing: p!([!activate] && [active]),
//             finished: p!([!activate] && [!active]),

//             *start: p!(enabled) => [ a!(!activate) ] / [],
//             finish: p!(executing) => [] / [ a!(!active) ],
//         },
//     }
// }

// // ie guard extraction to satisfy global spec.
// fn basic_model() -> Model {
//     let mut m = Model::new_root("dummy_robot_model", Vec::new());
//     m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
//     m
// }

// // ie guard extraction to satisfy global spec.
// fn model_with_global_spec() -> Model {
//     let mut m = Model::new_root("dummy_robot_model", Vec::new());

//     // Make resoureces
//     m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
//     m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

//     // Make some global stuff
//     let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling1").path();
//     let r2a = m.find_item("act_pos", &["r2"]).expect("check spelling2").path();

//     // (global offline) Specifications
//     let table_zone = p!(!([p:r1a == "at"] && [p:r2a == "at"]));
//     m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

//     m
// }

// // ie without guard extraction to satisfy global spec. needs the spec as invariant during planning
// fn model_without_spec() -> Model {
//     let mut m = Model::new_root("dummy_robot_model", Vec::new());

//     // Make resoureces
//     m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
//     m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

//     // !
//     // No specifications
//     // !

//     m
// }

fn main() {

    // let mut m = Model::new_root("tsi", Vec::new());

    // // Make resoureces
    // m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    // m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
    // let r1r = m.find_item("ref_pos", &["r1"]).expect("check spelling").path();
    // let r1active = m.find_item("active", &["r1"]).expect("check spelling").path();
    // let r1activate = m.find_item("activate", &["r1", "Control"]).expect("check spelling").path();

    // let r2a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();
    // let r2r = m.find_item("ref_pos", &["r2"]).expect("check spelling").path();
    // let r2active = m.find_item("active", &["r2"]).expect("check spelling").path();
    // let r2activate = m.find_item("activate", &["r2", "Control"]).expect("check spelling").path();


    // let table_zone = p!(!( [p:r1a == "at"] && [p:r2a == "at"]));
    // let new_table_zone = refine_invariant(&m, &table_zone);

    // // no guard extr.
    // let ts_model = TransitionSystemModel::from(&m);

    // let state = state! {
    //     r1a => "away",
    //     r1r => "away",
    //     r1active => false,
    //     r1activate => false,
    //     r2a => "away",
    //     r2r => "away",
    //     r2active => false,
    //     r2activate => false
    // };

    // // start planning test

    // let i1 = new_table_zone.clone();
    // let g1 = p!(p:r1a == "at");

    // let i2 = new_table_zone.clone();
    // let g2 = p!(p:r2a == "at");

    // let goals = vec![(g1, Some(i1)), (g2, Some(i2))];

    // // let result = SubParComputePlanSPModelZ3::plan(&ts_model, goals.as_slice(), &state, vec!(0,1,2,3,4,5,6,7,8,9,10,11,12,13));
    // let result = SeqComputePlanSPModelZ3::plan(&ts_model, goals.as_slice(), &state, 20);
    // assert!(result.plan_found);
    // println!("Z3 TIME: {:?}", result.time_to_solve);
    // // for f in &result.trace {
    // //     println!("==========================");
    // //     println!("{}", f.transition);
    // //     println!("==========================");
    // //     println!("{}", f.state);

    // // }
    // assert_eq!(result.trace.len(), 11);
    // assert_eq!(result.plan_length, 10);
}