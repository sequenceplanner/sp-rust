use sp_smt::*;

use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
// use sp_runner::*;

fn main() {

    // pub fn r(name: &str) -> Resource {
    
    //     let mut domain = vec!["home", "via1", "via2", "table"];

    //     resource!{
    //         name: name,
    //         estimated!{
    //             pose: domain,
    //         },
    
    //         ability!{
    //             name: movement,

    //             home_via_1: p!([pose == "home"]) => [a!(pose = "via1")] / [],
    //             home_via_2: p!([pose == "home"]) => [a!(pose = "via2")] / [],
    //             via_1_table: p!([pose == "via1"]) => [a!(pose = "table")] / [],
    //             via_2_table: p!([pose == "via2"]) => [a!(pose = "table")] / [],
    //         }
    //     }
    // }

    // let mut m = Model::new_root("model_x", Vec::new());
    // m.add_item(SPItem::Resource(r("resource_x")));

    // let pose = m.find_item("pose", &[]).expect("check spelling1").path();

    // let init_state = SPState::new_from_values(&[
    //     (pose.clone(), "home".to_spvalue()),
    // ]);

    // // let goal_state = SPState::new_from_values(&[
    // //     (pose.clone(), "table".to_spvalue()),
    // // ]);

    // let ts_model = TransitionSystemModel::from(&m);

    // let plan = SeqComputePlanSPModelZ3::plan(&ts_model, &[(p!([p:pose == "table"]), None)], &init_state, 10);
    // for l in plan {
    //     println!("{:?} : {:?} : {:?}", l.0, l.1, l.2);
    // }
}