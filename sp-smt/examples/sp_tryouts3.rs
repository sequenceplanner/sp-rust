use sp_smt::*;

use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
use sp_runner::*;

fn main() {

    pub fn r(name: &str) -> Resource {
    
        let mut domain = vec!["home", "via1", "via2", "table"];

        resource!{
            name: name,
            estimated!{
                pose: domain,
            },
    
            ability!{
                name: movement,
                home_via_1: p!([pose == "home"]) => [pose = "via1"],
                home_via_2: p!([pose == "home"]) => [pose = "via2"],
                via_1_table: p!([pose == "via1"]) => [pose = "table"],
                via_2_table: p!([pose == "via2"]) => [pose = "table"],
                // home_to_via_1: p!([[!x] && [y != "a"]] || [y != "b"] || [x == false]) => [ a!(x), a!(z <- y) ] / [a!(y = "c")]
            }
        }
    }

    let mut m = Model::new_root("model_x", Vec::new());
    m.add_item(SPItem::Resource(r("resource_x")));

    let pose = m.find_item("pose", &[]).expect("check spelling1").path();

    let state = SPState::new_from_values(&[
        (pose.clone(), "home".to_spvalue()),
    ]);

    let ts_model = TransitionSystemModel::from(&m);

    let plan = ComputePlanSPModelZ3::new(&ts_model, &[(p!([pose = "table"]), None)]);

}