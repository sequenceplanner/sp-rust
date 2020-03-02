use sp_smt::*;

use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
use sp_runner::*;

fn main() {

    pub fn r(name: &str) -> Resource {
    
        let mut domain = vec!["a", "b", "c"];

        resource!{
            name: name,
            estimated!{
                x: bool,
                y: domain,
                z: domain,
            },
    
            ability!{
                name: ability_x,
    
                change_x: p!([[!x] && [y != "a"]] || [y != "b"] || [x == false]) => [ a!(x), a!(z <- y) ] / [a!(y = "c")]
            }
        }
    }

    let mut m = Model::new_root("model_x", Vec::new());
    m.add_item(SPItem::Resource(r("resource_x")));

    let x = m.find_item("x", &[]).expect("check spelling1").path();
    let y = m.find_item("y", &[]).expect("check spelling2").path();
    let z = m.find_item("z", &[]).expect("check spelling3").path();

    let state = SPState::new_from_values(&[
        (x.clone(), false.to_spvalue()),
        (y.clone(), "a".to_spvalue()),
        (z.clone(), "b".to_spvalue()),
    ]);

    let ts_model = TransitionSystemModel::from(&m);

    println!("{:#?}", ts_model);

    let cfg = ConfigZ3::new();
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let init_state = GetInitialStateZ3::new(&ctx, &ts_model, &state);
    slv_assert_z3!(&ctx, &slv, init_state);
    println!("Added initial state to context");
    println!("{}", slv_to_string_z3!(&ctx, &slv));

    
    
    let mut step: u32 = 0;
    let max_steps: u32 = 10;
    // while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
    while step < max_steps {
        step = step + 1;
        // SlvPopZ3::new(&ctx, &slv, 1);
    // trans = Vec(trans_name, Vec(var, var_type, var_value))
        // let mut trans: Vec<(String, Z3_ast, Vec<(String, String, String)>)> = vec!();
        for t in &ts_model.transitions {
            // let mut tr: (Z3_ast, Z3_ast, Vec<(Z3_ast)>);
            println!("{}", ast_to_string_z3!(&ctx, GetSPPredicateZ3::new(&ctx, &ts_model, step - 1, t.guard())));
            println!("{}", ast_to_string_z3!(&ctx, GetSPUpdatesZ3::new(&ctx, &ts_model, &t, step)));
            // println!("updates: {:?}", updates);
        }
        // let goal = guard_to_z3(&ctx, &ts_model, step: u32, p: &Predicate); 
    }
}