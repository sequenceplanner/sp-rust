use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

fn main() {


    // sp stuff:  
    // let x = SPPath::from_string("x");
    // let y = SPPath::from_string("y");
    // let vars = vec![x, y];
    
    // let t = Transition::new("tx", p!(!x), vec![a!(x)], vec![], true);

    // let init = p!([!x] && [!y]);

    // println!("vars: {:?}", vars);
    // println!("t: {:?}",t );
    // println!("init: {:?}",init );


    // problem descriprion:
    println!("\n Find a plan to go from home to the table:
    
            home 
            . . 
           .   .
          .     . 
        via1    via2
          .     . 
           .   .
            . .  
           table\n");

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    let pose_sort = EnumSortZ3::new(&ctx, "pose_sort", vec!("home", "via1", "via2", "table"));
    let poses = &pose_sort.enum_asts;

    let mut step: usize  = 0;
    let max_steps: usize = 20;

    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose_s0");

    // initial state:
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[0]));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[3]));

    while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
        // println!("step: {:?}", step);
        step = step + 1;
        SlvPopZ3::new(&ctx, &slv, 1);

        // initial state:
        // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[0]));

        let current_step: &str = &format!("pose_s{}", step - 1);
        let next_step: &str = &format!("pose_s{}", step);
        

        // goal state:
        // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3]));

        let home_via1 = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[0]), 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[1]));

        let home_via2 = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[0]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[2]));

        let via1_table = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[1]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3]));

        let via2_table = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[2]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3]));

        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via1, bool_var_z3!(&ctx, &format!("home_via1_t{}", step))));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via1_table, bool_var_z3!(&ctx, &format!("via1_table_t{}", step))));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via2, bool_var_z3!(&ctx, &format!("home_via2_t{}", step))));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via2_table, bool_var_z3!(&ctx, &format!("via2_table_t{}", step))));

        slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, home_via1, via1_table, home_via2, via2_table));

        SlvPushZ3::new(&ctx, &slv);
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3]));
    }

    let model = slv_get_model_z3!(&ctx, &slv);
    let num = ModelGetNumConstsZ3::new(&ctx, model);
    let mut frames = vec!();
    unsafe {
    for i in 0..max_steps {
        let mut frame: (Vec<String>, String) = (vec!(), "".to_string());
        for j in 0..num {
            
            let decl = ModelGetConstDeclZ3::new(&ctx, model, j);
            let var = GetDeclNameZ3::new(&ctx, model, decl);
            let var_str_init = GetSymbolStringZ3::new(&ctx, var);
            let var_str = &CStr::from_ptr(var_str_init).to_str().unwrap().to_owned();
            let val = ModelGetConstInterpZ3::new(&ctx, model, decl);
            let val_str = AstToStringZ3::new(&ctx, val);

            if var_str.ends_with(&format!("_s{}", i)) {
                // var_str.replace(&format!("_s{}", i), "");
                frame.0.push(val_str.to_string());
            } else if var_str.ends_with(&format!("_t{}", i)) && val_str == "true" {
                // var_str.replace(&format!("_t{}", i), "");
                frame.1 = var_str.to_string();
            }
            }
            if frame.0.len() != 0 {
                frames.push(frame);
            } 
            
        }
    }

    for l in 0..frames.len() {
        println!("{:?} : {:?} : {:?}", l, frames[l].0, frames[l].1);
    }
    // for s in max_steps {
    //     println!("{}", model[])
    // }

    // let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

    // for model in models {
    //     println!("{}", model);
    // }
}