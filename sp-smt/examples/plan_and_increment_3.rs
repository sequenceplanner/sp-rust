use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

fn main() {

    // problem descriprion:
    println!("\n Find a plan to go from home to the table, concidering x as well:
    Why does it not find both ways for an even x?
    
               home and x = 0 
                   .  . 
           . .    .    .    . .
          .   .  .      .  .   .
        x+1   via1     via2    x+2
          .   .  .      .  .   .
           . .    .    .    . .
                   .  .  
            table and x = 11 (12)\n");

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    let pose_sort = EnumSortZ3::new(&ctx, "pose_sort", vec!("home", "via1", "via2", "table"));

    let zero = int_z3!(&ctx, 0);
    let one = int_z3!(&ctx, 1);
    let two = int_z3!(&ctx, 2);
    let goal_x = int_z3!(&ctx, 21);

    let int_sort = IntSortZ3::new(&ctx);

    let poses = &pose_sort.enum_asts;

    let mut step: i32 = 0;
    let mut step2: i32 = 0;
    let max_steps: i32 = 1000;

    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose_s0");

    let x0 = IntVarZ3::new(&ctx, &int_sort, "x_s0");

    // initial state:
    slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, eq_z3!(&ctx, x0, zero), eq_z3!(&ctx, pose0, poses[0])));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, eq_z3!(&ctx, x0, goal_x), eq_z3!(&ctx, pose0, poses[3])));

    // save to extract frames later:
    // let mut stuff = vec!();
    // let mut transitions_taken = vec!();

    let now = std::time::Instant::now();

    while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
        step = step + 1;
        step2 = step2 + 2;
        SlvPopZ3::new(&ctx, &slv, 1);

        let current_step: &str = &format!("pose_s{}", step - 1);
        let next_step: &str = &format!("pose_s{}", step);

        let current_x: &str = &format!("x_s{}", step - 1);
        let next_x: &str = &format!("x_s{}", step);

        // stuff.push(current_step);
        // stuff.push(next_step);
        // stuff.push(current_x);
        // stuff.push(next_x);

        // of course:
        // current_step = guard
        // next step = action

        let via1_via1 = and_z3!(&ctx,
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), IntZ3::new(&ctx, &int_sort, step - 2)),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[1]),
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), IntZ3::new(&ctx, &int_sort, step - 1)),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[1]));
            
        let via2_via2 = and_z3!(&ctx,
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), IntZ3::new(&ctx, &int_sort, step2 - 4)),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[2]), 
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), IntZ3::new(&ctx, &int_sort, step2 - 2)),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[2]));

        let home_via1 = and_z3!(&ctx,
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), zero),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[0]),
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), zero),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[1]));          

        let home_via2 = and_z3!(&ctx, 
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), zero),
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), zero),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[0]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[2]));

        let via1_table = and_z3!(&ctx, 
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), goal_x),
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), goal_x),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[1]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3]));

        let via2_table = and_z3!(&ctx,
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), goal_x),
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), goal_x),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[2]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3]));

        slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, via1_via1, via2_via2, home_via1, home_via2, via1_table, via2_table));
        
        let via1_via1_name = &format!("via1_via1_t{}", step);
        let via2_via2_name = &format!("via2_via2_t{}", step);
        let home_via1_name = &format!("home_via1_t{}", step);
        let via1_table_name = &format!("via1_table_t{}", step);
        let home_via2_name = &format!("home_via2_t{}", step);
        let via2_table_name = &format!("via2_table_t{}", step);
        
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via1_via1, bool_var_z3!(&ctx, via1_via1_name)));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via2_via2, bool_var_z3!(&ctx, via2_via2_name)));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via1, bool_var_z3!(&ctx, home_via1_name)));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via1_table, bool_var_z3!(&ctx, via1_table_name)));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via2, bool_var_z3!(&ctx, home_via2_name)));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via2_table, bool_var_z3!(&ctx, via2_table_name)));

        // stuff.push(via1_via1_name);
        // stuff.push(via2_via2_name);
        // stuff.push(home_via1_name);
        // stuff.push(via1_table_name);
        // stuff.push(home_via2_name);
        // stuff.push(via2_table_name);

        SlvPushZ3::new(&ctx, &slv);
        
        // goal state:
        slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, 
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), goal_x), 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3])));

    }

    println!("Solving time: {}ms\n", now.elapsed().as_millis());

    let model = slv_get_model_z3!(&ctx, &slv);

    println!("Model generation time: {}ms\n", now.elapsed().as_millis());

    // let num = ModelGetNumConstsZ3::new(&ctx, model);
    // let mut frames = vec!();

    // for i in 0..stuff.len() {
    //     let val = ModelGetConstInterpZ3::new(&ctx, model, stuff[i]);
    //     let val_str = AstToStringZ3::new(&ctx, val);
    // }


    // could be done faster maybe ?
    let num = ModelGetNumConstsZ3::new(&ctx, model);
    let mut frames = vec!();
    unsafe {
    for i in 0..step + 2 {
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
                let trimmed = var_str.trim_end_matches(&format!("_t{}", i));
                frame.1 = trimmed.to_string();
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

    println!("\nFrames extraction time: {}ms\n", now.elapsed().as_millis());
}