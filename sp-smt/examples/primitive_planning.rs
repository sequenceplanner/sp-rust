use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;
use std::time::{Duration, Instant};

fn main() {

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

    let mut step: u32 = 0;
    let mut path: u32 = 0;
    let max_steps: u32 = 100;
    let max_paths: u32 = 100;

    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose_s0");

    // initial state:
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[0]));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[3]));

    while path < max_paths {
        path = path + 1;
        while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
            step = step + 1;
            SlvPopZ3::new(&ctx, &slv, 1);

            let current_step: &str = &format!("pose_s{}", step - 1);
            let next_step: &str = &format!("pose_s{}", step);

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

            slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, home_via1, home_via2, via1_table, via2_table));
            
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via1, bool_var_z3!(&ctx, &format!("home_via1_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via1_table, bool_var_z3!(&ctx, &format!("via1_table_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via2, bool_var_z3!(&ctx, &format!("home_via2_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via2_table, bool_var_z3!(&ctx, &format!("via2_table_t{}", step))));

            SlvPushZ3::new(&ctx, &slv);
            
            // goal state:
            slv_assert_z3!(&ctx, &slv,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3]));

        }

    let now = Instant::now();

    println!("\nPath: {:?}", path);
    let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
    // let frames = GetPlanningFramesZ3::new(&ctx, model, step);
    
    // println!("Solving time: {} milliseconds", now.elapsed().as_millis());

    // for l in frames {
    //     println!("{:?} : {:?} : {:?}", l.0, l.1, l.2);
    //     }
    }
}