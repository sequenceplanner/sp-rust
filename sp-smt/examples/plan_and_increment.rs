use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;
use std::time::{Duration, Instant};

fn main() {

    // problem descriprion:
    println!("\n Find all paths to go from home to the table, concidering x as well:
    
              home and x = 0 
                   .  . 
           . .    .    .    . .
          .   .  .      .  .   .
        x+1   via1     via2    x+2
          .   .  .      .  .   .
           . .    .    .    . .
                   .  .  
             table and x = 12\n");

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    let pose_sort = EnumSortZ3::new(&ctx, "pose_sort", vec!("home", "via1", "via2", "table"));

    let zero = int_z3!(&ctx, 0);
    let one = int_z3!(&ctx, 1);
    let two = int_z3!(&ctx, 2);
    let goal_x = int_z3!(&ctx, 424);

    let int_sort = IntSortZ3::new(&ctx);

    let poses = &pose_sort.enum_asts;

    let mut step: u32 = 0;
    let mut path: u32 = 0;
    let mut step2: u32 = 0;
    let max_steps: u32 = 1000;
    let max_paths: u32 = 1000;

    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose_s0");

    let x0 = IntVarZ3::new(&ctx, &int_sort, "x_s0");

    // initial state:
    slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, eq_z3!(&ctx, x0, zero), eq_z3!(&ctx, pose0, poses[0])));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, eq_z3!(&ctx, x0, goal_x), eq_z3!(&ctx, pose0, poses[3])));

    while path < max_paths {
        path = path + 1;
        while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
            step = step + 1;
            step2 = step2 + 2;
            SlvPopZ3::new(&ctx, &slv, 1);

            let current_step: &str = &format!("pose_s{}", step - 1);
            let next_step: &str = &format!("pose_s{}", step);

            let current_x: &str = &format!("x_s{}", step - 1);
            let next_x: &str = &format!("x_s{}", step);

            let via1_via1 = and_z3!(&ctx,
                eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), IntZ3::new(&ctx, &int_sort, step as i32 - 2)),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[1]),
                eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), IntZ3::new(&ctx, &int_sort, step as i32 - 1)),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[1]));

            let via2_via2 = and_z3!(&ctx,
                eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), IntZ3::new(&ctx, &int_sort, step2 as i32 - 4)),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[2]), 
                eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), IntZ3::new(&ctx, &int_sort, step2 as i32 - 2)),
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
            
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via1_via1, bool_var_z3!(&ctx, &format!("via1_via1_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via2_via2, bool_var_z3!(&ctx,  &format!("via2_via2_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via1, bool_var_z3!(&ctx, &format!("home_via1_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via1_table, bool_var_z3!(&ctx, &format!("via1_table_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, home_via2, bool_var_z3!(&ctx, &format!("home_via2_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via2_table, bool_var_z3!(&ctx, &format!("via2_table_t{}", step))));

            SlvPushZ3::new(&ctx, &slv);
            
            // goal state:
            slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, 
                eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, next_x), goal_x), 
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, next_step), poses[3])));

        }

    let now = Instant::now();

    println!("\nPath: {:?}", path);
    let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
    let frames = GetPlanningFramesZ3::new(&ctx, model, step);
    
    println!("Solving time: {} milliseconds", now.elapsed().as_millis());

    for l in frames {
        println!("{:?} : {:?} : {:?}", l.0, l.1, l.2);
        }
    }


}