use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

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
    let max_steps: u32 = 20;

    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose0");

    // initial state:
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[0]));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[3]));

    while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
        println!("step: {:?}", step);
        step = step + 1;

        // println!("backtracking {:?}", SlvGetPopPointsZ3::new(&ctx, &slv));

        //if SlvGetPopPointsZ3::new(&ctx, &slv) != 0 {
        SlvPopZ3::new(&ctx, &slv, 1);
        //}

        // initial state:
        // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[0]));

        let current_step: &str = &format!("pose{}", step); // &pose_string[..];
        let previous_step: &str = &format!("pose{}", step - 1);

        // println!("current_step: {:?}", current_step);
        // println!("previous_step: {:?}", previous_step);

        let t_home_via1 = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, previous_step), poses[0]), 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[1]));

        let t_home_via2 = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, previous_step), poses[0]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[2]));

        let t_via1_table = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, previous_step), poses[1]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[3]));

        let t_via2_table = and_z3!(&ctx, 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, previous_step), poses[2]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[3]));

        SlvPushZ3::new(&ctx, &slv);

        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_home_via1, bool_var_z3!(&ctx, "t_home_via1")));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_via1_table, bool_var_z3!(&ctx, "t_via1_table")));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_home_via2, bool_var_z3!(&ctx, "t_home_via2")));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_via2_table, bool_var_z3!(&ctx, "t_via2_table")));
        slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, t_home_via1, t_home_via2, t_via1_table, t_via2_table));

        // goal state:
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[3]));

    }

    let model = slv_get_model_z3!(&ctx, &slv);

    let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

    for model in models {
        println!("{}", model);
    }
}