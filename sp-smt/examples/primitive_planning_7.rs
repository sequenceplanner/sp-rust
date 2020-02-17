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

    let zero = int_z3!(&ctx, 0);
    let one = int_z3!(&ctx, 1);
    let two = int_z3!(&ctx, 2);
    let hundred = int_z3!(&ctx, 10);

    // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, hundred_var, hundred));

    let int_sort = IntSortZ3::new(&ctx);

    let poses = &pose_sort.enum_asts;

    let mut step: i32 = 0;
    let max_steps: i32 = 102;

    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose0");
    let x0 = IntVarZ3::new(&ctx, &int_sort, "x0");
    // let x0 = int_var_z3!(&ctx, "x0");

    // initial state:
    slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, eq_z3!(&ctx, x0, zero), eq_z3!(&ctx, pose0, poses[0])));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, eq_z3!(&ctx, x0, hundred), eq_z3!(&ctx, pose0, poses[3])));

    let now = std::time::Instant::now();

    while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
        // println!("step: {:?}", step);
        step = step + 1;
        SlvPopZ3::new(&ctx, &slv, 1);

        // initial state:
        // slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, eq_z3!(&ctx, x0, zero), eq_z3!(&ctx, pose0, poses[0])));

        let current_step: &str = &format!("pose{}", step); // &pose_string[..];
        let previous_step: &str = &format!("pose{}", step - 1);

        let current_x: &str = &format!("x{}", step); // &pose_string[..];
        let previous_x: &str = &format!("x{}", step - 1);

        // let current_x = step; // &pose_string[..];
        // let previous_x = step - 1;

        // let inc1 = and_z3!(&ctx,
        //     eq_z3!(&ctx, IntVarZ3::new(&ctx, previous_x), IntZ3::new(&ctx, step - 1)), 
        //     eq_z3!(&ctx, IntVarZ3::new(&ctx, current_x), IntZ3::new(&ctx, step));

        let via1_via1 = and_z3!(&ctx,
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, previous_x), IntZ3::new(&ctx, &int_sort, step - 1)), 
            eq_z3!(&ctx, IntVarZ3::new(&ctx, &int_sort, current_x), IntZ3::new(&ctx, &int_sort, step)),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, previous_step), poses[1]), 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[1]));
            
        let via2_via2 = and_z3!(&ctx,
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, previous_step), poses[2]), 
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[2]));

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

        slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, via1_via1, t_home_via1, t_via1_table)); //, t_home_via2, t_via2_table));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, via1_via1, bool_var_z3!(&ctx, &format!("via1_via1_{}", step))));

        SlvPushZ3::new(&ctx, &slv);
    
        // slv_assert_z()
        
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_home_via1, bool_var_z3!(&ctx, "t_home_via1")));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_via1_table, bool_var_z3!(&ctx, "t_via1_table")));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_home_via2, bool_var_z3!(&ctx, "t_home_via2")));
        slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_via2_table, bool_var_z3!(&ctx, "t_via2_table")));

        // // goal state:
        // slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, 
        //     eq_z3!(&ctx, current_x, hundred), 
        //     eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, current_step), poses[3])));

    }

    println!("Solving {}ms\n", now.elapsed().as_millis());

    let model = slv_get_model_z3!(&ctx, &slv);

    let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

    for model in models {
        println!("{}", model);
    }
    println!("Models {}ms\n", now.elapsed().as_millis());
}