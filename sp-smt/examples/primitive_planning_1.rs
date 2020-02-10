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
    
    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose0");
    let pose1 = EnumVarZ3::new(&ctx, pose_sort.r, "pose1");
    let pose2 = EnumVarZ3::new(&ctx, pose_sort.r, "pose2");
    
    let poses = &pose_sort.enum_asts;

    // initial state:
    let init = eq_z3!(&ctx, pose0, poses[0]);

    // transitions:
    let t_home_via1 = and_z3!(&ctx, eq_z3!(&ctx, pose0, poses[0]), eq_z3!(&ctx, pose1, poses[1]));
    let t_home_via2 = and_z3!(&ctx, eq_z3!(&ctx, pose0, poses[0]), eq_z3!(&ctx, pose1, poses[2]));
    let t_via1_table = and_z3!(&ctx, eq_z3!(&ctx, pose1, poses[1]), eq_z3!(&ctx, pose2, poses[3]));
    let t_via2_table = and_z3!(&ctx, eq_z3!(&ctx, pose1, poses[2]), eq_z3!(&ctx, pose2, poses[3]));

    // ite transitions:
    // let t_home_via1 = ite_z3!(&ctx, eq_z3!(&ctx, pose0, poses[0]), eq_z3!(&ctx, pose1, poses[1]), eq_z3!(&ctx, pose1, poses[0]));
    // let t_home_via2 = ite_z3!(&ctx, eq_z3!(&ctx, pose0, poses[0]), eq_z3!(&ctx, pose1, poses[2]), eq_z3!(&ctx, pose1, poses[0]));
    // let t_via1_table = ite_z3!(&ctx, eq_z3!(&ctx, pose1, poses[1]), eq_z3!(&ctx, pose2, poses[3]), eq_z3!(&ctx, pose2, poses[1]));
    // let t_via2_table = ite_z3!(&ctx, eq_z3!(&ctx, pose1, poses[2]), eq_z3!(&ctx, pose2, poses[3]), eq_z3!(&ctx, pose2, poses[2]));

    // got the same results with transitions and ite transitions...

    // goal:
    // let goal = eq_z3!(&ctx, pose1, poses[2]); // why the wrong assignments?
    let goal = eq_z3!(&ctx, pose1, poses[1]);

    // track transitions
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_home_via1, bool_var_z3!(&ctx, "t_home_via1")));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_via1_table, bool_var_z3!(&ctx, "t_via1_table")));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_home_via2, bool_var_z3!(&ctx, "t_home_via2")));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, t_via2_table, bool_var_z3!(&ctx, "t_via2_table")));
    
    slv_assert_z3!(&ctx, &slv, init);
    slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, t_home_via1, t_home_via2, t_via1_table, t_via2_table));
    slv_assert_z3!(&ctx, &slv, goal);

    let res1 = slv_check_z3!(&ctx, &slv);
    if res1 == 1 {
        println!("SAT");
    } else if res1 == -1 {
        println!("UNSAT");
    } else {
        println!("UNDEF");
    };

    let model = slv_get_model_z3!(&ctx, &slv);

    let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

    for model in models {
        println!("{}", model);
    }
}