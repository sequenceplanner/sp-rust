use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

fn main() {

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    let r_c_sort = EnumSortZ3::new(&ctx, "r_c_sort", vec!("a", "b"));         // robot command
    let r_s_sort = EnumSortZ3::new(&ctx, "r_s_sort", vec!("a", "b", "u"));    // robot state
    let g_c_sort = EnumSortZ3::new(&ctx, "g_c_sort", vec!("o", "c"));         // gripper command
    let g_s_sort = EnumSortZ3::new(&ctx, "g_s_sort", vec!("o", "c", "u"));    // gripper state

    let r_c_poses = &r_c_sort.enum_asts;
    let r_s_poses = &r_s_sort.enum_asts;
    let g_c_poses = &g_c_sort.enum_asts;
    let g_s_poses = &g_s_sort.enum_asts;

    let robot_cmd = EnumVarZ3::new(&ctx, r_c_sort.r, "robot_cmd");
    let robot_state = EnumVarZ3::new(&ctx, r_s_sort.r, "robot_state");
    let gripper_cmd = EnumVarZ3::new(&ctx, g_c_sort.r, "gripper_cmd");
    let gripper_state = EnumVarZ3::new(&ctx, g_s_sort.r, "gripper_state");

    // all initial states:
    slv_assert_z3!(&ctx, &slv, 
        or_z3!(&ctx, 
            eq_z3!(&ctx, robot_cmd, r_c_poses[0]), 
            eq_z3!(&ctx, robot_cmd, r_c_poses[1])));

    slv_assert_z3!(&ctx, &slv, 
        or_z3!(&ctx, 
            eq_z3!(&ctx, robot_state, r_s_poses[0]), 
            eq_z3!(&ctx, robot_state, r_s_poses[1]),
            eq_z3!(&ctx, robot_state, r_s_poses[2])));
    
    slv_assert_z3!(&ctx, &slv, 
        or_z3!(&ctx, 
            eq_z3!(&ctx, gripper_cmd, g_c_poses[0]), 
            eq_z3!(&ctx, gripper_cmd, g_c_poses[1])));

    slv_assert_z3!(&ctx, &slv, 
        or_z3!(&ctx, 
            eq_z3!(&ctx, gripper_state, g_s_poses[0]), 
            eq_z3!(&ctx, gripper_state, g_s_poses[1]),
            eq_z3!(&ctx, gripper_state, g_s_poses[2])));

    // forbidden state combinations:
    slv_assert_z3!(&ctx, &slv,
        not_z3!(&ctx, 
            and_z3!(&ctx,
                eq_z3!(&ctx, robot_state, r_s_poses[1]),
                eq_z3!(&ctx, gripper_state, g_s_poses[1]))));
    
    // forbidden state combinations:
    slv_assert_z3!(&ctx, &slv,
        not_z3!(&ctx, 
            and_z3!(&ctx,
                eq_z3!(&ctx, robot_state, r_s_poses[1]),
                eq_z3!(&ctx, gripper_state, g_s_poses[2]))));

    let clauses_ast_vec = SlvGetAssertsZ3::new(&ctx, &slv);
    let clauses = Z3AstVectorToVectorAstZ3::new(&ctx, clauses_ast_vec);

    for i in 0..clauses.len() {
        println!("{:?} : {}", i, AstToStringZ3::new(&ctx, clauses[i]));
    }

    let models = slv_get_all_models_z3!(&ctx, &slv);
    for model in models {
        println!("{}", model);
    }

}