use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

fn main() {

    // let cfg = cfg_z3!();
    // let ctx = ctx_z3!(&cfg);
    // let slv = slv_z3!(&ctx);

    // let state_sort = EnumSortZ3::new(&ctx, "state_sort", vec!("a", "b", "c", "d", "e", "f", "g"));
    // let states = &state_sort.enum_asts;
    // let x = EnumVarZ3::new(&ctx, state_sort.r, "x_s0");
    // let y = EnumVarZ3::new(&ctx, state_sort.r, "y_s0");

    // let mut step: u32 = 0;
    // let mut path: u32 = 0;
    // let max_steps: u32 = 100;
    // let max_paths: u32 = 100;

    // // initial state:
    // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, state, states[0]));

    // // forbidden states:
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, state, states[6])));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, state, states[3])));

    // // goal state:
    // SlvPushZ3::new(&ctx, &slv);
    // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, state, states[2]));

    // pub fn new(ctx: &'ctx ContextZ3, x: Z3_ast, y: Z3_ast) -> Z3_ast {
    //     ORZ3::new(&ctx, vec!(y, ANDZ3::new(&ctx, vec!(x, new(&ctx, x: Z3_ast, y: Z3_ast)))))
    // }

    // while path < max_paths {
    //     path = path + 1;
    //     while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
    //         step = step + 1;

    //         SlvPopZ3::new(&ctx, &slv, 1);

    //         let current_step: &str = &format!("state_s{}", step - 1);
    //         let next_step: &str = &format!("state_s{}", step);

    //         // forbidden state (solve commented and uncommented):
    //         slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[6])));
    //         slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[3])));

    //         pub fn new(ctx: &'ctx ContextZ3, x: Z3_ast, y: Z3_ast) -> Z3_ast {
    //             ORZ3::new(&ctx, vec!(y, ANDZ3::new(&ctx, vec!(x, new(&ctx, x: Z3_ast, y: Z3_ast)))))
    //         }
            
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, a_b, bool_var_z3!(&ctx, &format!("a_b_t{}", step))));
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, a_e, bool_var_z3!(&ctx, &format!("a_e_t{}", step))));
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, a_g, bool_var_z3!(&ctx, &format!("a_g_t{}", step))));
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, e_f, bool_var_z3!(&ctx, &format!("e_f_t{}", step))));
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, f_c, bool_var_z3!(&ctx, &format!("f_c_t{}", step))));
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, b_c, bool_var_z3!(&ctx, &format!("b_c_t{}", step))));
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, b_d, bool_var_z3!(&ctx, &format!("b_d_t{}", step))));
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, g_c, bool_var_z3!(&ctx, &format!("g_c_t{}", step))));

    //         SlvPushZ3::new(&ctx, &slv);
    //         slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[2]));

    //     }

    //     println!("\nPath: {:?}", path);
    //     let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
    //     let frames = GetPlanningFramesZ3::new(&ctx, model, step);
        
    //     for l in frames {
    //         println!("{:?} : {:?} : {:?}", l.0, l.1, l.2);
    //     }
    // }
}