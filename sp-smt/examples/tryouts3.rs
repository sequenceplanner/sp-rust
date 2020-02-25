use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

fn main() {

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    let state_sort = EnumSortZ3::new(&ctx, "state_sort", vec!("a", "b", "c", "d", "e", "f", "g"));
    let states = &state_sort.enum_asts;
    let state = EnumVarZ3::new(&ctx, state_sort.r, "state_s0");

    let mut step: u32 = 0;
    let mut path: u32 = 0;
    let max_steps: u32 = 100;
    let max_paths: u32 = 100;

    // initial state:
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, state, states[0]));

    // forbidden states:
    slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, state, states[6])));
    slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, state, states[3])));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, state, states[2]));

    while path < max_paths {
        path = path + 1;
        while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
            step = step + 1;

            SlvPopZ3::new(&ctx, &slv, 1);

            let current_step: &str = &format!("state_s{}", step - 1);
            let next_step: &str = &format!("state_s{}", step);

            // forbidden state (solve commented and uncommented):
            slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[6])));
            slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[3])));

            let a_b = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[0]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[1]));
            
            let a_e = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[0]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[4]));
            
            let a_g = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[0]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[6]));
            
            let e_f = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[4]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[5]));

            let f_c = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[5]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[2]));

            let b_c = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[1]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[2]));
            
            let g_c = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[6]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[2]));
            
            let b_d = and_z3!(&ctx,
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, current_step), states[1]),
                eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[3]));

            slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, a_b, a_e, a_g, e_f, f_c, b_c, b_d, g_c));

            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, a_b, bool_var_z3!(&ctx, &format!("a_b_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, a_e, bool_var_z3!(&ctx, &format!("a_e_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, a_g, bool_var_z3!(&ctx, &format!("a_g_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, e_f, bool_var_z3!(&ctx, &format!("e_f_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, f_c, bool_var_z3!(&ctx, &format!("f_c_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, b_c, bool_var_z3!(&ctx, &format!("b_c_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, b_d, bool_var_z3!(&ctx, &format!("b_d_t{}", step))));
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, g_c, bool_var_z3!(&ctx, &format!("g_c_t{}", step))));

            SlvPushZ3::new(&ctx, &slv);
            slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, EnumVarZ3::new(&ctx, state_sort.r, next_step), states[2]));

        }

        println!("\nPath: {:?}", path);
        let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
        let frames = GetPlanningFramesZ3::new(&ctx, model, step);
        
        for l in frames {
            println!("{:?} : {:?} : {:?}", l.0, l.1, l.2);
        }
    }
}