use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;

fn main() {
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);
  

    // source state variables:
    let x = bool_var_z3!(&ctx, "x");
    let y = bool_var_z3!(&ctx, "y");
    
    let t = bool_z3!(&ctx, true);
    let f = bool_z3!(&ctx, false);

    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, x, t));
    // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, y, t));
    slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, y, not_z3!(&ctx, y)));

    
    let res1 = SlvCheckZ3::new(&ctx, &slv);

    let model = slv_get_model_z3!(&ctx, &slv);

    unsafe{
        let mut v = &mut BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), "v");
        Z3_model_eval(ctx.r, model, x, true, v);
    }

    let model = slv_get_model_z3!(&ctx, &slv);
    println!("{}", model_to_string_z3!(&ctx, model));
}