use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;

fn main() {
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    unsafe {
        Z3_solver_from_string(ctx.r, slv.r, CString::new("/home/endre/Desktop/dout.dimacs").unwrap().as_ptr());
    }
}
