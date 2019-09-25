//! Z3 solver

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;
use super::*;

pub struct SolverZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_solver,
}

/// Create a solver in the given context. This solver is a "combined solver" 
/// (see combined_solver module) that internally uses a non-incremental (solver1) 
/// and an incremental solver (solver2). This combined solver changes its 
/// behaviour based on how it is used and how its parameters are set.
impl <'ctx> SolverZ3<'ctx> {
    pub fn new(ctx: &ContextZ3) -> SolverZ3 {
        SolverZ3 {
            ctx,
            r: unsafe {
                let solv = Z3_mk_solver(ctx.r);
                Z3_solver_inc_ref(ctx.r, solv);
                solv
            }
        }
    }
}

impl <'ctx> Drop for SolverZ3<'ctx> {
    fn drop(&mut self) {
        unsafe { 
            Z3_solver_dec_ref(self.ctx.r, self.r)
        }
    }
}

/// Run test with -- --nocapture to see prints.
#[test]
fn test_solver(){
    let mut conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);
    unsafe{
        let solv_str = Z3_solver_to_string(ctx.r, solv.r);
        println!("Should print empty string, no assertions yet.");
        println!("{:?}", CStr::from_ptr(solv_str).to_str().unwrap());
    }
}