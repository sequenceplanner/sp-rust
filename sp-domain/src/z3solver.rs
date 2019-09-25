//! Z3 solver for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct SolverZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_solver,
}

impl <'ctx> SolverZ3<'ctx> {
    /// Create a new solver. This solver is a "combined solver" (see
    /// combined_solver module) that internally uses a non-incremental (solver1) and an
    /// incremental solver (solver2). This combined solver changes its behaviour based
    /// on how it is used and how its parameters are set.
    ///
    /// If the solver is used in a non incremental way (i.e. no calls to
    /// `Z3_solver_push()` or `Z3_solver_pop()`, and no calls to
    /// `Z3_solver_assert()` or `Z3_solver_assert_and_track()` after checking
    /// satisfiability without an intervening `Z3_solver_reset()`) then solver1
    /// will be used. This solver will apply Z3's "default" tactic.
    ///
    /// The "default" tactic will attempt to probe the logic used by the
    /// assertions and will apply a specialized tactic if one is supported.
    /// Otherwise the general `(and-then simplify smt)` tactic will be used.
    ///
    /// If the solver is used in an incremental way then the combined solver
    /// will switch to using solver2 (which behaves similarly to the general
    /// "smt" tactic).
    ///
    /// Note however it is possible to set the `solver2_timeout`,
    /// `solver2_unknown`, and `ignore_solver1` parameters of the combined
    /// solver to change its behaviour.
    ///
    /// The function [`Z3_solver_get_model`](fn.Z3_solver_get_model.html) retrieves a model if the
    /// assertions is satisfiable (i.e., the result is
    /// `Z3_L_TRUE`) and model construction is enabled.
    /// The function [`Z3_solver_get_model`](fn.Z3_solver_get_model.html) can also be used even
    /// if the result is `Z3_L_UNDEF`, but the returned model
    /// is not guaranteed to satisfy quantified assertions.
    ///
    /// NOTE: User must use [`Z3_solver_inc_ref`](fn.Z3_solver_inc_ref.html) and [`Z3_solver_dec_ref`](fn.Z3_solver_dec_ref.html) to manage solver objects.
    /// Even if the context was created using [`Z3_mk_context`](fn.Z3_mk_context.html) instead of [`Z3_mk_context_rc`](fn.Z3_mk_context_rc.html).
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
    /// Decrement the reference counter of the given solver.
    fn drop(&mut self) {
        unsafe { 
            Z3_solver_dec_ref(self.ctx.r, self.r)
        }
    }
}

/// Run test with -- --nocapture to see prints.
#[test]
fn test_solver(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);
    unsafe{
        let solv_str = Z3_solver_to_string(ctx.r, solv.r);
        println!("Should print empty string, no assertions yet.");
        println!("{:?}", CStr::from_ptr(solv_str).to_str().unwrap());
    }
}


// #[test]
// fn test_int_sort(){
//     unsafe {
//         let _conf = Config::new();
//         let _ctx = Context::new(&_conf);
//         let _sort = IntSort::new(&_ctx);
//         let _var = IntVar::new(&_ctx, &_sort, "x");
//         let _val = Int::new(&_ctx, &_sort, 7);
//         let _eq = EQ::new(&_ctx, _var.var, _val.val);

//         let _solv = Solver::new(&_ctx);
//         let _assert = Z3_solver_assert(_ctx.context, _solv.solver, _eq.rel);
//         let solv_str = Z3_solver_to_string(_ctx.context, _solv.solver);
//         println!("{}", CStr::from_ptr(solv_str).to_str().unwrap());
        
//         Z3_solver_check(_ctx.context, _solv.solver);

//         let solv_check_str = Z3_solver_to_string(_ctx.context, _solv.solver);
//         println!("{}", CStr::from_ptr(solv_check_str).to_str().unwrap());

//         let _sgm = Z3_solver_get_model(_ctx.context, _solv.solver);
//         let _msgm = Z3_model_to_string(_ctx.context, _sgm);
//         println!("{}", CStr::from_ptr(_msgm).to_str().unwrap());
//     }
// }
