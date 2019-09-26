//! Z3 optimizer for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct OptimizerZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_optimize,
}

pub struct OptAssertZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub cst: Z3_ast,
    pub r: ()
}

pub struct OptCheckZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub args: Vec<Z3_ast>,
    pub r: Z3_lbool
}

impl <'ctx> OptimizerZ3<'ctx> {
    /// Create a new optimize context.
    ///
    /// NOTE: User must use [`Z3_optimize_inc_ref`]
    /// and [`Z3_optimize_dec_ref`] to manage optimize objects,
    /// even if the context was created using [`Z3_mk_context`]
    /// instead of [`Z3_mk_context_rc`].

    /// [`Z3_mk_context`]: fn.Z3_mk_context.html
    /// [`Z3_mk_context_rc`]: fn.Z3_mk_context_rc.html
    /// [`Z3_optimize_dec_ref`]: fn.Z3_optimize_dec_ref.html)
    /// [`Z3_optimize_inc_ref`]: fn.Z3_optimize_inc_ref.html)
    pub fn new(ctx: &ContextZ3) -> OptimizerZ3 {
        OptimizerZ3 {
            ctx,
            r: unsafe {
                let opt = Z3_mk_optimize(ctx.r);
                Z3_optimize_inc_ref(ctx.r, opt);
                opt
            }
        }
    }
}

impl <'ctx, 'opt> OptAssertZ3<'ctx, 'opt> {
    /// Assert hard constraint to the optimization context.
    ///
    /// # See also:
    ///
    /// - [`Z3_optimize_assert_soft`](fn.Z3_optimize_assert_soft.html)
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, cst: Z3_ast) -> OptAssertZ3<'ctx, 'opt> {
        OptAssertZ3 {
            ctx,
            opt,
            cst,
            r: unsafe {
                Z3_optimize_assert(ctx.r, opt.r, cst);
            }
        }
    }
}

impl <'ctx, 'opt> OptCheckZ3<'ctx, 'opt> {
    /// Check consistency and produce optimal values.
    ///
    /// - `ctx`: - context
    /// - `opt`: - optimization context
    /// - `args`: - vector of additional assumptions
    ///
    /// # See also:
    ///
    /// - [`Z3_optimize_get_reason_unknown`](fn.Z3_optimize_get_reason_unknown.html)
    /// - [`Z3_optimize_get_model`](fn.Z3_optimize_get_model.html)
    /// - [`Z3_optimize_get_statistics`](fn.Z3_optimize_get_statistics.html)
    /// - [`Z3_optimize_get_unsat_core`](fn.Z3_optimize_get_unsat_core.html)
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, args: Vec<Z3_ast>) -> OptCheckZ3<'ctx, 'opt> {
        OptCheckZ3 {
            ctx,
            opt,
            r: unsafe {
                let args_slice = &args;
                let opt_res = Z3_optimize_check(ctx.r, opt.r, args_slice.len() as u32, args_slice.as_ptr());
                opt_res
            },
            args
        }
    }
}

impl <'ctx> Drop for OptimizerZ3<'ctx> {
    /// Decrement the reference counter of the given optimizer.
    fn drop(&mut self) {
        unsafe { 
            Z3_optimize_dec_ref(self.ctx.r, self.r)
        }
    }
}

#[test]
fn test_new_optimizer(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);
    unsafe{
        let opt_str = Z3_optimize_to_string(ctx.r, opt.r);
        println!("Should print empty string, opt context is still empty.");
        println!("{:?}", CStr::from_ptr(opt_str).to_str().unwrap());
    }
}

#[test]
fn test_new_oassert(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, y.r, real1.r);

    OptAssertZ3::new(&ctx, &opt, rel1.r);

    unsafe {
        let s1 = Z3_optimize_to_string(ctx.r, opt.r);
        println!("{}", CStr::from_ptr(s1).to_str().unwrap());

        let model = Z3_optimize_get_model(ctx.r, opt.r);
        let s2 = Z3_model_to_string(ctx.r, model);
        println!("{}", CStr::from_ptr(s2).to_str().unwrap());
    }
}

#[test]
fn test_new_ocheck(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);
 
    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);
 
    let rel1 = EQZ3::new(&ctx, y.r, real1.r);
 
    unsafe {
        let s1 = CStr::from_ptr(Z3_optimize_to_string(ctx.r, opt.r)).to_str().unwrap().to_owned();
        println!("This is the opt context without any assertions:");
        println!("{}", s1);
 
        OptAssertZ3::new(&ctx, &opt, rel1.r);
 
        let s2 = CStr::from_ptr(Z3_optimize_to_string(ctx.r, opt.r)).to_str().unwrap().to_owned();
        println!("This is the opt context with an assertion before the check:");
        println!("{}", s2);
 
        let res1 = OptCheckZ3::new(&ctx, &opt, vec!());
        println!("This is the return of the check:");
        println!("{}", res1.r);
 
        let s3 = CStr::from_ptr(Z3_optimize_to_string(ctx.r, opt.r)).to_str().unwrap().to_owned();
        println!("This is the opt context with an assertion after the check:");
        println!("{}", s3);
 
        let model = Z3_optimize_get_model(ctx.r, opt.r);
        let s4 = CStr::from_ptr(Z3_model_to_string(ctx.r, model)).to_str().unwrap().to_owned();
        println!("This is the solution");
        println!("{}", s4);
    }
}