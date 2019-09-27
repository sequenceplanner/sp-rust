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

pub struct OptMaximizeZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub cst: Z3_ast,
    pub r: ()
}

pub struct OptMinimizeZ3<'ctx, 'opt> {
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
        let z3 = unsafe {
            let opt = Z3_mk_optimize(ctx.r);
            Z3_optimize_inc_ref(ctx.r, opt);
            opt
        };
        OptimizerZ3 {
            ctx,
            r: z3
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

impl <'ctx, 'opt> OptMaximizeZ3<'ctx, 'opt> {
    /// Add a maximization constraint.
    /// - `ctx`: - context
    /// - `opt`: - optimization context
    /// - `cst`: - arithmetical term
    ///
    /// # See also:
    ///
    /// - [`Z3_optimize_minimize`](fn.Z3_optimize_minimize.html)
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, cst: Z3_ast) -> OptMaximizeZ3<'ctx, 'opt> {
        OptMaximizeZ3 {
            ctx,
            opt,
            cst,
            r: unsafe {
                Z3_optimize_maximize(ctx.r, opt.r, cst);
            }
        }
    }
}

impl <'ctx, 'opt> OptMinimizeZ3<'ctx, 'opt> {
    /// Add a minimization constraint.
    /// - `ctx`: - context
    /// - `opt`: - optimization context
    /// - `cst`: - arithmetical term
    ///
    /// # See also:
    ///
    /// - [`Z3_optimize_minimize`](fn.Z3_optimize_minimize.html)
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, cst: Z3_ast) -> OptMinimizeZ3<'ctx, 'opt> {
        OptMinimizeZ3 {
            ctx,
            opt,
            cst,
            r: unsafe {
                Z3_optimize_minimize(ctx.r, opt.r, cst);
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
    println!("Should print empty string, opt context is still empty.");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);
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

    println!("Now we have an assert in the opt context, should print it.");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print empty string, no check yet.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);
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

    println!("Should print empty string, opt context is still empty.");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    OptAssertZ3::new(&ctx, &opt, rel1.r);

    println!("Now we have an assert in the opt context, should print it.");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    let res1 = OptCheckZ3::new(&ctx, &opt, vec!());
    println!("This is the return of the check:");
    println!("{}", res1.r);

    println!("This is the opt context with an assertion after the check:");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print the solution, we did a check.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);
}

#[test]
fn test_new_maximize(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let int1 = IntZ3::new(&ctx, &intsort, 100);

    let lt1 = LTZ3::new(&ctx, x.r, int1.r);

    OptAssertZ3::new(&ctx, &opt, lt1.r);
    OptMaximizeZ3::new(&ctx, &opt, x.r);

    println!("Now we have an assert in the opt context, should print it.");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print empty string, no check yet.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);

    let res1 = OptCheckZ3::new(&ctx, &opt, vec!());
    println!("This is the return of the check:");
    println!("{}", res1.r);

    println!("This is the opt context with an assertion after the check:");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print the solution, we did a check.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);
}

/// DESCRIPTION:
///  - Decide about three activities (do or don't) and aim for maximum value
///  - Need to choose at least activity 1 or 2 (or both)
///  - The total time limit is 4 hours
///     - Activity 1 takes 1 hour
///     - Activity 2 takes 2 hours
///     - Activity 3 takes 3 hours
///  - Activity 3 is worth twice as much as activities 1 and 2
///
/// MODEL:
///  - This can be modelled as a linear mixed-integer Problem
///     - Binary variables x, y, z for activities 1, 2, 3
///     - Linear constraint for time limit
///     - Linear constraint for condition (1 or 2)
///
///     max x + y + 2z
///     so that: x + 2y + 3z <= 4
///              x + y >= 1
///     where x, y, z in {0, 1}
#[test]
fn test_example_1(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = IntVarZ3::new(&ctx, &intsort, "y");
    let z = IntVarZ3::new(&ctx, &intsort, "z");
    let obj = IntVarZ3::new(&ctx, &intsort, "obj");

    let zero = IntZ3::new(&ctx, &intsort, 0);
    let one = IntZ3::new(&ctx, &intsort, 1);
    let two = IntZ3::new(&ctx, &intsort, 2);
    let three = IntZ3::new(&ctx, &intsort, 3);
    let four = IntZ3::new(&ctx, &intsort, 4);

    let twoy = MULZ3::new(&ctx, vec!(two.r, y.r));
    let threez = MULZ3::new(&ctx, vec!(three.r, z.r));
    let add1 = ADDZ3::new(&ctx, vec!(x.r, twoy.r, threez.r));

    let constr1 = LEZ3::new(&ctx, add1.r, four.r);
    let constr2 = GEZ3::new(&ctx, ADDZ3::new(&ctx, vec!(x.r, y.r)).r, one.r);
    let add2 = ADDZ3::new(&ctx, vec!(x.r, y.r, MULZ3::new(&ctx, vec!(two.r, z.r)).r));
    let constr3 = EQZ3::new(&ctx, obj.r, add2.r);    
    
    OptAssertZ3::new(&ctx, &opt, GEZ3::new(&ctx, x.r, zero.r).r);
    OptAssertZ3::new(&ctx, &opt, GEZ3::new(&ctx, y.r, zero.r).r);
    OptAssertZ3::new(&ctx, &opt, GEZ3::new(&ctx, z.r, zero.r).r);
    OptAssertZ3::new(&ctx, &opt, LEZ3::new(&ctx, x.r, one.r).r);
    OptAssertZ3::new(&ctx, &opt, LEZ3::new(&ctx, y.r, one.r).r);
    OptAssertZ3::new(&ctx, &opt, LEZ3::new(&ctx, z.r, one.r).r);
    
    OptAssertZ3::new(&ctx, &opt, constr1.r);
    OptAssertZ3::new(&ctx, &opt, constr2.r);
    OptAssertZ3::new(&ctx, &opt, constr3.r);
    
    OptMaximizeZ3::new(&ctx, &opt, obj.r);

    println!("Now we have an assert in the opt context, should print it.");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print empty string, no check yet.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);

    let res1 = OptCheckZ3::new(&ctx, &opt, vec!());
    println!("This is the return of the check:");
    println!("{}", res1.r);

    println!("This is the opt context with an assertion after the check:");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print the solution, we did a check.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);
}
