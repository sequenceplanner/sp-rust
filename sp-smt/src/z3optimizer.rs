//! Z3 optimizer for SP

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

/// Z3 optimizer 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptimizerZ3::new(&ctx)
/// ```
/// Using the global context:
/// ```text
/// optz3!()
/// ```
/// Using a specific context:
/// ```text
/// optz3!(&ctx)
/// ```
#[macro_export]
macro_rules! optz3 {
    () => {
        OptimizerZ3::new(&CTX).r
    };
    ($ctx:expr) => {
        OptimizerZ3::new($ctx).r
    }
}

/// Z3 assert hard optimization constraint 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptAssertZ3::new(&ctx, &opt, a)
/// ```
/// Using the global context:
/// ```text
/// oasrtz3!(a, &opt)
/// ```
/// Using a specific context:
/// ```text
/// oasrt3!(&ctx, &opt, a)
/// ```
#[macro_export]
macro_rules! oasrtz3 {
    ($opt:expr, $a:expr) => {
        OptAssertZ3::new(&CTX, $opt, $a).r
    };
    ($ctx:expr, $opt:expr, $a:expr) => {
        OptAssertZ3::new($ctx, $opt, $a).r
    }
}

/// Z3 assert a maximization constraint 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptMaximizeZ3::new(&ctx, &opt, a)
/// ```
/// Using the global context:
/// ```text
/// omaxz3!(&opt, a)
/// ```
/// Using a specific context:
/// ```text
/// omaxt3!(&ctx, &opt, a)
/// ```
#[macro_export]
macro_rules! omaxz3 {
    ($opt:expr, $a:expr) => {
        OptMaximizeZ3::new(&CTX, $opt, $a).r
    };
    ($ctx:expr, $opt:expr, $a:expr) => {
        OptMaximizeZ3::new($ctx, $opt, $a).r
    }
}

/// Z3 assert a minimization constraint 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptMinimizeZ3::new(&ctx, &opt, a)
/// ```
/// Using the global context:
/// ```text
/// ominz3!(&opt, a)
/// ```
/// Using a specific context:
/// ```text
/// omint3!(&ctx, &opt, a)
/// ```
#[macro_export]
macro_rules! ominz3 {
    ($opt:expr, $a:expr) => {
        OptMinimizeZ3::new(&CTX, $opt, $a).r
    };
    ($ctx:expr, $opt:expr, $a:expr) => {
        OptMinimizeZ3::new($ctx, $opt, $a).r
    }
}

/// Z3 check consistency and produce optimal values 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptCheckZ3::new(&ctx, &opt, vec!(a, b, c))
/// ```
/// Using the global context:
/// ```text
/// ocheck3!(&opt, a, b, c)
/// ```
/// Using a specific context:
/// ```text
/// ocheck3!(&ctx, &opt, a, b, c)
/// ```
#[macro_export]
macro_rules! ocheckz3 {
    ( $opt:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            OptCheckZ3::new(&CTX, $opt, temp_vec).r
        }
    };
    ( $ctx:expr, $opt:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            OptCheckZ3::new($ctx, $opt, temp_vec).r
        }
    };
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

