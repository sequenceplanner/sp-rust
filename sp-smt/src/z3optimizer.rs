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

pub struct OptGetModelZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub r: Z3_model
}

pub struct OptGetStringZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub r: String
}

impl <'ctx> OptimizerZ3<'ctx> {
    /// Create a new optimization context.
    /// 
    /// NOTE: See macro! `opt_z3!`
    pub fn new(ctx: &ContextZ3) -> OptimizerZ3 {
        let z3 = unsafe {
            let opt = Z3_mk_optimize(ctx.r);
            Z3_optimize_inc_ref(ctx.r, opt);
            opt
        };
        OptimizerZ3 {ctx, r: z3}
    }
}

impl <'ctx, 'opt> OptAssertZ3<'ctx, 'opt> {
    /// Assert hard constraint to the optimization context.
    /// 
    /// NOTE: See macro! `opt_assert_z3!`
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, cst: Z3_ast) -> () {
        let z3 = unsafe {
            Z3_optimize_assert(ctx.r, opt.r, cst);
        };
        OptAssertZ3 {ctx, opt, cst, r: z3};
    }
}

impl <'ctx, 'opt> OptMaximizeZ3<'ctx, 'opt> {
    /// Add a maximization constraint.
    /// - `ctx`: - context
    /// - `opt`: - optimization context
    /// - `cst`: - arithmetical term
    /// 
    /// NOTE: See macro! `opt_maxizime_z3!`
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, cst: Z3_ast) -> () {
        let z3 = unsafe {
            Z3_optimize_maximize(ctx.r, opt.r, cst);
        };
        OptMaximizeZ3 {ctx, opt, cst, r: z3};
    }
}

impl <'ctx, 'opt> OptMinimizeZ3<'ctx, 'opt> {
    /// Add a minimization constraint.
    /// - `ctx`: - context
    /// - `opt`: - optimization context
    /// - `cst`: - arithmetical term
    /// 
    /// NOTE: See macro! `opt_minimize_z3!`
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, cst: Z3_ast) -> () {
        let z3 = unsafe {
            Z3_optimize_minimize(ctx.r, opt.r, cst);
        };
        OptMinimizeZ3 {ctx, opt, cst, r: z3};
    }
}

impl <'ctx, 'opt> OptCheckZ3<'ctx, 'opt> {
    /// Check consistency and produce optimal values.
    ///
    /// - `ctx`: - context
    /// - `opt`: - optimization context
    /// - `args`: - vector of additional assumptions
    /// 
    /// NOTE: See macro! `opt_check_z3!`
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>, args: Vec<Z3_ast>) -> Z3_lbool {
        let z3 = unsafe {
            let args_slice = &args;
            let opt_res = Z3_optimize_check(ctx.r, opt.r, args_slice.len() as u32, args_slice.as_ptr());
            opt_res
        };
        OptCheckZ3 {ctx, opt, r: z3, args}.r
    }
}

impl<'ctx, 'opt> OptGetModelZ3<'ctx, 'opt> {
    /// Retrieve the model for the last `OptCheckZ3::new`
    ///
    /// The error handler is invoked if a model is not available because
    /// the commands above were not invoked for the given optimization
    /// solver, or if the result was `Z3_L_FALSE`.
    /// 
    /// NOTE: See macro! `opt_get_model_z3!`
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>) -> Z3_model {
        let z3 = unsafe {
            Z3_optimize_get_model(ctx.r, opt.r)
        };
        OptGetModelZ3 {ctx, r: z3, opt}.r
    }
}

impl<'ctx, 'opt> OptGetStringZ3<'ctx, 'opt> {
    /// Z3 optimizer to readable string
    /// 
    /// NOTE: See macro! `opt_get_string_z3!`
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>) -> String {
        let z3 = unsafe {
            CStr::from_ptr(Z3_optimize_to_string(ctx.r, opt.r)).to_str().unwrap().to_owned()
        };
        OptGetStringZ3 {ctx, opt, r: z3}.r
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


/// create an optimization context 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptimizerZ3::new(&ctx)
/// ```
// / Using the global context:
// / ```text
// / opt_z3!()
// / ```
/// Using a specific context:
/// ```text
/// opt_z3!(&ctx)
/// ```
#[macro_export]
macro_rules! opt_z3 {
    // () => {
    //     OptimizerZ3::new(&CTX).r
    // };
    ($ctx:expr) => {
        OptimizerZ3::new($ctx)
    }
}

/// assert hard optimization constraint 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptAssertZ3::new(&ctx, &opt, a)
/// ```
// / Using the global context:
// / ```text
// / opt_assert_z3!(a, &opt)
// / ```
/// Using a specific context:
/// ```text
/// opt_assert_z3!(&ctx, &opt, a)
/// ```
#[macro_export]
macro_rules! opt_assert_z3 {
    // ($opt:expr, $a:expr) => {
    //     OptAssertZ3::new(&CTX, $opt, $a).r
    // };
    ($ctx:expr, $opt:expr, $a:expr) => {
        OptAssertZ3::new($ctx, $opt, $a)
    }
}

/// assert a maximization constraint 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptMaximizeZ3::new(&ctx, &opt, a)
/// ```
// / Using the global context:
// / ```text
// / omaxz3!(&opt, a)
// / ```
/// Using a specific context:
/// ```text
/// opt_maximize_z3!(&ctx, &opt, a)
/// ```
#[macro_export]
macro_rules! opt_maximize_z3 {
    // ($opt:expr, $a:expr) => {
    //     OptMaximizeZ3::new(&CTX, $opt, $a).r
    // };
    ($ctx:expr, $opt:expr, $a:expr) => {
        OptMaximizeZ3::new($ctx, $opt, $a)
    }
}

/// assert a minimization constraint 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptMinimizeZ3::new(&ctx, &opt, a)
/// ```
// / Using the global context:
// / ```text
// / opt_minimize_z3!(&opt, a)
// / ```
/// Using a specific context:
/// ```text
/// opt_minimize_z3!(&ctx, &opt, a)
/// ```
#[macro_export]
macro_rules! opt_minimize_z3 {
    // ($opt:expr, $a:expr) => {
    //     OptMinimizeZ3::new(&CTX, $opt, $a).r
    // };
    ($ctx:expr, $opt:expr, $a:expr) => {
        OptMinimizeZ3::new($ctx, $opt, $a)
    }
}

/// check consistency and produce optimal values 
/// 
/// Macro rule for:
/// ```text
/// z3optimizer::OptCheckZ3::new(&ctx, &opt, vec!(a, b, c))
/// ```
// / Using the global context:
// / ```text
// / ocheck3!(&opt, a, b, c)
// / ```
/// Using a specific context without additional assumptions:
/// ```text
/// ocheck3!(&ctx, &opt, )
/// ```
/// Using a specific context and passing additional assumptions:
/// ```text
/// ocheck3!(&ctx, &opt, a, b, c)
/// ```
/// Or make a vector firts and pass it:
/// ```text
/// let some = vec!(a, b, c);
/// ocheck3!(&ctx, &opt, some)
/// ```
#[macro_export]
macro_rules! opt_check_z3 {
    // ( $opt:expr, $( $x:expr ),* ) => {
    //     {
    //         let mut temp_vec = Vec::new();
    //         $(
    //             temp_vec.push($x);
    //         )*
    //         OptCheckZ3::new(&CTX, $opt, temp_vec).r
    //     }
    // };
    ($ctx:expr, $opt:expr, $b:expr) => {
        OptCheckZ3::new($ctx, $opt, $b)
    };
    ( $ctx:expr, $opt:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            OptCheckZ3::new($ctx, $opt, temp_vec)
        }
    };
}

/// get the model from the last check
#[macro_export]
macro_rules! opt_get_model_z3 {
    ($ctx:expr, $a:expr) => {
        OptGetModelZ3::new($ctx, $a)
    }
}

/// optimization context to readable string
#[macro_export]
macro_rules! opt_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        OptGetStringZ3::new($ctx, $a)
    }
}

#[test]
fn test_new_optimizer(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);
    assert_eq!("(check-sat)\n", opt_to_string_z3!(&ctx, &opt));
}

#[test]
fn test_new_oassert(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, y, real1);

    OptAssertZ3::new(&ctx, &opt, rel1);

    println!("Now we have an assert in the opt context");
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    println!("Model: Should be empty, no check yet.");
    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_new_ocheck(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);
 
    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);
 
    let rel1 = EQZ3::new(&ctx, y, real1);

    println!("Should print empty string, opt context is still empty.");
    assert_eq!("(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    OptAssertZ3::new(&ctx, &opt, rel1);

    println!("Now we have an assert in the opt context");
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = OptCheckZ3::new(&ctx, &opt, vec!());
    println!("This is the return of the check:");
    println!("{}", res1);

    println!("This is the opt context with an assertion after the check:");
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    println!("Model: Should print the solution, we did a check.");
    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("y -> (- (/ 271549371.0 500000.0))\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_new_maximize(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let int1 = IntZ3::new(&ctx, &intsort, 100);

    let lt1 = LTZ3::new(&ctx, x, int1);

    OptAssertZ3::new(&ctx, &opt, lt1);
    OptMaximizeZ3::new(&ctx, &opt, x);
    OptCheckZ3::new(&ctx, &opt, vec!());

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("x -> 99\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_new_minimize(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let int1 = IntZ3::new(&ctx, &intsort, 11);

    let gt1 = GTZ3::new(&ctx, x, int1);

    OptAssertZ3::new(&ctx, &opt, gt1);
    OptMinimizeZ3::new(&ctx, &opt, x);
    OptCheckZ3::new(&ctx, &opt, vec!());

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("x -> 12\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_opt_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let opt = opt_z3!(&ctx);
    assert_eq!("(check-sat)\n", opt_to_string_z3!(&ctx, &opt));
}

#[test]
fn test_oot_assert_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let opt = opt_z3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        eq_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, -543.098742)
        )
    );

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));
}

#[test]
fn test_ocheck_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let opt = opt_z3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        eq_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, -543.098742)
        )
    );

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = opt_check_z3!(&ctx, &opt, );
    println!("{}", res1);
}

#[test]
fn test_omax_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let opt = opt_z3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        lt_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 100.0)
        )
    );
    opt_maximize_z3!(&ctx, &opt, real_var_z3!(&ctx, "y"));

    assert_eq!("(declare-fun y () Real)
(assert (< y 100.0))
(maximize y)
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = opt_check_z3!(&ctx, &opt, );
    println!("{}", res1);

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("y -> 99.0\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_omin_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let opt = opt_z3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        gt_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
        )
    );
    opt_minimize_z3!(&ctx, &opt, real_var_z3!(&ctx, "y"));

    assert_eq!("(declare-fun y () Real)
(assert (> y 11.0))
(minimize y)
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = opt_check_z3!(&ctx, &opt, );
    println!("{}", res1);

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("y -> 12.0\n", model_to_string_z3!(&ctx, model));
}