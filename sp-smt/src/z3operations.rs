//! Z3 numerical operations for SP

use std::ffi::{CStr};
use z3_sys::*;
use super::*;

pub struct MULZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct DIVZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub r: Z3_ast
}

pub struct MODZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub r: Z3_ast
}

pub struct REMZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub r: Z3_ast
}

pub struct ADDZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct SUBZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct NEGZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub r: Z3_ast
}

pub struct POWZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub r: Z3_ast
}

impl<'ctx> MULZ3<'ctx> {
    /// Create an AST node representing `args[0] * ... * args[num_args-1]`.
    ///
    /// The `args` is a rust vector, use `vec!()`.
    /// All arguments must have int or real sort.
    ///
    /// NOTE: Z3 has limited support for non-linear arithmetic.
    /// 
    /// NOTE: The number of arguments must be greater than zero.
    /// 
    /// NOTE: See macro! mulz3!
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> Z3_ast {
        let args_slice = &args;
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_mul(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        MULZ3 {ctx, r: z3, args}.r
    }
}

impl <'ctx> DIVZ3<'ctx> {
    /// Create an AST node representing `arg1 div arg2`.
    ///
    /// NOTE: The arguments must either both have int type or both have real type.
    /// If the arguments have int type, then the result type is an int type, otherwise the
    /// the result type is real.
    /// 
    /// NOTE: See macro! divz3!
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_div(ctx.r, arg1, arg2)
        };
        DIVZ3 {ctx, r: z3, arg1, arg2}.r
    }
}

impl <'ctx> MODZ3<'ctx> {
    /// Create an AST node representing `arg1 mod arg2`.
    ///
    /// NOTE: The arguments must have int type.
    /// 
    /// NOTE: See macro! modz3!
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_mod(ctx.r, arg1, arg2)
        };
        MODZ3 {ctx, arg1, arg2, r: z3}.r
    }
}

impl <'ctx> REMZ3<'ctx> {
    /// Create an AST node representing `arg1 mod arg2`.
    ///
    /// NOTE: The arguments must have int type.
    /// 
    /// NOTE: See macro! remz3!
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_rem(ctx.r, arg1, arg2)
        };
        REMZ3 {ctx, arg1, arg2, r: z3}.r
    }
}

impl<'ctx> ADDZ3<'ctx> {
    /// Create an AST node representing `args[0] + ... + args[num_args-1]`.
    ///
    /// The `args` is a rust vector, use `vec!()`.
    /// 
    /// NOTE: All arguments must have int or real sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    /// 
    /// NOTE: See macro! addz3!
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> Z3_ast {
        let args_slice = &args;
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_add(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        ADDZ3 {ctx, r: z3, args}.r
    }
}

impl<'ctx> SUBZ3<'ctx> {
    /// Create an AST node representing `args[0] - ... - args[num_args - 1]`.
    ///
    /// The `args` is a rust vector, use `vec!()`.
    /// 
    /// NOTE: All arguments must have int or real sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    /// 
    /// NOTE: See macro! subz3!
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> Z3_ast {
        let args_slice = &args;
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_sub(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        SUBZ3 {ctx, r: z3, args}.r 
    }
}

impl<'ctx> NEGZ3<'ctx> {
    /// Create an AST node representing `- arg`.
    ///
    /// NOTE: The arguments must have int or real type.
    /// 
    /// NOTE: See macro! negz3!
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_unary_minus(ctx.r, arg)
        };
        NEGZ3 {ctx, r: z3, arg}.r  
    }
}

impl <'ctx> POWZ3<'ctx> {
    /// Create an AST node representing `arg1 ^ arg2`.
    ///
    /// NOTE: The arguments must have int or real type.
    /// 
    /// NOTE: See macro! powz3!
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_power(ctx.r, arg1, arg2)
        };
        POWZ3 {ctx, arg1, arg2, r: z3}.r
    }
}

/// Z3 a * b * c
/// 
/// Macro rule for:
/// ```text
/// z3operations::MULZ3::new(&ctx, vec!(a, b, c)).r
/// ```
/// Using the default context:
/// ```text
/// mulz3!(a, b, c)
/// ```
/// Using a specific context:
/// ```text
/// mulz3!(&ctx, a, b, c)
/// ```
/// Requires that a, b, c... are of Real or Int sort.
#[macro_export]
macro_rules! mulz3 {
    // ( $( $x:expr ),* ) => {
    //     {
    //         let mut temp_vec = Vec::new();
    //         $(
    //             temp_vec.push($x);
    //         )*
    //         MULZ3::new(&CTX, temp_vec)
    //     }
    // };
    ( $ctx:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            MULZ3::new($ctx, temp_vec)
        }
    };
}

/// Z3 a div b
/// 
/// Macro rule for:
/// ```text
/// z3operations::DIVZ3::new(&ctx, a, b).r
/// ```
/// Using the default context:
/// ```text
/// divz3!(a, b)
/// ```
/// Using a specific context:
/// ```text
/// divz3!(&ctx, a, b)
/// ```
/// Requires that a and b are of Real or Int sort.
#[macro_export]
macro_rules! divz3 {
    // ($a:expr, $b:expr) => {
    //     DIVZ3::new(&CTX, $a, $b)
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        DIVZ3::new($ctx, $b, $c)
    }
}

/// Z3 a mod b
/// 
/// Macro rule for:
/// ```text
/// z3operations::MODZ3::new(&ctx, a, b).r
/// ```
/// Using the default context:
/// ```text
/// modz3!(a, b)
/// ```
/// Using a specific context:
/// ```text
/// modz3!(&ctx, a, b)
/// ```
/// Requires that a and b are of Real or Int sort.
#[macro_export]
macro_rules! modz3 {
    // ($a:expr, $b:expr) => {
    //     MODZ3::new(&CTX, $a, $b)
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        MODZ3::new($ctx, $b, $c)
    }
}

/// Z3 a rem b
/// 
/// Macro rule for:
/// ```text
/// z3operations::REMZ3::new(&ctx, a, b).r
/// ```
/// Using the default context:
/// ```text
/// remz3!(a, b)
/// ```
/// Using a specific context:
/// ```text
/// remz3!(&ctx, a, b)
/// ```
/// Requires that a and b are of Real or Int sort.
#[macro_export]
macro_rules! remz3 {
    // ($a:expr, $b:expr) => {
    //     REMZ3::new(&CTX, $a, $b)
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        REMZ3::new($ctx, $b, $c)
    }
}

/// Z3 a + b + c
/// 
/// Macro rule for:
/// ```text
/// z3operations::ADDZ3::new(&ctx, vec!(a, b, c)).r
/// ```
/// Using the default context:
/// ```text
/// addz3!(a, b, c)
/// ```
/// Using a specific context:
/// ```text
/// addz3!(&ctx, a, b, c)
/// ```
/// Requires that a, b, c... are of Real or Int sort.
#[macro_export]
macro_rules! addz3 {
    // ( $( $x:expr ),* ) => {
    //     {
    //         let mut temp_vec = Vec::new();
    //         $(
    //             temp_vec.push($x);
    //         )*
    //         ADDZ3::new(&CTX, temp_vec)
    //     }
    // };
    ( $ctx:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            ADDZ3::new($ctx, temp_vec)
        }
    };
}

/// Z3 a - b - c
/// 
/// Macro rule for:
/// ```text
/// z3operations::SUBZ3::new(&ctx, vec!(a, b, c)).r
/// ```
/// Using the default context:
/// ```text
/// subz3!(a, b, c)
/// ```
/// Using a specific context:
/// ```text
/// subz3!(&ctx, a, b, c)
/// ```
/// Requires that a, b, c... are of Real or Int sort.
#[macro_export]
macro_rules! subz3 {
    // ( $( $x:expr ),* ) => {
    //     {
    //         let mut temp_vec = Vec::new();
    //         $(
    //             temp_vec.push($x);
    //         )*
    //         SUBZ3::new(&CTX, temp_vec)
    //     }
    // };
    ( $ctx:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            SUBZ3::new($ctx, temp_vec)
        }
    };
}

/// Z3 -a
/// 
/// Macro rule for:
/// ```text
/// z3operations::NEGZ3::new(&ctx, a).r
/// ```
/// Using the default context:
/// ```text
/// negz3!(a)
/// ```
/// Using a specific context:
/// ```text
/// negz3!(&ctx, a)
/// ```
/// Requires that a is of Real or Int sort.
#[macro_export]
macro_rules! negz3 {
    // ($a:expr) => {
    //     NEGZ3::new(&CTX, $a)
    // };
    ($ctx:expr, $b:expr) => {
        NEGZ3::new($ctx, $b)
    }
}

/// Z3 a ^ b
/// 
/// Macro rule for:
/// ```text
/// z3operations::POWZ3::new(&ctx, a, b).r
/// ```
/// Using the default context:
/// ```text
/// powz3!(a, b)
/// ```
/// Using a specific context:
/// ```text
/// powz3!(&ctx, a, b)
/// ```
/// Requires that a and b are of Real or Int sort.
#[macro_export]
macro_rules! powz3 {
    // ($a:expr, $b:expr) => {
    //     POWZ3::new(&CTX, $a, $b)
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        POWZ3::new($ctx, $b, $c)
    }
}

#[test]
fn test_new_mul(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");
    let x4 = RealVarZ3::new(&ctx, &realsort, "x4");

    let mul1 = MULZ3::new(&ctx, vec!(x1, x2, x3, x4, int1, int2, real1, real2));

    assert_eq!("(* (to_real x1)
   (to_real x2)
   x3
   x4
   (to_real 7)
   (to_real (- 1012))
   (/ 920241.0 125000.0)
   (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(ctx.r, mul1));
}

#[test]
fn test_new_div(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");

    let div1 = DIVZ3::new(&ctx, int1, int2);
    let div2 = DIVZ3::new(&ctx, int1, real2);
    let div3 = DIVZ3::new(&ctx, x1, real2);
    let div4 = DIVZ3::new(&ctx, x3, int1);
    let div5 = DIVZ3::new(&ctx, x3, real1);

    assert_eq!("(div 7 (- 1012))", ast_to_string_z3!(ctx.r, div1));
    assert_eq!("(div 7 (to_int (- (/ 59024591.0 250000.0))))", ast_to_string_z3!(ctx.r, div2));
    assert_eq!("(div x1 (to_int (- (/ 59024591.0 250000.0))))", ast_to_string_z3!(ctx.r, div3));
    assert_eq!("(/ x3 (to_real 7))", ast_to_string_z3!(ctx.r, div4));
    assert_eq!("(/ x3 (/ 920241.0 125000.0))", ast_to_string_z3!(ctx.r, div5));
}

#[test]
fn test_new_mod(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");

    let mod1 = MODZ3::new(&ctx, int1, int2);
    let mod2 = MODZ3::new(&ctx, x1, x2);
    let mod3 = MODZ3::new(&ctx, x1, int1);
    let mod4 = MODZ3::new(&ctx, int2, x2);

    assert_eq!("(mod 7 (- 1012))", ast_to_string_z3!(ctx.r, mod1));
    assert_eq!("(mod x1 x2)", ast_to_string_z3!(ctx.r, mod2));
    assert_eq!("(mod x1 7)", ast_to_string_z3!(ctx.r, mod3));
    assert_eq!("(mod (- 1012) x2)", ast_to_string_z3!(ctx.r, mod4));
}

#[test]
fn test_new_rem(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");

    let rem1 = REMZ3::new(&ctx, int1, int2);
    let rem2 = REMZ3::new(&ctx, x1, x2);
    let rem3 = REMZ3::new(&ctx, x1, int1);
    let rem4 = REMZ3::new(&ctx, int2, x2);

    assert_eq!("(rem 7 (- 1012))", ast_to_string_z3!(ctx.r, rem1));
    assert_eq!("(rem x1 x2)", ast_to_string_z3!(ctx.r, rem2));
    assert_eq!("(rem x1 7)", ast_to_string_z3!(ctx.r, rem3));
    assert_eq!("(rem (- 1012) x2)", ast_to_string_z3!(ctx.r, rem4));
}

#[test]
fn test_new_add(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");
    let x4 = RealVarZ3::new(&ctx, &realsort, "x4");

    let add1 = ADDZ3::new(&ctx, vec!(x1, x2, x3, x4, int1, int2, real1, real2));

    assert_eq!("(+ (to_real x1)
   (to_real x2)
   x3
   x4
   (to_real 7)
   (to_real (- 1012))
   (/ 920241.0 125000.0)
   (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(ctx.r, add1));
}

#[test]
fn test_new_sub(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");
    let x4 = RealVarZ3::new(&ctx, &realsort, "x4");

    let sub1 = SUBZ3::new(&ctx, vec!(x1, x2, x3, x4, int1, int2, real1, real2));

    assert_eq!("(- (- (- (- (- (- (to_real (- x1 x2)) x3) x4) (to_real 7)) (to_real (- 1012)))
      (/ 920241.0 125000.0))
   (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(ctx.r, sub1));
}

#[test]
fn test_new_neg(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");

    let neg1 = NEGZ3::new(&ctx, int1);
    let neg2 = NEGZ3::new(&ctx, int2);
    let neg3 = NEGZ3::new(&ctx, real1);
    let neg4 = NEGZ3::new(&ctx, real2);
    let neg5 = NEGZ3::new(&ctx, x1);

    assert_eq!("(- 7)", ast_to_string_z3!(ctx.r, neg1));
    assert_eq!("(- (- 1012))", ast_to_string_z3!(ctx.r, neg2));
    assert_eq!("(- (/ 920241.0 125000.0))", ast_to_string_z3!(ctx.r, neg3));
    assert_eq!("(- (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(ctx.r, neg4));
    assert_eq!("(- x1)", ast_to_string_z3!(ctx.r, neg5));
}

#[test]
fn test_new_pow(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");

    let pow1 = POWZ3::new(&ctx, int1, int2);
    let pow2 = POWZ3::new(&ctx, x1, x2);
    let pow3 = POWZ3::new(&ctx, x1, int1);
    let pow4 = POWZ3::new(&ctx, int2, x2);

    assert_eq!("(^ 7 (- 1012))", ast_to_string_z3!(ctx.r, pow1));
    assert_eq!("(^ x1 x2)", ast_to_string_z3!(ctx.r, pow2));
    assert_eq!("(^ x1 7)", ast_to_string_z3!(ctx.r, pow3));
    assert_eq!("(^ (- 1012) x2)", ast_to_string_z3!(ctx.r, pow4));
}

#[test]
fn test_mul_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let mul1 = mulz3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y")
    );
    assert_eq!("(* x 142 y)", ast_to_string_z3!(ctx.r, mul1));
}

#[test]
fn test_div_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let div1 = divz3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(div x 142)", ast_to_string_z3!(ctx.r, div1));
}

#[test]
fn test_mod_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let mod1 = modz3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(mod x 142)", ast_to_string_z3!(ctx.r, mod1));
}

#[test]
fn test_rem_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let rem1 = remz3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(rem x 142)", ast_to_string_z3!(ctx.r, rem1));
}

#[test]
fn test_add_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let add1 = addz3!(&ctx, 
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y"),
        int_z3!(&ctx, 1213442)
    );
    assert_eq!("(+ x 142 y 1213442)", ast_to_string_z3!(ctx.r, add1));
}

#[test]
fn test_sub_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let sub1 = subz3!(&ctx, 
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y"),
        int_z3!(&ctx, 1213442)
    );
    assert_eq!("(- (- (- x 142) y) 1213442)", ast_to_string_z3!(ctx.r, sub1));
}

#[test]
fn test_neg_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let neg1 = negz3!(&ctx,
        int_z3!(&ctx, 1213442)
    );
    assert_eq!("(- 1213442)", ast_to_string_z3!(ctx.r, neg1));
}

#[test]
fn test_pow_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let pow1 = powz3!(&ctx, 
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(^ x 142)", ast_to_string_z3!(ctx.r, pow1));
}