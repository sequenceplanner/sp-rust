//! Z3 numerical operations for SP

use std::ffi::{CStr};
use z3_sys::*;
use super::*;

pub struct MULZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub s: String,
    pub r: Z3_ast
}

pub struct DIVZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct MODZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct REMZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct ADDZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub s: String,
    pub r: Z3_ast
}

pub struct SUBZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub s: String,
    pub r: Z3_ast
}

pub struct NEGZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct POWZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

impl<'ctx> MULZ3<'ctx> {
    /// Create an AST node representing `args[0] * ... * args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have int or real sort.
    ///
    /// NOTE: Z3 has limited support for non-linear arithmetic.
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> MULZ3 {
        let args_slice = &args;
        let z3 = unsafe {
            Z3_mk_mul(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        MULZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            args
        }        
    }
}

impl <'ctx> DIVZ3<'ctx> {
    /// Create an AST node representing `arg1 div arg2`.
    ///
    /// The arguments must either both have int type or both have real type.
    /// If the arguments have int type, then the result type is an int type, otherwise the
    /// the result type is real.
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> DIVZ3 {
        let z3 = unsafe {
            Z3_mk_div(ctx.r, arg1, arg2)
        };
        DIVZ3 {
            ctx,
            arg1,
            arg2,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx> MODZ3<'ctx> {
    /// Create an AST node representing `arg1 mod arg2`.
    ///
    /// The arguments must have int type.
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> MODZ3 {
        let z3 = unsafe {
            Z3_mk_mod(ctx.r, arg1, arg2)
        };
        MODZ3 {
            ctx,
            arg1,
            arg2,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx> REMZ3<'ctx> {
    /// Create an AST node representing `arg1 mod arg2`.
    ///
    /// The arguments must have int type.
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> REMZ3 {
        let z3 = unsafe {
            Z3_mk_rem(ctx.r, arg1, arg2)
        };
        REMZ3 {
            ctx,
            arg1,
            arg2,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl<'ctx> ADDZ3<'ctx> {
    /// Create an AST node representing `args[0] + ... + args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have int or real sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> ADDZ3 {
        let args_slice = &args;
        let z3 = unsafe {
            Z3_mk_add(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        ADDZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            args
        }        
    }
}

impl<'ctx> SUBZ3<'ctx> {
    /// Create an AST node representing `args[0] - ... - args[num_args - 1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have int or real sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> SUBZ3 {
        let args_slice = &args;
        let z3 = unsafe {
            Z3_mk_sub(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        SUBZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            args
        }        
    }
}

impl<'ctx> NEGZ3<'ctx> {
    /// Create an AST node representing `- arg`.
    ///
    /// The arguments must have int or real type.
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> NEGZ3 {
        let z3 = unsafe {
            Z3_mk_unary_minus(ctx.r, arg)
        };
        NEGZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            arg
        }        
    }
}

impl <'ctx> POWZ3<'ctx> {
    /// Create an AST node representing `arg1 ^ arg2`.
    ///
    /// The arguments must have int or real type.
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> POWZ3 {
        let z3 = unsafe {
            Z3_mk_power(ctx.r, arg1, arg2)
        };
        POWZ3 {
            ctx,
            arg1,
            arg2,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
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

    let mul1 = MULZ3::new(&ctx, vec!(x1.r, x2.r, x3.r, x4.r, int1.r, int2.r, real1.r, real2.r));

    println!("{}", mul1.s);
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

    let div1 = DIVZ3::new(&ctx, int1.r, int2.r);
    let div2 = DIVZ3::new(&ctx, int1.r, real2.r);
    let div3 = DIVZ3::new(&ctx, x1.r, real2.r);
    let div4 = DIVZ3::new(&ctx, x3.r, int1.r);
    let div5 = DIVZ3::new(&ctx, x3.r, real1.r);

    println!("{}", div1.s);
    println!("{}", div2.s);
    println!("{}", div3.s);
    println!("{}", div4.s);
    println!("{}", div5.s);
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

    let mod1 = MODZ3::new(&ctx, int1.r, int2.r);
    let mod2 = MODZ3::new(&ctx, x1.r, x2.r);
    let mod3 = MODZ3::new(&ctx, x1.r, int1.r);
    let mod4 = MODZ3::new(&ctx, int2.r, x2.r);

    println!("{}", mod1.s);
    println!("{}", mod2.s);
    println!("{}", mod3.s);
    println!("{}", mod4.s);
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

    let rem1 = REMZ3::new(&ctx, int1.r, int2.r);
    let rem2 = REMZ3::new(&ctx, x1.r, x2.r);
    let rem3 = REMZ3::new(&ctx, x1.r, int1.r);
    let rem4 = REMZ3::new(&ctx, int2.r, x2.r);

    println!("{}", rem1.s);
    println!("{}", rem2.s);
    println!("{}", rem3.s);
    println!("{}", rem4.s);
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

    let add1 = ADDZ3::new(&ctx, vec!(x1.r, x2.r, x3.r, x4.r, int1.r, int2.r, real1.r, real2.r));

    println!("{}", add1.s);
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

    let sub1 = SUBZ3::new(&ctx, vec!(x1.r, x2.r, x3.r, x4.r, int1.r, int2.r, real1.r, real2.r));

    println!("{}", sub1.s);
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

    let neg1 = NEGZ3::new(&ctx, int1.r);
    let neg2 = NEGZ3::new(&ctx, int2.r);
    let neg3 = NEGZ3::new(&ctx, real1.r);
    let neg4 = NEGZ3::new(&ctx, real2.r);
    let neg5 = NEGZ3::new(&ctx, x1.r);

    println!("{}", neg1.s);
    println!("{}", neg2.s);
    println!("{}", neg3.s);
    println!("{}", neg4.s);
    println!("{}", neg5.s);
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

    let pow1 = POWZ3::new(&ctx, int1.r, int2.r);
    let pow2 = POWZ3::new(&ctx, x1.r, x2.r);
    let pow3 = POWZ3::new(&ctx, x1.r, int1.r);
    let pow4 = POWZ3::new(&ctx, int2.r, x2.r);

    println!("{}", pow1.s);
    println!("{}", pow2.s);
    println!("{}", pow3.s);
    println!("{}", pow4.s);
}