//! Z3 operations for SP

use std::ffi::{CStr, CString};
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

pub struct ANDZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct ORZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct NOTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
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
        MULZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let mulz3 = Z3_mk_mul(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                mulz3
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
        DIVZ3 {
            ctx,
            arg1,
            arg2,
            r: unsafe {
                let divz3 = Z3_mk_div(ctx.r, arg1, arg2);
                divz3
            }
        }
    }
}

impl <'ctx> MODZ3<'ctx> {
    /// Create an AST node representing `arg1 mod arg2`.
    ///
    /// The arguments must have int type.
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> MODZ3 {
        MODZ3 {
            ctx,
            arg1,
            arg2,
            r: unsafe {
                let modz3 = Z3_mk_mod(ctx.r, arg1, arg2);
                modz3
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
        ADDZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let addz3 = Z3_mk_add(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                addz3
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
        SUBZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let subz3 = Z3_mk_sub(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                subz3
            },
            args
        }        
    }
}

impl<'ctx> ANDZ3<'ctx> {
    /// Create an AST node representing `args[0] and ... and args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have Boolean sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> ANDZ3 {
        ANDZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let andz3 = Z3_mk_and(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                andz3
            },
            args
        }        
    }
}

impl<'ctx> ORZ3<'ctx> {
    /// Create an AST node representing `args[0] or ... or args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have Boolean sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> ORZ3 {
        ORZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let orz3 = Z3_mk_or(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                orz3
            },
            args
        }        
    }
}

impl<'ctx> NOTZ3<'ctx> {
    /// Create an AST node representing `not(a)`.
    ///
    /// The node `a` must have Boolean sort.
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> NOTZ3 {
        NOTZ3 {
            ctx,
            r: unsafe {
                let notz3 = Z3_mk_not(ctx.r, arg);
                notz3
            },
            arg
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

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, mul1.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
    }
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

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, div1.r)).to_str().unwrap().to_owned();
        let p2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, div2.r)).to_str().unwrap().to_owned();
        let p3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, div3.r)).to_str().unwrap().to_owned();
        let p4 = CStr::from_ptr(Z3_ast_to_string(ctx.r, div4.r)).to_str().unwrap().to_owned();
        let p5 = CStr::from_ptr(Z3_ast_to_string(ctx.r, div5.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
        println!("{}", p2);
        println!("{}", p3);
        println!("{}", p4);
        println!("{}", p5);
    }
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

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, mod1.r)).to_str().unwrap().to_owned();
        let p2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, mod2.r)).to_str().unwrap().to_owned();
        let p3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, mod3.r)).to_str().unwrap().to_owned();
        let p4 = CStr::from_ptr(Z3_ast_to_string(ctx.r, mod4.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
        println!("{}", p2);
        println!("{}", p3);
        println!("{}", p4);
    }
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

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, add1.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
    }
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

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, sub1.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
    }
}

#[test]
fn test_new_and(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx,false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let and1 = ANDZ3::new(&ctx, vec!(x1.r, x2.r, bool1.r, bool2.r));

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, and1.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
    }
}

#[test]
fn test_new_or(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx,false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let or1 = ORZ3::new(&ctx, vec!(x1.r, x2.r, bool1.r, bool2.r));

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, or1.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
    }
}

#[test]
fn test_new_not(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");

    let not1 = NOTZ3::new(&ctx, bool1.r);
    let not2 = NOTZ3::new(&ctx, x1.r);
    let not3 = NOTZ3::new(&ctx, not2.r);

    unsafe {
        let p1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, not1.r)).to_str().unwrap().to_owned();
        let p2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, not2.r)).to_str().unwrap().to_owned();
        let p3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, not3.r)).to_str().unwrap().to_owned();
        println!("{}", p1);
        println!("{}", p2);
        println!("{}", p3);
    }
}