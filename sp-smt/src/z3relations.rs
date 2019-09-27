//! Z3 realtions for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct EQZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct LEZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct LTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct GEZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct GTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

impl <'ctx> EQZ3<'ctx> {
    /// Create an AST node representing `left = right`.
    ///
    /// The nodes `left` and `right` must have the same type.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> EQZ3 {
        let z3 = unsafe {
            Z3_mk_eq(ctx.r, left, right)
        };
        EQZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx> LEZ3<'ctx> {
    /// Create less than or equal to.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> LEZ3 {
        let z3 = unsafe {
            Z3_mk_le(ctx.r, left, right)
        };
        LEZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx> LTZ3<'ctx> {
    /// Create less than.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> LTZ3 {
        let z3 = unsafe {
            Z3_mk_lt(ctx.r, left, right)
        };
        LTZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx> GEZ3<'ctx> {
    /// Create greater than or equal to.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> GEZ3 {
        let z3 = unsafe {
            Z3_mk_ge(ctx.r, left, right)
        };
        GEZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx> GTZ3<'ctx> {
    /// Create greater than or equal to.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> GTZ3 {
        let z3 = unsafe {
            Z3_mk_gt(ctx.r, left, right)
        };
        GTZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

#[test]
fn test_new_eq(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, x.r, int1.r);
    let rel2 = EQZ3::new(&ctx, y.r, real1.r);
    let rel3 = EQZ3::new(&ctx, y.r, x.r);
    let rel4 = EQZ3::new(&ctx, int1.r, real1.r);

    let string1 = rel1.s;
    let string2 = rel2.s;
    let string3 = rel3.s;
    let string4 = rel4.s;

    assert_eq!("(= x 7)", string1);
    assert_eq!("(= y (- (/ 271549371.0 500000.0)))", string2);
    assert_eq!("(= y (to_real x))", string3);
    assert_eq!("(= (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
}

#[test]
fn test_new_le(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = LEZ3::new(&ctx, x.r, int1.r);
    let rel2 = LEZ3::new(&ctx, y.r, real1.r);
    let rel3 = LEZ3::new(&ctx, y.r, x.r);
    let rel4 = LEZ3::new(&ctx, int1.r, real1.r);

    let string1 = rel1.s;
    let string2 = rel2.s;
    let string3 = rel3.s;
    let string4 = rel4.s;

    assert_eq!("(<= x 7)", string1);
    assert_eq!("(<= y (- (/ 271549371.0 500000.0)))", string2);
    assert_eq!("(<= y (to_real x))", string3);
    assert_eq!("(<= (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
}

#[test]
fn test_new_lt(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = LTZ3::new(&ctx, x.r, int1.r);
    let rel2 = LTZ3::new(&ctx, y.r, real1.r);
    let rel3 = LTZ3::new(&ctx, y.r, x.r);
    let rel4 = LTZ3::new(&ctx, int1.r, real1.r);

    let string1 = rel1.s;
    let string2 = rel2.s;
    let string3 = rel3.s;
    let string4 = rel4.s;

    assert_eq!("(< x 7)", string1);
    assert_eq!("(< y (- (/ 271549371.0 500000.0)))", string2);
    assert_eq!("(< y (to_real x))", string3);
    assert_eq!("(< (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
}

#[test]
fn test_new_ge(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = GEZ3::new(&ctx, x.r, int1.r);
    let rel2 = GEZ3::new(&ctx, y.r, real1.r);
    let rel3 = GEZ3::new(&ctx, y.r, x.r);
    let rel4 = GEZ3::new(&ctx, int1.r, real1.r);

    let string1 = rel1.s;
    let string2 = rel2.s;
    let string3 = rel3.s;
    let string4 = rel4.s;

    assert_eq!("(>= x 7)", string1);
    assert_eq!("(>= y (- (/ 271549371.0 500000.0)))", string2);
    assert_eq!("(>= y (to_real x))", string3);
    assert_eq!("(>= (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
}

#[test]
fn test_new_gt(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = GTZ3::new(&ctx, x.r, int1.r);
    let rel2 = GTZ3::new(&ctx, y.r, real1.r);
    let rel3 = GTZ3::new(&ctx, y.r, x.r);
    let rel4 = GTZ3::new(&ctx, int1.r, real1.r);

    let string1 = rel1.s;
    let string2 = rel2.s;
    let string3 = rel3.s;
    let string4 = rel4.s;

    assert_eq!("(> x 7)", string1);
    assert_eq!("(> y (- (/ 271549371.0 500000.0)))", string2);
    assert_eq!("(> y (to_real x))", string3);
    assert_eq!("(> (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
}