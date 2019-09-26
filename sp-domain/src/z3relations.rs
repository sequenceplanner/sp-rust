//! Z3 realtions for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct EQZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

pub struct LEZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

pub struct LTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

pub struct GEZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

pub struct GTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

impl <'ctx> EQZ3<'ctx> {
    /// Create an AST node representing `left = right`.
    ///
    /// The nodes `left` and `right` must have the same type.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> EQZ3 {
        EQZ3 {
            ctx,
            left,
            right,
            r: unsafe {
                let eq = Z3_mk_eq(ctx.r, left, right);
                eq
            }
        }
    }
}

impl <'ctx> LEZ3<'ctx> {
    /// Create less than or equal to.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> LEZ3 {
        LEZ3 {
            ctx,
            left,
            right,
            r: unsafe {
                let le = Z3_mk_le(ctx.r, left, right);
                le
            }
        }
    }
}

impl <'ctx> LTZ3<'ctx> {
    /// Create less than.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> LTZ3 {
        LTZ3 {
            ctx,
            left,
            right,
            r: unsafe {
                let lt = Z3_mk_lt(ctx.r, left, right);
                lt
            }
        }
    }
}

impl <'ctx> GEZ3<'ctx> {
    /// Create greater than or equal to.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> GEZ3 {
        GEZ3 {
            ctx,
            left,
            right,
            r: unsafe {
                let ge = Z3_mk_ge(ctx.r, left, right);
                ge
            }
        }
    }
}

impl <'ctx> GTZ3<'ctx> {
    /// Create greater than or equal to.
    ///
    /// The nodes `left` and `right` must have the same sort, and must be int or real.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> GTZ3 {
        GTZ3 {
            ctx,
            left,
            right,
            r: unsafe {
                let gt = Z3_mk_gt(ctx.r, left, right);
                gt
            }
        }
    }
}

#[test]
fn test_new_eq(){
    unsafe{
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

        let string1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel1.r)).to_str().unwrap().to_owned();
        let string2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel2.r)).to_str().unwrap().to_owned();
        let string3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel3.r)).to_str().unwrap().to_owned();
        let string4 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel4.r)).to_str().unwrap().to_owned();

        assert_eq!("(= x 7)", string1);
        assert_eq!("(= y (- (/ 271549371.0 500000.0)))", string2);
        assert_eq!("(= y (to_real x))", string3);
        assert_eq!("(= (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
    }
}

#[test]
fn test_new_le(){
    unsafe{
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

        let string1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel1.r)).to_str().unwrap().to_owned();
        let string2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel2.r)).to_str().unwrap().to_owned();
        let string3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel3.r)).to_str().unwrap().to_owned();
        let string4 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel4.r)).to_str().unwrap().to_owned();

        assert_eq!("(<= x 7)", string1);
        assert_eq!("(<= y (- (/ 271549371.0 500000.0)))", string2);
        assert_eq!("(<= y (to_real x))", string3);
        assert_eq!("(<= (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
    }
}

#[test]
fn test_new_lt(){
    unsafe{
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

        let string1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel1.r)).to_str().unwrap().to_owned();
        let string2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel2.r)).to_str().unwrap().to_owned();
        let string3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel3.r)).to_str().unwrap().to_owned();
        let string4 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel4.r)).to_str().unwrap().to_owned();

        assert_eq!("(< x 7)", string1);
        assert_eq!("(< y (- (/ 271549371.0 500000.0)))", string2);
        assert_eq!("(< y (to_real x))", string3);
        assert_eq!("(< (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
    }
}

#[test]
fn test_new_ge(){
    unsafe{
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

        let string1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel1.r)).to_str().unwrap().to_owned();
        let string2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel2.r)).to_str().unwrap().to_owned();
        let string3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel3.r)).to_str().unwrap().to_owned();
        let string4 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel4.r)).to_str().unwrap().to_owned();

        assert_eq!("(>= x 7)", string1);
        assert_eq!("(>= y (- (/ 271549371.0 500000.0)))", string2);
        assert_eq!("(>= y (to_real x))", string3);
        assert_eq!("(>= (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
    }
}

#[test]
fn test_new_gt(){
    unsafe{
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

        let string1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel1.r)).to_str().unwrap().to_owned();
        let string2 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel2.r)).to_str().unwrap().to_owned();
        let string3 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel3.r)).to_str().unwrap().to_owned();
        let string4 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel4.r)).to_str().unwrap().to_owned();

        assert_eq!("(> x 7)", string1);
        assert_eq!("(> y (- (/ 271549371.0 500000.0)))", string2);
        assert_eq!("(> y (to_real x))", string3);
        assert_eq!("(> (to_real 7) (- (/ 271549371.0 500000.0)))", string4);
    }
}