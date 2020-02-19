//! Z3 realtions for SP

use std::ffi::{CStr};
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
    /// NOTE: The nodes `left` and `right` must have the same type.
    /// 
    /// NOTE: See macro! `eq_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_eq(ctx.r, left, right)
        };
        EQZ3 {ctx, left, right, r: z3}.r
    }
}

impl <'ctx> LEZ3<'ctx> {
    /// Create less than or equal to.
    ///
    /// NOTE: The nodes `left` and `right` must have the same sort, and must be int or real.
    /// 
    /// NOTE: See macro! `le_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_le(ctx.r, left, right)
        };
        LEZ3 {ctx, left, right, r: z3}.r
    }
}

impl <'ctx> LTZ3<'ctx> {
    /// Create less than.
    ///
    /// NOTE: The nodes `left` and `right` must have the same sort, and must be int or real.
    /// 
    /// NOTE: See macro! `lt_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_lt(ctx.r, left, right)
        };
        LTZ3 {ctx, left, right, r: z3}.r
    }
}

impl <'ctx> GEZ3<'ctx> {
    /// Create greater than or equal to.
    ///
    /// NOTE: The nodes `left` and `right` must have the same sort, and must be int or real.
    /// 
    /// NOTE: See macro! `ge_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_ge(ctx.r, left, right)
        };
        GEZ3 {ctx, left, right, r: z3}.r
    }
}

impl <'ctx> GTZ3<'ctx> {
    /// Create greater than or equal to.
    ///
    /// NOTE: The nodes `left` and `right` must have the same sort, and must be int or real.
    /// 
    /// NOTE: See macro! `gt_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_gt(ctx.r, left, right)
        };
        GTZ3 {ctx, left, right, r: z3}.r
    }
}

/// a equal to b
/// 
/// Macro rule for:
/// ```text
/// z3relations::EQZ3::new(&ctx, a, b)
/// ```
/// Using a specific context:
/// ```text
/// eq_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are of the same sort.
#[macro_export]
macro_rules! eq_z3 {
    ($ctx:expr, $b:expr, $c:expr) => {
        EQZ3::new($ctx, $b, $c)
    }
}

/// a less than or equal to b
/// 
/// Macro rule for:
/// ```text
/// z3relations::LEZ3::new(&ctx, a, b)
/// ```
/// Using a specific context:
/// ```text
/// le_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are of the same sort.
#[macro_export]
macro_rules! le_z3 {
    ($ctx:expr, $b:expr, $c:expr) => {
        LEZ3::new($ctx, $b, $c)
    }
}

/// a less than b
/// 
/// Macro rule for:
/// ```text
/// z3relations::LTZ3::new(&ctx, a, b)
/// ```
/// Using a specific context:
/// ```text
/// lt_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are of the same sort.
#[macro_export]
macro_rules! lt_z3 {
    ($ctx:expr, $b:expr, $c:expr) => {
        LTZ3::new($ctx, $b, $c)
    }
}

/// a greater than or equal to b
/// 
/// Macro rule for:
/// ```text
/// z3relations::GEZ3::new(&ctx, a, b)
/// ```
/// Using a specific context:
/// ```text
/// ge_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are of the same sort.
#[macro_export]
macro_rules! ge_z3 {
    ($ctx:expr, $b:expr, $c:expr) => {
        GEZ3::new($ctx, $b, $c)
    }
}

/// a greater than b
/// 
/// Macro rule for:
/// ```text
/// z3relations::GTZ3::new(&ctx, a, b)
/// ```
/// Using a specific context:
/// ```text
/// gt_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are of the same sort.
#[macro_export]
macro_rules! gt_z3 {
    ($ctx:expr, $b:expr, $c:expr) => {
        GTZ3::new($ctx, $b, $c)
    }
}

#[test]
fn test_new_eq_1(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, x, int1);
    let rel2 = EQZ3::new(&ctx, y, real1);
    let rel3 = EQZ3::new(&ctx, y, x);
    let rel4 = EQZ3::new(&ctx, int1, real1);

    assert_eq!("(= x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(= y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(= y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(= (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
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

    let rel1 = LEZ3::new(&ctx, x, int1);
    let rel2 = LEZ3::new(&ctx, y, real1);
    let rel3 = LEZ3::new(&ctx, y, x);
    let rel4 = LEZ3::new(&ctx, int1, real1);

    assert_eq!("(<= x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(<= y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(<= y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(<= (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
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

    let rel1 = LTZ3::new(&ctx, x, int1);
    let rel2 = LTZ3::new(&ctx, y, real1);
    let rel3 = LTZ3::new(&ctx, y, x);
    let rel4 = LTZ3::new(&ctx, int1, real1);

    assert_eq!("(< x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(< y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(< y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(< (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
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

    let rel1 = GEZ3::new(&ctx, x, int1);
    let rel2 = GEZ3::new(&ctx, y, real1);
    let rel3 = GEZ3::new(&ctx, y, x);
    let rel4 = GEZ3::new(&ctx, int1, real1);

    assert_eq!("(>= x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(>= y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(>= y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(>= (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
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

    let rel1 = GTZ3::new(&ctx, x, int1);
    let rel2 = GTZ3::new(&ctx, y, real1);
    let rel3 = GTZ3::new(&ctx, y, x);
    let rel4 = GTZ3::new(&ctx, int1, real1);

    assert_eq!("(> x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(> y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(> y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(> (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
}

#[test]
fn test_eq_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let eq1 = eq_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(= y 11.0)", ast_to_string_z3!(&ctx, eq1));
}



#[test]
fn test_lt_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let lt1 = lt_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(< y 11.0)", ast_to_string_z3!(&ctx, lt1));
}

#[test]
fn test_gt_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let gt1 = gt_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(> y 11.0)", ast_to_string_z3!(&ctx, gt1));
}

#[test]
fn test_ge_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let ge1 = ge_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(>= y 11.0)", ast_to_string_z3!(&ctx, ge1));
}

#[test]
fn test_le_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let le1 = le_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(<= y 11.0)", ast_to_string_z3!(&ctx, le1));
}