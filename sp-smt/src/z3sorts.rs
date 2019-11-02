//! Z3 sorts for SP

use std::ffi::{CStr};
use z3_sys::*;
use super::*;

pub struct BoolSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

pub struct IntSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

pub struct RealSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

pub struct StringSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

pub struct GetSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub r: Z3_sort
}

impl <'ctx> BoolSortZ3<'ctx> {
    /// Create the Boolean type.
    ///
    /// This type is used to create propositional variables and predicates.
    pub fn new(ctx: &'ctx ContextZ3) -> BoolSortZ3 {
        let z3 = unsafe { 
            Z3_mk_bool_sort(ctx.r)
        };
        BoolSortZ3 {ctx, r: z3}
    }
}

impl <'ctx> IntSortZ3<'ctx> {
    /// Create the integer type.
    ///
    /// This type is not the int type found in programming languages.
    /// A machine integer can be represented using bit-vectors. The function
    /// [`Z3_mk_bv_sort`](fn.Z3_mk_bv_sort.html) creates a bit-vector type.
    ///
    /// # See also:
    ///
    /// - [`Z3_mk_bv_sort`](fn.Z3_mk_bv_sort.html)
    pub fn new(ctx: &'ctx ContextZ3) -> IntSortZ3 {
        let z3 = unsafe {
            Z3_mk_int_sort(ctx.r)
        };
        IntSortZ3 {ctx, r: z3}
    }
}

impl <'ctx> RealSortZ3<'ctx> {
    /// Create the real type.
    ///
    /// Note that this type is not a floating point number.
    pub fn new(ctx: &'ctx ContextZ3) -> RealSortZ3 {
        let z3 = unsafe {
            Z3_mk_real_sort(ctx.r)
        };
        RealSortZ3 {ctx, r: z3}
    }
}

impl <'ctx> StringSortZ3<'ctx> {
    /// Create a sort for 8 bit strings.
    ///
    /// This function creates a sort for ASCII strings.
    /// Each character is 8 bits.
    pub fn new(ctx: &'ctx ContextZ3) -> StringSortZ3 {
        let z3 = unsafe {
            Z3_mk_string_sort(ctx.r)
        };
        StringSortZ3 {ctx, r: z3}
    }
}

impl<'ctx> GetSortZ3<'ctx> {
    /// Return the sort of an AST node.
    /// 
    /// The AST node must be a constant, application, numeral, bound variable, or quantifier. 
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> GetSortZ3 {
        let z3 = unsafe {
            Z3_get_sort(ctx.r, arg)
        };
        GetSortZ3 {ctx, r: z3, arg}        
    }
}

/// Z3 sort to readable string
#[macro_export]
macro_rules! srtstrz3 {
    ($ctx:expr, $a:expr) => {
        unsafe {
            CStr::from_ptr(Z3_sort_to_string($ctx, $a)).to_str().unwrap().to_owned()
        }
    }
}

#[test]
fn test_bool_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = BoolSortZ3::new(&ctx);
    assert_eq!("Bool", srtstrz3!(ctx.r, sort.r));
}

#[test]
fn test_int_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = IntSortZ3::new(&ctx);
    assert_eq!("Int", srtstrz3!(ctx.r, sort.r));
}

#[test]
fn test_real_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = RealSortZ3::new(&ctx);
    assert_eq!("Real", srtstrz3!(ctx.r, sort.r));
}

#[test]
fn test_string_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = StringSortZ3::new(&ctx);
    assert_eq!("String", srtstrz3!(ctx.r, sort.r));
}