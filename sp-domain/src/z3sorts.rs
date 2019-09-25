//! Z3 sorts for SP

use std::ffi::{CStr, CString};
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

impl <'ctx> BoolSortZ3<'ctx> {
    /// Create the Boolean type.
    ///
    /// This type is used to create propositional variables and predicates.
    pub fn new(ctx: &'ctx ContextZ3) -> BoolSortZ3 {
        BoolSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_bool_sort(ctx.r);
                sort
            }
        }
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
        IntSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_int_sort(ctx.r);
                sort
            }
        }
    }
}

impl <'ctx> RealSortZ3<'ctx> {
    /// Create the real type.
    ///
    /// Note that this type is not a floating point number.
    pub fn new(ctx: &'ctx ContextZ3) -> RealSortZ3 {
        RealSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_real_sort(ctx.r);
                sort
            }
        }
    }
}

impl <'ctx> StringSortZ3<'ctx> {
    /// Create a sort for 8 bit strings.
    ///
    /// This function creates a sort for ASCII strings.
    /// Each character is 8 bits.
    pub fn new(ctx: &'ctx ContextZ3) -> StringSortZ3 {
        StringSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_string_sort(ctx.r);
                sort
            }
        }
    }
}

#[test]
fn test_bool_sort(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = BoolSortZ3::new(&ctx);
        let string = Z3_sort_to_string(ctx.r, sort.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
    }
}

#[test]
fn test_int_sort(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = IntSortZ3::new(&ctx);
        let string = Z3_sort_to_string(ctx.r, sort.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
    }
}

#[test]
fn test_real_sort(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = RealSortZ3::new(&ctx);
        let string = Z3_sort_to_string(ctx.r, sort.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
    }
}

#[test]
fn test_string_sort(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = StringSortZ3::new(&ctx);
        let string = Z3_sort_to_string(ctx.r, sort.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
    }
}