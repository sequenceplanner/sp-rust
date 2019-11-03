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
    /// 
    /// NOTE: See macro! bool_sort_z3!
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
    /// NOTE: This type is not the int type found in programming languages.
    /// A machine integer can be represented using bit-vectors. 
    /// 
    /// NOTE: See macro! int_sort_z3!
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
    /// NOTE: This type is not a floating point number.
    /// 
    /// NOTE: See macro! real_sort_z3!
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
    /// 
    /// NOTE: See macro! string_sort_z3!
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
    /// NOTE: The AST node must be a constant, application, numeral, bound variable, or quantifier. 
    /// 
    /// NOTE: See macro! get_sort_z3!
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> GetSortZ3 {
        let z3 = unsafe {
            Z3_get_sort(ctx.r, arg)
        };
        GetSortZ3 {ctx, r: z3, arg}        
    }
}

/// Define a bool sort 
#[macro_export]
macro_rules! bool_sort_z3 {
    ($ctx:expr) => {
        BoolSortZ3::new($ctx)
    }
}

/// Define an int sort 
#[macro_export]
macro_rules! int_sort_z3 {
    ($ctx:expr) => {
        IntSortZ3::new($ctx)
    }
}

/// Define a real sort 
#[macro_export]
macro_rules! real_sort_z3 {
    ($ctx:expr) => {
        RealSortZ3::new($ctx)
    }
}

/// Define a string sort 
#[macro_export]
macro_rules! string_sort_z3 {
    ($ctx:expr) => {
        StringSortZ3::new($ctx)
    }
}

/// Define a string sort 
#[macro_export]
macro_rules! get_sort_z3 {
    ($ctx:expr, $a:expr) => {
        GetSortZ3::new($ctx, $a).r
    }
}

/// Z3 sort to readable string
#[macro_export]
macro_rules! sort_to_string_z3 {
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
    assert_eq!("Bool", sort_to_string_z3!(ctx.r, sort.r));
}

#[test]
fn test_int_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = IntSortZ3::new(&ctx);
    assert_eq!("Int", sort_to_string_z3!(ctx.r, sort.r));
}

#[test]
fn test_real_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = RealSortZ3::new(&ctx);
    assert_eq!("Real", sort_to_string_z3!(ctx.r, sort.r));
}

#[test]
fn test_string_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = StringSortZ3::new(&ctx);
    assert_eq!("String", sort_to_string_z3!(ctx.r, sort.r));
}


#[test]
fn test_bool_sort_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let bsrt = bool_sort_z3!(&ctx);
    assert_eq!("Bool", sort_to_string_z3!(ctx.r, bsrt.r));
}

#[test]
fn test_int_sort_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let isrt = int_sort_z3!(&ctx);
    assert_eq!("Int", sort_to_string_z3!(ctx.r, isrt.r));
}

#[test]
fn test_real_sort_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let rsrt = real_sort_z3!(&ctx);
    assert_eq!("Real", sort_to_string_z3!(ctx.r, rsrt.r));
}

#[test]
fn test_string_sort_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let ssrt = string_sort_z3!(&ctx);
    assert_eq!("String", sort_to_string_z3!(ctx.r, ssrt.r));
}