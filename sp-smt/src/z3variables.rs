//! Z3 variables for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct BoolVarZ3<'ctx, 'bsrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub bsrt: &'bsrt BoolSortZ3<'ctx>,
    pub name: &'a str,
    pub r: Z3_ast,
}

pub struct IntVarZ3<'ctx, 'isrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub isrt: &'isrt IntSortZ3<'ctx>,
    pub name: &'a str,
    pub r: Z3_ast,
}

pub struct RealVarZ3<'ctx, 'rsrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub rsrt: &'rsrt RealSortZ3<'ctx>,
    pub name: &'a str,
    pub r: Z3_ast,
}

pub struct StringVarZ3<'ctx, 'ssrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub ssrt: &'ssrt StringSortZ3<'ctx>,
    pub name: &'a str,
    pub r: Z3_ast,
}

pub struct EnumVarZ3<'ctx, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub esrt: Z3_sort,
    pub name: &'a str,
    pub r: Z3_ast,
}

impl <'ctx, 'bsrt, 'a> BoolVarZ3<'ctx, 'bsrt, 'a> {
    /// Declare and create a Boolean type variable.
    /// 
    /// NOTE: See macro! `bool_var_z3!`
    pub fn new(ctx: &'ctx ContextZ3, bsrt: &'bsrt BoolSortZ3<'ctx>, name: &'a str) -> Z3_ast {
        let bool_sort = bsrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), bool_sort)
        };
        BoolVarZ3 {ctx, bsrt, name, r: z3}.r
    }
}

impl <'ctx, 'isrt, 'a> IntVarZ3<'ctx, 'isrt, 'a> {
    /// Declare and create an Integer type variable.
    /// 
    /// NOTE: See macro! `int_var_z3!`
    pub fn new(ctx: &'ctx ContextZ3, isrt: &'isrt IntSortZ3<'ctx>, name: &'a str) -> Z3_ast {
        let int_sort = isrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), int_sort)
        };
        IntVarZ3 {ctx, isrt, name,r: z3}.r
    }
}

impl <'ctx, 'rsrt, 'a> RealVarZ3<'ctx, 'rsrt, 'a> {
    /// Declare and create an Real type variable.
    /// 
    /// NOTE: See macro! `real_var_z3!`
    pub fn new(ctx: &'ctx ContextZ3, rsrt: &'rsrt RealSortZ3<'ctx>, name: &'a str) -> Z3_ast{
        let real_sort = rsrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), real_sort)
        };
        RealVarZ3 {ctx, rsrt, name, r: z3}.r
    }
}

impl <'ctx, 'ssrt, 'a> StringVarZ3<'ctx, 'ssrt, 'a> {
    /// Declare and create an String type variable
    /// 
    /// NOTE: See macro! `string_var_z3!`
    pub fn new(ctx: &'ctx ContextZ3, ssrt: &'ssrt StringSortZ3<'ctx>, name: &'a str) -> Z3_ast{
        let string_sort = ssrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), string_sort)
        };
        StringVarZ3 {ctx, ssrt, name, r: z3}.r
    }
}

impl <'ctx, 'a> EnumVarZ3<'ctx, 'a> {
    /// Declare and create an Enum type variable
    /// 
    /// NOTE: See macro! `enum_var_z3!`
    pub fn new(ctx: &'ctx ContextZ3, esrt: Z3_sort, name: &'a str) -> Z3_ast{
        // let enum_sort = esrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), esrt)
        };
        EnumVarZ3 {ctx, esrt, name, r: z3}.r
    }
}

/// create a boolean variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::BoolVarZ3::new(&ctx, a)
/// ```
// / Using the global context:
// / ```text
// / bool_var_z3!(a)
// / ```
/// Using a specific context:
/// ```text
/// bool_var_z3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! bool_var_z3 {
    // ($a:expr) => {
    //     BoolVarZ3::new(&CTX, &BoolSortZ3::new(&CTX), $a).r
    // };
    ($ctx:expr, $a:expr) => {
        BoolVarZ3::new($ctx, &BoolSortZ3::new($ctx), $a)
    }
}

/// create an integer variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::IntVarZ3::new(&ctx, a)
/// ```
// / Using the global context:
// / ```text
// / int_var_z3!(a)
// / ```
/// Using a specific context:
/// ```text
/// int_var_z3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! int_var_z3 {
    // ($a:expr) => {
    //     IntVarZ3::new(&CTX, &IntSortZ3::new(&CTX), $a).r
    // };
    ($ctx:expr, $a:expr) => {
        IntVarZ3::new($ctx, &IntSortZ3::new($ctx), $a)
    }
}

/// create a real variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::RealVarZ3::new(&ctx, a)
/// ```
// / Using the global context:
// / ```text
// / real_var_z3!(a)
// / ```
/// Using a specific context:
/// ```text
/// real_var_z3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! real_var_z3 {
    // ($a:expr) => {
    //     RealVarZ3::new(&CTX, $a).r
    // };
    ($ctx:expr, $a:expr) => {
        RealVarZ3::new($ctx, &RealSortZ3::new($ctx), $a)
    }
}

/// create a string variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::StringVarZ3::new(&ctx, a)
/// ```
// / Using the global context:
// / ```text
// / string_var_z3!(a)
// / ```
/// Using a specific context:
/// ```text
/// string_var_z3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! string_var_z3 {
    // ($a:expr) => {
    //     StringVarZ3::new(&CTX, $a).r
    // };
    ($ctx:expr, $a:expr) => {
        StringVarZ3::new($ctx, &StringSortZ3::new($ctx), $a)
    }
}

/// create an enumeration variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::EnumVarZ3::new(&ctx, a)
/// ```
// / Using the global context:
// / ```text
// / str_var_z3!(a)
// / ```
/// Using a specific context:
/// ```text
/// str_var_z3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! enum_var_z3 {
    // ($a:expr) => {
    //     EnumVarZ3::new(&CTX, $a).r
    // };
    ($ctx:expr, $a:expr, $b:expr) => {
        EnumVarZ3::new($ctx, $a, $b)
    }
}

#[test]
fn test_new_bool_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = BoolSortZ3::new(&ctx);

    let x = BoolVarZ3::new(&ctx, &sort, "x");
    let y = BoolVarZ3::new(&ctx, &sort, "y");

    assert_eq!("x", ast_to_string_z3!(&ctx, x));
    assert_eq!("y", ast_to_string_z3!(&ctx, y));
}

#[test]
fn test_new_int_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = IntSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &sort, "x");
    let y = IntVarZ3::new(&ctx, &sort, "y");

    assert_eq!("x", ast_to_string_z3!(&ctx, x));
    assert_eq!("y", ast_to_string_z3!(&ctx, y));
}

#[test]
fn test_new_real_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = RealSortZ3::new(&ctx);

    let x = RealVarZ3::new(&ctx, &sort, "x");
    let y = RealVarZ3::new(&ctx, &sort, "y");

    assert_eq!("x", ast_to_string_z3!(&ctx, x));
    assert_eq!("y", ast_to_string_z3!(&ctx, y));
}

#[test]
fn test_new_string_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = StringSortZ3::new(&ctx);

    let x = StringVarZ3::new(&ctx, &sort, "x");

    assert_eq!("x", ast_to_string_z3!(&ctx, x));
}

#[test]
fn test_bool_var_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let bool1 = bool_var_z3!(&ctx, "x");
    assert_eq!("x", ast_to_string_z3!(&ctx, bool1));
    assert_eq!("Bool", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, bool1)));
}

#[test]
fn test_int_var_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let int1 = int_var_z3!(&ctx, "x");
    assert_eq!("x", ast_to_string_z3!(&ctx, int1));
    assert_eq!("Int", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, int1)));
}

#[test]
fn test_real_var_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let real1 = real_var_z3!(&ctx, "x");
    assert_eq!("x", ast_to_string_z3!(&ctx, real1));
    assert_eq!("Real", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, real1)));
}

#[test]
fn test_string_var_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let string1 = string_var_z3!(&ctx, "x");
    assert_eq!("x", ast_to_string_z3!(&ctx, string1));
    assert_eq!("String", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, string1)));
}