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

pub struct EnumSortZ3<'ctx, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub name: &'a str,
    pub nr: u32,
    pub enum_names: Vec<Z3_symbol>,
    pub enum_consts: Vec<Z3_func_decl>,
    pub enum_testers: Vec<Z3_func_decl>,
    pub enum_asts: Vec<Z3_ast>,
    pub r: Z3_sort
}

pub struct GetSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub r: Z3_sort
}
pub struct SortToStringZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub what: Z3_sort,
    pub r: String
}

impl<'ctx> SortToStringZ3<'ctx> {
    /// Z3 optimizer to readable string
    /// 
    /// NOTE: See macro! `sort_to_string_z3!`
    pub fn new(ctx: &'ctx ContextZ3, what: Z3_sort) -> String {
        let z3 = unsafe {
            CStr::from_ptr(Z3_sort_to_string(ctx.r, what)).to_str().unwrap().to_owned()
        };
        SortToStringZ3 {ctx, what, r: z3}.r
    }
}

impl <'ctx> BoolSortZ3<'ctx> {
    /// Create the Boolean type.
    ///
    /// This type is used to create propositional variables and predicates.
    /// 
    /// NOTE: See macro! `bool_sort_z3!`
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
    /// NOTE: See macro! `int_sort_z3!`
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
    /// NOTE: See macro! `real_sort_z3!`
    pub fn new(ctx: &'ctx ContextZ3) -> RealSortZ3 {
        let z3 = unsafe {
            Z3_mk_real_sort(ctx.r)
        };
        RealSortZ3 {ctx, r: z3}
    }
}

impl <'ctx, 'a> EnumSortZ3<'ctx, 'a> {
    /// Create an enumeration type.
    ///
    /// NOTE: See macro! `enum_sort_z3!`
    pub fn new(ctx: &'ctx ContextZ3, name: &'a str, enum_elements: Vec<&str>) -> EnumSortZ3<'ctx, 'a> {
        
        let z3 = unsafe {
            let len = enum_elements.len() as u32;
            let enum_name = CString::new(name).unwrap(); 
            let enum_name_symbol = Z3_mk_string_symbol(ctx.r, enum_name.as_ptr());  

            let mut enum_names: Vec<Z3_symbol> = vec![std::ptr::null_mut(); enum_elements.len()];
            let mut enum_consts: Vec<Z3_func_decl> = vec![std::ptr::null_mut(); enum_elements.len()];
            let mut enum_testers: Vec<Z3_func_decl> = vec![std::ptr::null_mut(); enum_elements.len()];
            let mut enum_asts: Vec<Z3_ast> = vec![std::ptr::null_mut(); enum_elements.len()];

            for s in &enum_elements {
                let index = enum_elements.iter().position(|&r| r == *s).unwrap();
                enum_names[index] = Z3_mk_string_symbol(ctx.r, CString::new(*s).unwrap().as_ptr());
            }

            let enum1 = Z3_mk_enumeration_sort(ctx.r, enum_name_symbol, enum_names.len() as u32, enum_names.as_ptr(), enum_consts.as_mut_ptr(), enum_testers.as_mut_ptr());
            
            for c in &enum_names {
                let index = enum_names.iter().position(|&r| r == *c).unwrap();
                enum_asts[index] = Z3_mk_app(ctx.r, enum_consts[index], 0, [].as_ptr());
            }

            (len, enum_names, enum_consts, enum_testers, enum_asts, enum1)
        };
        EnumSortZ3 {ctx, name, nr: z3.0, enum_names: z3.1, enum_consts: z3.2, enum_testers: z3.3, enum_asts: z3.4, r: z3.5}
    }
}

impl <'ctx> StringSortZ3<'ctx> {
    /// Create a sort for 8 bit strings.
    ///
    /// This function creates a sort for ASCII strings.
    /// Each character is 8 bits.
    /// 
    /// NOTE: See macro! `string_sort_z3!`
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
    /// NOTE: See macro! `get_sort_z3!`
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> GetSortZ3 {
        let z3 = unsafe {
            Z3_get_sort(ctx.r, arg)
        };
        GetSortZ3 {ctx, r: z3, arg}        
    }
}

/// define a bool sort 
#[macro_export]
macro_rules! bool_sort_z3 {
    ($ctx:expr) => {
        BoolSortZ3::new($ctx)
    }
}

/// define an int sort 
#[macro_export]
macro_rules! int_sort_z3 {
    ($ctx:expr) => {
        IntSortZ3::new($ctx)
    }
}

/// define a real sort 
#[macro_export]
macro_rules! real_sort_z3 {
    ($ctx:expr) => {
        RealSortZ3::new($ctx)
    }
}

/// define a string sort 
#[macro_export]
macro_rules! string_sort_z3 {
    ($ctx:expr) => {
        StringSortZ3::new($ctx)
    }
}

/// get sort 
#[macro_export]
macro_rules! get_sort_z3 {
    ($ctx:expr, $a:expr) => {
        GetSortZ3::new($ctx, $a).r
    }
}

/// define an enumeration sort 
#[macro_export]
macro_rules! enum_sort_z3 {
    ($ctx:expr, $a:expr, $b:expr) => {
        EnumSortZ3::new($ctx, $a, $b)
    }
}

/// sort to readable string
#[macro_export]
macro_rules! sort_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        SortToStringZ3::new($ctx, $a)
    }
}

#[test]
fn test_bool_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = BoolSortZ3::new(&ctx);
    assert_eq!("Bool", sort_to_string_z3!(&ctx, sort.r));
}

#[test]
fn test_int_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = IntSortZ3::new(&ctx);
    assert_eq!("Int", sort_to_string_z3!(&ctx, sort.r));
}

#[test]
fn test_real_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = RealSortZ3::new(&ctx);
    assert_eq!("Real", sort_to_string_z3!(&ctx, sort.r));
}

#[test]
fn test_string_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = StringSortZ3::new(&ctx);
    assert_eq!("String", sort_to_string_z3!(&ctx, sort.r));
}

#[test]
fn test_enum_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = EnumSortZ3::new(&ctx, "fruit", vec!("apple", "banana", "orange"));
    assert_eq!("fruit", sort_to_string_z3!(&ctx, sort.r));
}


#[test]
fn test_bool_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let bsrt = bool_sort_z3!(&ctx);
    assert_eq!("Bool", sort_to_string_z3!(&ctx, bsrt.r));
}

#[test]
fn test_int_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let isrt = int_sort_z3!(&ctx);
    assert_eq!("Int", sort_to_string_z3!(&ctx, isrt.r));
}

#[test]
fn test_real_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let rsrt = real_sort_z3!(&ctx);
    assert_eq!("Real", sort_to_string_z3!(&ctx, rsrt.r));
}

#[test]
fn test_string_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let ssrt = string_sort_z3!(&ctx);
    assert_eq!("String", sort_to_string_z3!(&ctx, ssrt.r));
}