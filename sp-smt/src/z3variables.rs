//! Z3 variables for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct BoolVarZ3<'ctx, 'bsrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub bsrt: &'bsrt BoolSortZ3<'ctx>,
    pub name: &'a str,
    // pub s: String,
    pub r: Z3_ast,
}

pub struct IntVarZ3<'ctx, 'isrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub isrt: &'isrt IntSortZ3<'ctx>,
    pub name: &'a str,
    // pub s: String,
    pub r: Z3_ast,
}

pub struct RealVarZ3<'ctx, 'rsrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub rsrt: &'rsrt RealSortZ3<'ctx>,
    pub name: &'a str,
    // pub s: String,
    pub r: Z3_ast,
}

pub struct StringVarZ3<'ctx, 'ssrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub ssrt: &'ssrt StringSortZ3<'ctx>,
    pub name: &'a str,
    // pub s: String,
    pub r: Z3_ast,
}

// /// Create a enumeration sort.
//     ///
//     /// An enumeration sort with `n` elements.
//     /// This function will also declare the functions corresponding to the enumerations.
//     ///
//     /// - `c`: logical context
//     /// - `name`: name of the enumeration sort.
//     /// - `n`: number of elements in enumeration sort.
//     /// - `enum_names`: names of the enumerated elements.
//     /// - `enum_consts`: constants corresponding to the enumerated elements.
//     /// - `enum_testers`: predicates testing if terms of the enumeration sort correspond to an enumeration.
//     ///
//     /// For example, if this function is called with three symbols A, B, C and the name S, then
//     /// `s` is a sort whose name is S, and the function returns three terms corresponding to A, B, C in
//     /// `enum_consts`. The array `enum_testers` has three predicates of type `(s -> Bool)`.
//     /// The first predicate (corresponding to A) is true when applied to A, and false otherwise.
//     /// Similarly for the other predicates.
//     pub fn Z3_mk_enumeration_sort(
//         c: Z3_context,
//         name: Z3_symbol,
//         n: ::std::os::raw::c_uint,
//         enum_names: *const Z3_symbol,
//         enum_consts: *mut Z3_func_decl,
//         enum_testers: *mut Z3_func_decl,
//     ) -> Z3_sort;

pub struct EnumVarZ3<'ctx, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub name: &'a str,
    pub args: Vec<(Z3_symbol, Z3_func_decl)>,
    pub s: String,
    pub r: Z3_sort
}

impl <'ctx, 'bsrt, 'a> BoolVarZ3<'ctx, 'bsrt, 'a> {
    /// Declare and create an Boolean variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, bsrt: &'bsrt BoolSortZ3<'ctx>, name: &'a str) -> Z3_ast {
        let bool_sort = bsrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), bool_sort)
        };
        BoolVarZ3 {ctx, bsrt, name, r: z3}.r
    }
}

impl <'ctx, 'isrt, 'a> IntVarZ3<'ctx, 'isrt, 'a> {
    /// Declare and create an Integer variable (constant).
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
    /// Declare and create an Real variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, rsrt: &'rsrt RealSortZ3<'ctx>, name: &'a str) -> Z3_ast{
        let real_sort = rsrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), real_sort)
        };
        RealVarZ3 {ctx, rsrt, name, r: z3}.r
    }
}

// impl <'ctx, 'ssrt, 'a> StringVarZ3<'ctx, 'ssrt, 'a> {
//     /// Declare and create an Real variable (constant).
//     pub fn new(ctx: &'ctx ContextZ3, ssrt: &'ssrt StringSortZ3<'ctx>, name: &'a str) -> StringVarZ3<'ctx, 'ssrt, 'a> {
//         let string_sort = ssrt.r;
//         let str_name = CString::new(name).unwrap();
//         let z3 = unsafe {
//             Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), string_sort)
//         };
//         StringVarZ3 {
//             ctx,
//             ssrt,
//             name,
//             r: z3,
//             s: unsafe {
//                 CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
//             }
//         }
//     }
// }

/// Z3 bool type variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::BoolVarZ3::new(&ctx, a)
/// ```
/// Using the global context:
/// ```text
/// bvrz3!(a)
/// ```
/// Using a specific context:
/// ```text
/// bvrz3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! bvrz3 {
    // ($a:expr) => {
    //     BoolVarZ3::new(&CTX, &BoolSortZ3::new(&CTX), $a).r
    // };
    ($ctx:expr, $a:expr) => {
        BoolVarZ3::new($ctx, &BoolSortZ3::new($ctx), $a)
    }
}

/// Z3 integer type variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::IntVarZ3::new(&ctx, a)
/// ```
/// Using the global context:
/// ```text
/// ivrz3!(a)
/// ```
/// Using a specific context:
/// ```text
/// ivrz3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! ivrz3 {
    // ($a:expr) => {
    //     IntVarZ3::new(&CTX, &IntSortZ3::new(&CTX), $a).r
    // };
    ($ctx:expr, $a:expr) => {
        IntVarZ3::new($ctx, &IntSortZ3::new($ctx), $a)
    }
}

/// Z3 real type variable
/// 
/// Macro rule for:
/// ```text
/// z3variables::RealVarZ3::new(&ctx, a)
/// ```
/// Using the global context:
/// ```text
/// rvrz3!(a)
/// ```
/// Using a specific context:
/// ```text
/// rvrz3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! rvrz3 {
    // ($a:expr) => {
    //     RealVarZ3::new(&CTX, $a).r
    // };
    ($ctx:expr, $a:expr) => {
        RealVarZ3::new($ctx, &RealSortZ3::new($ctx), $a)
    }
}

// #[test]
// fn test_new_bool_var(){
//     let conf = ConfigZ3::new();
//     let ctx = ContextZ3::new(&conf);
//     let sort = BoolSortZ3::new(&ctx);

//     let x = BoolVarZ3::new(&ctx, &sort, "x");
//     let y = BoolVarZ3::new(&ctx, &sort, "y");

//     let stringx = x.s;
//     let stringy = y.s;

//     assert_eq!("x", stringx);
//     assert_eq!("y", stringy);
// }

// #[test]
// fn test_new_int_var(){
//     let conf = ConfigZ3::new();
//     let ctx = ContextZ3::new(&conf);
//     let sort = IntSortZ3::new(&ctx);

//     let x = IntVarZ3::new(&ctx, &sort, "x");
//     let y = IntVarZ3::new(&ctx, &sort, "y");

//     let stringx = x.s;
//     let stringy = y.s;

//     assert_eq!("x", stringx);
//     assert_eq!("y", stringy);
// }

// #[test]
// fn test_new_real_var(){
//         let conf = ConfigZ3::new();
//         let ctx = ContextZ3::new(&conf);
//         let sort = RealSortZ3::new(&ctx);

//         let x = RealVarZ3::new(&ctx, &sort, "x");
//         let y = RealVarZ3::new(&ctx, &sort, "y");

//         let stringx = x.s;
//         let stringy = y.s;

//         assert_eq!("x", stringx);
//         assert_eq!("y", stringy);
// }

// #[test]
// fn test_new_string_var(){
//         let conf = ConfigZ3::new();
//         let ctx = ContextZ3::new(&conf);
//         let sort = StringSortZ3::new(&ctx);

//         let x = StringVarZ3::new(&ctx, &sort, "x");

//         let stringx = x.s;

//         assert_eq!("x", stringx);
// }
