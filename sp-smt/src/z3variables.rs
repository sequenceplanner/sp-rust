//! Z3 variables for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct BoolVarZ3<'ctx, 'bsrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub bsrt: &'bsrt BoolSortZ3<'ctx>,
    pub name: &'a str,
    pub s: String,
    pub r: Z3_ast,
}

pub struct IntVarZ3<'ctx, 'isrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub isrt: &'isrt IntSortZ3<'ctx>,
    pub name: &'a str,
    pub s: String,
    pub r: Z3_ast,
}

pub struct RealVarZ3<'ctx, 'rsrt, 'a> {
    pub ctx: &'ctx ContextZ3,
    pub rsrt: &'rsrt RealSortZ3<'ctx>,
    pub name: &'a str,
    pub s: String,
    pub r: Z3_ast,
}

impl <'ctx, 'bsrt, 'a> BoolVarZ3<'ctx, 'bsrt, 'a> {
    /// Declare and create an Boolean variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, bsrt: &'bsrt BoolSortZ3<'ctx>, name: &'a str) -> BoolVarZ3<'ctx, 'bsrt, 'a> {
        let bool_sort = bsrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), bool_sort)
        };
        BoolVarZ3 {
            ctx,
            bsrt,
            name,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx, 'isrt, 'a> IntVarZ3<'ctx, 'isrt, 'a> {
    /// Declare and create an Integer variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, isrt: &'isrt IntSortZ3<'ctx>, name: &'a str) -> IntVarZ3<'ctx, 'isrt, 'a> {
        let int_sort = isrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), int_sort)
        };
        IntVarZ3 {
            ctx,
            isrt,
            name,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}


impl <'ctx, 'rsrt, 'a> RealVarZ3<'ctx, 'rsrt, 'a> {
    /// Declare and create an Real variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, rsrt: &'rsrt RealSortZ3<'ctx>, name: &'a str) -> RealVarZ3<'ctx, 'rsrt, 'a> {
        let real_sort = rsrt.r;
        let str_name = CString::new(name).unwrap();
        let z3 = unsafe {
            Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, str_name.as_ptr()), real_sort)
        };
        RealVarZ3 {
            ctx,
            rsrt,
            name,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

#[test]
fn test_new_bool_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = BoolSortZ3::new(&ctx);

    let x = BoolVarZ3::new(&ctx, &sort, "x");
    let y = BoolVarZ3::new(&ctx, &sort, "y");

    let stringx = x.s;
    let stringy = y.s;

    assert_eq!("x", stringx);
    assert_eq!("y", stringy);
}

#[test]
fn test_new_int_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = IntSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &sort, "x");
    let y = IntVarZ3::new(&ctx, &sort, "y");

    let stringx = x.s;
    let stringy = y.s;

    assert_eq!("x", stringx);
    assert_eq!("y", stringy);
}

#[test]
fn test_new_real_var(){
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = RealSortZ3::new(&ctx);

        let x = RealVarZ3::new(&ctx, &sort, "x");
        let y = RealVarZ3::new(&ctx, &sort, "y");

        let stringx = x.s;
        let stringy = y.s;

        assert_eq!("x", stringx);
        assert_eq!("y", stringy);
}
