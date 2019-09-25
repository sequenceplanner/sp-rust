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

impl <'ctx, 'bsrt, 'a> BoolVarZ3<'ctx, 'bsrt, 'a> {
    /// Declare and create an Boolean variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, bsrt: &'bsrt BoolSortZ3<'ctx>, name: &'a str) -> BoolVarZ3<'ctx, 'bsrt, 'a> {
        BoolVarZ3 {
            ctx,
            bsrt,
            name,
            r: unsafe {
                let bool_sort = bsrt.r;
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.r, str_name.as_ptr());
                let bool_var = Z3_mk_const(ctx.r, sym_name, bool_sort);
                bool_var
            }
        }
    }
}

impl <'ctx, 'isrt, 'a> IntVarZ3<'ctx, 'isrt, 'a> {
    /// Declare and create an Integer variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, isrt: &'isrt IntSortZ3<'ctx>, name: &'a str) -> IntVarZ3<'ctx, 'isrt, 'a> {
        IntVarZ3 {
            ctx,
            isrt,
            name,
            r: unsafe {
                let int_sort = isrt.r;
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.r, str_name.as_ptr());
                let int_var = Z3_mk_const(ctx.r, sym_name, int_sort);
                int_var
            }
        }
    }
}


impl <'ctx, 'rsrt, 'a> RealVarZ3<'ctx, 'rsrt, 'a> {
    /// Declare and create an Real variable (constant).
    pub fn new(ctx: &'ctx ContextZ3, rsrt: &'rsrt RealSortZ3<'ctx>, name: &'a str) -> RealVarZ3<'ctx, 'rsrt, 'a> {
        RealVarZ3 {
            ctx,
            rsrt,
            name,
            r: unsafe {
                let real_sort = rsrt.r;
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.r, str_name.as_ptr());
                let real_var = Z3_mk_const(ctx.r, sym_name, real_sort);
                real_var
            }
        }
    }
}

#[test]
fn test_new_bool_var(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = BoolSortZ3::new(&ctx);

        let x = BoolVarZ3::new(&ctx, &sort, "x");
        let y = BoolVarZ3::new(&ctx, &sort, "y");

        let stringx = CStr::from_ptr(Z3_ast_to_string(ctx.r, x.r)).to_str().unwrap().to_owned();
        let stringy = CStr::from_ptr(Z3_ast_to_string(ctx.r, y.r)).to_str().unwrap().to_owned();

        let whatx = Z3_get_sort(ctx.r, x.r);
        let whaty = Z3_get_sort(ctx.r, y.r);
        let whatstringx = CStr::from_ptr(Z3_sort_to_string(ctx.r, whatx)).to_str().unwrap().to_owned();
        let wahtstringy = CStr::from_ptr(Z3_sort_to_string(ctx.r, whaty)).to_str().unwrap().to_owned();

        assert_eq!("x", stringx);
        assert_eq!("y", stringy);
        assert_eq!("Bool", whatstringx);
        assert_eq!("Bool", wahtstringy);
    }
}

#[test]
fn test_new_int_var(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = IntSortZ3::new(&ctx);

        let x = IntVarZ3::new(&ctx, &sort, "x");
        let y = IntVarZ3::new(&ctx, &sort, "y");

        let stringx = CStr::from_ptr(Z3_ast_to_string(ctx.r, x.r)).to_str().unwrap().to_owned();
        let stringy = CStr::from_ptr(Z3_ast_to_string(ctx.r, y.r)).to_str().unwrap().to_owned();

        let whatx = Z3_get_sort(ctx.r, x.r);
        let whaty = Z3_get_sort(ctx.r, y.r);
        let whatstringx = CStr::from_ptr(Z3_sort_to_string(ctx.r, whatx)).to_str().unwrap().to_owned();
        let wahtstringy = CStr::from_ptr(Z3_sort_to_string(ctx.r, whaty)).to_str().unwrap().to_owned();

        assert_eq!("x", stringx);
        assert_eq!("y", stringy);
        assert_eq!("Int", whatstringx);
        assert_eq!("Int", wahtstringy);
    }
}

#[test]
fn test_new_real_var(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = RealSortZ3::new(&ctx);

        let x = RealVarZ3::new(&ctx, &sort, "x");
        let y = RealVarZ3::new(&ctx, &sort, "y");

        let stringx = CStr::from_ptr(Z3_ast_to_string(ctx.r, x.r)).to_str().unwrap().to_owned();
        let stringy = CStr::from_ptr(Z3_ast_to_string(ctx.r, y.r)).to_str().unwrap().to_owned();

        let whatx = Z3_get_sort(ctx.r, x.r);
        let whaty = Z3_get_sort(ctx.r, y.r);
        let whatstringx = CStr::from_ptr(Z3_sort_to_string(ctx.r, whatx)).to_str().unwrap().to_owned();
        let wahtstringy = CStr::from_ptr(Z3_sort_to_string(ctx.r, whaty)).to_str().unwrap().to_owned();

        assert_eq!("x", stringx);
        assert_eq!("y", stringy);
        assert_eq!("Real", whatstringx);
        assert_eq!("Real", wahtstringy);
    }
}
