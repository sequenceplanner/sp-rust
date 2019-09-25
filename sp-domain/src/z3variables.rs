//! Some Z3 variables for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;
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

///Create an integer variable (took some time to figure out the lifetime stuff)
impl <'ctx, 'isrt, 'a> IntVarZ3<'ctx, 'isrt, 'a> {
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

impl <'ctx, 'bsrt, 'a> BoolVarZ3<'ctx, 'bsrt, 'a> {
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

impl <'ctx, 'rsrt, 'a> RealVarZ3<'ctx, 'rsrt, 'a> {
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
        let string = Z3_ast_to_string(ctx.r, x.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
        let what = Z3_get_sort(ctx.r, x.r);
        let string2 = Z3_sort_to_string(ctx.r, what);
        println!("{:?}", CStr::from_ptr(string2).to_str().unwrap());
    }
}

#[test]
fn test_new_int_var(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = IntSortZ3::new(&ctx);
        let x = IntVarZ3::new(&ctx, &sort, "x");
        let string = Z3_ast_to_string(ctx.r, x.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
        let what = Z3_get_sort(ctx.r, x.r);
        let string2 = Z3_sort_to_string(ctx.r, what);
        println!("{:?}", CStr::from_ptr(string2).to_str().unwrap());
    }
}

#[test]
fn test_new_real_var(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = RealSortZ3::new(&ctx);
        let x = RealVarZ3::new(&ctx, &sort, "x");
        let string = Z3_ast_to_string(ctx.r, x.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
        let what = Z3_get_sort(ctx.r, x.r);
        let string2 = Z3_sort_to_string(ctx.r, what);
        println!("{:?}", CStr::from_ptr(string2).to_str().unwrap());
    }
}
