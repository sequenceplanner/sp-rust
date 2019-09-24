//! Some Z3 variables for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;
use super::*;

/// TODO: add Sort struct and include it in the generalized var struct
pub struct IntVar<'ctx, 'isrt, 'a> {
    pub ctx: &'ctx Context,
    pub isrt: &'isrt IntSort<'ctx>,
    pub name: &'a str,
    pub r: Z3_ast,
}

pub struct BoolVar<'ctx, 'bsrt, 'a> {
    pub ctx: &'ctx Context,
    pub bsrt: &'bsrt BoolSort<'ctx>,
    pub name: &'a str,
    pub r: Z3_ast,
}

pub struct RealVar<'ctx, 'rsrt, 'a> {
    pub ctx: &'ctx Context,
    pub rsrt: &'rsrt RealSort<'ctx>,
    pub name: &'a str,
    pub r: Z3_ast,
}

///Create an integer variable (took some time to figure out the lifetime stuff)
impl <'ctx, 'isrt, 'a> IntVar<'ctx, 'isrt, 'a> {
    pub fn new(ctx: &'ctx Context, isrt: &'isrt IntSort<'ctx>, name: &'a str) -> IntVar<'ctx, 'isrt, 'a> {
        IntVar {
            ctx,
            isrt,
            name,
            r: unsafe {
                let int_sort = isrt.r;
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.context, str_name.as_ptr());
                let int_var = Z3_mk_const(ctx.context, sym_name, int_sort);
                int_var
            }
        }
    }
}

impl <'ctx, 'bsrt, 'a> BoolVar<'ctx, 'bsrt, 'a> {
    pub fn new(ctx: &'ctx Context, bsrt: &'bsrt BoolSort<'ctx>, name: &'a str) -> BoolVar<'ctx, 'bsrt, 'a> {
        BoolVar {
            ctx,
            bsrt,
            name,
            r: unsafe {
                let bool_sort = bsrt.r;
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.context, str_name.as_ptr());
                let bool_var = Z3_mk_const(ctx.context, sym_name, bool_sort);
                bool_var
            }
        }
    }
}

impl <'ctx, 'rsrt, 'a> RealVar<'ctx, 'rsrt, 'a> {
    pub fn new(ctx: &'ctx Context, rsrt: &'rsrt RealSort<'ctx>, name: &'a str) -> RealVar<'ctx, 'rsrt, 'a> {
        RealVar {
            ctx,
            rsrt,
            name,
            r: unsafe {
                let real_sort = rsrt.r;
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.context, str_name.as_ptr());
                let real_var = Z3_mk_const(ctx.context, sym_name, real_sort);
                real_var
            }
        }
    }
}

#[test]
fn test_new_bool_var(){
    unsafe {
        let conf = Config::new();
        let ctx = Context::new(&conf);
        let sort = BoolSort::new(&ctx);
        let x = BoolVar::new(&ctx, &sort, "x");
        let string = Z3_ast_to_string(ctx.context, x.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
        let what = Z3_get_sort(ctx.context, x.r);
        let string2 = Z3_sort_to_string(ctx.context, what);
        println!("{:?}", CStr::from_ptr(string2).to_str().unwrap());
    }
}

#[test]
fn test_new_int_var(){
    unsafe {
        let conf = Config::new();
        let ctx = Context::new(&conf);
        let sort = IntSort::new(&ctx);
        let x = IntVar::new(&ctx, &sort, "x");
        let string = Z3_ast_to_string(ctx.context, x.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
        let what = Z3_get_sort(ctx.context, x.r);
        let string2 = Z3_sort_to_string(ctx.context, what);
        println!("{:?}", CStr::from_ptr(string2).to_str().unwrap());
    }
}

#[test]
fn test_new_real_var(){
    unsafe {
        let conf = Config::new();
        let ctx = Context::new(&conf);
        let sort = RealSort::new(&ctx);
        let x = RealVar::new(&ctx, &sort, "x");
        let string = Z3_ast_to_string(ctx.context, x.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
        let what = Z3_get_sort(ctx.context, x.r);
        let string2 = Z3_sort_to_string(ctx.context, what);
        println!("{:?}", CStr::from_ptr(string2).to_str().unwrap());
    }
}
