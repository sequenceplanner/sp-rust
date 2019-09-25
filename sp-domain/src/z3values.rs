//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::f64;
use super::*;

pub struct Bool<'ctx, 'bsrt> {
    pub ctx: &'ctx Context,
    pub bsrt: &'bsrt BoolSort<'ctx>,
    pub r: Z3_ast
}

pub struct Int<'ctx, 'isrt> {
    pub ctx: &'ctx Context,
    pub isrt: &'isrt IntSort<'ctx>,
    pub r: Z3_ast
}

pub struct Real<'ctx, 'ssrt> {
    pub ctx: &'ctx Context,
    pub ssrt: &'ssrt StringSort<'ctx>,
    pub r: Z3_ast
}

pub struct Z3String<'ctx, 'a> {
    pub ctx: &'ctx Context,
    pub string: &'a str,
    pub r: Z3_string
}

impl <'ctx, 'bsrt> Bool<'ctx, 'bsrt> {
    pub fn new(ctx: &'ctx Context, bsrt: &'bsrt BoolSort<'ctx>, val: bool) -> Bool<'ctx, 'bsrt> {
        Bool {
            ctx,
            bsrt,
            r: unsafe {
                let _bool_sort = bsrt.r;
                if val == true {
                    let _bool = Z3_mk_true(ctx.context);
                    _bool
                } else {
                    let _bool = Z3_mk_false(ctx.context);
                    _bool
                }
            }
        }
    }
}

impl <'ctx, 'isrt> Int<'ctx, 'isrt> {
    pub fn new(ctx: &'ctx Context, isrt: &'isrt IntSort<'ctx>, val: i32) -> Int<'ctx, 'isrt> {
        Int {
            ctx,
            isrt,
            r: unsafe {
                let _int = Z3_mk_int(ctx.context, val, isrt.r);
                _int
            }
        }
    }
}

impl <'ctx, 'a> Z3String<'ctx, 'a> {
    pub fn new(ctx: &'ctx Context, string: &'a str) -> Z3String<'ctx, 'a> {
        Z3String {
            ctx,
            string,
            r: unsafe {
                let _str: Z3_string = string.as_ptr() as *const i8;
                _str
            }
        }
    }
}

//Why do we need to make real sort then? aha ok , for
impl <'ctx, 'ssrt> Real<'ctx, 'ssrt> {
    pub fn new(ctx: &'ctx Context, ssrt: &'ssrt StringSort<'ctx>, val: f64) -> Real<'ctx, 'ssrt> {
        Real {
            ctx,
            ssrt,
            r: unsafe {
                let num_string = val.to_string();
                let string = Z3String::new(&ctx, &num_string);
                let _real = Z3_mk_numeral(ctx.context, string.string.as_ptr() as *const i8, ssrt.r);
                _real
            }
        }
    }
}


#[test]
fn test_new_int(){
    unsafe {
        let conf = Config::new();
        let ctx = Context::new(&conf);
        let intsort = IntSort::new(&ctx);
        let seven = Int::new(&ctx, &intsort, 7);
        let string = Z3_ast_to_string(ctx.context, seven.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
    }
}

#[test]
fn test_new_real(){
    unsafe {
        let conf = Config::new();
        let ctx = Context::new(&conf);
        let string_sort = StringSort::new(&ctx);
        let val = Real::new(&ctx, &string_sort,7.9874);
        let string = Z3_ast_to_string(ctx.context, val.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
    }
}
