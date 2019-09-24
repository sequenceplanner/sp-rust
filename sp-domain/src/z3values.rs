//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct Bool<'ctx, 'bsrt> {
    pub ctx: &'ctx Context,
    pub bsrt: &'bsrt BoolSort<'ctx>,
    pub val: Z3_ast
}

pub struct Int<'ctx, 'isrt> {
    pub ctx: &'ctx Context,
    pub isrt: &'isrt IntSort<'ctx>,
    pub val: Z3_ast
}

pub struct Real<'ctx, 'rsrt> {
    pub ctx: &'ctx Context,
    pub rsrt: &'rsrt RealSort<'ctx>,
    pub val: Z3_ast
}

impl <'ctx, 'bsrt> Bool<'ctx, 'bsrt> {
    pub fn new(ctx: &'ctx Context, bsrt: &'bsrt BoolSort<'ctx>, val: bool) -> Bool<'ctx, 'bsrt> {
        Bool {
            ctx,
            bsrt,
            val: unsafe {
                let _bool_sort = bsrt.sort;
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
            val: unsafe {
                let _int_sort = Z3_mk_int_sort(ctx.context); // make a sort impl
                let _int = Z3_mk_int(ctx.context, val, _int_sort);
                _int
            }
        }
    }
}


#[test]
fn test_new_int(){
    unsafe {
        let _conf = Config::new();
        let _ctx = Context::new(&_conf);
        let _sort = IntSort::new(&_ctx);
        let _val = Int::new(&_ctx, &_sort, 7);
        let _str_var = Z3_ast_to_string(_ctx.context, _val.val);
        println!("{:?}", CStr::from_ptr(_str_var).to_str().unwrap());
    }
}
