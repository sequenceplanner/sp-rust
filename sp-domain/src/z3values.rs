//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct Int<'ctx> {
    pub ctx: &'ctx Context,
    pub int: Z3_ast
}

impl <'ctx> Int<'ctx> {
    pub fn new(ctx: &'ctx Context, val: i32) -> Int{
        Int {
            ctx,
            int: unsafe {
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
        let _val = Int::new(&_ctx, 7);
        let _str_var = Z3_ast_to_string(_ctx.context, _val.int);
        println!("{:?}", CStr::from_ptr(_str_var).to_str().unwrap());
    }
}
