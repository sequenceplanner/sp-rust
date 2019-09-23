//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;
use super::*;

pub struct EQ<'ctx> {
    pub ctx: &'ctx Context,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub rel: Z3_ast
}

impl <'ctx> EQ<'ctx> {
    pub fn new(ctx: &'ctx Context, left: Z3_ast, right: Z3_ast) -> EQ {
        EQ {
            ctx,
            left,
            right,
            rel: unsafe {
                let _eq = Z3_mk_eq(ctx.context, left, right);
                _eq
            }
        }
    }
}


#[test]
fn test_new_eq(){
    unsafe {
        let _conf = Config::new();
        let _ctx = Context::new(&_conf);
        let _var = IntVar::new(&_ctx, "x");
        let _val = Int::new(&_ctx, 7);
        let _eq = EQ::new(&_ctx, _var.var, _val.int);

        let _solv = Solver::new(&_ctx);
        let _assert = Z3_solver_assert(_ctx.context, _solv.solver, _eq.rel);
        let solv_str = Z3_solver_to_string(_ctx.context, _solv.solver);
        println!("{}", CStr::from_ptr(solv_str).to_str().unwrap());
        
        Z3_solver_check(_ctx.context, _solv.solver);

        let solv_check_str = Z3_solver_to_string(_ctx.context, _solv.solver);
        println!("{}", CStr::from_ptr(solv_check_str).to_str().unwrap());

        let _sgm = Z3_solver_get_model(_ctx.context, _solv.solver);
        let _msgm = Z3_model_to_string(_ctx.context, _sgm);
        println!("{}", CStr::from_ptr(_msgm).to_str().unwrap());
    }
}
