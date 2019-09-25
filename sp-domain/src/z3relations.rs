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
    pub r: Z3_ast
}

impl <'ctx> EQ<'ctx> {
    pub fn new(ctx: &'ctx Context, left: Z3_ast, right: Z3_ast) -> EQ {
        EQ {
            ctx,
            left,
            right,
            r: unsafe {
                let _eq = Z3_mk_eq(ctx.context, left, right);
                _eq
            }
        }
    }
}


#[test]
fn test_new_eq(){
    unsafe {
        let conf = Config::new();
        let ctx = Context::new(&conf);
        let sort = IntSort::new(&ctx);
        let x = IntVar::new(&ctx, &sort, "x");
        let seven = Int::new(&ctx, &sort, 7);
        let eq = EQ::new(&ctx, x.r, seven.r);

        let string = Z3_ast_to_string(ctx.context, eq.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());

        // let _solv = Solver::new(&_ctx);
        // let _assert = Z3_solver_assert(_ctx.context, _solv.solver, _eq.rel);
        // let solv_str = Z3_solver_to_string(_ctx.context, _solv.solver);
        // println!("{}", CStr::from_ptr(solv_str).to_str().unwrap());
        
        // Z3_solver_check(_ctx.context, _solv.solver);

        // let solv_check_str = Z3_solver_to_string(_ctx.context, _solv.solver);
        // println!("{}", CStr::from_ptr(solv_check_str).to_str().unwrap());

        // let _sgm = Z3_solver_get_model(_ctx.context, _solv.solver);
        // let _msgm = Z3_model_to_string(_ctx.context, _sgm);
        // println!("{}", CStr::from_ptr(_msgm).to_str().unwrap());
    }
}
