//! Z3 operations for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct MULZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct DIVZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub a: Z3_ast,
    pub b: Z3_ast,
    pub r: Z3_ast
}

pub struct MODZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub a: Z3_ast,
    pub b: Z3_ast,
    pub r: Z3_ast
}

pub struct ADDZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct SUBZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

impl<'ctx> MULZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> MULZ3 {
        MULZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let mul = Z3_mk_mul(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                mul
            },
            args
        }        
    }
}

impl <'ctx> DIVZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, a: Z3_ast, b: Z3_ast) -> DIVZ3 {
        DIVZ3 {
            ctx,
            a,
            b,
            r: unsafe {
                let eq = Z3_mk_div(ctx.r, a, b);
                eq
            }
        }
    }
}

impl <'ctx> MODZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, a: Z3_ast, b: Z3_ast) -> MODZ3 {
        MODZ3 {
            ctx,
            a,
            b,
            r: unsafe {
                let eq = Z3_mk_mod(ctx.r, a, b);
                eq
            }
        }
    }
}

impl<'ctx> ADDZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> ADDZ3 {
        ADDZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let add = Z3_mk_add(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                add
            },
            args
        }        
    }
}

impl<'ctx> SUBZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> SUBZ3 {
        SUBZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let sub = Z3_mk_sub(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                sub
            },
            args
        }        
    }
}

// #[test]
// fn test_new_mul_with_ints() {
//     unsafe{
//         let conf = Config::new();
//         let ctx = Context::new(&conf);
//         let sort = IntSort::new(&ctx);
//         let eight = Int::new(&ctx, &sort, 8);
//         let x = IntVar::new(&ctx, &sort, "x");
//         let eightx = MUL::new(&ctx, vec!(eight.r, x.r));
//         let string = Z3_ast_to_string(ctx.context, eightx.r);
//         println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
//     }
// }

// #[test]
// fn test_new_mul(){
//     unsafe {
//         let _conf = Config::new();
//         let _ctx = Context::new(&_conf);
//         let _sort = IntSort::new(&_ctx);
//         let _var = IntVar::new(&_ctx, &_sort, "x");
//         let _val1 = Int::new(&_ctx, &_sort, 2);
//         let _val = Int::new(&_ctx, &_sort, 8);
//         let _mul = MUL::new(&_ctx, vec!(_val1.val, _var.var));
//         let _add = ADD::new(&_ctx, vec!(_mul.r, _val1.val));
//         let _eq = EQ::new(&_ctx, _add.r, _val.val);

//         let _solv = Solver::new(&_ctx);
//         let _assert = Z3_solver_assert(_ctx.context, _solv.solver, _eq.rel);
//         let solv_str = Z3_solver_to_string(_ctx.context, _solv.solver);
//         println!("{}", CStr::from_ptr(solv_str).to_str().unwrap());
        
//         Z3_solver_check(_ctx.context, _solv.solver);

//         let solv_check_str = Z3_solver_to_string(_ctx.context, _solv.solver);
//         println!("{}", CStr::from_ptr(solv_check_str).to_str().unwrap());

//         let _sgm = Z3_solver_get_model(_ctx.context, _solv.solver);
//         let _msgm = Z3_model_to_string(_ctx.context, _sgm);
//         println!("{}", CStr::from_ptr(_msgm).to_str().unwrap());
//     }
// }
