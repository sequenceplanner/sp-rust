//! Z3 sorts

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct BoolSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

pub struct IntSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

pub struct RealSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

pub struct StringSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_sort
}

impl <'ctx> BoolSortZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3) -> BoolSortZ3 {
        BoolSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_bool_sort(ctx.r);
                sort
            }
        }
    }
}

impl <'ctx> IntSortZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3) -> IntSortZ3 {
        IntSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_int_sort(ctx.r);
                sort
            }
        }
    }
}

impl <'ctx> RealSortZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3) -> RealSortZ3 {
        RealSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_real_sort(ctx.r);
                sort
            }
        }
    }
}

impl <'ctx> StringSortZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3) -> StringSortZ3 {
        StringSortZ3 {
            ctx,
            r: unsafe {
                let sort = Z3_mk_string_sort(ctx.r);
                sort
            }
        }
    }
}

// #[test]
// fn test_int_sort(){
//     unsafe {
//         let _conf = Config::new();
//         let _ctx = Context::new(&_conf);
//         let _sort = IntSort::new(&_ctx);
//         let _var = IntVar::new(&_ctx, &_sort, "x");
//         let _val = Int::new(&_ctx, &_sort, 7);
//         let _eq = EQ::new(&_ctx, _var.var, _val.val);

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
