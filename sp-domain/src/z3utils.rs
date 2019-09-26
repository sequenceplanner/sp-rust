//! Z3 utilities for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct GetSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub s: String,
    pub r: Z3_sort
}

pub struct GetSolvStringZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub s: String
}

pub struct GetOptStringZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub s: String
}

pub struct GetSolvModelZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub s: String,
    pub r: Z3_model
}

pub struct GetOptModelZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub s: String,
    pub r: Z3_model
}

impl<'ctx> GetSortZ3<'ctx> {
    /// Return the sort of an AST node.
    /// 
    /// The AST node must be a constant, application, numeral, bound variable, or quantifier. 
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> GetSortZ3 {
        let z3 = unsafe {
            Z3_get_sort(ctx.r, arg)
        };
        GetSortZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_sort_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            arg
        }        
    }
}

impl<'ctx, 'slv> GetSolvModelZ3<'ctx, 'slv> {
    /// Retrieve the model for the last [`SolvCheckZ3::new`]
    ///
    /// The error handler is invoked if a model is not available because
    /// the commands above were not invoked for the given solver, or if the result was `Z3_L_FALSE`.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> GetSolvModelZ3<'ctx, 'slv> {
        let z3 = unsafe {
            Z3_solver_get_model(ctx.r, slv.r)
        };
        GetSolvModelZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_model_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            slv
        }        
    }
}

impl<'ctx, 'slv> GetSolvStringZ3<'ctx, 'slv> {
    /// Get the solver context as a string.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> GetSolvStringZ3<'ctx, 'slv> {
        GetSolvStringZ3 {
            ctx,
            s: unsafe {
                CStr::from_ptr(Z3_solver_to_string(ctx.r, slv.r)).to_str().unwrap().to_owned()
            },
            slv
        }        
    }
}

impl<'ctx, 'opt> GetOptModelZ3<'ctx, 'opt> {
    /// Retrieve the model for the last [`OptCheckZ3::new`]
    ///
    /// The error handler is invoked if a model is not available because
    /// the commands above were not invoked for the given optimization
    /// solver, or if the result was `Z3_L_FALSE`.
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>) -> GetOptModelZ3<'ctx, 'opt> {
        let z3 = unsafe {
            Z3_optimize_get_model(ctx.r, opt.r)
        };
        GetOptModelZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_model_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            opt
        }        
    }
}

impl<'ctx, 'opt> GetOptStringZ3<'ctx, 'opt> {
    /// Get the optimize context as a string.
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>) -> GetOptStringZ3<'ctx, 'opt> {
        GetOptStringZ3 {
            ctx,
            s: unsafe {
                CStr::from_ptr(Z3_optimize_to_string(ctx.r, opt.r)).to_str().unwrap().to_owned()
            },
            opt
        }        
    }
}