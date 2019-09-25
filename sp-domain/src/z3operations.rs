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
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
    pub r: Z3_ast
}

pub struct MODZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg1: Z3_ast,
    pub arg2: Z3_ast,
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
    /// Create an AST node representing `args[0] * ... * args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have int or real sort.
    ///
    /// NOTE: Z3 has limited support for non-linear arithmetic.
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> MULZ3 {
        MULZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let mulz3 = Z3_mk_mul(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                mulz3
            },
            args
        }        
    }
}

impl <'ctx> DIVZ3<'ctx> {
    /// Create an AST node representing `arg1 div arg2`.
    ///
    /// The arguments must either both have int type or both have real type.
    /// If the arguments have int type, then the result type is an int type, otherwise the
    /// the result type is real.
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> DIVZ3 {
        DIVZ3 {
            ctx,
            arg1,
            arg2,
            r: unsafe {
                let divz3 = Z3_mk_div(ctx.r, arg1, arg2);
                divz3
            }
        }
    }
}

impl <'ctx> MODZ3<'ctx> {
    /// Create an AST node representing `arg1 mod arg2`.
    ///
    /// The arguments must have int type.
    pub fn new(ctx: &'ctx ContextZ3, arg1: Z3_ast, arg2: Z3_ast) -> MODZ3 {
        MODZ3 {
            ctx,
            arg1,
            arg2,
            r: unsafe {
                let modz3 = Z3_mk_mod(ctx.r, arg1, arg2);
                modz3
            }
        }
    }
}

impl<'ctx> ADDZ3<'ctx> {
    /// Create an AST node representing `args[0] + ... + args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have int or real sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> ADDZ3 {
        ADDZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let addz3 = Z3_mk_add(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                addz3
            },
            args
        }        
    }
}

impl<'ctx> SUBZ3<'ctx> {
    /// Create an AST node representing `args[0] - ... - args[num_args - 1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have int or real sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> SUBZ3 {
        SUBZ3 {
            ctx,
            r: unsafe {
                let args_slice = &args;
                let subz3 = Z3_mk_sub(ctx.r, args_slice.len() as u32, args_slice.as_ptr());
                subz3
            },
            args
        }        
    }
}
