//! Z3 utilities for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct AstToStringZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub what: Z3_ast,
    pub r: String
}

pub struct ModelToStringZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub what: Z3_model,
    pub r: String
}

impl<'ctx> AstToStringZ3<'ctx> {
    /// Z3 optimizer to readable string
    /// 
    /// NOTE: See macro! `ast_to_string_z3!`
    pub fn new(ctx: &'ctx ContextZ3, what: Z3_ast) -> String {
        let z3 = unsafe {
            CStr::from_ptr(Z3_ast_to_string(ctx.r, what)).to_str().unwrap().to_owned()
        };
        AstToStringZ3 {ctx, what, r: z3}.r
    }
}

impl<'ctx> ModelToStringZ3<'ctx> {
    /// Z3 optimizer to readable string
    /// 
    /// NOTE: See macro! `model_to_string_z3!`
    pub fn new(ctx: &'ctx ContextZ3, what: Z3_model) -> String {
        let z3 = unsafe {
            CStr::from_ptr(Z3_model_to_string(ctx.r, what)).to_str().unwrap().to_owned()
        };
        ModelToStringZ3 {ctx, what, r: z3}.r
    }
}

/// abstract static tree to readable string
#[macro_export]
macro_rules! ast_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        AstToStringZ3::new($ctx, $a)
    }
}

/// model to readable string
#[macro_export]
macro_rules! model_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        ModelToStringZ3::new($ctx, $a)
    }
}