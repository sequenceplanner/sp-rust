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

pub struct ModelGetNumConstsZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub r: ::std::os::raw::c_uint
}

pub struct ModelGetConstDeclZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub index : ::std::os::raw::c_uint,
    pub r: Z3_func_decl
}

pub struct GetDeclNameZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub decl : Z3_func_decl,
    pub r: Z3_symbol
}

pub struct ModelGetConstInterpZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub decl : Z3_func_decl,
    pub r: Z3_ast
}

pub struct GetSymbolStringZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub symbol: Z3_symbol,
    pub r: Z3_string
}

impl<'ctx> AstToStringZ3<'ctx> {
    /// AST to readable string
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
    /// Model to readable string
    /// 
    /// NOTE: See macro! `model_to_string_z3!`
    pub fn new(ctx: &'ctx ContextZ3, what: Z3_model) -> String {
        let z3 = unsafe {
            CStr::from_ptr(Z3_model_to_string(ctx.r, what)).to_str().unwrap().to_owned()
        };
        ModelToStringZ3 {ctx, what, r: z3}.r
    }
}

impl<'ctx> ModelGetNumConstsZ3<'ctx> {
    /// Get the number of constants in a model
    /// 
    /// NOTE: See macro! `model_get_num_consts_z3!`
    pub fn new(ctx: &'ctx ContextZ3, model: Z3_model) -> ::std::os::raw::c_uint {
        let z3 = unsafe {
            Z3_model_get_num_consts(ctx.r, model)
        };
        ModelGetNumConstsZ3 {ctx, model, r: z3}.r
    }
}

impl<'ctx> ModelGetConstDeclZ3<'ctx> {
    /// Get declaration of the i-th const in a model
    /// 
    /// NOTE: See macro! `model_get_const_decl_z3!`
    pub fn new(ctx: &'ctx ContextZ3, model: Z3_model, index: ::std::os::raw::c_uint) -> Z3_func_decl {
        let z3 = unsafe {
            Z3_model_get_const_decl(ctx.r, model, index)
        };
        ModelGetConstDeclZ3 {ctx, model, index, r: z3}.r
    }
}

impl<'ctx> GetDeclNameZ3<'ctx> {
    /// Get the name (symbol) of a declaration
    /// 
    /// NOTE: See macro! `get_decl_name_z3!`
    pub fn new(ctx: &'ctx ContextZ3, model: Z3_model, decl: Z3_func_decl) -> Z3_symbol {
        let z3 = unsafe {
            Z3_get_decl_name(ctx.r, decl)
        };
        GetDeclNameZ3 {ctx, model, decl, r: z3}.r
    }
}

impl<'ctx> ModelGetConstInterpZ3<'ctx> {
    /// Get interpretation of of a declaration
    /// 
    /// NOTE: See macro! `model_get_const_interp_z3!`
    pub fn new(ctx: &'ctx ContextZ3, model: Z3_model, decl: Z3_func_decl) -> Z3_ast {
        let z3 = unsafe {
            Z3_model_get_const_interp(ctx.r, model, decl)
        };
        ModelGetConstInterpZ3 {ctx, model, decl, r: z3}.r
    }
}

impl<'ctx> GetSymbolStringZ3<'ctx> {
    /// Symbol to Z3 string
    pub fn new(ctx: &'ctx ContextZ3, symbol: Z3_symbol) -> Z3_string {
        let z3 = unsafe {
            Z3_get_symbol_string(ctx.r, symbol)
        };
        GetSymbolStringZ3 {ctx, symbol, r: z3}.r
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