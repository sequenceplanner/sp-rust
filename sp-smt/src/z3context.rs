//! Z3 context

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct ContextZ3 {
    pub r: Z3_context
}

pub struct UpdateParamZ3<'ctx, 'p, 'v> {
    pub ctx: &'ctx ContextZ3,
    pub param: &'p str,
    pub value: &'v str,
}

impl ContextZ3 {
    /// Create a context using the given configuration.
    ///
    /// After a context is created, the configuration cannot be changed,
    /// although some parameters can be changed using:
    /// 
    /// `UpdateParamZ3::new`
    /// 
    /// All main interaction with Z3 happens in the context of a `Z3_context`.
    /// 
    /// NOTE: See macro! `ctx_z3!`
    pub fn new(cfg: &ConfigZ3) -> ContextZ3 {
        ContextZ3 {
            r: unsafe {
                let ctx = Z3_mk_context(cfg.r);
                ctx
            }
        }
    }
}

impl <'ctx, 'p, 'v> UpdateParamZ3<'ctx, 'p, 'v> {
    /// Update a value of a context parameter. 
    /// 
    /// NOTE: See macro! `update_param_z3!`
    pub fn new(ctx: &'ctx ContextZ3, param: &'p str, value: &'v str) -> () {
        let str_param = CString::new(param).unwrap();
        let str_value = CString::new(value).unwrap();
        unsafe {
            Z3_update_param_value(ctx.r, str_param.as_ptr(), str_value.as_ptr());
        }
    }
}

impl Default for ContextZ3 {
    /// Create a default logical context using the given configuration..
    fn default() -> Self {
        Self::new(&ConfigZ3::default())
    }
}

impl Drop for ContextZ3 {
    /// Delete the given logical context.
    fn drop(&mut self) {
        unsafe {
            Z3_del_context(self.r)
        }
    }
}

/// create a context using the given configuration
#[macro_export]
macro_rules! ctx_z3 {
    ($a:expr) => {
        ContextZ3::new($a)
    }
}

/// update a value of a context parameter
#[macro_export]
macro_rules! update_param_z3 {
    ($a:expr, $b:expr, $c:expr) => {
        UpdateParamZ3::new($a, $b, $c)
    }
}

#[test]
fn test_ctx(){
    let conf = ConfigZ3::new();
    ContextZ3::new(&conf);
}

#[test]
fn test_default_ctx(){
    ContextZ3::default();
}

#[test]
fn test_update_param(){
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true");
    let ctx = ContextZ3::default();
    UpdateParamZ3::new(&ctx, "proof", "false");
}

#[test]
fn test_ctx_macro(){
    let cfg = cfg_z3!();
    ctx_z3!(&cfg);
}

#[test]
fn test_update_param_macro(){
    let cfg = cfg_z3!();
    set_param_z3!(&cfg, "proof", "true");
    let ctx = ctx_z3!(&cfg);
    update_param_z3!(&ctx, "proof", "false");
}