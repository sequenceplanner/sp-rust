//! Z3 context

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;
use super::*;

pub struct ContextZ3 {
    pub r: Z3_context
}

/// Create a context using the given configuration.
impl ContextZ3 {
    pub fn new(cfg: &ConfigZ3) -> ContextZ3 {
        ContextZ3 {
            r: unsafe {
                let ctx = Z3_mk_context(cfg.r);
                ctx
            }
        }
    }
}

impl Default for ContextZ3 {
    fn default() -> Self {
        Self::new(&ConfigZ3::default())
    }
}

impl Drop for ContextZ3 {
    fn drop(&mut self) {
        unsafe { 
            Z3_del_context(self.r)
        }
    }
}

#[test]
fn test_ctx(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
}

#[test]
fn test_default_ctx(){
    let ctx = ContextZ3::default();
}