//! Z3 config

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;

pub struct ConfigZ3 {
    pub r: Z3_config
}

/// Create a configuration object for the Z3 context object.
/// Currently default only, without parameter assigning.
/// 
/// TODO: Since configurations are created in order to assign parameters prior to 
/// creating contexts for Z3 interaction, add posibility to assign parameters.
impl ConfigZ3 {
    pub fn new() -> ConfigZ3 {
        ConfigZ3 {
            r: unsafe {
                let conf = Z3_mk_config();
                conf
            }
        }
    }
}

impl Default for ConfigZ3 {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for ConfigZ3 {
    fn drop(&mut self) {
        unsafe { 
            Z3_del_config(self.r)
        }
    }
}

#[test]
fn test_new_cfg(){
    let conf = ConfigZ3::new();
}

#[test]
fn test_default_cfg(){
    let conf = ConfigZ3::default();
}

#[test]
fn test_drop_cfg(){
}