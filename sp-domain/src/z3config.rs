//! Z3 config

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;

pub struct ConfigZ3 {
    pub r: Z3_config
}

impl ConfigZ3 {
    /// Create a configuration object for the Z3 context object.
    ///
    /// Configurations are created in order to assign parameters prior to creating
    /// contexts for Z3 interaction. For example, if the users wishes to use proof
    /// generation, then call:
    ///
    /// `Z3_set_param_value(cfg, "proof", "true")`
    ///
    /// NOTE: In previous versions of Z3, the `Z3_config` was used to store
    /// global and module configurations. Now, we should use `Z3_global_param_set`.
    ///
    /// The following parameters can be set:
    ///
    /// - proof  (Boolean)           Enable proof generation
    /// - debug_ref_count (Boolean)  Enable debug support for `Z3_ast` reference counting
    /// - trace  (Boolean)           Tracing support for VCC
    /// - trace_file_name (String)   Trace out file for VCC traces
    /// - timeout (unsigned)         default timeout (in milliseconds) used for solvers
    /// - well_sorted_check          type checker
    /// - auto_config                use heuristics to automatically select solver and configure it
    /// - model                      model generation for solvers, this parameter can be overwritten when creating a solver
    /// - model_validate             validate models produced by solvers
    /// - unsat_core                 unsat-core generation for solvers, this parameter can be overwritten when creating a solver
    ///
    /// # See also:
    ///
    /// - [`Z3_set_param_value`](fn.Z3_set_param_value.html)
    /// - [`Z3_del_config`](fn.Z3_del_config.html)
    /// 
    /// TODO: Since configurations are created in order to assign parameters prior to 
    /// creating contexts for Z3 interaction, add posibility to assign parameters.
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
    /// Create a default configuration object for the Z3 context object.
    ///
    /// # See also:
    ///
    /// - [`Z3_mk_config`](fn.Z3_mk_config.html)
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for ConfigZ3 {
    /// Delete the given configuration object.
    ///
    /// # See also:
    ///
    /// - [`Z3_mk_config`](fn.Z3_mk_config.html)
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