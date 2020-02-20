//! Z3 config

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::sync::Mutex;

pub struct ConfigZ3 {
    pub r: Z3_config
}

pub struct SetParamZ3<'cfg, 'p, 'v> {
    pub cfg: &'cfg ConfigZ3,
    pub param: &'p str,
    pub value: &'v str,
}

impl ConfigZ3 {
    /// Create a configuration object for the Z3 context object.
    /// 
    /// Configurations are created in order to assign parameters prior to creating
    /// contexts for Z3 interaction. See `SetParamZ3::new` or `set_param_z3!` for parameter setting.
    ///
    /// NOTE: See macro! `cfg_z3!`
    pub fn new() -> ConfigZ3 {
        ConfigZ3 {
            r: unsafe {
                Z3_mk_config()
            }
        }
    }
}

impl<'cfg, 'p, 'v> SetParamZ3<'cfg, 'p, 'v> {
    /// Set a configuration parameter.
    ///
    /// The following parameters can be set:
    ///
    /// - `proof`  (Boolean)           Enable proof generation
    /// - `debug_ref_count` (Boolean)  Enable debug support for `Z3_ast` reference counting
    /// - `trace`  (Boolean)           Tracing support for VCC
    /// - `trace_file_name` (String)   Trace out file for VCC traces
    /// - `timeout` (unsigned)         default timeout (in milliseconds) used for solvers
    /// - `well_sorted_check`          type checker
    /// - `auto_config`                use heuristics to automatically select solver and configure it
    /// - `model`                      model generation for solvers, this parameter can be overwritten when creating a solver
    /// - `model_validate`             validate models produced by solvers
    /// - `unsat_core`                 unsat-core generation for solvers, this parameter can be overwritten when creating a solver
    /// - `dot_proof_file`
    /// - `dump_models`
    /// - `model_compress`
    /// - `rlimit`
    /// - `smtlib2_compliant`
    /// - `stats`
    /// - `type_check`
    /// 
    /// For example, if the user wishes to use proof
    /// generation, then call:
    ///
    /// `SetParamZ3::new(&ctx, "proof", "true")`
    /// 
    /// NOTE: See macro! `set_param_z3!`
    pub fn new(cfg: &'cfg ConfigZ3, param: &'p str, value: &'v str) -> () {
        let str_param = CString::new(param).unwrap();
        let str_value = CString::new(value).unwrap();
        unsafe {
            Z3_set_param_value(cfg.r, str_param.as_ptr(), str_value.as_ptr());
        }
    }
}

unsafe impl Sync for ConfigZ3 {}

impl Default for ConfigZ3 {
    /// Create a default configuration object for the Z3 context object.
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for ConfigZ3 {
    /// Delete the given configuration object.
    fn drop(&mut self) {
        unsafe { 
            Z3_del_config(self.r)
        }
    }
}

/// create a configuration object for the Z3 context object
#[macro_export]
macro_rules! cfg_z3 {
    () => {
        ConfigZ3::new()
    }
}

/// set a configuration parameter
///
/// The following parameters can be set:
///
/// - `proof`  (Boolean)           Enable proof generation
/// - `debug_ref_count` (Boolean)  Enable debug support for `Z3_ast` reference counting
/// - `trace`  (Boolean)           Tracing support for VCC
/// - `trace_file_name` (String)   Trace out file for VCC traces
/// - `timeout` (unsigned)         default timeout (in milliseconds) used for solvers
/// - `well_sorted_check`          type checker
/// - `auto_config`                use heuristics to automatically select solver and configure it
/// - `model`                      model generation for solvers, this parameter can be overwritten when creating a solver
/// - `model_validate`             validate models produced by solvers
/// - `unsat_core`                 unsat-core generation for solvers, this parameter can be overwritten when creating a solver
/// - `dot_proof_file`
/// - `dump_models`
/// - `model_compress`
/// - `rlimit`
/// - `smtlib2_compliant`
/// - `stats`
/// - `type_check`
/// 
/// For example, if the user wishes to use proof
/// generation, then call:
///
/// `set_param_z3!(&cfg, "proof", "true")`
#[macro_export]
macro_rules! set_param_z3 {
    ($a:expr, $b:expr, $c:expr) => {
        SetParamZ3::new($a, $b, $c)
    }
}

#[test]
fn test_new_cfg(){
    ConfigZ3::new();
}

#[test]
fn test_default_cfg(){
    ConfigZ3::default();
}

#[test]
fn test_param_set(){
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true")
}

#[test]
fn test_cfg_macro_1(){
    cfg_z3!();
}

#[test]
fn test_macro_param_set(){
    let cfg = cfg_z3!();
    set_param_z3!(&cfg, "model_compress", "true");
}