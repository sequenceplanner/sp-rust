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
    /// contexts for Z3 interaction. For example, if the user wishes to use proof
    /// generation, then call:
    ///
    /// `SetParamZ3::new(&cfg, "proof", "true")`
    ///
    /// Or, use a macro! to do the same thing:
    /// 
    /// Default configuration: `cspz3!("proof", "true")`
    /// 
    /// Specific configuration: `cspz3!(&cfg, "proof", "true")`
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
    /// A public reference to a default configuration for simplicity: `&CFG`
    pub fn new() -> ConfigZ3 {
        ConfigZ3 {
            r: unsafe {
                // Z3_MUTEX.lock().unwrap();
                let conf = Z3_mk_config();
                conf
            }
        }
    }
}

impl<'cfg, 'p, 'v> SetParamZ3<'cfg, 'p, 'v> {
    /// Set a configuration parameter.
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
    /// For example, if the users wishes to use proof
    /// generation, then call:
    ///
    /// `SetParamZ3::new(&ctx, "proof", "true")`
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

// Taken from the original library, credits: Bruce Mitchener, Graydon Hoare 
// Z3 appears to be only mostly-threadsafe, a few initializers
// and such race; so we mutex-guard all access to the library.
lazy_static! {
    pub static ref Z3_MUTEX: Mutex<()> = Mutex::new(());
}

// // Unsafe sync takes toll... avoid this. 
// lazy_static! {
//     /// A public reference to a default configuration.
//     /// Avoids passing the config around.
//     pub static ref CFG: ConfigZ3 = {
//         // let guard = Z3_MUTEX.lock().unwrap();
//         ConfigZ3::new()
//     }
// }

/// Create a configuration object for the Z3 context object.
#[macro_export]
macro_rules! cfgz3 {
    () => {
        ConfigZ3::new()
    }
}

/// Set a configuration parameter.
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

// #[test]
// fn test_static_cfg(){
//     &CFG;
// }