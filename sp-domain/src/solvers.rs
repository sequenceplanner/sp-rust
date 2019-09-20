//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;

pub struct Config {
    pub config: Z3_config
}

pub struct Context {
    pub context: Z3_context
}

/// TODO: Figure out this lifetime to have a public trait Default for Solver.
pub struct Solver<'ctx> {
    pub solver: Z3_solver,
    ctx: &'ctx Context
}

pub struct Optimize {
    pub optimize: Z3_optimize
}

pub struct Model {
    pub model: Z3_model
}

/// Create a configuration object for the Z3 context object.
/// Currently default only, without parameter assigning.
/// 
/// TODO: Since configurations are created in order to assign parameters prior to creating contexts for Z3 interaction, add posibility to assign parameters.
impl Config {
    pub fn new() -> Config {
        Config {
            config: unsafe {
                let conf = Z3_mk_config();
                conf
            }
        }
    }
}

/// Create a context using the given configuration.
impl Context {
    pub fn new(cfg: &Config) -> Context {
        Context {
            context: unsafe {
                let ctx = Z3_mk_context(cfg.config);
                ctx
            }
        }
    }
}

/// Create a solver in the given context. This solver is a "combined solver" 
/// (see combined_solver module) that internally uses a non-incremental (solver1) 
/// and an incremental solver (solver2). This combined solver changes its 
/// behaviour based on how it is used and how its parameters are set.
/// /// TODO: Figure out this lifetime to have a public trait Default for Solver.
impl <'ctx> Solver<'ctx> {
    pub fn new(ctx: &Context) -> Solver {
        Solver {
            ctx,
            solver: unsafe {
                let solv = Z3_mk_solver(ctx.context);
                solv
            }
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for Context {
    fn default() -> Self {
        Self::new(&Config::default())
    }
}

/// TODO: Figure out this lifetime to have a public trait Default for Solver.
// impl <'ctx> Default for Solver<'ctx> {
    // fn default() -> Self {
        // Self::new(ctx.context)
    // }
// }

impl Drop for Config {
    fn drop(&mut self) {
        unsafe { 
            Z3_del_config(self.config)
        }
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        unsafe { 
            Z3_del_context(self.context)
        }
    }
}

impl <'ctx> Drop for Solver<'ctx> {
    fn drop(&mut self) {
        unsafe { 
            Z3_solver_dec_ref(self.ctx.context, self.solver)
        }
    }
}

#[test]
fn test_cfg(){
    let _conf = Config::new();
}

#[test]
fn test_ctx(){
    let _conf = Config::new();
    let _ctx = Context::new(&_conf);
}

/// Run test with -- --nocapture to see prints.
#[test]
fn test_solver(){
    let _conf = Config::new();
    let mut _ctx = Context::new(&_conf);
    let _solv = Solver::new(&_ctx);
    unsafe{
        let solv_str = Z3_solver_to_string(_ctx.context, _solv.solver);
        println!("Should print empty quote, nothing is defined yet.");
        println!("{:?}", CStr::from_ptr(solv_str).to_str().unwrap());
    }
}

#[test]
fn test_default_cfg(){
    let _c = Config::default();
}

#[test]
fn test_default_ctx(){
    let _c = Config::default();
}