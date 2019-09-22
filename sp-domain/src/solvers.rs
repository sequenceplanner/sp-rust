//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;

pub struct Config {
    pub config: Z3_config
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

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Config {
    fn drop(&mut self) {
        unsafe { 
            Z3_del_config(self.config)
        }
    }
}

#[test]
fn test_cfg(){
    let _conf = Config::new();
}

#[test]
fn test_default_cfg(){
    let _c = Config::default();
}

///TODO: write a config drop test
// #[test]
// fn test_drop_cfg(){
    // }


pub struct Context {
    pub context: Z3_context
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

impl Default for Context {
    fn default() -> Self {
        Self::new(&Config::default())
    }
}


impl Drop for Context {
    fn drop(&mut self) {
        unsafe { 
            Z3_del_context(self.context)
        }
    }
}

#[test]
fn test_ctx(){
    let _conf = Config::new();
    let _ctx = Context::new(&_conf);
}

#[test]
fn test_default_ctx(){
    let _c = Context::default();
}

///TODO: write a context drop test
// #[test]
// fn test_drop_ctx(){
    // }


pub struct Solver<'ctx> {
    pub ctx: &'ctx Context,
    pub solver: Z3_solver,
}

/// Create a solver in the given context. This solver is a "combined solver" 
/// (see combined_solver module) that internally uses a non-incremental (solver1) 
/// and an incremental solver (solver2). This combined solver changes its 
/// behaviour based on how it is used and how its parameters are set.
impl <'ctx> Solver<'ctx> {
    pub fn new(ctx: &Context) -> Solver {
        Solver {
            ctx,
            solver: unsafe {
                let _solv = Z3_mk_solver(ctx.context);
                Z3_solver_inc_ref(ctx.context, _solv);
                _solv
            }
        }
    }
}

/// TODO: Figure out this lifetime to have a public trait Default for Solver.
// impl <'ctx> Default for Solver<'ctx> {
    // fn default() -> Self {
        // Self::new(&Context::default())
    // }
// }

impl <'ctx> Drop for Solver<'ctx> {
    fn drop(&mut self) {
        unsafe { 
            Z3_solver_dec_ref(self.ctx.context, self.solver)
        }
    }
}

/// Run test with -- --nocapture to see prints.
#[test]
fn test_solver(){
    let mut _conf = Config::new();
    let _ctx = Context::new(&_conf);
    let _solv = Solver::new(&_ctx);
    unsafe{
        let solv_str = Z3_solver_to_string(_ctx.context, _solv.solver);
        println!("Should print empty string, no assertions yet.");
        println!("{:?}", CStr::from_ptr(solv_str).to_str().unwrap());
    }
}

///TODO: write a solver default test
// #[test]
// fn test_default_solver(){
    // }

///TODO: write a solver drop test
// #[test]
// fn test_drop_solver(){
    // }


/// TODO: add Sort struct and include it in the generalized var struct
pub struct IntVar<'ctx, 'a> {
    pub ctx: &'ctx Context,
    pub name: &'a str,
    pub var: Z3_ast,
    // pub sort: Z3_sort,
}

///Create an integer variable (took some time to figure out the lifetime stuff)
impl <'ctx, 'a> IntVar<'ctx, 'a> {
    pub fn new(ctx: &'ctx Context, name: &'a str) -> IntVar<'ctx, 'a> {
        IntVar {
            ctx,
            name,
            var: unsafe {
                let int_sort = Z3_mk_int_sort(ctx.context);
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.context, str_name.as_ptr());
                let int_var = Z3_mk_const(ctx.context, sym_name, int_sort);
                int_var
            }
        }
    }
}

///TODO: write a default trait for IntVar
// impl <'ctx, 'a> Default for IntVar<'ctx, 'a> {
    // fn default() -> Self {
        // Self::new()
    // }
// }

#[test]
fn test_new_int_var(){
    unsafe {
        let _conf = Config::new();
        let _ctx = Context::new(&_conf);
        let _var = IntVar::new(&_ctx, "x");
        let _str_var = Z3_ast_to_string(_ctx.context, _var.var);
        println!("{:?}", CStr::from_ptr(_str_var).to_str().unwrap());
    }
}

//TODO: write a intvar default test
// #[test]
// fn test_default_intvar(){
    // }

//TODO: write a intvar drop test
// #[test]
// fn test_drop_solver(){
    // }

// references from the C api examples...

// Z3_ast mk_var(Z3_context ctx, const char * name, Z3_sort ty)
// {
//     Z3_symbol   s  = Z3_mk_string_symbol(ctx, name);
//     return Z3_mk_const(ctx, s, ty);
// }
// // 
// /**
//    \brief Create a boolean variable using the given name.
// */
// Z3_ast mk_bool_var(Z3_context ctx, const char * name)
// {
//     Z3_sort ty = Z3_mk_bool_sort(ctx);
//     return mk_var(ctx, name, ty);
// }
// 
// impl Var {
//     pub fn new(ctx: &Context, sort: &Sort, name: &str) -> Var {
//         Var {
//             sort: 
//         }
//     }
// }

// Z3_solver mk_solver(Z3_context ctx)
// {
//   Z3_solver s = Z3_mk_solver(ctx);
//   Z3_solver_inc_ref(ctx, s);
//   return s;
// }
// 
// void del_solver(Z3_context ctx, Z3_solver s)
// {
//   Z3_solver_dec_ref(ctx, s);
// }
