//! Z3 utilities for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct GetSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub s: String,
    pub r: Z3_sort
}

pub struct GetSolvStringZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub s: String
}

pub struct GetOptStringZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub s: String
}

pub struct GetSolvModelZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub s: String,
    pub r: Z3_model
}

pub struct GetOptModelZ3<'ctx, 'opt> {
    pub ctx: &'ctx ContextZ3,
    pub opt: &'opt OptimizerZ3<'ctx>,
    pub s: String,
    pub r: Z3_model
}

pub struct GetSolvParamDescrZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub s: String
}

pub struct GetSolvProofZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_ast,
    pub s: String
}

pub struct GetSolvSolutionZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_ast,
    pub s: String
}

pub struct GetSolvAllSolutionsZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_ast,
    pub s: Vec<String>
}

pub struct GetSolvUnsatCoreZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_ast_vector,
    pub s: String
}

pub struct SetParamZ3<'cfg, 'p, 'v> {
    pub cfg: &'cfg ConfigZ3,
    pub param: &'p str,
    pub value: &'v str,
}

impl<'ctx> GetSortZ3<'ctx> {
    /// Return the sort of an AST node.
    /// 
    /// The AST node must be a constant, application, numeral, bound variable, or quantifier. 
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> GetSortZ3 {
        let z3 = unsafe {
            Z3_get_sort(ctx.r, arg)
        };
        GetSortZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_sort_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            arg
        }        
    }
}

impl<'ctx, 'slv> GetSolvModelZ3<'ctx, 'slv> {
    /// Retrieve the model for the last [`SolvCheckZ3::new`]
    ///
    /// The error handler is invoked if a model is not available because
    /// the commands above were not invoked for the given solver, or if the result was `Z3_L_FALSE`.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> GetSolvModelZ3<'ctx, 'slv> {
        let z3 = unsafe {
            Z3_solver_get_model(ctx.r, slv.r)
        };
        GetSolvModelZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_model_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            slv
        }        
    }
}

impl<'ctx, 'slv> GetSolvStringZ3<'ctx, 'slv> {
    /// Get the solver context as a string.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> GetSolvStringZ3<'ctx, 'slv> {
        GetSolvStringZ3 {
            ctx,
            s: unsafe {
                CStr::from_ptr(Z3_solver_to_string(ctx.r, slv.r)).to_str().unwrap().to_owned()
            },
            slv
        }        
    }
}

impl<'ctx, 'opt> GetOptModelZ3<'ctx, 'opt> {
    /// Retrieve the model for the last [`OptCheckZ3::new`]
    ///
    /// The error handler is invoked if a model is not available because
    /// the commands above were not invoked for the given optimization
    /// solver, or if the result was `Z3_L_FALSE`.
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>) -> GetOptModelZ3<'ctx, 'opt> {
        let z3 = unsafe {
            Z3_optimize_get_model(ctx.r, opt.r)
        };
        GetOptModelZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_model_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            opt
        }        
    }
}

impl<'ctx, 'opt> GetOptStringZ3<'ctx, 'opt> {
    /// Get the optimize context as a string.
    pub fn new(ctx: &'ctx ContextZ3, opt: &'opt OptimizerZ3<'ctx>) -> GetOptStringZ3<'ctx, 'opt> {
        GetOptStringZ3 {
            ctx,
            s: unsafe {
                CStr::from_ptr(Z3_optimize_to_string(ctx.r, opt.r)).to_str().unwrap().to_owned()
            },
            opt
        }        
    }
}

impl <'ctx, 'slv> GetSolvParamDescrZ3<'ctx, 'slv> {
    /// Return a string that is the parameter description set for the given solver object.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> String {
        unsafe {
            let descr = Z3_solver_get_param_descrs(ctx.r, slv.r);
            let desc_str = Z3_param_descrs_to_string(ctx.r, descr);
            CStr::from_ptr(desc_str).to_str().unwrap().to_owned()
        }
    }
}

impl <'ctx, 'slv> GetSolvProofZ3<'ctx, 'slv> {
    /// Retrieve the proof for the last [`Z3_solver_check`](fn.Z3_solver_check.html) or [`Z3_solver_check_assumptions`](fn.Z3_solver_check_assumptions.html)
    ///
    /// The error handler is invoked if proof generation is not enabled,
    /// or if the commands above were not invoked for the given solver,
    /// or if the result was different from `Z3_L_FALSE`.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> GetSolvProofZ3<'ctx, 'slv> {
        let z3 = unsafe {
            Z3_solver_get_proof(ctx.r, slv.r)
        };
        GetSolvProofZ3 {
            ctx,
            slv,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx, 'slv> GetSolvUnsatCoreZ3<'ctx, 'slv> {
    /// Retrieve the unsat core for the last [`Z3_solver_check_assumptions`](fn.Z3_solver_check_assumptions.html)
    /// The unsat core is a subset of the assumptions `a`.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> GetSolvUnsatCoreZ3<'ctx, 'slv> {
        let z3 = unsafe {
            Z3_solver_get_unsat_core(ctx.r, slv.r)
        };
        GetSolvUnsatCoreZ3 {
            ctx,
            slv,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_vector_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
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

/// Z3 get solver as string 
/// 
/// Macro rule for:
/// ```text
/// z3utils::GetSolvStringZ3::new(&ctx, &slv)
/// ```
/// Using the global context:
/// ```text
/// gssz3!(&slv)
/// ```
/// Using a specific context:
/// ```text
/// gssz3!(&ctx, &slv, a)
/// ```
#[macro_export]
macro_rules! gssz3 {
    ($slv:expr) => {
        GetSolvStringZ3::new(&CTX, $slv).s
    };
    ($ctx:expr, $slv:expr) => {
        GetSolvStringZ3::new($ctx, $slv).s
    }
}

#[test]
fn print_parameter_set_test() {
    let cfg = ConfigZ3::new();
    let ctx = ContextZ3::new(&cfg);
    let solv = SolverZ3::new(&ctx);
    println!("{}", GetSolvParamDescrZ3::new(&ctx, &solv))
}

#[test]
fn set_proof_param_test() {
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true");
    let ctx = ContextZ3::new(&cfg);
    SolverZ3::new(&ctx);
}

#[test]
fn get_proof_test() {
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true");
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let sort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &sort, "x");
    let three = IntZ3::new(&ctx, &sort, 3);
    let two = IntZ3::new(&ctx, &sort, 2);

    SolvAssertZ3::new(&ctx, &slv, ANDZ3::new(&ctx, vec!(GTZ3::new(&ctx, x.r, three.r).r, LTZ3::new(&ctx, x.r, two.r).r)).r);

    SolvCheckZ3::new(&ctx, &slv);
    println!("{}", GetSolvProofZ3::new(&ctx, &slv).s)

}

#[test]
fn get_unsat_core_test() {
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true");
    SetParamZ3::new(&cfg, "unsat_core", "true");
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let sort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &sort, "x");
    let three = IntZ3::new(&ctx, &sort, 3);
    let two = IntZ3::new(&ctx, &sort, 2);
    let one = IntZ3::new(&ctx, &sort, 1);

    SolvAssertAndTrackZ3::new(&ctx, &slv, GTZ3::new(&ctx, x.r, three.r).r, "a1");
    SolvAssertAndTrackZ3::new(&ctx, &slv, LTZ3::new(&ctx, x.r, two.r).r, "a2");
    SolvAssertAndTrackZ3::new(&ctx, &slv, EQZ3::new(&ctx, x.r, one.r).r, "a3");

    SolvCheckZ3::new(&ctx, &slv);
    println!("{}", GetSolvUnsatCoreZ3::new(&ctx, &slv).s);
    println!("{}", GetSolvProofZ3::new(&ctx, &slv).s);
}