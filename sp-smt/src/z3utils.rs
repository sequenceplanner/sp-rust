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

pub struct GetTseitinCnfZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: String
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

impl<'ctx> GetTseitinCnfZ3<'ctx> {
    /// Get cnf
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> String {
        let z3 = unsafe {
            let goal = Z3_mk_goal(ctx.r, false, false, false);
            for formula in args {
                Z3_goal_assert(ctx.r, goal, formula);
                Z3_goal_inc_ref(ctx.r, goal);
            }
            let tactic = Z3_mk_tactic(ctx.r, CString::new("tseitin-cnf".to_string()).unwrap().as_ptr());
            Z3_tactic_inc_ref(ctx.r, tactic);
            Z3_tactic_apply(ctx.r, tactic, goal);            
            CStr::from_ptr(Z3_goal_to_string(ctx.r, goal)).to_str().unwrap().to_owned()
        };
        GetTseitinCnfZ3 {ctx, r: z3}.r
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

#[test]
fn test_tseitin(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = IntVarZ3::new(&ctx, &intsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 1);
    let int2 = IntZ3::new(&ctx, &intsort, 10);

    let asrt1 = GEZ3::new(&ctx, x, int1);
    let asrt2 = LEZ3::new(&ctx, x, int2);

    let asrt3 = GEZ3::new(&ctx, y, int1);
    let asrt4 = LEZ3::new(&ctx, y, int2);

    let asrt5 = DISTINCTZ3::new(&ctx, vec!(x, y));

    let tseit = GetTseitinCnfZ3::new(&ctx, vec!(asrt1, asrt2, asrt3, asrt4, asrt5));

    assert_eq!("(goal
  (>= x 1)
  (<= x 10)
  (>= y 1)
  (<= y 10)
  (distinct x y))", tseit);
}