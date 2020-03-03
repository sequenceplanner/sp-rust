//! Z3 utilities for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct AstToStringZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub what: Z3_ast,
    pub r: String
}

pub struct AstVectorToStringZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub what: Z3_ast_vector,
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

pub struct Z3StringToStringZ3 {
    pub cstr: Z3_string,
    pub r: String
}

pub struct Z3AstVectorToVectorAstZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub ast_vec: Z3_ast_vector,
    pub r: Vec<Z3_ast>
}

pub struct GetCnfVectorZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub cnf: Vec<Z3_ast>
}

pub struct FreshModelZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model

}

impl<'ctx> FreshModelZ3<'ctx> {
    /// New empty model
    pub fn new(ctx: &'ctx ContextZ3) -> Z3_model {
        unsafe {
            Z3_mk_model(ctx.r)
        }
    }
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

impl<'ctx> AstVectorToStringZ3<'ctx> {
    /// AST vector to readable string
    /// 
    /// NOTE: See macro! `ast_vector_to_string_z3!`
    pub fn new(ctx: &'ctx ContextZ3, what: Z3_ast_vector) -> String {
        let z3 = unsafe {
            CStr::from_ptr(Z3_ast_vector_to_string(ctx.r, what)).to_str().unwrap().to_owned()
        };
        AstVectorToStringZ3 {ctx, what, r: z3}.r
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

impl Z3StringToStringZ3 {
    /// Get usable String from a Z3_string..
    pub fn new(cstr: Z3_string) -> String {
        let z3 = unsafe {
            CStr::from_ptr(cstr).to_str().unwrap().to_owned()
        };
        Z3StringToStringZ3 {cstr, r: z3}.r
    }
}

impl <'ctx> Z3AstVectorToVectorAstZ3<'ctx> {
    /// transform to extract
    pub fn new(ctx: &'ctx ContextZ3, ast_vec: Z3_ast_vector) -> Vec<Z3_ast> {
        let mut vec: Vec<Z3_ast> = vec!();
        unsafe {
            let size = Z3_ast_vector_size(ctx.r, ast_vec);
            for i in 0..size {
                vec.push(Z3_ast_vector_get(ctx.r, ast_vec, i));
            }
        }
        vec
    }
}

impl<'ctx> GetCnfVectorZ3<'ctx> {
    /// Get cnf. Will return false if unsat.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> Vec<Z3_ast> {
        let z3 = unsafe {
            let goal = Z3_mk_goal(ctx.r, false, false, false);
            for formula in args {
                Z3_goal_assert(ctx.r, goal, formula);
                Z3_goal_inc_ref(ctx.r, goal);
            }
            let tactic = Z3_mk_tactic(ctx.r, CString::new("tseitin-cnf".to_string()).unwrap().as_ptr());
            Z3_tactic_inc_ref(ctx.r, tactic);
            let applied = Z3_tactic_apply(ctx.r, tactic, goal);
            let nr_subgoals: u32 = Z3_apply_result_get_num_subgoals(ctx.r, applied);
            let subgoal = Z3_apply_result_get_subgoal(ctx.r, applied, 0);
            let subgoal_size = Z3_goal_size(ctx.r, subgoal);
            let mut cnf_vec: Vec<Z3_ast> = vec!();
            for i in 0..subgoal_size {
                cnf_vec.push(Z3_goal_formula(ctx.r, subgoal, i));
            }
            cnf_vec
        };
        GetCnfVectorZ3 {ctx, cnf: z3}.cnf
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

    let sort = BoolSortZ3::new(&ctx);
    let x = BoolVarZ3::new(&ctx, &sort, "x");
    let y = BoolVarZ3::new(&ctx, &sort, "y");
    let z = BoolVarZ3::new(&ctx, &sort, "z");
    
    let asrt1 = NOTZ3::new(&ctx, x);
    let asrt2 = XORZ3::new(&ctx, x, y);
    let asrt2 = XORZ3::new(&ctx, x, z);

    let asrt3 = IMPZ3::new(&ctx, y, x);

    let asrt4 = DISTINCTZ3::new(&ctx, vec!(x, y));
    let asrt5 = XORZ3::new(&ctx, x, y);

    let tseit = GetCnfVectorZ3::new(&ctx, vec!(asrt1, asrt2, asrt3, asrt4, asrt5));

    // println!("{}", tseit);
//     assert_eq!("(goals
// (goal
//   (not x)
//   (or (not x) (not z))
//   (or x z)
//   (or x (not y))
//   (or (not x) (not y))
//   (or x y))
// )", tseit);
}