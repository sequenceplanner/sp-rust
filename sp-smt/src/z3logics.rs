//! Z3 logical operations for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct ANDZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub s: String,
    pub r: Z3_ast
}

pub struct ORZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub s: String,
    pub r: Z3_ast
}

pub struct NOTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct ITEZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub ifz3: Z3_ast,
    pub thenz3: Z3_ast,
    pub elsez3: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct IFFZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct IMPZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

pub struct XORZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub s: String,
    pub r: Z3_ast
}

impl<'ctx> ANDZ3<'ctx> {
    /// Create an AST node representing `args[0] and ... and args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have Boolean sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> ANDZ3 {
        let args_slice = &args;
        let z3 = unsafe {
            Z3_mk_and(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        ANDZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            args
        }        
    }
}

impl<'ctx> ORZ3<'ctx> {
    /// Create an AST node representing `args[0] or ... or args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have Boolean sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> ORZ3 {
        let args_slice = &args;
        let z3 = unsafe {
            Z3_mk_or(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        ORZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            args
        }        
    }
}

impl<'ctx> NOTZ3<'ctx> {
    /// Create an AST node representing `not(arg)`.
    ///
    /// The node `arg` must have Boolean sort.
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> NOTZ3 {
        let z3 = unsafe {
            Z3_mk_not(ctx.r, arg)
        };
        NOTZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            },
            arg
        }        
    }
}

impl<'ctx> ITEZ3<'ctx> {
    /// Create an AST node representing an if-then-else: `ite(ifz3, thenz3, elsez3)`.
    ///
    /// The node `ifz3` must have Boolean sort, `thenz3` and `elsez3` must have the same sort.
    /// The sort of the new node is equal to the sort of `thenz3` and `elsez3`.
    pub fn new(ctx: &'ctx ContextZ3, ifz3: Z3_ast, thenz3: Z3_ast, elsez3: Z3_ast) -> ITEZ3 {
        let z3 = unsafe {
            Z3_mk_ite(ctx.r, ifz3, thenz3, elsez3)
        };
        ITEZ3 {
            ctx,
            ifz3,
            thenz3,
            elsez3,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }        
    }
}

impl<'ctx> IFFZ3<'ctx> {
    /// Create an AST node representing `left iff right`.
    ///
    /// The nodes `left` and `right` must have Boolean sort.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> IFFZ3 {
        let z3 = unsafe {
            Z3_mk_iff(ctx.r, left, right)
        };
        IFFZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }        
    }
}

impl<'ctx> IMPZ3<'ctx> {
    /// Create an AST node representing `left implies right`.
    ///
    /// The nodes `left` and `right` must have Boolean sort.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> IMPZ3 {
        let z3 = unsafe {
            Z3_mk_implies(ctx.r, left, right)
        };
        IMPZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }        
    }
}

impl<'ctx> XORZ3<'ctx> {
    /// Create an AST node representing `left xor right`.
    ///
    /// The nodes `left` and `right` must have Boolean sort.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> XORZ3 {
        let z3 = unsafe {
            Z3_mk_xor(ctx.r, left, right)
        };
        XORZ3 {
            ctx,
            left,
            right,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }        
    }
}



#[test]
fn test_new_and(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx,false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let and1 = ANDZ3::new(&ctx, vec!(x1.r, x2.r, bool1.r, bool2.r));

    println!("{}", and1.s);
}

#[test]
fn test_new_or(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx,false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let or1 = ORZ3::new(&ctx, vec!(x1.r, x2.r, bool1.r, bool2.r));

    println!("{}", or1.s);
}

#[test]
fn test_new_not(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");

    let not1 = NOTZ3::new(&ctx, bool1.r);
    let not2 = NOTZ3::new(&ctx, x1.r);
    let not3 = NOTZ3::new(&ctx, not2.r);

    println!("{}", not1.s);
    println!("{}", not2.s);
    println!("{}", not3.s);

}

#[test]
fn test_new_ite(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let int1 = IntZ3::new(&ctx, &intsort, 3);
    let int2 = IntZ3::new(&ctx, &intsort, 7);

    let ite1 = ITEZ3::new(&ctx, bool1.r, int1.r, int2.r);
    let ite2 = ITEZ3::new(&ctx, bool2.r, int1.r, int2.r);
    
    println!("{}", ite1.s);
    println!("{}", ite2.s);
}

#[test]
fn test_new_iff(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let iff1 = IFFZ3::new(&ctx, bool1.r, bool2.r);
    let iff2 = IFFZ3::new(&ctx, bool1.r, x1.r);
    let iff3 = IFFZ3::new(&ctx, x2.r, bool2.r);
    let iff4 = IFFZ3::new(&ctx, x1.r, x2.r);
    
    println!("{}", iff1.s);
    println!("{}", iff2.s);
    println!("{}", iff3.s);
    println!("{}", iff4.s);
}

#[test]
fn test_new_imp(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let imp1 = IMPZ3::new(&ctx, bool1.r, bool2.r);
    let imp2 = IMPZ3::new(&ctx, bool1.r, x1.r);
    let imp3 = IMPZ3::new(&ctx, x2.r, bool2.r);
    let imp4 = IMPZ3::new(&ctx, x1.r, x2.r);
    
    println!("{}", imp1.s);
    println!("{}", imp2.s);
    println!("{}", imp3.s);
    println!("{}", imp4.s);
}

#[test]
fn test_new_xor(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let xor1 = XORZ3::new(&ctx, bool1.r, bool2.r);
    let xor2 = XORZ3::new(&ctx, bool1.r, x1.r);
    let xor3 = XORZ3::new(&ctx, x2.r, bool2.r);
    let xor4 = XORZ3::new(&ctx, x1.r, x2.r);
    
    println!("{}", xor1.s);
    println!("{}", xor2.s);
    println!("{}", xor3.s);
    println!("{}", xor4.s);
}