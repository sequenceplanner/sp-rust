//! Z3 realtions for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct EQZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

impl <'ctx> EQZ3<'ctx> {
    /// Create an AST node representing `left = right`.
    ///
    /// The nodes `left` and `right` must have the same type.
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> EQZ3 {
        EQZ3 {
            ctx,
            left,
            right,
            r: unsafe {
                let eq = Z3_mk_eq(ctx.r, left, right);
                eq
            }
        }
    }
}


 #[test]
fn test_new_eq(){
    unsafe{
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let intsort = IntSortZ3::new(&ctx);

        let x = IntVarZ3::new(&ctx, &intsort, "x");
        let int1 = IntZ3::new(&ctx, &intsort, 7);

        let rel1 = EQZ3::new(&ctx, x.r, int1.r);

        let string1 = CStr::from_ptr(Z3_ast_to_string(ctx.r, rel1.r)).to_str().unwrap().to_owned();

        assert_eq!("(= x 7)", string1);
    }
}