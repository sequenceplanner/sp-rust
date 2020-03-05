//! Z3 plans

use super::*;
use std::ffi::{CStr, CString};
use sp_domain::*;
use z3_sys::*;

pub struct UntilZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub x: Z3_ast,
    pub y: Z3_ast
}

// test this somehow
impl <'ctx> UntilZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, x: &Predicate, y: &Predicate, mut from_step: u32, until_step: u32) -> Z3_ast {
        let mut unrolled: Z3_ast = GetSPPredicateZ3::new(&ctx, ts_model, from_step, x);
        if from_step <= until_step {
            from_step = from_step + 1;
            unrolled = ORZ3::new(&ctx, vec!(
                    GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, y),
                    ANDZ3::new(&ctx, vec!(
                            GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, x),
                            UntilZ3::new(&ctx, ts_model, &x, &y, from_step, until_step)
                        )
                    )
                )
            );
        } else {
            unrolled
        }
    }
}