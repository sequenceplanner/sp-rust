//! Z3 ltl

use super::*;
use std::ffi::{CStr, CString};
use sp_domain::*;
use z3_sys::*;

pub struct UntilZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub x: Z3_ast,
    pub y: Z3_ast
}

pub struct UntilAndInvarZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub x: Z3_ast,
    pub y: Z3_ast
}

pub struct AtLeastOnceZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub x: Z3_ast,
    pub y: Z3_ast
}

// maybe also make a pure z3 impl and on top of that hte SP connection?
impl <'ctx> UntilZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel,
        x: &Predicate, y: &Predicate, mut from_step: u32, until_step: u32) -> Z3_ast {
        if from_step < until_step {
            from_step = from_step + 1;
            ORZ3::new(&ctx, vec!(
                    GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, y),
                    ANDZ3::new(&ctx, vec!(
                            GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, x),
                            UntilZ3::new(&ctx, ts_model, &x, &y, from_step, until_step)
                        )
                    )
                )
            )
        } else if from_step == until_step {
            ORZ3::new(&ctx, vec!(
                GetSPPredicateZ3::new(&ctx, ts_model, from_step, y),
                GetSPPredicateZ3::new(&ctx, ts_model, from_step, x)
                )
            )
        } else {
            panic! ("from_step > until_step")
        }
    }
}

// just for now, make a generatl one later...
impl <'ctx> UntilAndInvarZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel,
        x: &Predicate, y: &Predicate, mut from_step: u32, until_step: u32) -> Z3_ast {
        if from_step < until_step {
            from_step = from_step + 1;
            ORZ3::new(&ctx, vec!(
                    ANDZ3::new(&ctx, vec!(GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, x), GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, y))),
                    ANDZ3::new(&ctx, vec!(
                            ANDZ3::new(&ctx, vec!(GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, x), GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, y))),
                            UntilZ3::new(&ctx, ts_model, &x, &y, from_step, until_step)
                        )
                    )
                )
            )
        } else if from_step == until_step {
            ORZ3::new(&ctx, vec!(
                ANDZ3::new(&ctx, vec!(GetSPPredicateZ3::new(&ctx, ts_model, from_step, x), GetSPPredicateZ3::new(&ctx, ts_model, from_step, y))),
                GetSPPredicateZ3::new(&ctx, ts_model, from_step, x)
                )
            )
        } else {
            panic! ("from_step > until_step")
        }
    }
}

// In a finite length trace, property has to hold at least in one step (a disjunction basically)
impl <'ctx> AtLeastOnceZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, 
        x: &Predicate, from_step: u32, until_step: u32) -> Z3_ast {
        let mut disj: Vec<Z3_ast> = vec!();
        for step in from_step..until_step + 1 {
            disj.push(GetSPPredicateZ3::new(&ctx, ts_model, step, x));
        }
        ORZ3::new(&ctx, disj)
    }
}