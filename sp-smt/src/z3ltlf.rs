//! Z3 ltlf

use super::*;
use std::ffi::{CStr, CString};
use sp_domain::*;
use z3_sys::*;

pub struct UntilZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub x: Z3_ast,
    pub y: Z3_ast
}

pub struct AtLeastOnceZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub x: Z3_ast,
    pub y: Z3_ast
}

impl <'ctx> UntilZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel,
        x: &Predicate, y: &Vec<&Predicate>, mut from_step: u32, until_step: u32) -> Z3_ast {
        let mut y_vec: Vec<Z3_ast> = vec!();
        for sub_y in y {
            y_vec.push(GetSPPredicateZ3::new(&ctx, ts_model, from_step, sub_y));
        }        
        // let y_vec = ANDZ3::new(&ctx, GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, y);
        if from_step < until_step {
            from_step = from_step + 1;
            ORZ3::new(&ctx, vec!(
                    ANDZ3::new(&ctx, y_vec),
                    ANDZ3::new(&ctx, vec!(
                            GetSPPredicateZ3::new(&ctx, ts_model, from_step - 1, x),
                            UntilZ3::new(&ctx, ts_model, &x, &y, from_step, until_step)
                        )
                    )
                )
            )
        } else if from_step == until_step {
            ORZ3::new(&ctx, vec!(
                ANDZ3::new(&ctx, y_vec),
                // GetSPPredicateZ3::new(&ctx, ts_model, from_step, y_vec),
                GetSPPredicateZ3::new(&ctx, ts_model, from_step, x)
                )
            )
        } else {
            panic! ("from_step > until_step")
        }
    }
}

// In a finite length trace, property has to hold at least in one step (a disjunction basically).
// Property defined as a conjunction of predicates
impl <'ctx> AtLeastOnceZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, 
        x: Vec<&Predicate>, from_step: u32, until_step: u32) -> Z3_ast {
        let mut disj: Vec<Z3_ast> = vec!();
        for step in from_step..until_step + 1 {
            let mut conj: Vec<Z3_ast> = vec!();
            for pred in &x {
                conj.push(GetSPPredicateZ3::new(&ctx, ts_model, step, pred));
            }
            disj.push(ANDZ3::new(&ctx, conj));
        }
        ORZ3::new(&ctx, disj)
    }
}