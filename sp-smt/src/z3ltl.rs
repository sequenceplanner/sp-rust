//! Z3 plans

use super::*;
use std::ffi::{CStr, CString};
use z3_sys::*;

pub struct UntilZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub x: Z3_ast,
    pub y: Z3_ast,
    pub r: Z3_ast
}

impl <'ctx> UntilZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, x: Z3_ast, y: Z3_ast) -> Z3_ast {
        ORZ3::new(&ctx, vec!(y, ANDZ3::new(&ctx, args: Vec<Z3_ast>)))
    }
}



impl <'ctx> GetPlanningFramesZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, model: Z3_model, nr_steps: u32) -> Vec<(u32, Vec<String>, String)> {
        let model_str = ModelToStringZ3::new(&ctx, model);
        let mut model_vec = vec!();

        let num = ModelGetNumConstsZ3::new(&ctx, model);
        let mut lines = model_str.lines();
        let mut i: u32 = 0;
        let mut frames: Vec<(u32, Vec<String>, String)> = vec!();

        while i <= num {
            model_vec.push(lines.next().unwrap_or(""));
            i = i + 1;
        }

        for i in 0..nr_steps + 2 {
            let mut frame: (u32, Vec<String>, String) = (0, vec!(), "".to_string());
            for j in &model_vec {
                let sep: Vec<&str> = j.split(" -> ").collect();
                frame.0 = i;
                if sep[0].ends_with(&format!("_s{}", i)){
                    frame.1.push(sep[1].to_string());
                } else if sep[0].ends_with(&format!("_t{}", i)) && sep[1] == "true" {
                    let trimmed = sep[0].trim_end_matches(&format!("_t{}", i));
                    frame.2 = trimmed.to_string();
                }
            }
            if frame.1.len() != 0 {
                frames.push(frame);
            } 
        }
        frames
    }
}