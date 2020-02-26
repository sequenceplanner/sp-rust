//! Z3 plans

use super::*;
use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
use sp_runner::*;

pub struct ComputePlanSPModelZ3 {
    pub mdoel: TransitionSystemModel,
    pub state: SPState,
    pub goals: Vec<(Predicate, Option<Predicate>)>,
    pub max_steps: u32,
    pub r: Z3_model
}

pub struct GetPlanningFramesZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub nr_steps: u32,
    pub frames: Vec<(i32, Vec<String>, String)> 
}

impl Planner for ComputePlanSPModelZ3 {
    fn plan(model: &TransitionSystemModel,
            goals: &[(Predicate, Option<Predicate>)],
            state: &SPState,
            max_steps: u32) -> PlanningResult {

        let cfg = ConfigZ3::new();
        let ctx = ContextZ3::new(&cfg);
        let slv = SolverZ3::new(&ctx);

        let v: Vec<_> = state.clone().extract().iter().map(|(path,value)|path.clone()).collect();
        let vs: Vec<_> = v.iter().map(|x|x.to_string()).collect();
        let init_name =  vs.join(",");

        let init_type: SPValueType = match init.sp_value_from_path(&x) {
            Some(x) => x.has_type(),
            None    => SPValueType::Unknown,
        };
    
        let init_value: String = match init.sp_value_from_path(&x) {
            Some(x) => x.to_string(),
            None    => SPValue::Unknown.to_string(),
        };

        if init_type == SPValueType::Bool {
            let bool_sort = BoolSortZ3::new(&ctx);
            let init = BoolVarZ3::new(&ctx, &bool_sort, &format!("{}_s0", init_name);
            SlvAssertZ3::new(&ctx, &slv, cst: Z3_ast)
        } else {
            let enum_sort = EnumSortZ3::new(&ctx, &format!("{}_sort", init_name), 
        }

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