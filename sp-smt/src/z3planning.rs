//! Z3 plans

use super::*;
use std::ffi::{CStr, CString};
use z3_sys::*;

// pub struct ComputePlanFiniteDomainZ3<'ctx, 'slv> {
//     pub ctx: &'ctx ContextZ3,
//     pub slv: &'slv SolverZ3<'ctx>,
//     pub vars: Vec<Z3_ast>,
//     pub init: Z3_ast,
//     pub goal: Z3_ast, // for now... (add Vec<Z3_ast> later instead)
//     pub trans: Vec<Z3_ast>,
//     pub max_steps: u32,
//     pub r: Z3_model
// }

pub struct GetPlanningFramesZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub nr_steps: u32,
    pub frames: Vec<(i32, Vec<String>, String)> 
}

// impl <'ctx, 'slv> ComputePlanFiniteDomainZ3<'ctx, 'slv> {
//     /// Get a plan from the initial to the goal state. 
//     ///
//     /// NOTE: See macro! `compute_plan_z3!`
//     pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>, vars: Vec<Z3_ast>, init: Z3_ast, goal: Z3_ast, trans: Vec<Z3_ast>, max_steps: u32) -> (Z3_model, u32) {
//         let mut step: u32 = 0;

//         SlvAssertZ3::new(&ctx, &slv, init);

//         SlvPushZ3::new(&ctx, &slv);
//         SlvAssertZ3::new(&ctx, &slv, goal);

//         while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {

//             step = step + 1;
//             SlvPopZ3::new(&ctx, &slv, 1);

//             let current_step: &str = &format!("pose_s{}", step);
//             let next_step: &str = &format!("pose_s{}", step + 1);

//             for tran in trans {
//                 SlvAssertZ3::new(&ctx, &slv, tran);

//             }

//         }

//         ComputePlanZ3 {ctx, slv, r: z3}.r
//     }
// }

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