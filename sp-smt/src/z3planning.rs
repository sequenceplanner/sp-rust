//! Z3 plans

use super::*;
use std::ffi::{CStr, CString};
use z3_sys::*;

// pub struct ComputePlanZ3<'ctx, 'slv> {
//     pub ctx: &'ctx ContextZ3,
//     pub slv: &'slv SolverZ3<'ctx>,
//     pub init: Z3_ast,
//     pub goal: Z3_ast, // for now... (add Vec<Z3_ast> later instead)
//     // pub invars: Vec<Z3_ast>,
//     pub trans: Vec<Z3_ast>,
//     pub steps: u32,
//     pub r: Z3_model // for now...
// }

// impl <'ctx, 'slv> ComputePlanZ3<'ctx, 'slv> {
//     /// Get a plan from the initial to the goal state. 
//     ///
//     /// NOTE: See macro! `compute_plan_z3!`
//     pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>, init: Z3_ast, goal: Z3_ast, trans: Vec<Z3_ast>, steps: u32) -> Z3_model {
//         let mut step: u32 = 0;

//         SlvAssertZ3::new(&ctx, &slv, init);

//         SlvPushZ3::new(&ctx, &slv);
//         SlvAssertZ3::new(&ctx, &slv, goal);

//         while SlvCheckZ3::new(&ctx, &slv) != 1 && step < steps {

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