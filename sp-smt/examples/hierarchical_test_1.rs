use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;
use std::time::{Duration, Instant};

fn main() {

    // problem descriprion:
    println!("\n Find a plan to go from home to the table:
    
            home 
            . . 
           .   .
          .     . 
        via1    via2
          .     . 
           .   .
            . .  
           table\n");

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    let pose_sort = EnumSortZ3::new(&ctx, "pose_sort", vec!("a", "b", "c"));
    let poses = &pose_sort.enum_asts;

    let mut depth: u32 = 0;
    let max_depth: u32 = 10;

    let pose0 = EnumVarZ3::new(&ctx, pose_sort.r, "pose_s0");

    // initial state:
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[0]));

    // goal state:
    SlvPushZ3::new(&ctx, &slv);
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, pose0, poses[2]));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("primitive"))));

    while SlvCheckZ3::new(&ctx, &slv) != 1 && depth < max_depth {
        depth = depth + 1;
        SlvPopZ3::new(&ctx, &slv, 1);

        let current_depth: &str = &format!("pose_s{}", depth - 1);
        let next_depth: &str = &format!("pose_s{}", depth);

        let a_c = and_z3!(&ctx,
            eq_z3!(&ctx, bool_z3!(&ctx, false), bool_var_z3!(&ctx, &format!("primitive"))),
            // eq_z3!(&ctx, bool_z3!(&ctx, false), bool_var_z3!(&ctx, &format!("primitive_t{}", depth))),
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("a_c_t{}", depth))),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, "p0"), poses[0]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, "p1"), poses[2]));
        
        let a_b = and_z3!(&ctx,
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("a_c_t{}", depth - 1))),
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("primitive"))),
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("a_b_t{}", depth))),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, "p0"), poses[0]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, "p1"), poses[1]));

        let b_c = and_z3!(&ctx,
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("a_c_t{}", depth - 1))),
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("primitive"))),
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("b_c_t{}", depth))),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, "p0"), poses[1]),
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, "p1"), poses[2]));

        slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, a_c, and_z3!(&ctx, a_b, b_c)));
        

        SlvPushZ3::new(&ctx, &slv);
        
        // goal state:
        slv_assert_z3!(&ctx, &slv,
            eq_z3!(&ctx, EnumVarZ3::new(&ctx, pose_sort.r, "p1"), poses[2]));
        slv_assert_z3!(&ctx, &slv,
            eq_z3!(&ctx, bool_z3!(&ctx, true), bool_var_z3!(&ctx, &format!("primitive"))));
            
    }

    let model = SlvGetModelZ3::new(&ctx, &slv);
    // let frames = GetPlanningFramesZ3::new(&ctx, model, step);
    
    println!("{}", model_to_string_z3!(&ctx, model));

    // for l in frames {
    //     println!("{:?} : {:?} : {:?}", l.0, l.1, l.2);
    //     }
}