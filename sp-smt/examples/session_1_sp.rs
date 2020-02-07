use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

fn main() {
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);


  // sp stuff:  
    let x = SPPath::from_string("x");
    let y = SPPath::from_string("y");
    let vars = vec![x, y];
    
    let t = Transition::new("tx", p!(!x), vec![a!(x)], vec![], true);

    let init = p!([!x] && [!y]);

    println!("vars: {:?}", vars);
    println!("t: {:?}",t );
    println!("init: {:?}",init );

    // source state variables:
    let x0 = bool_var_z3!(&ctx, "x0");
    let y0 = bool_var_z3!(&ctx, "y0");
    let x1 = bool_var_z3!(&ctx, "x1");
    let y1 = bool_var_z3!(&ctx, "y1");
    let x2 = bool_var_z3!(&ctx, "x2");
    let y2 = bool_var_z3!(&ctx, "y2");

    // name the trans:
    let tx_0 = bool_var_z3!(&ctx, "tx_0");
    let ty_0 = bool_var_z3!(&ctx, "ty_0");
    let tx_1 = bool_var_z3!(&ctx, "tx_1");
    let ty_1 = bool_var_z3!(&ctx, "ty_1");

    let init = and_z3!(&ctx, not_z3!(&ctx, x0), not_z3!(&ctx, y0));

    
    
    
    //transitions:
    let t1_0 = and_z3!(&ctx, not_z3!(&ctx, x0), x1, imp_z3!(&ctx, y0, y1), imp_z3!(&ctx, y1, y0));//, tx_0, not_z3!(&ctx, ty_0), not_z3!(&ctx, tx_1), not_z3!(&ctx, ty_1));
    let t2_0 = and_z3!(&ctx, not_z3!(&ctx, y0), y1, imp_z3!(&ctx, x0, x1), imp_z3!(&ctx, x1, x0));// , ty_0, not_z3!(&ctx, tx_0), not_z3!(&ctx, tx_1), not_z3!(&ctx, ty_1));
    let t1_1 = and_z3!(&ctx, not_z3!(&ctx, x1), x2, imp_z3!(&ctx, y1, y2), imp_z3!(&ctx, y2, y1));//, tx_1);//, tx_1, not_z3!(&ctx, ty_1));
    let t2_1 = and_z3!(&ctx, not_z3!(&ctx, y1), y2, imp_z3!(&ctx, x1, x2), imp_z3!(&ctx, x2, x1));//, ty_1, not_z3!(&ctx, tx_1));

    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, tx_0, t1_0));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, ty_0, t2_0));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, tx_1, t1_1));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, ty_1, t2_1));

    //  goal:
    let goal = and_z3!(&ctx, x2, y2);
    
    slv_assert_z3!(&ctx, &slv, init);
    slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, t1_0, t1_1, t2_0, t2_1));
    slv_assert_z3!(&ctx, &slv, goal);

    let res1 = slv_check_z3!(&ctx, &slv);
    if res1 == 1 {
        println!("SAT");
    } else if res1 == -1 {
        println!("UNSAT");
    } else {
        println!("UNDEF");
    };

    let model = slv_get_model_z3!(&ctx, &slv);

    // unsafe{
    //     let mut v = &mut BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), "v");
    //     Z3_model_eval(ctx.r, model, x, true, v);
    // }

    // let model = slv_get_model_z3!(&ctx, &slv);
    // println!("{}", model_to_string_z3!(&ctx, model));

    let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

    for model in models {
        println!("{}", model);
    }

}