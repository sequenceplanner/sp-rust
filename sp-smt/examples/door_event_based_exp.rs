use sp_smt::*;

use std::ffi::{CStr};
use z3_sys::*;


fn main() {
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    let state = string_var_z3!(&ctx, "state");
    let state_next = string_var_z3!(&ctx, "state_next");

    let opened = string_z3!(&ctx, "opened");
    let closed = string_z3!(&ctx, "closed");

    let event = string_var_z3!(&ctx, "event");

    let open = string_z3!(&ctx, "open");
    let close = string_z3!(&ctx, "close");
    let none = string_z3!(&ctx, "none");

    let open_enabled = and_z3!(&ctx,
        eq_z3!(&ctx, event, open),
        eq_z3!(&ctx, state, closed)
    );

    let close_enabled = and_z3!(&ctx,
        eq_z3!(&ctx, event, close),
        eq_z3!(&ctx, state, opened)
    );

    // initial state:
    slv_assert_z3!(&ctx, &slv, 
        eq_z3!(&ctx, state, closed)
        // and_z3!(&ctx, 
        //     eq_z3!(&ctx, state, opened)
            // eq_z3!(&ctx, event, open)
        // )
    );

    // Even here we need transitioning states, since if we put event == "open"
    // and initial state == "opened", the next state will be closed for some reason.

    let t1 = eq_z3!(&ctx, 
        state_next,
        ite_z3!(&ctx, open_enabled, opened, closed)
    );

    let t2 = eq_z3!(&ctx, 
        state_next,
        ite_z3!(&ctx, close_enabled, closed, opened)
    );

    let t3 = eq_z3!(&ctx, 
        state_next,
        ite_z3!(&ctx, open_enabled, opened, closed)
    );

    let trans = vec!(t1, t2, t3);

    for t in trans {
         slv_assert_z3!(&ctx, &slv, t);
        let res1 = slv_check_z3!(&ctx, &slv);

        if res1 == 1 {
            println!("SAT");
        } else if res1 == -1 {
            println!("UNSAT");
        } else {
            println!("UNDEF");
        };

        // uncomment if you want to print the context:
        // println!("{}", slv_to_string_z3!(&ctx, &opt));

        let model = slv_get_model_z3!(&ctx, &slv);
        println!("{}", model_to_string_z3!(&ctx, model));
    }

   
    // let models = SlvGetAllModelsZ3::new(&ctx, &slv, 10).s;

    // for model in models {
    //     println!("{}", model);
    // }
}