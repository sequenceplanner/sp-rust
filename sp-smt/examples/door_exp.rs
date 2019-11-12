use sp_smt::*;
use std::ffi::{CStr};

fn main() {
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    // variables:
    let opened_c = bool_var_z3!(&ctx, "opened_c");
    let opened_m = bool_var_z3!(&ctx, "opened_m");
    let closed_c = bool_var_z3!(&ctx, "closed_c");
    let closed_m = bool_var_z3!(&ctx, "closed_m");

    let t = bool_z3!(&ctx, true);
    let f = bool_z3!(&ctx, false);

    // deliberation var:
    let delib = bool_var_z3!(&ctx, "delib");
    
    // open door ability states:
    let open_enabled = and_z3!(&ctx, eq_z3!(&ctx, opened_c, f), eq_z3!(&ctx, opened_m, f));
    let open_executing = and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m));
    let open_finishing = and_z3!(&ctx, opened_c, opened_m);
    let open_done = and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m);

    // close door ability statesopened_mopened_m:
    let close_enabled = and_z3!(&ctx, not_z3!(&ctx, closed_c), not_z3!(&ctx, closed_m));
    let close_executing = and_z3!(&ctx, closed_c, not_z3!(&ctx, closed_m));
    let close_finishing = and_z3!(&ctx, closed_c, closed_m);
    let close_done = and_z3!(&ctx, not_z3!(&ctx, closed_c), closed_m);

    // forbidden behavior (this has not to hold in the assert):
    let forb1 = not_z3!(&ctx, and_z3!(&ctx, closed_c, opened_c));
    let forb2 = not_z3!(&ctx, and_z3!(&ctx, closed_m, opened_m));

    // transitions:
    // let t1 = ite_z3!(&ctx, open_enabled, eq_z3!(&ctx, opened_c, t), or_z3!(&ctx, eq_z3!(&ctx, opened_c, t), eq_z3!(&ctx, opened_c, f)));
    let t1 = ite_z3!(&ctx, open_enabled, opened_c, or_z3!(&ctx, opened_c, not_z3!(&ctx, opened_c)));
    let t2 = ite_z3!(&ctx, open_executing, open_finishing, open_executing);
    let t3 = ite_z3!(&ctx, open_finishing, open_done, open_finishing);

    slv_assert_z3!(&ctx, &slv, t1);
    slv_assert_z3!(&ctx, &slv, forb1);
    slv_assert_z3!(&ctx, &slv, forb2);

    slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m, not_z3!(&ctx, closed_c), not_z3!(&ctx, closed_m))));
    slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m, closed_c, not_z3!(&ctx, closed_m))));
    slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m), not_z3!(&ctx, closed_c), not_z3!(&ctx, closed_m))));
    slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, opened_m, not_z3!(&ctx, closed_c), not_z3!(&ctx, closed_m))));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m), not_z3!(&ctx, closed_c), closed_m)));

    // slv_assert_and_track_z3!(&ctx, &slv, and_z3!(&ctx, t1), "t1");
    // slv_assert_and_track_z3!(&ctx, &slv, and_z3!(&ctx, forb1), "forb1");
    // slv_assert_and_track_z3!(&ctx, &slv, and_z3!(&ctx, forb2), "forb2");

    slv_check_z3!(&ctx, &slv);

    // let unsat_core = slv_get_unsat_core_z3!(&ctx, &slv);
    // println!("{}", slv_unsat_core_to_string_z3!(&ctx, unsat_core));

    assert_eq!(1, slv_check_z3!(&ctx, &slv));

    let model = slv_get_model_z3!(&ctx, &slv);
    model_to_string_z3!(&ctx, model);
    println!("{}", model_to_string_z3!(&ctx, model));
}