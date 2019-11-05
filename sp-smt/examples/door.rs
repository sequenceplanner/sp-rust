use sp_smt::*;
use std::ffi::{CStr};

fn main() {
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);

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
    let open_enabled = andz3!(&ctx, eqz3!(&ctx, opened_c, f), eqz3!(&ctx, opened_m, f));
    let open_executing = andz3!(&ctx, opened_c, notz3!(&ctx, opened_m));
    let open_finishing = andz3!(&ctx, opened_c, opened_m);
    let open_done = andz3!(&ctx, notz3!(&ctx, opened_c), opened_m);

    // close door ability statesopened_mopened_m:
    let close_enabled = andz3!(&ctx, notz3!(&ctx, closed_c), notz3!(&ctx, closed_m));
    let close_executing = andz3!(&ctx, closed_c, notz3!(&ctx, closed_m));
    let close_finishing = andz3!(&ctx, closed_c, closed_m);
    let close_done = andz3!(&ctx, notz3!(&ctx, closed_c), closed_m);

    // forbidden behavior (this has not to hold in the assert):
    let forb1 = notz3!(&ctx, andz3!(&ctx, closed_c, opened_c));
    let forb2 = notz3!(&ctx, andz3!(&ctx, closed_m, opened_m));

    // transitions:
    // let t1 = itez3!(&ctx, open_enabled, eqz3!(&ctx, opened_c, t), orz3!(&ctx, eqz3!(&ctx, opened_c, t), eqz3!(&ctx, opened_c, f)));
    let t1 = itez3!(&ctx, open_enabled, opened_c, orz3!(&ctx, opened_c, notz3!(&ctx, opened_c)));
    let t2 = itez3!(&ctx, open_executing, open_finishing, open_executing);
    let t3 = itez3!(&ctx, open_finishing, open_done, open_finishing);

    slv_assert_z3!(&ctx, &slv, t1);
    slv_assert_z3!(&ctx, &slv, forb1);
    slv_assert_z3!(&ctx, &slv, forb2);

    slv_assert_z3!(&ctx, &slv, notz3!(&ctx, andz3!(&ctx, notz3!(&ctx, opened_c), opened_m, notz3!(&ctx, closed_c), notz3!(&ctx, closed_m))));
    slv_assert_z3!(&ctx, &slv, notz3!(&ctx, andz3!(&ctx, notz3!(&ctx, opened_c), opened_m, closed_c, notz3!(&ctx, closed_m))));
    slv_assert_z3!(&ctx, &slv, notz3!(&ctx, andz3!(&ctx, opened_c, notz3!(&ctx, opened_m), notz3!(&ctx, closed_c), notz3!(&ctx, closed_m))));
    slv_assert_z3!(&ctx, &slv, notz3!(&ctx, andz3!(&ctx, opened_c, opened_m, notz3!(&ctx, closed_c), notz3!(&ctx, closed_m))));
    slv_assert_z3!(&ctx, &slv, notz3!(&ctx, andz3!(&ctx, opened_c, notz3!(&ctx, opened_m), notz3!(&ctx, closed_c), closed_m)));

    // slv_assert_and_track_z3!(&ctx, &slv, andz3!(&ctx, t1), "t1");
    // slv_assert_and_track_z3!(&ctx, &slv, andz3!(&ctx, forb1), "forb1");
    // slv_assert_and_track_z3!(&ctx, &slv, andz3!(&ctx, forb2), "forb2");

    slv_check_z3!(&ctx, &slv);

    // let unsat_core = slv_get_unsat_core_z3!(&ctx, &slv);
    // println!("{}", slv_unsat_core_to_string_z3!(&ctx, unsat_core));

    assert_eq!(1, slv_check_z3!(&ctx, &slv));

    let model = slv_get_model_z3!(&ctx, &slv);
    model_to_string_z3!(&ctx, model);
    println!("{}", model_to_string_z3!(&ctx, model));
}