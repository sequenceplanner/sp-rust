use sp_smt::*;

use std::ffi::{CStr};
use z3_sys::*;
// use super::*;

// use std::ffi::{CStr};

fn main() {
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    // source state variables:
    let opened_c = bool_var_z3!(&ctx, "opened_c");
    let opened_m = bool_var_z3!(&ctx, "opened_m");
    let closed_c = bool_var_z3!(&ctx, "closed_c");
    let closed_m = bool_var_z3!(&ctx, "closed_m");
    // let locked_c = bool_var_z3!(&ctx, "locked_c");
    // let locked_m = bool_var_z3!(&ctx, "locked_m");
    // let unlocked_c = bool_var_z3!(&ctx, "unlocked_c");
    // let unlocked_m = bool_var_z3!(&ctx, "unlocked_m");

    let t = bool_z3!(&ctx, true);
    let f = bool_z3!(&ctx, false);

    // deliberation var:
    let delib = bool_var_z3!(&ctx, "delib");

    // initialize some values:
    // opened_c = t;

    
    // open door ability states:
    let open_enabled = and_z3!(&ctx, not_z3!(&ctx, opened_c), not_z3!(&ctx, opened_m));
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
    // let forb3 = not_z3!(&ctx, and_z3!(&ctx, open_executing, locked_m));

    // open door transitions:
    // let open_start = ite_z3!(&ctx, and_z3!(&ctx, open_enabled, not_z3!(&ctx, locked_m)), opened_c, not_z3!(&ctx, opened_c));
    let open_start = ite_z3!(&ctx, open_enabled, opened_c, not_z3!(&ctx, opened_c));
    let open_finish = ite_z3!(&ctx, open_executing, opened_m, or_z3!(&ctx, opened_m, not_z3!(&ctx, opened_m)));
    let open_reset = ite_z3!(&ctx, open_finishing, not_z3!(&ctx, opened_c), opened_c);

    // close door transitions:
    let close_start = ite_z3!(&ctx, close_enabled, closed_c, not_z3!(&ctx, closed_c));
    let close_finish = ite_z3!(&ctx, close_executing, closed_m, or_z3!(&ctx, closed_m, not_z3!(&ctx, closed_m)));
    let close_reset = ite_z3!(&ctx, close_finishing, not_z3!(&ctx, closed_c), closed_c);

    // slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, t, f));

    // slv_assert_z3!(&ctx, &slv, close_done);
    slv_assert_z3!(&ctx, &slv, open_start);
    // slv_assert_z3!(&ctx, &slv, open_finish);
    // slv_assert_z3!(&ctx, &slv, open_reset);
    // slv_assert_z3!(&ctx, &slv, close_start);
    // slv_assert_z3!(&ctx, &slv, close_finish);
    // slv_assert_z3!(&ctx, &slv, close_reset);
    slv_assert_z3!(&ctx, &slv, forb1);
    slv_assert_z3!(&ctx, &slv, forb2);
    // slv_assert_z3!(&ctx, &slv, forb3);

    let models = SlvGetAllModelsDontCareZ3::new(&ctx, &slv).s;

    for model in models {
        println!("{}", model);
    }
}