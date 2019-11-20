use sp_smt::*;

use std::ffi::{CStr};
use z3_sys::*;
// use super::*;

// use std::ffi::{CStr};

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

    // open door transitions:
    let open_start = ite_z3!(&ctx, open_enabled, opened_c, or_z3!(&ctx, opened_c, not_z3!(&ctx, opened_c)));
    let open_finish = ite_z3!(&ctx, open_executing, opened_m, or_z3!(&ctx, opened_m, not_z3!(&ctx, opened_m)));
    let open_reset = ite_z3!(&ctx, open_finishing, not_z3!(&ctx, opened_c), or_z3!(&ctx, opened_c, not_z3!(&ctx, opened_c)));

    // close door transitions:
    let close_start = ite_z3!(&ctx, close_enabled, closed_c, or_z3!(&ctx, closed_c, not_z3!(&ctx, closed_c)));
    let close_finish = ite_z3!(&ctx, close_executing, closed_m, or_z3!(&ctx, closed_m, not_z3!(&ctx, closed_m)));
    let close_reset = ite_z3!(&ctx, close_finishing, not_z3!(&ctx, closed_c), or_z3!(&ctx, closed_c, not_z3!(&ctx, closed_c)));

    slv_assert_z3!(&ctx, &slv, open_start);
    // slv_assert_z3!(&ctx, &slv, open_finish);
    // slv_assert_z3!(&ctx, &slv, open_reset);
    //  slv_assert_z3!(&ctx, &slv, close_start);
    // slv_assert_z3!(&ctx, &slv, close_finish);
    // slv_assert_z3!(&ctx, &slv, close_reset);
    slv_assert_z3!(&ctx, &slv, forb1);
    slv_assert_z3!(&ctx, &slv, forb2);

    while slv_check_z3!(&ctx, &slv) == 1 {
        let model = slv_get_model_z3!(&ctx, &slv);
        model_to_string_z3!(&ctx, model);
        println!("{}", model_to_string_z3!(&ctx, model));
        let mut to_assert = Vec::new();
        unsafe{
            let fnum = Z3_model_get_num_consts(ctx.r, model);
            for i in 0..fnum {
                let fd = Z3_model_get_const_decl(ctx.r, model, i);
                let fname = Z3_get_decl_name(ctx.r, fd);
                let fd_ast = Z3_func_decl_to_ast(ctx.r, fd);
                let val = Z3_model_get_const_interp(ctx.r, model, fd);
                let val_str = ast_to_string_z3!(&ctx, val);
                if val_str == "true" {
                    to_assert.push(bool_var_z3!(&ctx, &CStr::from_ptr(Z3_get_symbol_string(ctx.r, fname)).to_str().unwrap().to_owned()));
                    println!("TRUE{}", val_str);
                } else {
                    to_assert.push(not_z3!(&ctx, bool_var_z3!(&ctx, &CStr::from_ptr(Z3_get_symbol_string(ctx.r, fname)).to_str().unwrap().to_owned())));
                    println!("FALSE{}", val_str)
                }
                // to_assert.push(bool_var_z3!(&ctx, &CStr::from_ptr(Z3_get_symbol_string(ctx.r, fname)).to_str().unwrap().to_owned()));
                // Z3_get_symbol_string(&ctx, fname);
                // println!("{:#?}", CStr::from_ptr(Z3_get_symbol_string(ctx.r, fname)).to_str().unwrap().to_owned());
                // println!("{:#?}", fname);
                
                // let sort = get_sort_z3!(&ctx, fd_ast);
                // println!("{}", sort_to_string_z3!(&ctx, sort));
                let ci = Z3_model_get_const_interp(ctx.r, model, fd);
                // to_assert.push(fd_ast);
        }
        let asrt = and_z3!(&ctx, to_assert);
        println!("{}", ast_to_string_z3!(&ctx, asrt));
        slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, asrt));
        // slv_check_z3!(&ctx, &slv);
            // println!("{}", fnum);
            // let fd = Z3_model_get_const_decl(ctx.r, model, 3);
            // let fd_ast = Z3_func_decl_to_ast(ctx.r, fd);
            // let ci = Z3_model_get_const_interp(ctx.r, model, fd);
            // println!("=====================================");
            // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, ci));
            // println!("=====================================");
            // println!("{}", slv_to_string_z3!(&ctx, &slv));
            // let asdf = not_z3!(&ctx, ci);
            // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, fd_ast));
            // let fd = Z3_model_get_func_decl(ctx.r, model, 1);
            // let fi = Z3_model_get_func_interp(ctx.r, model, fd);
            // println!("{:#?}", CStr::from_ptr(Z3_ast_to_string(ctx.r, fd_ast)).to_str().unwrap().to_owned());
            // CStr::from_ptr(Z3_ast_to_string(ctx.r, opt.r)).to_str().unwrap().to_owned()
        }
        
        // slv_assert_z3!(&ctx, &slv, slv_get_model_z3!(&ctx, &slv));
    }

    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m)));

    // Assertions without forbidden states:
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m, closed_c, not_z3!(&ctx, closed_m))));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m), closed_c, not_z3!(&ctx, closed_m))));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, opened_m, closed_c, not_z3!(&ctx, closed_m))));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m, not_z3!(&ctx, closed_c), closed_m)));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, opened_m, closed_c, closed_m)));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, opened_m, not_z3!(&ctx, closed_c), closed_m)));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m, closed_c, closed_m)));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m), not_z3!(&ctx, closed_c), closed_m)));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m), closed_c, closed_m)));

    // Assertions with forbidden states (open_start and close_start):
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m, closed_c, not_z3!(&ctx, closed_m))));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m), not_z3!(&ctx, closed_c), closed_m)));

    // Assertions with forbidden states (open_start and close_start): wrong asserts, have to extract them actually with a nice method.
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m, not_z3!(&ctx, closed_c), not_z3!(&ctx, closed_m))));
    // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m), closed_c, not_z3!(&ctx, closed_m))));

    slv_check_z3!(&ctx, &slv);

    // let unsat_core = slv_get_unsat_core_z3!(&ctx, &slv);
    // println!("{}", slv_unsat_core_to_string_z3!(&ctx, unsat_core));

    assert_eq!(1, slv_check_z3!(&ctx, &slv));

    let model = slv_get_model_z3!(&ctx, &slv);
    model_to_string_z3!(&ctx, model);
    println!("{}", model_to_string_z3!(&ctx, model));
}