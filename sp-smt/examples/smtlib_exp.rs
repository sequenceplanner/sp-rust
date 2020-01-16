use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;

fn main() {
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    unsafe {
        let int_sort = int_sort_z3!(&ctx).r;
        let symbol = Z3_mk_string_symbol(ctx.r, CString::new("g").unwrap().as_ptr());
        let domain = vec!(int_sort, int_sort);
        let domain_slice = &domain;
        let g = Z3_mk_func_decl(ctx.r, symbol, 2, domain_slice.as_ptr(), int_sort);
        let g_slice = &g;

        let t = Z3_get_range(ctx.r, g);
        let t_slice = &t;
        let f_name = Z3_mk_string_symbol(ctx.r, CString::new("f").unwrap().as_ptr());
        let f_name_slice = &f_name;
        let t_name = Z3_mk_string_symbol(ctx.r, CString::new("T").unwrap().as_ptr());
        let t_name_slice = &t_name;

        let q = Z3_parse_smtlib2_string(ctx.r, 
                                        CString::new("(assert (forall ((x T) (y T)) (= (f x y) (f y x))))").unwrap().as_ptr(), 
                                        1, t_name_slice, t_slice, 
                                        1, f_name_slice, g_slice);
        
        let what = Z3_ast_vector_to_string(ctx.r, q);
        let string1 = CStr::from_ptr(what).to_str().unwrap().to_owned();

        println!("{:#?}", string1);

        for i in 0..Z3_ast_vector_size(ctx.r, q) {
            let ast = Z3_ast_vector_get(ctx.r, q, i);
            let what2 = Z3_ast_to_string(ctx.r, ast);
            let string2 = CStr::from_ptr(what2).to_str().unwrap().to_owned();
            println!("{:#?}", string2);
        }
    }


    // // open door ability states:
    // let open_enabled = and_z3!(&ctx, not_z3!(&ctx, opened_c), not_z3!(&ctx, opened_m));
    // let open_executing = and_z3!(&ctx, opened_c, not_z3!(&ctx, opened_m));
    // let open_finishing = and_z3!(&ctx, opened_c, opened_m);
    // let open_done = and_z3!(&ctx, not_z3!(&ctx, opened_c), opened_m);

    // // close door ability statesopened_mopened_m:
    // let close_enabled = and_z3!(&ctx, not_z3!(&ctx, closed_c), not_z3!(&ctx, closed_m));
    // let close_executing = and_z3!(&ctx, closed_c, not_z3!(&ctx, closed_m));
    // let close_finishing = and_z3!(&ctx, closed_c, closed_m);
    // let close_done = and_z3!(&ctx, not_z3!(&ctx, closed_c), closed_m);

    // // forbidden behavior (this has not to hold in the assert):
    // let forb1 = not_z3!(&ctx, and_z3!(&ctx, closed_c, opened_c));
    // let forb2 = not_z3!(&ctx, and_z3!(&ctx, closed_m, opened_m));
    // // let forb3 = not_z3!(&ctx, and_z3!(&ctx, open_executing, locked_m));

    // open door transitions:
    // let open_start = ite_z3!(&ctx, and_z3!(&ctx, open_enabled, not_z3!(&ctx, locked_m)), opened_c, not_z3!(&ctx, opened_c));
    // let open_start = ite_z3!(&ctx, t, opened_c, not_z3!(&ctx, opened_c));
    // let open_finish = ite_z3!(&ctx, t, opened_m2, not_z3!(&ctx, opened_m2));
    // let open_reset = ite_z3!(&ctx, f, not_z3!(&ctx, opened_c2), opened_c2);

    // // close door transitions:
    // let close_start = ite_z3!(&ctx, close_enabled, closed_c, not_z3!(&ctx, closed_c));
    // let close_finish = ite_z3!(&ctx, close_executing, closed_m, or_z3!(&ctx, closed_m, not_z3!(&ctx, closed_m)));
    // let close_reset = ite_z3!(&ctx, close_finishing, not_z3!(&ctx, closed_c), closed_c);

        // slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, t, f));

    // slv_assert_z3!(&ctx, &slv, close_done);
    // slv_assert_z3!(&ctx, &slv, opened_c);
    // slv_assert_z3!(&ctx, &slv, open_finish);
    // slv_assert_z3!(&ctx, &slv, open_reset);
    // slv_assert_z3!(&ctx, &slv, close_start);
    // slv_assert_z3!(&ctx, &slv, close_finish);
    // slv_assert_z3!(&ctx, &slv, close_reset);
    // slv_assert_z3!(&ctx, &slv, forb1);
    // slv_assert_z3!(&ctx, &slv, forb2);
    // slv_assert_z3!(&ctx, &slv, forb3);

    // let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

    // for model in models {
    //     println!("{}", model);
    // }
}