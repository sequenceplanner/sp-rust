// use sp_smt::*;

// use std::ffi::{CStr, CString};
// use z3_sys::*;

fn main() {
}
//     let cfg = cfg_z3!();
//     let ctx = ctx_z3!(&cfg);
//     let slv = slv_z3!(&ctx);
  
//     // source state variables:
//     let x0 = bool_var_z3!(&ctx, "x0");
//     let y0 = bool_var_z3!(&ctx, "y0");
//     let x1 = bool_var_z3!(&ctx, "x1");
//     let y1 = bool_var_z3!(&ctx, "y1");
//     let x2 = bool_var_z3!(&ctx, "x2");
//     let y2 = bool_var_z3!(&ctx, "y2");

//     // name the trans:
//     let tx_0 = bool_var_z3!(&ctx, "tx_0");
//     let ty_0 = bool_var_z3!(&ctx, "ty_0");
//     let tx_1 = bool_var_z3!(&ctx, "tx_1");
//     let ty_1 = bool_var_z3!(&ctx, "ty_1");

//     let init = and_z3!(&ctx, not_z3!(&ctx, x0), not_z3!(&ctx, y0));
    
    
//     //transitions:
//     let t1_0 = and_z3!(&ctx, not_z3!(&ctx, x0), x1, imp_z3!(&ctx, y0, y1), imp_z3!(&ctx, y1, y0)); //, tx_0, not_z3!(&ctx, ty_0));
//     let t2_0 = and_z3!(&ctx, not_z3!(&ctx, y0), y1, imp_z3!(&ctx, x0, x1), imp_z3!(&ctx, x1, x0));//, ty_0, not_z3!(&ctx, tx_0));
//     let t1_1 = and_z3!(&ctx, not_z3!(&ctx, x1), x2, imp_z3!(&ctx, y1, y2), imp_z3!(&ctx, y2, y1));//, tx_1, not_z3!(&ctx, ty_1));
//     let t2_1 = and_z3!(&ctx, not_z3!(&ctx, y1), y2, imp_z3!(&ctx, x1, x2), imp_z3!(&ctx, x2, x1));//, ty_1, not_z3!(&ctx, tx_1));

//     //  goal:
//     let goal = and_z3!(&ctx, x2, y2);
    
//     slv_assert_z3!(&ctx, &slv, init);
//     slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, t1_0, t1_1, t2_0, t2_1));
//     slv_assert_z3!(&ctx, &slv, goal);

//     let res1 = slv_check_z3!(&ctx, &slv);
//     if res1 == 1 {
//         println!("SAT");
//     } else if res1 == -1 {
//         println!("UNSAT");
//     } else {
//         println!("UNDEF");
//     };

//     let model = slv_get_model_z3!(&ctx, &slv);

//     // unsafe{
//     //     let mut v = &mut BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), "v");
//     //     Z3_model_eval(ctx.r, model, x, true, v);
//     // }

//     // let model = slv_get_model_z3!(&ctx, &slv);
//     // println!("{}", model_to_string_z3!(&ctx, model));

//     let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

//     for model in models {
//         println!("{}", model);
//     }

// }

// void enum_example() {
//     Z3_context ctx = mk_context();
//     Z3_solver s = mk_solver(ctx);
//     Z3_sort fruit;
//     Z3_symbol name = Z3_mk_string_symbol(ctx, "fruit");
//     Z3_symbol enum_names[3];
//     Z3_func_decl enum_consts[3];
//     Z3_func_decl enum_testers[3];
//     Z3_ast apple, orange, banana, fruity;
//     Z3_ast ors[3];

//     printf("\nenum_example\n");
//     LOG_MSG("enum_example");

//     enum_names[0] = Z3_mk_string_symbol(ctx,"apple");
//     enum_names[1] = Z3_mk_string_symbol(ctx,"banana");
//     enum_names[2] = Z3_mk_string_symbol(ctx,"orange");

//     fruit = Z3_mk_enumeration_sort(ctx, name, 3, enum_names, enum_consts, enum_testers);

//     printf("%s\n", Z3_func_decl_to_string(ctx, enum_consts[0]));
//     printf("%s\n", Z3_func_decl_to_string(ctx, enum_consts[1]));
//     printf("%s\n", Z3_func_decl_to_string(ctx, enum_consts[2]));

//     printf("%s\n", Z3_func_decl_to_string(ctx, enum_testers[0]));
//     printf("%s\n", Z3_func_decl_to_string(ctx, enum_testers[1]));
//     printf("%s\n", Z3_func_decl_to_string(ctx, enum_testers[2]));

//     apple  = Z3_mk_app(ctx, enum_consts[0], 0, 0);
//     banana = Z3_mk_app(ctx, enum_consts[1], 0, 0);
//     orange = Z3_mk_app(ctx, enum_consts[2], 0, 0);

//     /* Apples are different from oranges */
//     prove(ctx, s, Z3_mk_not(ctx, Z3_mk_eq(ctx, apple, orange)), true);

//     /* Apples pass the apple test */
//     prove(ctx, s, Z3_mk_app(ctx, enum_testers[0], 1, &apple), true);

//     /* Oranges fail the apple test */
//     prove(ctx, s, Z3_mk_app(ctx, enum_testers[0], 1, &orange), false);
//     prove(ctx, s, Z3_mk_not(ctx, Z3_mk_app(ctx, enum_testers[0], 1, &orange)), true);

//     fruity = mk_var(ctx, "fruity", fruit);

//     /* If something is fruity, then it is an apple, banana, or orange */
//     ors[0] = Z3_mk_eq(ctx, fruity, apple);
//     ors[1] = Z3_mk_eq(ctx, fruity, banana);
//     ors[2] = Z3_mk_eq(ctx, fruity, orange);

//     prove(ctx, s, Z3_mk_or(ctx, 3, ors), true);

//     /* delete logical context */
//     del_solver(ctx, s);
//     Z3_del_context(ctx);
// }