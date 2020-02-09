use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;

fn main() {

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);
  
    let fruit_cstr = CString::new("fruit").unwrap();
    let apple_cstr = CString::new("apple").unwrap();
    let banana_cstr = CString::new("banana").unwrap();
    let orange_cstr = CString::new("orange").unwrap();
    let fruity_cstr = CString::new("fruity").unwrap();

    unsafe {
        
        let name: Z3_symbol = Z3_mk_string_symbol(ctx.r, fruit_cstr.as_ptr());

        let mut enum_names: Vec<Z3_symbol> = vec![std::ptr::null_mut(); 3];
        let mut enum_consts: Vec<Z3_func_decl> = vec![std::ptr::null_mut(); 3];
        let mut enum_testers: Vec<Z3_func_decl> = vec![std::ptr::null_mut(); 3];
        
        enum_names[0] = Z3_mk_string_symbol(ctx.r, apple_cstr.as_ptr());
        enum_names[1] = Z3_mk_string_symbol(ctx.r, banana_cstr.as_ptr());
        enum_names[2] = Z3_mk_string_symbol(ctx.r, orange_cstr.as_ptr());

        let string_sort = StringSortZ3::new(&ctx);
 //     ty: Z3_sort)(ctx.r, enum_consts[0], 0, 0);

        println!("{:?}", enum_names);

        let enum_sort = Z3_mk_enumeration_sort(ctx.r, name, 3, enum_names.as_ptr(), enum_consts.as_mut_ptr(), enum_testers.as_mut_ptr());

        // let enum_var = Z3_mk_const(ctx.r, name, enum1);

        // Z3_mk_app(c: Z3_context, d: Z3_func_decl, num_args: ::std::os::raw::c_uint, args: *const Z3_ast)
        // Z3_mk_func_decl(c: Z3_context, s: Z3_symbol, domain_size: ::std::os::raw::c_uint, domain: *const Z3_sort, range: Z3_sort)

        let apple = Z3_mk_app(ctx.r, enum_consts[0], 0, [].as_ptr());// ;(ctx.r, enum_consts[0], enum_sort);
        let banana = Z3_mk_app(ctx.r, enum_consts[1], 0, [].as_ptr());
        let orange = Z3_mk_app(ctx.r, enum_consts[2], 0, [].as_ptr());
        // let banana = Z3_mk_const(ctx.r, enum_names[1], enum_sort);
        // let orange = Z3_mk_const(ctx.r, enum_names[2], enum_sort);

        let fruity = Z3_mk_const(ctx.r, Z3_mk_string_symbol(ctx.r, fruity_cstr.as_ptr()), enum_sort);

        let asrt1 = eq_z3!(&ctx, fruity, apple);
        let asrt2 = eq_z3!(&ctx, fruity, banana);
        let asrt3 = eq_z3!(&ctx, fruity, orange);

        // slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, apple, orange));
        slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, asrt1, asrt2, asrt3));
        // slv_assert_z3!(&ctx, &slv, not_z3!(&ctx, eq_z3!(&ctx, apple, fruity)));

        // let fruity_string = string_z3!(&ctx, "fruity");
        // let eq_1 = eq_z3!(&ctx, enum_var, enum_var);

        // println!("{}", sort_to_string_z3!(&ctx, enum1));
        // slv_assert_z3!(&ctx, &slv, eq_1);

        println!("{}", slv_to_string_z3!(&ctx, &slv));


        let res1 = slv_check_z3!(&ctx, &slv);
        if res1 == 1 {
            println!("SAT");
        } else if res1 == -1 {
            println!("UNSAT");
        } else {
            println!("UNDEF");
        };


        let model = slv_get_model_z3!(&ctx, &slv);
        println!("{}", model_to_string_z3!(&ctx, model));

    };
}

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