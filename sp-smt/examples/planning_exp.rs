use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;

 fn main() {
    unsafe{
    let cfg = Z3_mk_config();
    let ctx = Z3_mk_context(cfg);
    // Z3_set_param_value(cfg, ":fixedpoint.explanations-on-relation-level", "true");
 
    let dl = Z3_mk_fixedpoint(ctx);
    Z3_fixedpoint_inc_ref(ctx, dl);
     
    // printf("all params: %s\n", Z3_fixedpoint_get_help(ctx, dl));
    let params = Z3_mk_params(ctx);
    Z3_params_set_symbol(ctx, params, Z3_mk_string_symbol(ctx, CString::new(":engine").unwrap().as_ptr()), Z3_mk_string_symbol(ctx, CString::new("datalog").unwrap().as_ptr()));
    Z3_params_set_bool(ctx, params, Z3_mk_string_symbol(ctx, CString::new(":datalog.generate_explanations").unwrap().as_ptr()), true);
    // Z3_params_set_bool(ctx, params, Z3_mk_string_symbol(ctx, CString::new(":eager-emptiness-checking").unwrap().as_ptr()), false);
    Z3_fixedpoint_set_params(ctx, dl, params);

    let bool_sort = Z3_mk_bool_sort(ctx);
    let a = Z3_mk_const(ctx,
            Z3_mk_string_symbol(ctx, CString::new("a").unwrap().as_ptr()), bool_sort);
    let b = Z3_mk_const(ctx,
            Z3_mk_string_symbol(ctx, CString::new("b").unwrap().as_ptr()), bool_sort);
    let c = Z3_mk_const(ctx,
            Z3_mk_string_symbol(ctx, CString::new("c").unwrap().as_ptr()), bool_sort);
     
    //Z3_fixedpoint_register_relation(ctx, dl, Z3_to_func_decl(ctx, a));
    Z3_fixedpoint_register_relation(ctx, dl, Z3_get_app_decl(ctx, Z3_to_app(ctx, a)));
    Z3_fixedpoint_register_relation(ctx, dl, Z3_get_app_decl(ctx, Z3_to_app(ctx, b)));
    Z3_fixedpoint_register_relation(ctx, dl, Z3_get_app_decl(ctx, Z3_to_app(ctx, c)));
 
    //SWIGTYPE_p__Z3_ast rule1 = Z3.Z3_mk_implies(ctx, a, b);
    //Z3.Z3_assert_cnstr(ctx, a);
    Z3_fixedpoint_add_rule(ctx, dl, Z3_mk_implies(ctx, a, b), Z3_mk_int_symbol(ctx, 1));
    Z3_fixedpoint_add_rule(ctx, dl, Z3_mk_implies(ctx, b, c), Z3_mk_int_symbol(ctx, 2));
     
    let mut r = Z3_fixedpoint_query(ctx, dl, c); //bool

    if r == 1 {
        println!("SAT");
    } else if r == -1 {
        println!("UNSAT");
    } else {
        println!("UNDEF");
    };
 
    Z3_fixedpoint_add_rule(ctx, dl, a, Z3_mk_int_symbol(ctx, 3));
    r = Z3_fixedpoint_query(ctx, dl, c);
    
     if r == 1 {
        println!("SAT");
    } else if r == -1 {
        println!("UNSAT");
    } else {
        println!("UNDEF");
    };
 
    // println!("{}", solv_to_string_z3!(&ctx, &)

    let answer = Z3_fixedpoint_get_answer(ctx, dl);
     
    println!("{}", CStr::from_ptr(Z3_ast_to_string(ctx, answer)).to_str().unwrap().to_owned());
    // println!("{:?}", CStr::from_ptr(Z3_fixedpoint_to_string(ctx, dl, 3, []).to_str().unwrap().to_owned()));
 
    Z3_fixedpoint_dec_ref(ctx, dl);
    Z3_del_context(ctx);
    }
}