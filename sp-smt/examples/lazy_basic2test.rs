use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;
use sp_domain::*;

fn main() {

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);
    let prop_slv = slv_z3!(&ctx);
    let th_slv = slv_z3!(&ctx);

    let fruit_sort = EnumSortZ3::new(&ctx, "fruit", vec!("apple", "banana", "orange", "avocado"));
    let fruits = &fruit_sort.enum_asts;

    let fruit_0 = EnumVarZ3::new(&ctx, fruit_sort.r, "fruit_0");
    let fruit_1 = EnumVarZ3::new(&ctx, fruit_sort.r, "fruit_1");
    let fruit_2 = EnumVarZ3::new(&ctx, fruit_sort.r, "fruit_2");

    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, fruit_0, fruits[0]));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, fruit_1, fruits[1]));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, fruit_2, fruits[2]));
    // slv_assert_z3!(&ctx, &slv, and_z3!(&ctx, not_z3!(&ctx, eq_z3!(&ctx, fruit_0, fruit_1)), eq_z3!(&ctx, fruit_1, fruit_2)));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, fruit_0, fruit_1, fruit_2));

    let cls_ast_vec = SlvGetAssertsZ3::new(&ctx, &slv);
    let cls = Z3AstVectorToVectorAstZ3::new(&ctx, cls_ast_vec);

    // fn lazy_basic(clauses: Vec<Z3_ast>) -> String {
    //     for 
    //     let prop_formula = 
    // }

    for i in 0..cls.len() {
        println!("{:?} : {}", i, AstToStringZ3::new(&ctx, cls[i]));
    }   
    println!("====================");

    let cnf = GetCnfVectorZ3::new(&ctx, cls);
    for i in 0..cnf.len() {
        println!("{:?} : {}", i, AstToStringZ3::new(&ctx, cnf[i]));
    }  
    // println!("{}", cnf);
}