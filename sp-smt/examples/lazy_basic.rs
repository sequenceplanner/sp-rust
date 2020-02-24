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

    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, fruit_0, fruits[0]));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, fruit_1, fruits[1]));

    let clauses_ast_vec = SlvGetAssertsZ3::new(&ctx, &slv);
    let clauses = Z3AstVectorToVectorAstZ3::new(&ctx, clauses_ast_vec);

    

    for i in 0..clauses.len() {
        println!("{:?} : {}", i, AstToStringZ3::new(&ctx, clauses[i]));
    }   
}