use sp_smt::*;

use std::ffi::{CStr, CString};
use z3_sys::*;

fn main() {

    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);
  
    let fruit_sort = EnumSortZ3::new(&ctx, "fruit", vec!("apple", "banana", "orange", "avocado"));

    let fruits = &fruit_sort.enum_asts;

    let fruity = EnumVarZ3::new(&ctx, fruit_sort.r, "fruity");

    let asrt1 = eq_z3!(&ctx, fruity, fruits[0]);
    let asrt2 = eq_z3!(&ctx, fruity, fruits[1]);
    let asrt3 = eq_z3!(&ctx, fruity, fruits[2]);
    let asrt4 = eq_z3!(&ctx, fruity, fruits[3]);

    slv_assert_z3!(&ctx, &slv, or_z3!(&ctx, asrt1, asrt2, asrt3, asrt4));

    println!("{}", slv_to_string_z3!(&ctx, &slv));

    let res1 = slv_check_z3!(&ctx, &slv);
    if res1 == 1 {
        println!("SAT");
    } else if res1 == -1 {
        println!("UNSAT");
    } else {
        println!("UNDEF");
    };

    // let model = slv_get_model_z3!(&ctx, &slv);
    // println!("{}", model_to_string_z3!(&ctx, model));

    let models = SlvGetAllModelsZ3::new(&ctx, &slv).s;

    for model in models {
        println!("{}", model);
    }
}