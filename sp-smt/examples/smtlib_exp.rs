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
}