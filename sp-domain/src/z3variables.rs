//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::ptr;
use std::fmt;
use super::*;

/// TODO: add Sort struct and include it in the generalized var struct
pub struct IntVar<'ctx, 'a> {
    pub ctx: &'ctx Context,
    pub name: &'a str,
    pub var: Z3_ast,
    // pub sort: Z3_sort,
}

///Create an integer variable (took some time to figure out the lifetime stuff)
impl <'ctx, 'a> IntVar<'ctx, 'a> {
    pub fn new(ctx: &'ctx Context, name: &'a str) -> IntVar<'ctx, 'a> {
        IntVar {
            ctx,
            name,
            var: unsafe {
                let int_sort = Z3_mk_int_sort(ctx.context);
                let str_name = CString::new(name).unwrap();
                let sym_name = Z3_mk_string_symbol(ctx.context, str_name.as_ptr());
                let int_var = Z3_mk_const(ctx.context, sym_name, int_sort);
                int_var
            }
        }
    }
}

///TODO: write a default trait for IntVar
// impl <'ctx, 'a> Default for IntVar<'ctx, 'a> {
    // fn default() -> Self {
        // Self::new()
    // }
// }

#[test]
fn test_new_int_var(){
    unsafe {
        let _conf = Config::new();
        let _ctx = Context::new(&_conf);
        let _var = IntVar::new(&_ctx, "x");
        let _str_var = Z3_ast_to_string(_ctx.context, _var.var);
        println!("{:?}", CStr::from_ptr(_str_var).to_str().unwrap());
    }
}

//TODO: write a intvar default test
// #[test]
// fn test_default_intvar(){
    // }

//TODO: write a intvar drop test
// #[test]
// fn test_drop_solver(){
    // }

// references from the C api examples...

// Z3_ast mk_var(Z3_context ctx, const char * name, Z3_sort ty)
// {
//     Z3_symbol   s  = Z3_mk_string_symbol(ctx, name);
//     return Z3_mk_const(ctx, s, ty);
// }
// // 
// /**
//    \brief Create a boolean variable using the given name.
// */
// Z3_ast mk_bool_var(Z3_context ctx, const char * name)
// {
//     Z3_sort ty = Z3_mk_bool_sort(ctx);
//     return mk_var(ctx, name, ty);
// }
// 
// impl Var {
//     pub fn new(ctx: &Context, sort: &Sort, name: &str) -> Var {
//         Var {
//             sort: 
//         }
//     }
// }

// Z3_solver mk_solver(Z3_context ctx)
// {
//   Z3_solver s = Z3_mk_solver(ctx);
//   Z3_solver_inc_ref(ctx, s);
//   return s;
// }
// 
// void del_solver(Z3_context ctx, Z3_solver s)
// {
//   Z3_solver_dec_ref(ctx, s);
// }
