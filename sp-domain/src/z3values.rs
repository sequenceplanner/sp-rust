//! Z3 stuff for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::f64;
use super::*;

pub struct BoolZ3<'ctx, 'bsrt> {
    pub ctx: &'ctx ContextZ3,
    pub bsrt: &'bsrt BoolSortZ3<'ctx>,
    pub r: Z3_ast
}

pub struct IntZ3<'ctx, 'isrt> {
    pub ctx: &'ctx ContextZ3,
    pub isrt: &'isrt IntSortZ3<'ctx>,
    pub r: Z3_ast
}

pub struct RealZ3<'ctx, 'ssrt> {
    pub ctx: &'ctx ContextZ3,
    pub ssrt: &'ssrt StringSortZ3<'ctx>,
    pub r: Z3_ast
}

// pub struct StringZ3<'ctx, 'a> {
//     pub ctx: &'ctx ContextZ3,
//     pub string: &'a str,
//     pub r: Z3_string
// }

// bool sort maybe not needed here, remove
impl <'ctx, 'bsrt> BoolZ3<'ctx, 'bsrt> {
    pub fn new(ctx: &'ctx ContextZ3, bsrt: &'bsrt BoolSortZ3<'ctx>, val: bool) -> BoolZ3<'ctx, 'bsrt> {
        BoolZ3 {
            ctx,
            bsrt,
            r: unsafe {
                let bool_sort = bsrt.r;
                if val == true {
                    let bool = Z3_mk_true(ctx.r);
                    bool
                } else {
                    let bool = Z3_mk_false(ctx.r);
                    bool
                }
            }
        }
    }
}

impl <'ctx, 'isrt> IntZ3<'ctx, 'isrt> {
    pub fn new(ctx: &'ctx ContextZ3, isrt: &'isrt IntSortZ3<'ctx>, val: i32) -> IntZ3<'ctx, 'isrt> {
        IntZ3 {
            ctx,
            isrt,
            r: unsafe {
                let int = Z3_mk_int(ctx.r, val, isrt.r);
                int
            }
        }
    }
}

// impl <'ctx, 'a> StringZ3<'ctx, 'a> {
//     pub fn new(ctx: &'ctx Context, string: &'a str) -> StringZ3<'ctx, 'a> {
//         StringZ3 {
//             ctx,
//             string,
//             r: unsafe {
//                 let _str: Z3_string = string.as_ptr() as *const i8;
//                 _str
//             }
//         }
//     }
// }

// this looks a bit iffy...
//Why do we need to make real sort then? aha ok , for
// impl <'ctx, 'ssrt> RealZ3<'ctx, 'ssrt> {
//     pub fn new(ctx: &'ctx ContextZ3, ssrt: &'ssrt StringSortZ3<'ctx>, val: f64) -> RealZ3<'ctx, 'ssrt> {
//         Real {
//             ctx,
//             ssrt,
//             r: unsafe {
//                 let num_string = val.to_string();
//                 let string = Z3String::new(&ctx, &num_string);
//                 let _real = Z3_mk_numeral(ctx.context, string.string.as_ptr() as *const i8, ssrt.r);
//                 _real
//             }
//         }
//     }
// }

#[test]
fn test_new_int(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let intsort = IntSortZ3::new(&ctx);
        let seven = IntZ3::new(&ctx, &intsort, 7);
        let string = Z3_ast_to_string(ctx.r, seven.r);
        println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
    }
}

// #[test]
// fn test_new_real(){
//     unsafe {
//         let conf = Config::new();
//         let ctx = Context::new(&conf);
//         let string_sort = StringSort::new(&ctx);
//         let val = Real::new(&ctx, &string_sort,7.9874);
//         let string = Z3_ast_to_string(ctx.context, val.r);
//         println!("{:?}", CStr::from_ptr(string).to_str().unwrap());
//     }
// }
