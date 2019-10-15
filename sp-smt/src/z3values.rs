//! Z3 values for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use std::f64;
use super::*;

pub struct BoolZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub s: String,
    pub r: Z3_ast
}

pub struct IntZ3<'ctx, 'isrt> {
    pub ctx: &'ctx ContextZ3,
    pub isrt: &'isrt IntSortZ3<'ctx>,
    pub s: String,
    pub r: Z3_ast
}

pub struct RealZ3<'ctx, 'rsrt> {
    pub ctx: &'ctx ContextZ3,
    pub rsrt: &'rsrt RealSortZ3<'ctx>,
    pub s: String,
    pub r: Z3_ast
}

impl <'ctx> BoolZ3<'ctx> {
    /// Create an AST node representing `true` or 'false'.
    pub fn new(ctx: &'ctx ContextZ3, val: bool) -> BoolZ3<'ctx> {
        let z3 = if val == true { unsafe {
                Z3_mk_true(ctx.r)
            }} else { unsafe { 
                Z3_mk_false(ctx.r)
            }
        };
        BoolZ3 {
            ctx,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx, 'isrt> IntZ3<'ctx, 'isrt> {
    /// Create a numeral of an int, bit-vector, or finite-domain sort.
    ///
    /// This function can be use to create numerals that fit in a machine integer.
    /// It is slightly faster than [`Z3_mk_numeral`](fn.Z3_mk_numeral.html) since it is not necessary to parse a string.
    ///
    /// # See also:
    ///
    /// - [`Z3_mk_numeral`](fn.Z3_mk_numeral.html)
    pub fn new(ctx: &'ctx ContextZ3, isrt: &'isrt IntSortZ3<'ctx>, val: i32) -> IntZ3<'ctx, 'isrt> {
        let z3 = unsafe {
            Z3_mk_int(ctx.r, val, isrt.r)
        };
        IntZ3 {
            ctx,
            isrt,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

impl <'ctx, 'rsrt> RealZ3<'ctx, 'rsrt> {
    /// Create a real from a rust float.
    ///
    /// - `ctx`: logical context.
    /// - `rsrt`: real sort.
    /// - `val`: float to be realized.
    ///
    /// # See also:
    ///
    /// - [`Z3_mk_numeral`](fn.Z3_mk_numeral.html)
    /// - [`Z3_mk_int`](fn.Z3_mk_int.html)
    /// - [`Z3_mk_unsigned_int`](fn.Z3_mk_unsigned_int.html)
    pub fn new(ctx: &'ctx ContextZ3, rsrt: &'rsrt RealSortZ3<'ctx>, val: f64) -> RealZ3<'ctx, 'rsrt> {
        let num_string = val.to_string();
        let cstring = CString::new(num_string).unwrap();
        let z3 = unsafe {
            Z3_mk_numeral(ctx.r, cstring.as_ptr(), rsrt.r)
        };
        RealZ3 {
            ctx,
            rsrt,
            r: z3,
            s: unsafe {
                CStr::from_ptr(Z3_ast_to_string(ctx.r, z3)).to_str().unwrap().to_owned()
            }
        }
    }
}

#[test]
fn test_new_bool_val(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);

    let t = BoolZ3::new(&ctx, true);
    let f = BoolZ3::new(&ctx, false);

    let stringt = t.s;
    let stringf = f.s;

    assert_eq!("true", stringt);
    assert_eq!("false", stringf);
}

#[test]
fn test_new_int_val(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012390);

    let string1 = int1.s;
    let string2 = int2.s;

    assert_eq!("7", string1);
    assert_eq!("(- 1012390)", string2);
}

#[test]
fn test_new_real_val(){
    unsafe {
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let realsort = RealSortZ3::new(&ctx);

        let r1 = 7.361928;
        let r2 = -236487.098364;

        let real1 = RealZ3::new(&ctx, &realsort, r1);
        let real2 = RealZ3::new(&ctx, &realsort, r2);

        let real1numast = Z3_get_numerator(ctx.r, real1.r);
        let real2numast = Z3_get_numerator(ctx.r, real2.r);
        let real1denast = Z3_get_denominator(ctx.r, real1.r);
        let real2denast = Z3_get_denominator(ctx.r, real2.r);

        let mut real1numstring = CStr::from_ptr(Z3_ast_to_string(ctx.r, real1numast)).to_str().unwrap().to_owned();
        let mut real2numstring = CStr::from_ptr(Z3_ast_to_string(ctx.r, real2numast)).to_str().unwrap().to_owned();
        let mut real1denstring = CStr::from_ptr(Z3_ast_to_string(ctx.r, real1denast)).to_str().unwrap().to_owned();
        let mut real2denstring = CStr::from_ptr(Z3_ast_to_string(ctx.r, real2denast)).to_str().unwrap().to_owned();

        real1numstring.retain(|c| c!='(');
        real1numstring.retain(|c| c!=')');
        real2numstring.retain(|c| c!='(');
        real2numstring.retain(|c| c!=')');
        real1denstring.retain(|c| c!='(');
        real1denstring.retain(|c| c!=')');
        real2denstring.retain(|c| c!='(');
        real2denstring.retain(|c| c!=')');
        real1numstring.retain(|c| c!=' ');
        real2numstring.retain(|c| c!=' ');
        real1denstring.retain(|c| c!=' ');
        real2denstring.retain(|c| c!=' ');

        let real1num: i64 = real1numstring.parse().unwrap();
        let real2num: i64 = real2numstring.parse().unwrap();
        let real1den: i64 = real1denstring.parse().unwrap();
        let real2den: i64 = real2denstring.parse().unwrap();

        let real1n = real1num as f64 / real1den as f64;
        let real2n = real2num as f64 / real2den as f64;

        let real1nstr = real1n.to_string();
        let real2nstr = real2n.to_string();

        let what1 = Z3_get_sort(ctx.r, real1.r);
        let what2 = Z3_get_sort(ctx.r, real2.r);
        let whatstring1 = CStr::from_ptr(Z3_sort_to_string(ctx.r, what1)).to_str().unwrap().to_owned();
        let wahtstring2 = CStr::from_ptr(Z3_sort_to_string(ctx.r, what2)).to_str().unwrap().to_owned();

        assert_eq!(real1nstr, r1.to_string());
        assert_eq!(real2nstr, r2.to_string());
        assert_eq!("Real", whatstring1);
        assert_eq!("Real", wahtstring2);
    }
}