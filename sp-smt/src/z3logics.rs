//! Z3 logical operations for SP

use std::ffi::{CStr};
use z3_sys::*;
use super::*;

pub struct ANDZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct ORZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct DISTINCTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub args: Vec<Z3_ast>,
    pub r: Z3_ast
}

pub struct NOTZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub arg: Z3_ast,
    pub r: Z3_ast
}

pub struct ITEZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub ifz3: Z3_ast,
    pub thenz3: Z3_ast,
    pub elsez3: Z3_ast,
    pub r: Z3_ast
}

pub struct IFFZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

pub struct IMPZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

pub struct EQUIVZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

pub struct NTRUEZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_ast
}

pub struct XORZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub left: Z3_ast,
    pub right: Z3_ast,
    pub r: Z3_ast
}

impl<'ctx> ANDZ3<'ctx> {
    /// Create an AST node representing `args[0] and ... and args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have Boolean sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    /// 
    /// NOTE: See macro! `and_z3!`
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> Z3_ast {
        let args_slice = &args;
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_and(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        ANDZ3{ctx, r: z3, args}.r   
    }
}

impl<'ctx> ORZ3<'ctx> {
    /// Create an AST node representing `args[0] or ... or args[num_args-1]`.
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have Boolean sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    /// 
    /// NOTE: See macro! `or_z3!`
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> Z3_ast {
        let args_slice = &args;
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_or(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        ORZ3 {ctx, r: z3, args}.r       
    }
}

impl<'ctx> DISTINCTZ3<'ctx> {
    /// Create an AST node representing distinct(args[0], ..., args[num_args-1]). 
    ///
    /// The `args` arg is a rust vector.
    /// All arguments must have the same sort.
    ///
    /// NOTE: The number of arguments must be greater than zero.
    /// 
    /// NOTE: See macro! `distinct_z3!`
    pub fn new(ctx: &'ctx ContextZ3, args: Vec<Z3_ast>) -> Z3_ast {
        let args_slice = &args;
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_distinct(ctx.r, args_slice.len() as u32, args_slice.as_ptr())
        };
        DISTINCTZ3 {ctx, r: z3, args}.r       
    }
}

impl<'ctx> NOTZ3<'ctx> {
    /// Create an AST node representing `not(arg)`.
    ///
    /// NOTE: The node `arg` must have Boolean sort.
    /// 
    /// NOTE: See macro! `not_z3!`
    pub fn new(ctx: &'ctx ContextZ3, arg: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_not(ctx.r, arg)
        };
        NOTZ3 {ctx, r: z3, arg}.r  
    }
}

impl<'ctx> ITEZ3<'ctx> {
    /// Create an AST node representing an if-then-else: `ite(ifz3, thenz3, elsez3)`.
    ///
    /// NOTE: The node `ifz3` must be of Boolean sort, `thenz3` and `elsez3` must have the same sort.
    /// The sort of the new node is equal to the sort of `thenz3` and `elsez3`.
    /// 
    /// NOTE: See macro! `ite_z3!`
    pub fn new(ctx: &'ctx ContextZ3, ifz3: Z3_ast, thenz3: Z3_ast, elsez3: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            // Z3_MUTEX.lock().unwrap();
            Z3_mk_ite(ctx.r, ifz3, thenz3, elsez3)
        };
        ITEZ3 {ctx, ifz3, thenz3, elsez3, r: z3}.r
    }
}

impl<'ctx> IFFZ3<'ctx> {
    /// Create an AST node representing `left iff right`.
    ///
    /// NOTE: The nodes `left` and `right` must have Boolean sort.
    /// 
    /// NOTE: See macro! `iff_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_iff(ctx.r, left, right)
        };
        IFFZ3 {ctx, left, right, r: z3}.r        
    }
}

impl<'ctx> IMPZ3<'ctx> {
    /// Create an AST node representing `left implies right`.
    ///
    /// NOTE: The nodes `left` and `right` must have Boolean sort.
    /// 
    /// NOTE: See macro! `imp_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_implies(ctx.r, left, right)
        };
        IMPZ3 {ctx, left, right, r: z3}.r       
    }
}

impl<'ctx> EQUIVZ3<'ctx> {
    /// Create an AST node representing `left implies right and right implies left`.
    ///
    /// NOTE: The nodes `left` and `right` must have Boolean sort.
    /// 
    /// NOTE: See macro! `equiv_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            let vec = vec!(Z3_mk_implies(ctx.r, left, right), Z3_mk_implies(ctx.r, right, left));
            Z3_mk_and(ctx.r, 2 as u32, vec.as_ptr())
        };
        EQUIVZ3 {ctx, left, right, r: z3}.r       
    }
}

impl<'ctx> NTRUEZ3<'ctx> {
    /// Create an AST node representing `t1 and t2 and not t3 and not t4 and not ...`.
    ///
    /// NOTE: The nodes must have Boolean sort.
    /// 
    /// NOTE: See macro! `ntrue_z3!`
    pub fn new(ctx: &'ctx ContextZ3, t: Vec<Z3_ast>, f: Vec<Z3_ast>) -> Z3_ast {

        let z3 = unsafe {
           
            let mut falses_init = Vec::new();
            for fs in &f {
               falses_init.push(Z3_mk_not(ctx.r, *fs));
            }

            let together = [t, falses_init].concat();
            Z3_mk_and(ctx.r, together.len() as u32, together.as_ptr())
        };

        NTRUEZ3 {ctx, r: z3}.r       
    }
}

impl<'ctx> XORZ3<'ctx> {
    /// Create an AST node representing `left xor right`.
    ///
    /// NOTE: The nodes `left` and `right` must have Boolean sort.
    /// 
    /// NOTE: See macro! `xor_z3!`
    pub fn new(ctx: &'ctx ContextZ3, left: Z3_ast, right: Z3_ast) -> Z3_ast {
        let z3 = unsafe {
            Z3_mk_xor(ctx.r, left, right)
        };
        XORZ3 {ctx, left, right, r: z3}.r  
    }
}

/// a and b and c and ...
/// 
/// Macro rule for:
/// ```text
/// z3logics::ANDZ3::new(&ctx, vec!(a, b, c)).r
/// ```
// / Using the default context:
// / ```text
// / and_z3!(a, b, c)
// / ```
/// Using a specific context and passing elements:
/// ```text
/// and_z3!(&ctx, a, b, c)
/// ```
/// Or make a vector firts and pass it:
/// ```text
/// let some = vec!(a, b, c);
/// and_z3!(&ctx, some)
/// ```
/// Requires that a, b, c... are of Bool sort.
#[macro_export]
macro_rules! and_z3 {
    // ( $( $x:expr ),* ) => {
    //     {
    //         let mut temp_vec = Vec::new();
    //         $(
    //             temp_vec.push($x);
    //         )*
    //         ANDZ3::new(&CTX, temp_vec).r
    //     }
    // };
    ($ctx:expr, $b:expr) => {
        ANDZ3::new($ctx, $b)
    };
    ( $ctx:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            ANDZ3::new($ctx, temp_vec)
        }
    };
}

/// a or b or c or ...
/// 
/// Macro rule for:
/// ```text
/// z3logics::ORZ3::new(&ctx, vec!(a, b, c)).r
/// ```
// / Using the default context:
// / ```text
// / or_z3!(a, b, c)
// / ```
/// Using a specific context and passing elements:
/// ```text
/// or_z3!(&ctx, a, b, c)
/// ```
/// Or make a vector firts and pass it:
/// ```text
/// let some = vec!(a, b, c);
/// or_z3!(&ctx, some)
/// ```
/// Requires that a, b, c... are of Bool sort.
#[macro_export]
macro_rules! or_z3 {
    // ( $( $x:expr ),* ) => {
    //     {
    //         let mut temp_vec = Vec::new();
    //         $(
    //             temp_vec.push($x);
    //         )*
    //         ORZ3::new(&CTX, temp_vec).r
    //     }
    // };
    ($ctx:expr, $b:expr) => {
        ORZ3::new($ctx, $b)
    };
    ( $ctx:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            ORZ3::new($ctx, temp_vec)
        }
    };
}

/// all different (a, b, c...)
/// 
/// Macro rule for:
/// ```text
/// z3logics::DISTINCTZ3::new(&ctx, vec!(a, b, c)).r
/// ```
// / Using the default context:
// / ```text
// / distinct_z3!(a, b, c)
// / ```
/// Using a specific context and passing elements:
/// ```text
/// distinct_z3!(&ctx, a, b, c)
/// ```
/// Or make a vector firts and pass it:
/// ```text
/// let some = vec!(a, b, c);
/// distinct_z3!(&ctx, some)
/// ```
/// Requires that a, b, c... are of the same sort.
#[macro_export]
macro_rules! distinct_z3 {
    // ( $( $x:expr ),* ) => {
    //     {
    //         let mut temp_vec = Vec::new();
    //         $(
    //             temp_vec.push($x);
    //         )*
    //         DISTINCTZ3::new(&CTX, temp_vec).r
    //     }
    // };
    ($ctx:expr, $b:expr) => {
        DISTINCTZ3::new($ctx, $b)
    };
    ( $ctx:expr, $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            DISTINCTZ3::new($ctx, temp_vec)
        }
    };
}

/// not a
/// 
/// Macro rule for:
/// ```text
/// z3logics::NOTZ3::new(&ctx, a).r
/// ```
// / Using the default context:
// / ```text
// / not_z3!(a)
// / ```
/// Using a specific context:
/// ```text
/// not_z3!(&ctx, a)
/// ```
/// Requires that a is of Bool sort.
#[macro_export]
macro_rules! not_z3 {
    // ($a:expr) => {
    //     NOTZ3::new(&CTX, $a).r
    // };
    ($ctx:expr, $b:expr) => {
        NOTZ3::new($ctx, $b)
    }
}

/// if a then b else c
/// 
/// Macro rule for:
/// ```text
/// z3logics::ITEZ3::new(&ctx, a, b, c).r
/// ```
// / Using the default context:
// / ```text
// / ite_z3!(a, b, c)
// / ```
/// Using a specific context:
/// ```text
/// ite_z3!(&ctx, a, b, c)
/// ```
/// Requires that a is Bool sort and that b and c are of same sort.
#[macro_export]
macro_rules! ite_z3 {
    // ($a:expr, $b:expr, $c:expr) => {
    //     ITEZ3::new(&CTX, $a, $b, $c).r
    // };
    ($ctx:expr, $b:expr, $c:expr, $d:expr) => {
        ITEZ3::new($ctx, $b, $c, $d)
    }
}

/// a if and only if b
/// 
/// Macro rule for:
/// ```text
/// z3logics::IFFZ3::new(&ctx, a, b).r
/// ```
// / Using the default context:
// / ```text
// / iff_z3!(a, b)
// / ```
/// Using a specific context:
/// ```text
/// iff_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are Bool sort.
#[macro_export]
macro_rules! iff_z3 {
    // ($a:expr, $b:expr) => {
    //     IFFZ3::new(&CTX, $a, $b).r
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        IFFZ3::new($ctx, $b, $c)
    }
}

/// a implies b
/// 
/// Macro rule for:
/// ```text
/// z3logics::IMPZ3::new(&ctx, a, b).r
/// ```
// / Using the default context:
// / ```text
// / imp_z3!(a, b)
// / ```
/// Using a specific context:
/// ```text
/// imp_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are Bool sort.
#[macro_export]
macro_rules! imp_z3 {
    // ($a:expr, $b:expr) => {
    //     IMPZ3::new(&CTX, $a, $b).r
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        IMPZ3::new($ctx, $b, $c)
    }
}

/// a bi-implication b
/// 
/// Macro rule for:
/// ```text
/// z3logics::BIMPZ3::new(&ctx, a, b).r
/// ```
// / Using the default context:
// / ```text
// / bimp_z3!(a, b)
// / ```
/// Using a specific context:
/// ```text
/// bimp_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are Bool sort.
#[macro_export]
macro_rules! equiv_z3 {
    // ($a:expr, $b:expr) => {
    //     IMPZ3::new(&CTX, $a, $b).r
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        EQUIVZ3::new($ctx, $b, $c)
    }
}

/// either a or b
/// 
/// Macro rule for:
/// ```text
/// z3logics::XORZ3::new(&ctx, a, b).r
/// ```
// / Using the default context:
// / ```text
// / xor_z3!(a, b)
// / ```
/// Using a specific context:
/// ```text
/// xor_z3!(&ctx, a, b)
/// ```
/// Requires that a and b are Bool sort.
#[macro_export]
macro_rules! xor_z3 {
    // ($a:expr, $b:expr) => {
    //     XORZ3::new(&CTX, $a, $b).r
    // };
    ($ctx:expr, $b:expr, $c:expr) => {
        XORZ3::new($ctx, $b, $c)
    }
}

#[test]
fn test_new_and(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx,false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let and1 = ANDZ3::new(&ctx, vec!(x1, x2, bool1, bool2));

    assert_eq!("(and x1 x2 true false)", ast_to_string_z3!(&ctx, and1));
}

#[test]
fn test_new_or(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx,false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let or1 = ORZ3::new(&ctx, vec!(x1, x2, bool1, bool2));

    assert_eq!("(or x1 x2 true false)", ast_to_string_z3!(&ctx, or1));
}

#[test]
fn test_new_distinct(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");

    let dist1 = DISTINCTZ3::new(&ctx, vec!(x1, bool1));

    assert_eq!("(distinct x1 true)", ast_to_string_z3!(&ctx, dist1));
}

#[test]
fn test_new_not(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");

    // let not1 = NOTZ3::new(&ctx, vec!(bool1));
    // let not2 = NOTZ3::new(&ctx, vec!(x1));
    // let not3 = NOTZ3::new(&ctx, vec!(not2));

    let not1 = NOTZ3::new(&ctx, bool1);
    let not2 = NOTZ3::new(&ctx, x1);
    let not3 = NOTZ3::new(&ctx, not2);

    assert_eq!("(not true)", ast_to_string_z3!(&ctx, not1));
    assert_eq!("(not x1)", ast_to_string_z3!(&ctx, not2));
    assert_eq!("(not (not x1))", ast_to_string_z3!(&ctx, not3));
}

#[test]
fn test_new_ite(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let int1 = IntZ3::new(&ctx, &intsort, 3);
    let int2 = IntZ3::new(&ctx, &intsort, 7);

    let ite1 = ITEZ3::new(&ctx, bool1, int1, int2);
    let ite2 = ITEZ3::new(&ctx, bool2, int1, int2);
    
    assert_eq!("(ite true 3 7)", ast_to_string_z3!(&ctx, ite1));
    assert_eq!("(ite false 3 7)", ast_to_string_z3!(&ctx, ite2));
}

#[test]
fn test_new_iff(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let iff1 = IFFZ3::new(&ctx, bool1, bool2);
    let iff2 = IFFZ3::new(&ctx, bool1, x1);
    let iff3 = IFFZ3::new(&ctx, x2, bool2);
    let iff4 = IFFZ3::new(&ctx, x1, x2);
    
    assert_eq!("(= true false)", ast_to_string_z3!(&ctx, iff1));
    assert_eq!("(= true x1)", ast_to_string_z3!(&ctx, iff2));
    assert_eq!("(= x2 false)", ast_to_string_z3!(&ctx, iff3));
    assert_eq!("(= x1 x2)", ast_to_string_z3!(&ctx, iff4));
}

#[test]
fn test_new_imp(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let imp1 = IMPZ3::new(&ctx, bool1, bool2);
    let imp2 = IMPZ3::new(&ctx, bool1, x1);
    let imp3 = IMPZ3::new(&ctx, x2, bool2);
    let imp4 = IMPZ3::new(&ctx, x1, x2);
    
    assert_eq!("(=> true false)", ast_to_string_z3!(&ctx, imp1));
    assert_eq!("(=> true x1)", ast_to_string_z3!(&ctx, imp2));
    assert_eq!("(=> x2 false)", ast_to_string_z3!(&ctx, imp3));
    assert_eq!("(=> x1 x2)", ast_to_string_z3!(&ctx, imp4));
}


#[test]
fn test_new_equiv(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let equiv1 = EQUIVZ3::new(&ctx, x1, x2);

    assert_eq!("(and (=> x1 x2) (=> x2 x1))", ast_to_string_z3!(&ctx, equiv1));
}

#[test]
fn test_new_ntrue(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");
    let x3 = BoolVarZ3::new(&ctx, &boolsort, "x3");
    let x4 = BoolVarZ3::new(&ctx, &boolsort, "x4");
    let x5 = BoolVarZ3::new(&ctx, &boolsort, "x5");
    let x6 = BoolVarZ3::new(&ctx, &boolsort, "x6");

    let ntrue1 = NTRUEZ3::new(&ctx, vec!(x2), vec!(x1, x3, x4, x5, x6));

    assert_eq!("(and x2 (not x1) (not x3) (not x4) (not x5) (not x6))", ast_to_string_z3!(&ctx, ntrue1));
}

#[test]
fn test_new_ntrue_2(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");
    let x3 = BoolVarZ3::new(&ctx, &boolsort, "x3");
    let x4 = BoolVarZ3::new(&ctx, &boolsort, "x4");
    let x5 = BoolVarZ3::new(&ctx, &boolsort, "x5");
    let x6 = BoolVarZ3::new(&ctx, &boolsort, "x6");

    let ntrue1 = NTRUEZ3::new(&ctx, vec!(), vec!(x1, x2, x3, x4, x5, x6));

    assert_eq!("(and (not x1) (not x2) (not x3) (not x4) (not x5) (not x6))", ast_to_string_z3!(&ctx, ntrue1));
}

#[test]
fn test_new_xor(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let boolsort = BoolSortZ3::new(&ctx);

    let bool1 = BoolZ3::new(&ctx, true);
    let bool2 = BoolZ3::new(&ctx, false);

    let x1 = BoolVarZ3::new(&ctx, &boolsort, "x1");
    let x2 = BoolVarZ3::new(&ctx, &boolsort, "x2");

    let xor1 = XORZ3::new(&ctx, bool1, bool2);
    let xor2 = XORZ3::new(&ctx, bool1, x1);
    let xor3 = XORZ3::new(&ctx, x2, bool2);
    let xor4 = XORZ3::new(&ctx, x1, x2);
    
    assert_eq!("(xor true false)", ast_to_string_z3!(&ctx, xor1));
    assert_eq!("(xor true x1)", ast_to_string_z3!(&ctx, xor2));
    assert_eq!("(xor x2 false)", ast_to_string_z3!(&ctx, xor3));
    assert_eq!("(xor x1 x2)", ast_to_string_z3!(&ctx,xor4));
}

#[test]
fn test_and_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let some = vec!(
        bool_var_z3!(&ctx, "x"),
        bool_z3!(&ctx, true),
        bool_var_z3!(&ctx, "y")
    );
    let and1 = and_z3!(&ctx, some);
    assert_eq!("(and x true y)", ast_to_string_z3!(&ctx, and1));
}

#[test]
fn test_and_macro_2(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let and1 = and_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_z3!(&ctx, true),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(and x true y)", ast_to_string_z3!(&ctx, and1));
}

#[test]
fn test_or_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let or1 = or_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_z3!(&ctx, true),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(or x true y)", ast_to_string_z3!(&ctx, or1));
}

#[test]
fn test_or_macro_2(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let some = vec!(
        bool_var_z3!(&ctx, "x"),
        bool_z3!(&ctx, true),
        bool_var_z3!(&ctx, "y")
    );
    let or1 = or_z3!(&ctx, some);
    assert_eq!("(or x true y)", ast_to_string_z3!(&ctx, or1));
}

#[test]
fn test_distinct_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let dist1 = distinct_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_z3!(&ctx, true)
    );
    assert_eq!("(distinct x true)", ast_to_string_z3!(&ctx, dist1));
}

#[test]
fn test_distinct_macro_2(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let some = vec!(
        bool_var_z3!(&ctx, "x"),
        bool_z3!(&ctx, true)
    );
    let dist1 = distinct_z3!(&ctx, some);
    assert_eq!("(distinct x true)", ast_to_string_z3!(&ctx, dist1));
}

#[test]
fn test_distinct_macro_3(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);
    let some = vec!(
        int_var_z3!(&ctx, "x"),
        int_var_z3!(&ctx, "y"),
        int_var_z3!(&ctx, "z"),
        int_var_z3!(&ctx, "m"),
        int_z3!(&ctx, -1)
    );
    let dist1 = distinct_z3!(&ctx, some);
    slv_assert_z3!(&ctx, &slv, dist1);
    slv_check_z3!(&ctx, &slv);
    let model = slv_get_model_z3!(&ctx, &slv);
    println!("{}", model_to_string_z3!(&ctx, model));
    assert_eq!("z -> (- 3)
y -> (- 4)
x -> (- 5)
m -> (- 2)
", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_not_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let not1 = not_z3!(&ctx,
        bool_z3!(&ctx, true)
    );
    assert_eq!("(not true)", ast_to_string_z3!(&ctx, not1));
}

#[test]
fn test_ite_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let ite1 = ite_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_z3!(&ctx, true),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(ite x true y)", ast_to_string_z3!(&ctx, ite1));
}

#[test]
fn test_iff_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let iff1 = iff_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(= x y)", ast_to_string_z3!(&ctx, iff1));
}

#[test]
fn test_imp_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let imp1 = imp_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(=> x y)", ast_to_string_z3!(&ctx, imp1));
}

#[test]
fn test_bimp_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let equiv1 = equiv_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(and (=> x y) (=> y x))", ast_to_string_z3!(&ctx, equiv1));
}

#[test]
fn test_xor_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let xor1 = xor_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(xor x y)", ast_to_string_z3!(&ctx, xor1));
}