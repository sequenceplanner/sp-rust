use sp_smt::*;

#[test]
fn test_new_bool(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);

    let t = BoolZ3::new(&ctx, true);
    let f = BoolZ3::new(&ctx, false);

    assert_eq!("true", ast_to_string_z3!(&ctx, t));
    assert_eq!("false", ast_to_string_z3!(&ctx, f));
}

#[test]
fn test_new_int(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012390);

    assert_eq!("7", ast_to_string_z3!(&ctx, int1));
    assert_eq!("(- 1012390)", ast_to_string_z3!(&ctx, int2));
}

#[test]
fn test_new_string(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);

    let str1 = StringZ3::new(&ctx, "hello!@#$ @# $%@#$ ");

    assert_eq!("\"hello!@#$ @# $%@#$ \"", ast_to_string_z3!(&ctx, str1));
}

#[test]
fn test_bool_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let bool1 = bool_z3!(&ctx, true);
    assert_eq!("true", ast_to_string_z3!(&ctx, bool1));
    assert_eq!("Bool", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, bool1)));
}

#[test]
fn test_int_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let int1 = int_z3!(&ctx, 76);
    assert_eq!("76", ast_to_string_z3!(&ctx, int1));
    assert_eq!("Int", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, int1)));
}

#[test]
fn test_real_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let real1 = real_z3!(&ctx, 76.456);
    assert_eq!("(/ 9557.0 125.0)", ast_to_string_z3!(&ctx, real1));
    assert_eq!("Real", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, real1)));
}

#[test]
fn test_string_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let string1 = string_z3!(&ctx, "asdf_ASDF_!@#$");
    assert_eq!("\"asdf_ASDF_!@#$\"", ast_to_string_z3!(&ctx, string1));
    assert_eq!("String", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, string1)));
}