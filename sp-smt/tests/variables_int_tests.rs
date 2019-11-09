use sp_smt::*;

#[test]
fn test_new_bool_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = BoolSortZ3::new(&ctx);

    let x = BoolVarZ3::new(&ctx, &sort, "x");
    let y = BoolVarZ3::new(&ctx, &sort, "y");

    assert_eq!("x", ast_to_string_z3!(&ctx, x));
    assert_eq!("y", ast_to_string_z3!(&ctx, y));
}

#[test]
fn test_new_int_var(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = IntSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &sort, "x");
    let y = IntVarZ3::new(&ctx, &sort, "y");

    assert_eq!("x", ast_to_string_z3!(&ctx, x));
    assert_eq!("y", ast_to_string_z3!(&ctx, y));
}

#[test]
fn test_new_real_var(){
        let conf = ConfigZ3::new();
        let ctx = ContextZ3::new(&conf);
        let sort = RealSortZ3::new(&ctx);

        let x = RealVarZ3::new(&ctx, &sort, "x");
        let y = RealVarZ3::new(&ctx, &sort, "y");

        assert_eq!("x", ast_to_string_z3!(&ctx, x));
        assert_eq!("y", ast_to_string_z3!(&ctx, y));
}

#[test]
fn test_bool_var_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let bool1 = bool_var_z3!(&ctx, "x");
    assert_eq!("x", ast_to_string_z3!(&ctx, bool1));
    assert_eq!("Bool", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, bool1)));
}

#[test]
fn test_int_var_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let int1 = int_var_z3!(&ctx, "x");
    assert_eq!("x", ast_to_string_z3!(&ctx, int1));
    assert_eq!("Int", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, int1)));
}

#[test]
fn test_real_var_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let real1 = real_var_z3!(&ctx, "x");
    assert_eq!("x", ast_to_string_z3!(&ctx, real1));
    assert_eq!("Real", sort_to_string_z3!(&ctx, get_sort_z3!(&ctx, real1)));
}