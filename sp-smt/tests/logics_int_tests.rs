use sp_smt::*;

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
fn test_xor_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let xor1 = xor_z3!(&ctx,
        bool_var_z3!(&ctx, "x"),
        bool_var_z3!(&ctx, "y")
    );
    assert_eq!("(xor x y)", ast_to_string_z3!(&ctx, xor1));
}