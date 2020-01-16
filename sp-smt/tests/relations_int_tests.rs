use sp_smt::*;

#[test]
fn test_new_eq(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, x, int1);
    let rel2 = EQZ3::new(&ctx, y, real1);
    let rel3 = EQZ3::new(&ctx, y, x);
    let rel4 = EQZ3::new(&ctx, int1, real1);

    assert_eq!("(= x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(= y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(= y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(= (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
}

#[test]
fn test_new_le(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = LEZ3::new(&ctx, x, int1);
    let rel2 = LEZ3::new(&ctx, y, real1);
    let rel3 = LEZ3::new(&ctx, y, x);
    let rel4 = LEZ3::new(&ctx, int1, real1);

    assert_eq!("(<= x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(<= y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(<= y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(<= (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
}

#[test]
fn test_new_lt(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = LTZ3::new(&ctx, x, int1);
    let rel2 = LTZ3::new(&ctx, y, real1);
    let rel3 = LTZ3::new(&ctx, y, x);
    let rel4 = LTZ3::new(&ctx, int1, real1);

    assert_eq!("(< x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(< y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(< y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(< (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
}

#[test]
fn test_new_ge(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = GEZ3::new(&ctx, x, int1);
    let rel2 = GEZ3::new(&ctx, y, real1);
    let rel3 = GEZ3::new(&ctx, y, x);
    let rel4 = GEZ3::new(&ctx, int1, real1);

    assert_eq!("(>= x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(>= y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(>= y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(>= (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
}

#[test]
fn test_new_gt(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = GTZ3::new(&ctx, x, int1);
    let rel2 = GTZ3::new(&ctx, y, real1);
    let rel3 = GTZ3::new(&ctx, y, x);
    let rel4 = GTZ3::new(&ctx, int1, real1);

    assert_eq!("(> x 7)", ast_to_string_z3!(&ctx, rel1));
    assert_eq!("(> y (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel2));
    assert_eq!("(> y (to_real x))", ast_to_string_z3!(&ctx, rel3));
    assert_eq!("(> (to_real 7) (- (/ 271549371.0 500000.0)))", ast_to_string_z3!(&ctx, rel4));
}

#[test]
fn test_eq_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let eq1 = eq_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(= y 11.0)", ast_to_string_z3!(&ctx, eq1));
}

#[test]
fn test_lt_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let lt1 = lt_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(< y 11.0)", ast_to_string_z3!(&ctx, lt1));
}

#[test]
fn test_gt_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let gt1 = gt_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(> y 11.0)", ast_to_string_z3!(&ctx, gt1));
}

#[test]
fn test_ge_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let ge1 = ge_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(>= y 11.0)", ast_to_string_z3!(&ctx, ge1));
}

#[test]
fn test_le_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let le1 = le_z3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
    );    
    assert_eq!("(<= y 11.0)", ast_to_string_z3!(&ctx, le1));
}