use sp_smt::*;

#[test]
fn test_new_mul(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");
    let x4 = RealVarZ3::new(&ctx, &realsort, "x4");

    let mul1 = MULZ3::new(&ctx, vec!(x1, x2, x3, x4, int1, int2, real1, real2));

    assert_eq!("(* (to_real x1)
   (to_real x2)
   x3
   x4
   (to_real 7)
   (to_real (- 1012))
   (/ 920241.0 125000.0)
   (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(&ctx, mul1));
}

#[test]
fn test_new_div(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");

    let div1 = DIVZ3::new(&ctx, int1, int2);
    let div2 = DIVZ3::new(&ctx, int1, real2);
    let div3 = DIVZ3::new(&ctx, x1, real2);
    let div4 = DIVZ3::new(&ctx, x3, int1);
    let div5 = DIVZ3::new(&ctx, x3, real1);

    assert_eq!("(div 7 (- 1012))", ast_to_string_z3!(&ctx, div1));
    assert_eq!("(div 7 (to_int (- (/ 59024591.0 250000.0))))", ast_to_string_z3!(&ctx, div2));
    assert_eq!("(div x1 (to_int (- (/ 59024591.0 250000.0))))", ast_to_string_z3!(&ctx, div3));
    assert_eq!("(/ x3 (to_real 7))", ast_to_string_z3!(&ctx, div4));
    assert_eq!("(/ x3 (/ 920241.0 125000.0))", ast_to_string_z3!(&ctx, div5));
}

#[test]
fn test_new_mod(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");

    let mod1 = MODZ3::new(&ctx, int1, int2);
    let mod2 = MODZ3::new(&ctx, x1, x2);
    let mod3 = MODZ3::new(&ctx, x1, int1);
    let mod4 = MODZ3::new(&ctx, int2, x2);

    assert_eq!("(mod 7 (- 1012))", ast_to_string_z3!(&ctx, mod1));
    assert_eq!("(mod x1 x2)", ast_to_string_z3!(&ctx, mod2));
    assert_eq!("(mod x1 7)", ast_to_string_z3!(&ctx, mod3));
    assert_eq!("(mod (- 1012) x2)", ast_to_string_z3!(&ctx, mod4));
}

#[test]
fn test_new_rem(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");

    let rem1 = REMZ3::new(&ctx, int1, int2);
    let rem2 = REMZ3::new(&ctx, x1, x2);
    let rem3 = REMZ3::new(&ctx, x1, int1);
    let rem4 = REMZ3::new(&ctx, int2, x2);

    assert_eq!("(rem 7 (- 1012))", ast_to_string_z3!(&ctx, rem1));
    assert_eq!("(rem x1 x2)", ast_to_string_z3!(&ctx, rem2));
    assert_eq!("(rem x1 7)", ast_to_string_z3!(&ctx, rem3));
    assert_eq!("(rem (- 1012) x2)", ast_to_string_z3!(&ctx, rem4));
}

#[test]
fn test_new_add(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");
    let x4 = RealVarZ3::new(&ctx, &realsort, "x4");

    let add1 = ADDZ3::new(&ctx, vec!(x1, x2, x3, x4, int1, int2, real1, real2));

    assert_eq!("(+ (to_real x1)
   (to_real x2)
   x3
   x4
   (to_real 7)
   (to_real (- 1012))
   (/ 920241.0 125000.0)
   (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(&ctx, add1));
}

#[test]
fn test_new_sub(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");
    let x3 = RealVarZ3::new(&ctx, &realsort, "x3");
    let x4 = RealVarZ3::new(&ctx, &realsort, "x4");

    let sub1 = SUBZ3::new(&ctx, vec!(x1, x2, x3, x4, int1, int2, real1, real2));

    assert_eq!("(- (- (- (- (- (- (to_real (- x1 x2)) x3) x4) (to_real 7)) (to_real (- 1012)))
      (/ 920241.0 125000.0))
   (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(&ctx, sub1));
}

#[test]
fn test_new_neg(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);
    let realsort = RealSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);
    let real1 = RealZ3::new(&ctx, &realsort, 7.361928);
    let real2 = RealZ3::new(&ctx, &realsort, -236.098364);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");

    let neg1 = NEGZ3::new(&ctx, int1);
    let neg2 = NEGZ3::new(&ctx, int2);
    let neg3 = NEGZ3::new(&ctx, real1);
    let neg4 = NEGZ3::new(&ctx, real2);
    let neg5 = NEGZ3::new(&ctx, x1);

    assert_eq!("(- 7)", ast_to_string_z3!(&ctx, neg1));
    assert_eq!("(- (- 1012))", ast_to_string_z3!(&ctx, neg2));
    assert_eq!("(- (/ 920241.0 125000.0))", ast_to_string_z3!(&ctx, neg3));
    assert_eq!("(- (- (/ 59024591.0 250000.0)))", ast_to_string_z3!(&ctx, neg4));
    assert_eq!("(- x1)", ast_to_string_z3!(&ctx, neg5));
}

#[test]
fn test_new_pow(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let intsort = IntSortZ3::new(&ctx);

    let int1 = IntZ3::new(&ctx, &intsort, 7);
    let int2 = IntZ3::new(&ctx, &intsort, -1012);

    let x1 = IntVarZ3::new(&ctx, &intsort, "x1");
    let x2 = IntVarZ3::new(&ctx, &intsort, "x2");

    let pow1 = POWZ3::new(&ctx, int1, int2);
    let pow2 = POWZ3::new(&ctx, x1, x2);
    let pow3 = POWZ3::new(&ctx, x1, int1);
    let pow4 = POWZ3::new(&ctx, int2, x2);

    assert_eq!("(^ 7 (- 1012))", ast_to_string_z3!(&ctx, pow1));
    assert_eq!("(^ x1 x2)", ast_to_string_z3!(&ctx, pow2));
    assert_eq!("(^ x1 7)", ast_to_string_z3!(&ctx, pow3));
    assert_eq!("(^ (- 1012) x2)", ast_to_string_z3!(&ctx, pow4));
}

#[test]
fn test_mul_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let some = vec!(
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y")
    );
    let mul1 = mul_z3!(&ctx, some);
    assert_eq!("(* x 142 y)", ast_to_string_z3!(&ctx, mul1));
}

#[test]
fn test_mul_macro_2(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let mul1 = mul_z3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y")
    );
    assert_eq!("(* x 142 y)", ast_to_string_z3!(&ctx, mul1));
}

#[test]
fn test_div_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let div1 = div_z3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(div x 142)", ast_to_string_z3!(&ctx, div1));
}

#[test]
fn test_mod_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let mod1 = mod_z3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(mod x 142)", ast_to_string_z3!(&ctx, mod1));
}

#[test]
fn test_rem_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let rem1 = rem_z3!(&ctx,
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(rem x 142)", ast_to_string_z3!(&ctx, rem1));
}

#[test]
fn test_add_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let some = vec!(
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y"),
        int_z3!(&ctx, 1213442)
    );
    let add1 = add_z3!(&ctx, some);
    assert_eq!("(+ x 142 y 1213442)", ast_to_string_z3!(&ctx, add1));
}

#[test]
fn test_add_macro_2(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let add1 = add_z3!(&ctx, 
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y"),
        int_z3!(&ctx, 1213442)
    );
    assert_eq!("(+ x 142 y 1213442)", ast_to_string_z3!(&ctx, add1));
}

#[test]
fn test_sub_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let some = vec!(
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y"),
        int_z3!(&ctx, 1213442)
    );
    let sub1 = sub_z3!(&ctx, some);
    assert_eq!("(- (- (- x 142) y) 1213442)", ast_to_string_z3!(&ctx, sub1));
}

#[test]
fn test_sub_macro_2(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let sub1 = sub_z3!(&ctx, 
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142),
        int_var_z3!(&ctx, "y"),
        int_z3!(&ctx, 1213442)
    );
    assert_eq!("(- (- (- x 142) y) 1213442)", ast_to_string_z3!(&ctx, sub1));
}

#[test]
fn test_neg_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let neg1 = neg_z3!(&ctx,
        int_z3!(&ctx, 1213442)
    );
    assert_eq!("(- 1213442)", ast_to_string_z3!(&ctx, neg1));
}

#[test]
fn test_pow_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let pow1 = pow_z3!(&ctx, 
        int_var_z3!(&ctx, "x"),
        int_z3!(&ctx, 142)
    );
    assert_eq!("(^ x 142)", ast_to_string_z3!(&ctx, pow1));
}