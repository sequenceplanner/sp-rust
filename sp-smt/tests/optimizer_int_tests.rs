use sp_smt::*;

#[test]
fn test_new_optimizer(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);
    assert_eq!("(check-sat)\n", opt_to_string_z3!(&ctx, &opt));
}

#[test]
fn test_new_oassert(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, y, real1);

    OptAssertZ3::new(&ctx, &opt, rel1);

    println!("Now we have an assert in the opt context");
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    println!("Model: Should be empty, no check yet.");
    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_new_ocheck(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);
 
    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);
 
    let rel1 = EQZ3::new(&ctx, y, real1);

    println!("Should print empty string, opt context is still empty.");
    assert_eq!("(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    OptAssertZ3::new(&ctx, &opt, rel1);

    println!("Now we have an assert in the opt context");
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = OptCheckZ3::new(&ctx, &opt, vec!());
    println!("This is the return of the check:");
    println!("{}", res1);

    println!("This is the opt context with an assertion after the check:");
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    println!("Model: Should print the solution, we did a check.");
    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("y -> (- (/ 271549371.0 500000.0))\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_new_maximize(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let int1 = IntZ3::new(&ctx, &intsort, 100);

    let lt1 = LTZ3::new(&ctx, x, int1);

    OptAssertZ3::new(&ctx, &opt, lt1);
    OptMaximizeZ3::new(&ctx, &opt, x);
    OptCheckZ3::new(&ctx, &opt, vec!());

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("x -> 99\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_new_minimize(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let int1 = IntZ3::new(&ctx, &intsort, 11);

    let gt1 = GTZ3::new(&ctx, x, int1);

    OptAssertZ3::new(&ctx, &opt, gt1);
    OptMinimizeZ3::new(&ctx, &opt, x);
    OptCheckZ3::new(&ctx, &opt, vec!());

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("x -> 12\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_opt_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let opt = optz3!(&ctx);
    assert_eq!("(check-sat)\n", opt_to_string_z3!(&ctx, &opt));
}

#[test]
fn test_oot_assert_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let opt = optz3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        eqz3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, -543.098742)
        )
    );

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));
}

#[test]
fn test_ocheck_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let opt = optz3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        eqz3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, -543.098742)
        )
    );

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = opt_check_z3!(&ctx, &opt, );
    println!("{}", res1);
}

#[test]
fn test_omax_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let opt = optz3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        ltz3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 100.0)
        )
    );
    opt_maximize_z3!(&ctx, &opt, real_var_z3!(&ctx, "y"));

    assert_eq!("(declare-fun y () Real)
(assert (< y 100.0))
(maximize y)
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = opt_check_z3!(&ctx, &opt, );
    println!("{}", res1);

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("y -> 99.0\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_omin_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let opt = optz3!(&ctx);

    opt_assert_z3!(&ctx, &opt,
        gtz3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, 11.0)
        )
    );
    opt_minimize_z3!(&ctx, &opt, real_var_z3!(&ctx, "y"));

    assert_eq!("(declare-fun y () Real)
(assert (> y 11.0))
(minimize y)
(check-sat)\n", opt_to_string_z3!(&ctx, &opt));

    let res1 = opt_check_z3!(&ctx, &opt, );
    println!("{}", res1);

    let model = opt_get_model_z3!(&ctx, &opt);
    assert_eq!("y -> 12.0\n", model_to_string_z3!(&ctx, model));
}