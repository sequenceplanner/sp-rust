use sp_smt::*;

#[test]
fn test_new_solver(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);
    assert_eq!("", slv_to_string_z3!(&ctx, &solv));
}

#[test]
fn test_new_solver_assert(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);

    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, y, real1);

    SlvAssertZ3::new(&ctx, &solv, rel1);
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))\n", slv_to_string_z3!(&ctx, &solv));
}

#[test]
fn test_new_solver_assert_and_track(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);

    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, y, real1);

    SlvAssertAndTrackZ3::new(&ctx, &solv, rel1, "track");
    assert_eq!("(declare-fun y () Real)
(declare-fun track () Bool)
(assert (=> track (= y (- (/ 271549371.0 500000.0)))))\n", slv_to_string_z3!(&ctx, &solv));
}

#[test]
fn test_new_scheck(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);

    let realsort = RealSortZ3::new(&ctx);
    let y = RealVarZ3::new(&ctx, &realsort, "y");
    let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

    let rel1 = EQZ3::new(&ctx, y, real1);

    assert_eq!("", slv_to_string_z3!(&ctx, &solv));

    SlvAssertZ3::new(&ctx, &solv, rel1);
    
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))\n", slv_to_string_z3!(&ctx, &solv));

    let res1 = SlvCheckZ3::new(&ctx, &solv);

    assert_eq!(1, res1);

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(rmodel->model-converter-wrapper
y -> (- (/ 271549371.0 500000.0))
)\n", slv_to_string_z3!(&ctx, &solv));

    let model = slv_get_model_z3!(&ctx, &solv);
    assert_eq!("y -> (- (/ 271549371.0 500000.0))\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_new_solver_get_param_descr(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);

    SlvGetParamDescrZ3::new(&ctx, &solv);
}

#[test]
fn get_proof_test() {
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true");
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let sort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &sort, "x");
    let three = IntZ3::new(&ctx, &sort, 3);
    let two = IntZ3::new(&ctx, &sort, 2);

    SlvAssertZ3::new(&ctx, &slv, ANDZ3::new(&ctx, vec!(GTZ3::new(&ctx, x, three), LTZ3::new(&ctx, x, two))));

    SlvCheckZ3::new(&ctx, &slv);
    let proof = SlvGetProofZ3::new(&ctx, &slv);
    assert_eq!("(unit-resolution (th-lemma (or (>= x 2) (<= x 3)))
                 (mp (mp (and-elim (asserted (and (> x 3) (< x 2))) (> x 3))
                         (rewrite (= (> x 3) (not (<= x 3))))
                         (not (<= x 3)))
                     (rewrite (= (not (<= x 3)) (not (<= x 3))))
                     (not (<= x 3)))
                 (mp (mp (mp (and-elim (asserted (and (> x 3) (< x 2))) (< x 2))
                             (rewrite (= (< x 2) (not (<= 2 x))))
                             (not (<= 2 x)))
                         (rewrite (= (not (<= 2 x)) (not (<= 2 x))))
                         (not (<= 2 x)))
                     (monotonicity (rewrite (= (<= 2 x) (>= x 2)))
                                   (= (not (<= 2 x)) (not (>= x 2))))
                     (not (>= x 2)))
                 false)", slv_proof_to_string_z3!(&ctx, proof));

}

#[test]
fn get_unsat_core_test() {
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true");
    SetParamZ3::new(&cfg, "unsat_core", "true");
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let sort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &sort, "x");
    let three = IntZ3::new(&ctx, &sort, 3);
    let two = IntZ3::new(&ctx, &sort, 2);
    let one = IntZ3::new(&ctx, &sort, 1);

    SlvAssertAndTrackZ3::new(&ctx, &slv, GTZ3::new(&ctx, x, three), "a1");
    SlvAssertAndTrackZ3::new(&ctx, &slv, LTZ3::new(&ctx, x, two), "a2");
    SlvAssertAndTrackZ3::new(&ctx, &slv, EQZ3::new(&ctx, x, one), "a3");

    SlvCheckZ3::new(&ctx, &slv);
    let unsat_core = SlvGetUnsatCoreZ3::new(&ctx, &slv);
    assert_eq!("(ast-vector
  a1
  a2)", slv_unsat_core_to_string_z3!(&ctx, unsat_core));
}

#[test]
fn test_slv_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);
    assert_eq!("", slv_to_string_z3!(&ctx, &slv));
}

#[test]
fn test_slv_assert_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);

    slv_assert_z3!(&ctx, &slv,
        eqz3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, -543.098742)
        )
    );

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))\n", slv_to_string_z3!(&ctx, &slv));
}

#[test]
fn test_slv_assert_and_track_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);

    slv_assert_and_track_z3!(&ctx, &slv,
        eqz3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, -543.098742)
        ),
        "tracker"
    );

    assert_eq!("(declare-fun y () Real)
(declare-fun tracker () Bool)
(assert (=> tracker (= y (- (/ 271549371.0 500000.0)))))\n", slv_to_string_z3!(&ctx, &slv));
}

#[test]
fn test_slv_check_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);

    slv_assert_z3!(&ctx, &slv,
        eqz3!(&ctx,
            real_var_z3!(&ctx, "y"),
            real_z3!(&ctx, -543.098742)
        )
    );

    let res1 = slv_check_z3!(&ctx, &slv);

    assert_eq!(1, res1);

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(rmodel->model-converter-wrapper
y -> (- (/ 271549371.0 500000.0))
)\n", slv_to_string_z3!(&ctx, &slv));

    let model = slv_get_model_z3!(&ctx, &slv);
    assert_eq!("y -> (- (/ 271549371.0 500000.0))\n", model_to_string_z3!(&ctx, model));
}

#[test]
fn test_slv_param_descr_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);
    slv_get_param_descr_z3!(&ctx, &slv);
}

#[test]
fn test_get_proof_macro_1(){
    let cfg = cfgz3!();
    set_param_z3!(&cfg, "proof", "true");
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);

    let sort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &sort, "x");
    let three = IntZ3::new(&ctx, &sort, 3);
    let two = IntZ3::new(&ctx, &sort, 2);

    SlvAssertZ3::new(&ctx, &slv, ANDZ3::new(&ctx, vec!(GTZ3::new(&ctx, x, three), LTZ3::new(&ctx, x, two))));

    SlvCheckZ3::new(&ctx, &slv);
    let proof = slv_get_proof_z3!(&ctx, &slv);
    assert_eq!("(unit-resolution (th-lemma (or (>= x 2) (<= x 3)))
                 (mp (mp (and-elim (asserted (and (> x 3) (< x 2))) (> x 3))
                         (rewrite (= (> x 3) (not (<= x 3))))
                         (not (<= x 3)))
                     (rewrite (= (not (<= x 3)) (not (<= x 3))))
                     (not (<= x 3)))
                 (mp (mp (mp (and-elim (asserted (and (> x 3) (< x 2))) (< x 2))
                             (rewrite (= (< x 2) (not (<= 2 x))))
                             (not (<= 2 x)))
                         (rewrite (= (not (<= 2 x)) (not (<= 2 x))))
                         (not (<= 2 x)))
                     (monotonicity (rewrite (= (<= 2 x) (>= x 2)))
                                   (= (not (<= 2 x)) (not (>= x 2))))
                     (not (>= x 2)))
                 false)", slv_proof_to_string_z3!(&ctx, proof));


    slv_assert_z3!(&ctx, &slv,
        andz3!(&ctx,
            eqz3!(&ctx,
                int_var_z3!(&ctx, "y"),
                int_z3!(&ctx, -543)
            ),
            ltz3!(&ctx,
                int_var_z3!(&ctx, "y"),
                int_z3!(&ctx, -600)
            ),
            gtz3!(&ctx,
                int_var_z3!(&ctx, "y"),
                int_z3!(&ctx, 300)
            )
        )    
    );

    let res1 = slv_check_z3!(&ctx, &slv);
    assert_eq!(-1, res1);

    let proof = slv_get_proof_z3!(&ctx, &slv);
    assert_eq!("(mp (and-elim (mp (asserted (and (= y (- 543)) (< y (- 600)) (> y 300)))
                  (monotonicity (trans (rewrite (= (< y (- 600))
                                                   (not (<= (- 600) y))))
                                       (monotonicity (rewrite (= (<= (- 600) y)
                                                                 (>= y (- 600))))
                                                     (= (not (<= (- 600) y))
                                                        (not (>= y (- 600)))))
                                       (= (< y (- 600)) (not (>= y (- 600)))))
                                (rewrite (= (> y 300) (not (<= y 300))))
                                (= (and (= y (- 543)) (< y (- 600)) (> y 300))
                                   (and (= y (- 543))
                                        (not (>= y (- 600)))
                                        (not (<= y 300)))))
                  (and (= y (- 543)) (not (>= y (- 600))) (not (<= y 300))))
              (not (<= y 300)))
    (trans (monotonicity (trans (monotonicity (and-elim (mp (asserted (and (= y
                                                                              (- 543))
                                                                           (< y
                                                                              (- 600))
                                                                           (> y
                                                                              300)))
                                                            (monotonicity (trans (rewrite (= (< y
                                                                                                (- 600))
                                                                                             (not (<= (- 600)
                                                                                                      y))))
                                                                                 (monotonicity (rewrite (= (<= (- 600)
                                                                                                               y)
                                                                                                           (>= y
                                                                                                               (- 600))))
                                                                                               (= (not (<= (- 600)
                                                                                                           y))
                                                                                                  (not (>= y
                                                                                                           (- 600)))))
                                                                                 (= (< y
                                                                                       (- 600))
                                                                                    (not (>= y
                                                                                             (- 600)))))
                                                                          (rewrite (= (> y
                                                                                         300)
                                                                                      (not (<= y
                                                                                               300))))
                                                                          (= (and (= y
                                                                                     (- 543))
                                                                                  (< y
                                                                                     (- 600))
                                                                                  (> y
                                                                                     300))
                                                                             (and (= y
                                                                                     (- 543))
                                                                                  (not (>= y
                                                                                           (- 600)))
                                                                                  (not (<= y
                                                                                           300)))))
                                                            (and (= y (- 543))
                                                                 (not (>= y
                                                                          (- 600)))
                                                                 (not (<= y 300))))
                                                        (= y (- 543)))
                                              (= (<= y 300) (<= (- 543) 300)))
                                (rewrite (= (<= (- 543) 300) true))
                                (= (<= y 300) true))
                         (= (not (<= y 300)) (not true)))
           (rewrite (= (not true) false))
           (= (not (<= y 300)) false))
    false)", slv_proof_to_string_z3!(&ctx, proof));
    slv_get_param_descr_z3!(&ctx, &slv);
}

#[test]
fn test_get_unsat_core_macro_1() {
    let cfg = ConfigZ3::new();
    SetParamZ3::new(&cfg, "proof", "true");
    SetParamZ3::new(&cfg, "unsat_core", "true");
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let sort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &sort, "x");
    let three = IntZ3::new(&ctx, &sort, 3);
    let two = IntZ3::new(&ctx, &sort, 2);
    let one = IntZ3::new(&ctx, &sort, 1);

    SlvAssertAndTrackZ3::new(&ctx, &slv, GTZ3::new(&ctx, x, three), "a1");
    SlvAssertAndTrackZ3::new(&ctx, &slv, LTZ3::new(&ctx, x, two), "a2");
    SlvAssertAndTrackZ3::new(&ctx, &slv, EQZ3::new(&ctx, x, one), "a3");

    SlvCheckZ3::new(&ctx, &slv);
    let unsat_core = slv_get_unsat_core_z3!(&ctx, &slv);
    assert_eq!("(ast-vector
  a1
  a2)", slv_unsat_core_to_string_z3!(&ctx, unsat_core));
}