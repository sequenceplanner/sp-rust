//! Z3 solver for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct SolverZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_solver,
}

pub struct SlvAssertZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub cst: Z3_ast,
    pub r: ()
}

pub struct SlvAssertAndTrackZ3<'ctx, 'slv, 't> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub cst: Z3_ast,
    pub tracker: &'t str,
    pub r: ()
}

pub struct SlvCheckZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_lbool
}

pub struct SlvGetModelZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_model
}

pub struct SlvGetParamDescrZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>
}

pub struct SlvGetProofZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_ast
}

pub struct SlvGetAllSolutionsZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_ast,
    pub s: Vec<String>
}

pub struct SlvGetUnsatCoreZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_ast_vector
}

impl <'ctx> SolverZ3<'ctx> {
    /// Create a new solver. This solver is a "combined solver" (see
    /// combined_solver module) that internally uses a non-incremental (solver1) and an
    /// incremental solver (solver2). This combined solver changes its behaviour based
    /// on how it is used and how its parameters are set.
    ///
    /// The "default" tactic will attempt to probe the logic used by the
    /// assertions and will apply a specialized tactic if one is supported.
    /// Otherwise the general `(and-then simplify smt)` tactic will be used.
    ///
    /// If the solver is used in an incremental way then the combined solver
    /// will switch to using solver2 (which behaves similarly to the general
    /// "smt" tactic).
    ///
    /// Note however it is possible to set the `solver2_timeout`,
    /// `solver2_unknown`, and `ignore_solver1` parameters of the combined
    /// solver to change its behaviour.
    ///
    /// NOTE: See macro! slvz3!
    pub fn new(ctx: &ContextZ3) -> SolverZ3 {
        let z3 = unsafe {
            let solv = Z3_mk_solver(ctx.r);
            Z3_solver_inc_ref(ctx.r, solv);
            solv
        };
        SolverZ3 {ctx, r: z3}
    }
}

impl <'ctx, 'slv> SlvAssertZ3<'ctx, 'slv> {
    /// Assert a new constraint into the solver.
    ///
    /// NOTE: See macro! slv_assert_z3!
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>, cst: Z3_ast) -> () {
        let z3 = unsafe {
            Z3_solver_assert(ctx.r, slv.r, cst)
        };
        SlvAssertZ3 {ctx, slv, cst, r: z3}.r
    }
}

impl <'ctx, 'slv, 't> SlvAssertAndTrackZ3<'ctx, 'slv, 't> {
    /// Assert a constraint `cst` into the solver, and track it (in the
    /// unsat) core using the Boolean constant `tracker`. Used for extracting
    /// unsat cores.
    /// 
    /// NOTE: See macro! slv_assert_and_track_z3!
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>, cst: Z3_ast, tracker: &'t str) -> () {
        let z3 = unsafe {
                let sort = BoolSortZ3::new(&ctx);
                let var = BoolVarZ3::new(&ctx, &sort, tracker);
                Z3_solver_assert_and_track(ctx.r, slv.r, cst, var)
            };
        SlvAssertAndTrackZ3 {ctx, slv, cst, tracker, r: z3}.r
    }
}

impl <'ctx, 'slv> SlvCheckZ3<'ctx, 'slv> {
    /// Check whether the assertions in a given solver are consistent or not.
    ///
    /// NOTE: See macro! slv_check_z3!
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> Z3_lbool {
        let z3 = unsafe {
            Z3_solver_check(ctx.r, slv.r)
        };
        SlvCheckZ3 {ctx, slv, r: z3}.r
    }
}

impl<'ctx, 'slv> SlvGetModelZ3<'ctx, 'slv> {
    /// Retrieve the model for the last `SolvCheckZ3::new`
    ///
    /// NOTE: The error handler is invoked if a model is not available because
    /// the commands above were not invoked for the given solver, or if the result was `Z3_L_FALSE`.
    /// 
    /// NOTE: See macro! slv_get_model_z3!
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> Z3_model {
        let z3 = unsafe {
            Z3_solver_get_model(ctx.r, slv.r)
        };
        SlvGetModelZ3 {ctx, r: z3, slv}.r        
    }
}

impl <'ctx, 'slv> SlvGetParamDescrZ3<'ctx, 'slv> {
    /// Return a string that is the parameter description set for the given solver object.
    /// 
    /// NOTE: See macro! slv_get_param_descr!
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> String {
        unsafe {
            let descr = Z3_solver_get_param_descrs(ctx.r, slv.r);
            let desc_str = Z3_param_descrs_to_string(ctx.r, descr);
            CStr::from_ptr(desc_str).to_str().unwrap().to_owned()
        }
    }
}

impl <'ctx, 'slv> SlvGetProofZ3<'ctx, 'slv> {
    /// Retrieve the proof for the last `SolvCheckZ3::new`
    ///
    /// NOTE: The error handler is invoked if proof generation is not enabled,
    /// or if the commands above were not invoked for the given solver,
    /// or if the result was different from `Z3_L_FALSE`.
    /// 
    /// NOTE: See macro! slv_get_proof!
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> Z3_ast {
        let z3 = unsafe {
            Z3_solver_get_proof(ctx.r, slv.r)
        };
        SlvGetProofZ3 {ctx, slv, r: z3}.r
    }
}

impl <'ctx, 'slv> SlvGetUnsatCoreZ3<'ctx, 'slv> {
    /// Retrieve the unsat core for the last `SolvCheckZ3::new`
    /// 
    /// NOTE: The unsat core is a subset of the assumptions `a`.
    /// 
    /// NOTE: See macro! slv_get_unsat_core!
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> Z3_ast_vector {
        let z3 = unsafe {
            Z3_solver_get_unsat_core(ctx.r, slv.r)
        };
        SlvGetUnsatCoreZ3 {ctx, slv, r: z3}.r
    }
}

impl <'ctx> Drop for SolverZ3<'ctx> {
    /// Decrement the reference counter of the given solver.
    fn drop(&mut self) {
        unsafe { 
            Z3_solver_dec_ref(self.ctx.r, self.r)
        }
    }
}

/// Z3 solver 
/// 
/// Macro rule for:
/// ```text
/// z3solver::SolverZ3::new(&ctx)
/// ```
/// Using the global context:
/// ```text
/// slvz3!()
/// ```
/// Using a specific context:
/// ```text
/// slvz3!(&ctx)
/// ```
#[macro_export]
macro_rules! slvz3 {
    // () => {
    //     SolverZ3::new(&CTX)
    // };
    ($ctx:expr) => {
        SolverZ3::new($ctx)
    }
}

/// Z3 assert constraint 
/// 
/// Macro rule for:
/// ```text
/// z3solver::SlvAssertZ3::new(&ctx, &slv, a)
/// ```
/// Using the global context:
/// ```text
/// sasrtz3!(a)
/// ```
/// Using a specific context:
/// ```text
/// sasrtz3!(&ctx, a)
/// ```
#[macro_export]
macro_rules! slv_assert_z3 {
    // ($slv:expr, $a:expr) => {
    //     SlvAssertZ3::new(&CTX, $slv, $a).r
    // };
    ($ctx:expr, $slv:expr, $a:expr) => {
        SlvAssertZ3::new($ctx, $slv, $a)
    }
}

/// Z3 assert constraint and track for unsat core extraction
/// 
/// Macro rule for:
/// ```text
/// z3solver::SlvAssertAndTrackZ3::new(&ctx, &slv, constraint, tracker)
/// ```
/// Using the global context:
/// ```text
/// sasrtatz3!(&slv, constraint, tracker)
/// ```
/// Using a specific context:
/// ```text
/// sasrtaz3!(&ctx, &slv, constraint, tracker)
/// ```
/// tracker must be a Boolean variable.
#[macro_export]
macro_rules! slv_assert_and_track_z3 {
    // ($slv:expr, $a:expr, $b:expr) => {
    //     SlvAssertAndTrackZ3::new(&CTX, $slv, $a, $b).r
    // };
    ($ctx:expr, $slv:expr, $a:expr, $b:expr) => {
        SlvAssertAndTrackZ3::new($ctx, $slv, $a, $b)
    }
}

/// Z3 check whether the assertions in a given solver are consistent or not
/// 
/// Macro rule for:
/// ```text
/// z3solver::SlvCheckZ3::new(&ctx, &slv, a)
/// ```
/// Using the global context:
/// ```text
/// scheckz3!(a, &slv)
/// ```
/// Using a specific context:
/// ```text
/// scheckz3!(&ctx, &slv, a)
/// ```
#[macro_export]
macro_rules! slv_check_z3 {
    // ($slv:expr) => {
    //     SlvCheckZ3::new(&CTX, $slv).r
    // // };
    ($ctx:expr, $slv:expr) => {
        SlvCheckZ3::new($ctx, $slv)
    }
}

/// Get the model from slv
#[macro_export]
macro_rules! slv_get_model_z3 {
    ($ctx:expr, $a:expr) => {
        SlvGetModelZ3::new($ctx, $a)
    }
}

/// Get the parameter description
#[macro_export]
macro_rules! slv_get_param_descr_z3 {
    ($ctx:expr, $a:expr) => {
        SlvGetParamDescrZ3::new($ctx, $a)
    }
}

/// Get the proof if unsat
#[macro_export]
macro_rules! slv_get_proof_z3 {
    ($ctx:expr, $a:expr) => {
        SlvGetProofZ3::new($ctx, $a)
    }
}

/// Get the unsat core if unsat
#[macro_export]
macro_rules! slv_get_unsat_core_z3 {
    ($ctx:expr, $a:expr) => {
        SlvGetUnsatCoreZ3::new($ctx, $a)
    }
}

/// Z3 solver to readable string
#[macro_export]
macro_rules! slv_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        unsafe {
            CStr::from_ptr(Z3_solver_to_string($ctx, $a)).to_str().unwrap().to_owned()
        }
    }
}

/// Z3 unsat core to readable string
#[macro_export]
macro_rules! slv_unsat_core_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        unsafe {
            CStr::from_ptr(Z3_ast_vector_to_string($ctx, $a)).to_str().unwrap().to_owned()
        }
    }
}

/// Z3 proof to readable string
#[macro_export]
macro_rules! slv_proof_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        unsafe {
            CStr::from_ptr(Z3_ast_to_string($ctx, $a)).to_str().unwrap().to_owned()
        }
    }
}

#[test]
fn test_new_solver(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let solv = SolverZ3::new(&ctx);
    assert_eq!("", slv_to_string_z3!(ctx.r, solv.r));
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
(assert (= y (- (/ 271549371.0 500000.0))))\n", slv_to_string_z3!(ctx.r, solv.r));
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
(assert (=> track (= y (- (/ 271549371.0 500000.0)))))\n", slv_to_string_z3!(ctx.r, solv.r));
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

    assert_eq!("", slv_to_string_z3!(ctx.r, solv.r));

    SlvAssertZ3::new(&ctx, &solv, rel1);
    
    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))\n", slv_to_string_z3!(ctx.r, solv.r));

    let res1 = SlvCheckZ3::new(&ctx, &solv);

    assert_eq!(1, res1);

    assert_eq!("(declare-fun y () Real)
(assert (= y (- (/ 271549371.0 500000.0))))
(rmodel->model-converter-wrapper
y -> (- (/ 271549371.0 500000.0))
)\n", slv_to_string_z3!(ctx.r, solv.r));

    let model = slv_get_model_z3!(&ctx, &solv);
    assert_eq!("y -> (- (/ 271549371.0 500000.0))\n", model_to_string_z3!(ctx.r, model));
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
                 false)", slv_proof_to_string_z3!(ctx.r, proof));

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
  a2)", slv_unsat_core_to_string_z3!(ctx.r, unsat_core));
}

#[test]
fn test_slv_macro_1(){
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let slv = slvz3!(&ctx);
    assert_eq!("", slv_to_string_z3!(ctx.r, slv.r));
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
(assert (= y (- (/ 271549371.0 500000.0))))\n", slv_to_string_z3!(ctx.r, slv.r));
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
(assert (=> tracker (= y (- (/ 271549371.0 500000.0)))))\n", slv_to_string_z3!(ctx.r, slv.r));
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
)\n", slv_to_string_z3!(ctx.r, slv.r));

    let model = slv_get_model_z3!(&ctx, &slv);
    assert_eq!("y -> (- (/ 271549371.0 500000.0))\n", model_to_string_z3!(ctx.r, model));
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
                 false)", slv_proof_to_string_z3!(ctx.r, proof));


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
    false)", slv_proof_to_string_z3!(ctx.r, proof));
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
  a2)", slv_unsat_core_to_string_z3!(ctx.r, unsat_core));
}