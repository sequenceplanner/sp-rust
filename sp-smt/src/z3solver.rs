//! Z3 solver for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct SolverZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub r: Z3_solver,
}

pub struct SolvAssertZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub cst: Z3_ast,
    pub r: ()
}

pub struct SolvAssertAndTrackZ3<'ctx, 'slv, 't> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub cst: Z3_ast,
    pub tracker: &'t str,
    pub r: ()
}

pub struct SolvCheckZ3<'ctx, 'slv> {
    pub ctx: &'ctx ContextZ3,
    pub slv: &'slv SolverZ3<'ctx>,
    pub r: Z3_lbool
}

impl <'ctx> SolverZ3<'ctx> {
    /// Create a new solver. This solver is a "combined solver" (see
    /// combined_solver module) that internally uses a non-incremental (solver1) and an
    /// incremental solver (solver2). This combined solver changes its behaviour based
    /// on how it is used and how its parameters are set.
    ///
    /// If the solver is used in a non incremental way (i.e. no calls to
    /// `Z3_solver_push()` or `Z3_solver_pop()`, and no calls to
    /// `Z3_solver_assert()` or `Z3_solver_assert_and_track()` after checking
    /// satisfiability without an intervening `Z3_solver_reset()`) then solver1
    /// will be used. This solver will apply Z3's "default" tactic.
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
    /// The function [`Z3_solver_get_model`](fn.Z3_solver_get_model.html) retrieves a model if the
    /// assertions is satisfiable (i.e., the result is
    /// `Z3_L_TRUE`) and model construction is enabled.
    /// The function [`Z3_solver_get_model`](fn.Z3_solver_get_model.html) can also be used even
    /// if the result is `Z3_L_UNDEF`, but the returned model
    /// is not guaranteed to satisfy quantified assertions.
    ///
    /// NOTE: User must use [`Z3_solver_inc_ref`](fn.Z3_solver_inc_ref.html) and [`Z3_solver_dec_ref`](fn.Z3_solver_dec_ref.html) to manage solver objects.
    /// Even if the context was created using [`Z3_mk_context`](fn.Z3_mk_context.html) instead of [`Z3_mk_context_rc`](fn.Z3_mk_context_rc.html).
    pub fn new(ctx: &ContextZ3) -> SolverZ3 {
        let z3 = unsafe {
            let solv = Z3_mk_solver(ctx.r);
            Z3_solver_inc_ref(ctx.r, solv);
            solv
        };
        SolverZ3 {
ctx,
            r: z3
        }
    }
}

impl <'ctx, 'slv> SolvAssertZ3<'ctx, 'slv> {
    /// Assert a new constraint into the solver.
    ///
    /// The functions [`SCheckZ3::check()`](#method.check) and
    /// [`SCheckZ3::check_assumptions()`](#method.check_assumptions) should be
    /// used to check whether the logical context is consistent or not.
    ///
    /// # See also:
    ///
    /// - [`Solver::assert_and_track()`](#method.assert_and_track)
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>, cst: Z3_ast) -> SolvAssertZ3<'ctx, 'slv> {
        SolvAssertZ3 {
            ctx,
            slv,
            cst,
            r: unsafe {
                Z3_solver_assert(ctx.r, slv.r, cst);
            }
        }
    }
}

impl <'ctx, 'slv, 't> SolvAssertAndTrackZ3<'ctx, 'slv, 't> {
    /// Assert a constraint `cst` into the solver, and track it (in the
    /// unsat) core using the Boolean constant `tracker`. Used for extracting
    /// unsat cores.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>, cst: Z3_ast, tracker: &'t str) -> SolvAssertAndTrackZ3<'ctx, 'slv, 't> {
        SolvAssertAndTrackZ3 {
            ctx,
            slv,
            cst,
            tracker,
            r: unsafe {
                let sort = BoolSortZ3::new(&ctx);
                let var = BoolVarZ3::new(&ctx, &sort, tracker);
                Z3_solver_assert_and_track(ctx.r, slv.r, cst, var)
            }
        }
    }
}

impl <'ctx, 'slv> SolvCheckZ3<'ctx, 'slv> {
    /// Check whether the assertions in a given solver are consistent or not.
    ///
    /// The function [`Solver::get_model()`](#method.get_model)
    /// retrieves a model if the assertions is satisfiable (i.e., the
    /// result is `SatResult::Sat`) and [model construction is enabled].
    /// Note that if the call returns `SatResult::Unknown`, Z3 does not
    /// ensure that calls to [`Solver::get_model()`](#method.get_model)
    /// succeed and any models produced in this case are not guaranteed
    /// to satisfy the assertions.
    pub fn new(ctx: &'ctx ContextZ3, slv: &'slv SolverZ3<'ctx>) -> SolvCheckZ3<'ctx, 'slv> {
        let z3 = unsafe {
            Z3_solver_check(ctx.r, slv.r)
        };
        SolvCheckZ3 {
            ctx,
            slv,
            r: z3
        }
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
    () => {
        SolverZ3::new(&CTX)
    };
    ($ctx:expr) => {
        SolverrZ3::new($ctx)
    }
}

/// Z3 assert constraint 
/// 
/// Macro rule for:
/// ```text
/// z3solver::SolvAssertZ3::new(&ctx, &slv, a)
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
macro_rules! sasrtz3 {
    ($slv:expr, $a:expr) => {
        SolvAssertZ3::new(&CTX, $slv, $a).r
    };
    ($ctx:expr, $slv:expr, $a:expr) => {
        SolvAssertZ3::new($ctx, $slv, $a).r
    }
}

/// Z3 assert constraint and track for unsat core extraction
/// 
/// Macro rule for:
/// ```text
/// z3solver::SolvAssertAndTrackZ3::new(&ctx, &slv, constraint, tracker)
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
macro_rules! sasrtatz3 {
    ($slv:expr, $a:expr, $b:expr) => {
        SolvAssertAndTrackZ3::new(&CTX, $slv, $a, $b).r
    };
    ($ctx:expr, $slv:expr, $a:expr, $b:expr) => {
        SolvAssertAndTrackZ3::new($ctx, $slv, $a, $b).r
    }
}

/// Z3 check whether the assertions in a given solver are consistent or not
/// 
/// Macro rule for:
/// ```text
/// z3solver::SolvCheckZ3::new(&ctx, &slv, a)
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
macro_rules! scheckz3 {
    // ($slv:expr) => {
    //     SolvCheckZ3::new(&CTX, $slv).r
    // // };
    ($ctx:expr, $slv:expr, $a:expr) => {
        SolvCheckZ3::new($ctx, $slv, $a).r
    }
}

// #[test]
// fn test_new_solver(){
//     let conf = ConfigZ3::new();
//     let ctx = ContextZ3::new(&conf);
//     let solv = SolverZ3::new(&ctx);
//     println!("Should print empty string, no assertions yet.");
//     println!("{}", GetSolvStringZ3::new(&ctx, &solv).s);
// }

// #[test]
// fn test_new_sassert(){
//     let conf = ConfigZ3::new();
//     let ctx = ContextZ3::new(&conf);
//     let solv = SolverZ3::new(&ctx);

//     let realsort = RealSortZ3::new(&ctx);
//     let y = RealVarZ3::new(&ctx, &realsort, "y");
//     let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

//     let rel1 = EQZ3::new(&ctx, y.r, real1.r);

//     SolvAssertZ3::new(&ctx, &solv, rel1.r);
//     println!("Should print stuff now, we have asserted constraints.");
//     println!("{}", GetSolvStringZ3::new(&ctx, &solv).s);
// }

// #[test]
// fn test_new_scheck(){
//     let conf = ConfigZ3::new();
//     let ctx = ContextZ3::new(&conf);
//     let solv = SolverZ3::new(&ctx);

//     let realsort = RealSortZ3::new(&ctx);
//     let y = RealVarZ3::new(&ctx, &realsort, "y");
//     let real1 = RealZ3::new(&ctx, &realsort, -543.098742);

//     let rel1 = EQZ3::new(&ctx, y.r, real1.r);

//     println!("This is the solvers context without any assertions:");
//     println!("{}", GetSolvStringZ3::new(&ctx, &solv).s);

//     SolvAssertZ3::new(&ctx, &solv, rel1.r);
    
//     println!("This is the solvers context with an assertion before the check:");
//     println!("{}", GetSolvStringZ3::new(&ctx, &solv).s);

//     let res1 = SolvCheckZ3::new(&ctx, &solv);

//     println!("This is the return of the check:");
//     println!("{}", res1.r);

//     println!("This is the solver with an assertion after the check:");
//     println!("{}", GetSolvStringZ3::new(&ctx, &solv).s);

//     println!("This is the solution (model retreived from the solver):");
//     println!("{}", GetSolvModelZ3::new(&ctx, &solv).s);
// }

// #[test]
// fn door(){
//     let slv = slvz3!();

//     // deliberation var:
//     let delib = bvrz3!("delib");

//     // open door ability states:
//     let open_enabled = andz3!(notz3!(bvrz3!("opened_c")), notz3!(bvrz3!("opened_m")));
//     let open_executing = andz3!(bvrz3!("opened_c"), notz3!(bvrz3!("opened_m")));
//     let open_finishing = andz3!(bvrz3!("opened_c"), bvrz3!("opened_m"));
//     let open_done = andz3!(notz3!(bvrz3!("opened_c")), bvrz3!("opened_m"));

//     // close door ability states:
//     let closed_enabled = andz3!(notz3!(bvrz3!("closed_c")), notz3!(bvrz3!("closed_m")));
//     let closed_executing = andz3!(bvrz3!("closed_c"), notz3!(bvrz3!("closed_m")));
//     let closed_finishing = andz3!(bvrz3!("closed_c"), bvrz3!("closed_m"));
//     let closed_done = andz3!(notz3!(bvrz3!("closed_c")), bvrz3!("closed_m"));

//     // forbidden behavior:
//     let forb1 = notz3!(andz3!(bvrz3!("opened_c"), bvrz3!("closed_c")));
//     let forb2 = notz3!(andz3!(bvrz3!("opened_m"), bvrz3!("closed_m")));

//     // trasnitions:
//     let t1 = sasrtz3!(&slv, itez3!(&CTX, open_enabled, open_executing, open_enabled));

//     scheckz3!(&slv);
//     // println!("{}", );



//     // sasrtz3!(&slv,
//     //     notz3!(
//     //         andz3!(
//     //             bvrz3!("closed_c"),
//     //             bvrz3!("opened_c")
//     //         )
//     //     )
//     // );

//     // sasrtz3!(&slv, 
//     //     itez3!(
//     //         andz3!(
//     //             notz3!(bvrz3!("cc")), 
//     //             notz3!(bvrz3!("cm"))
//     //         ), 
//     //         bvrz3!("cc"), 
//     //         notz3!(bvrz3!("cc"))
//     //     )
//     // );  
// }