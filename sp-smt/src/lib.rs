//! # Z3 prover for SP

// #[macro_use]
// extern crate lazy_static;

pub mod z3config;
pub use crate::z3config::{ConfigZ3, SetParamZ3}; // CFG, Z3_MUTEX};

pub mod z3context;
pub use crate::z3context::{ContextZ3}; //, CTX};

pub mod z3solver;
pub use crate::z3solver::{SolverZ3, SlvAssertZ3, SlvAssertAndTrackZ3, SlvCheckZ3, SlvGetModelZ3, SlvGetParamDescrZ3,
    SlvGetProofZ3, SlvGetUnsatCoreZ3, SlvToStringZ3, SlvUnsatCoreToStringZ3, SlvProofToStringZ3};

pub mod z3optimizer;
pub use crate::z3optimizer::{OptimizerZ3, OptAssertZ3, OptCheckZ3, OptMaximizeZ3, OptMinimizeZ3, OptGetModelZ3, OptGetStringZ3};

pub mod z3sorts;
pub use crate::z3sorts::{IntSortZ3, BoolSortZ3, RealSortZ3, StringSortZ3, GetSortZ3, SortToStringZ3};

pub mod z3values;
pub use crate::z3values::{BoolZ3, IntZ3, RealZ3, StringZ3};

pub mod z3variables;
pub use crate::z3variables::{BoolVarZ3, IntVarZ3, RealVarZ3};

pub mod z3relations;
pub use crate::z3relations::{EQZ3, LEZ3, LTZ3, GEZ3, GTZ3};

pub mod z3operations;
pub use crate::z3operations::{MULZ3, DIVZ3, MODZ3, REMZ3, ADDZ3, SUBZ3, NEGZ3, POWZ3};

pub mod z3logics;
pub use crate::z3logics::{ANDZ3, ORZ3, NOTZ3, ITEZ3, IFFZ3, IMPZ3, XORZ3};

pub mod z3utils;
pub use crate::z3utils::{AstToStringZ3, ModelToStringZ3};