//! Z3 utilities for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

// Z3 AST to readable string
#[macro_export]
macro_rules! ast_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        unsafe {
            CStr::from_ptr(Z3_ast_to_string($ctx, $a)).to_str().unwrap().to_owned()
        }
    }
}

/// Z3 model to readable string
#[macro_export]
macro_rules! model_to_string_z3 {
    ($ctx:expr, $a:expr) => {
        unsafe {
            CStr::from_ptr(Z3_model_to_string($ctx, $a)).to_str().unwrap().to_owned()
        }
    }
}

// #[macro_export]
// macro_rules! inf_z3_type {
//     ($arg:expr) => {{
//         trait InferIntegerZ3 {
//             fn infer($arg) -> Z3_ast {
//                 ivlz3!($arg)
//             }
//         }
//         impl InferIntegerZ3 for i32 {
//             ivlz3!($arg)
//         }
//     }};
// }

#[macro_export]
macro_rules! f {
    ($($arg:expr),*) => {{
        trait PrintInteger {
            fn as_printed(&self) -> &'static str {
                "Integer"
            }
        }
        impl PrintInteger for i32 {}
        trait PrintOther {
            fn as_printed(&self) -> &Self {
                self
            }
        }
        impl<T: std::fmt::Display> PrintOther for &T {}
        $(
            println!("{}", (&$arg).as_printed());
        )*
    }};
}

/// Z3 set configuration parameters 
/// 
/// Macro rule for:
/// ```text
/// SetParamZ3::new(&cfg, param, value)
/// ```
/// Using the global configuration:
/// ```text
/// scpz3!(param, value)
/// ```
/// Using a specific configuration:
/// ```text
/// scpz3!(&cfg, param, value)
/// ```
#[macro_export]
macro_rules! cspz3 {
    // ($p:expr, $v:expr) => {
    //     SetParamZ3::new(&CFG, param, value)
    // };
    ($cfg:expr, $p:expr, $v:expr) => {
        SetParamZ3::new($cfg, param, value)
    }
}
