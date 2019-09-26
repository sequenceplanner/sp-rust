//! Z3 sorts for SP

use std::ffi::{CStr, CString};
use z3_sys::*;
use super::*;

pub struct GetSortZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub what: Z3_ast,
    pub s: String,
    pub r: Z3_sort
}