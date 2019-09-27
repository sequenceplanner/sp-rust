//! Z3 context

use z3_sys::*;
use super::*;

pub struct ContextZ3 {
    pub r: Z3_context
}

impl ContextZ3 {
    /// Create a context using the given configuration.
    ///
    /// After a context is created, the configuration cannot be changed,
    /// although some parameters can be changed using [`Z3_update_param_value`](fn.Z3_update_param_value.html).
    /// All main interaction with Z3 happens in the context of a `Z3_context`.
    ///
    /// In contrast to [`Z3_mk_context_rc`](fn.Z3_mk_context_rc.html), the
    /// life time of `Z3_ast` objects are determined by the scope level of
    /// [`Z3_solver_push`](fn.Z3_solver_push.html) and
    /// [`Z3_solver_pop`](fn.Z3_solver_pop.html).
    /// In other words, a `Z3_ast` object remains valid until there is a
    /// call to [`Z3_solver_pop`](fn.Z3_solver_pop.html) that
    /// takes the current scope below the level where
    /// the object was created.
    ///
    /// Note that all other reference counted objects, including `Z3_model`,
    /// `Z3_solver`, `Z3_func_interp` have to be managed by the caller.
    /// Their reference counts are not handled by the context.
    ///
    /// Further remarks:
    /// - `Z3_sort`, `Z3_func_decl`, `Z3_app`, `Z3_pattern` are `Z3_ast`'s.
    /// - Z3 uses hash-consing, i.e., when the same `Z3_ast` is created twice,
    ///   Z3 will return the same pointer twice.
    ///
    /// # See also:
    ///
    /// - [`Z3_del_context`](fn.Z3_del_context.html)
    pub fn new(cfg: &ConfigZ3) -> ContextZ3 {
        ContextZ3 {
            r: unsafe {
                let ctx = Z3_mk_context(cfg.r);
                ctx
            }
        }
    }
}

impl Default for ContextZ3 {
    /// Create a default logical context using the given configuration..
    ///
    /// # See also:
    ///
    /// - [`Z3_mk_context`](fn.Z3_mk_context.html)
    fn default() -> Self {
        Self::new(&ConfigZ3::default())
    }
}

impl Drop for ContextZ3 {
    /// Delete the given logical context.
    ///
    /// # See also:
    ///
    /// - [`Z3_mk_context`](fn.Z3_mk_context.html)
    fn drop(&mut self) {
        unsafe { 
            Z3_del_context(self.r)
        }
    }
}

#[test]
fn test_ctx(){
    let conf = ConfigZ3::new();
    ContextZ3::new(&conf);
}

#[test]
fn test_default_ctx(){
    ContextZ3::default();
}