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
    /// although some parameters can be changed using:
    /// 
    /// `TODO: Add a parameter update utility.`
    /// 
    /// All main interaction with Z3 happens in the context of a `Z3_context`.
    /// 
    /// A public reference to a default context for simplicity: `&CTX`
    pub fn new(cfg: &ConfigZ3) -> ContextZ3 {
        ContextZ3 {
            r: unsafe {
                // Z3_MUTEX.lock().unwrap();
                let ctx = Z3_mk_context(cfg.r);
                ctx
            }
        }
    }
}

unsafe impl Sync for ContextZ3 {
    // let guard = Z3_MUTEX.lock().unwrap();
}

impl Default for ContextZ3 {
    /// Create a default logical context using the given configuration..
    fn default() -> Self {
        Self::new(&ConfigZ3::default())
    }
}

impl Drop for ContextZ3 {
    /// Delete the given logical context.
    fn drop(&mut self) {
        unsafe { 
            // Z3_MUTEX.lock().unwrap();
            Z3_del_context(self.r)
        }
    }
}

// // Unsafe sync takes toll... avoid this. 
// lazy_static! {
//     // Z3_MUTEX.lock().unwrap();
//     pub static ref CTX: ContextZ3 = {
//         ContextZ3::new(&CFG)
//     };
// }

/// Create a context using the given configuration.
#[macro_export]
macro_rules! ctxz3 {
    ($a:expr) => {
        ContextZ3::new($a)
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

// #[test]
// fn test_static_ctx(){
//     &CTX;
// }