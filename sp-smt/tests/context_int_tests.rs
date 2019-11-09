use sp_smt::*;

#[test]
fn test_ctx(){
    let conf = ConfigZ3::new();
    ContextZ3::new(&conf);
}

#[test]
fn test_default_ctx(){
    ContextZ3::default();
}

#[test]
fn test_ctx_macro(){
    let cfg = cfg_z3!();
    ctx_z3!(&cfg);
}