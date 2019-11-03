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