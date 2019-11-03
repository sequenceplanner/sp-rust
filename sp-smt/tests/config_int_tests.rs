use sp_smt::*;

#[test]
fn test_new_cfg(){
    ConfigZ3::new();
}

#[test]
fn test_default_cfg(){
    ConfigZ3::default();
}