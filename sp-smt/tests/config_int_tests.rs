use sp_smt::*;

#[test]
fn test_new_cfg(){
    ConfigZ3::new();
}

#[test]
fn test_default_cfg(){
    ConfigZ3::default();
}

#[test]
fn test_cfg_macro_1(){
    cfg_z3!();
}