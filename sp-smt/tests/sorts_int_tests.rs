use sp_smt::*;

#[test]
fn test_bool_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = BoolSortZ3::new(&ctx);
    assert_eq!("Bool", sort_to_string_z3!(&ctx, sort.r));
}

#[test]
fn test_int_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = IntSortZ3::new(&ctx);
    assert_eq!("Int", sort_to_string_z3!(&ctx, sort.r));
}

#[test]
fn test_real_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = RealSortZ3::new(&ctx);
    assert_eq!("Real", sort_to_string_z3!(&ctx, sort.r));
}

#[test]
fn test_string_sort(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let sort = StringSortZ3::new(&ctx);
    assert_eq!("String", sort_to_string_z3!(&ctx, sort.r));
}


#[test]
fn test_bool_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let bsrt = bool_sort_z3!(&ctx);
    assert_eq!("Bool", sort_to_string_z3!(&ctx, bsrt.r));
}

#[test]
fn test_int_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let isrt = int_sort_z3!(&ctx);
    assert_eq!("Int", sort_to_string_z3!(&ctx, isrt.r));
}

#[test]
fn test_real_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let rsrt = real_sort_z3!(&ctx);
    assert_eq!("Real", sort_to_string_z3!(&ctx, rsrt.r));
}

#[test]
fn test_string_sort_macro_1(){
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let ssrt = string_sort_z3!(&ctx);
    assert_eq!("String", sort_to_string_z3!(&ctx, ssrt.r));
}