
use sp_domain::*;



/// Testing show we know that SPValues and support can be used outside
#[test]
fn sp_value_testing_external() {
    assert_eq!(true.to_spvalue(), SPValue::Bool(true));
    let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

    let v = SPPath::from_str(&["a", "b"]);

    let eq = Predicate::EQ(predicates::PredicateValue::SPValue(2.to_spvalue()), predicates::PredicateValue::SPPath(v.clone()));

    let p = p!{v == 2};
    println!("TEST: {:?}", &p);
    let p2 = p!{{["a", "b"]} == 2};
    println!("TEST: {:?}", &p2);

    let x = pr!{p2 && p && p && p};
    println!("TEST2: {:?}", x);

    let y = pr!{{p!{{["a", "b"]} == 10}} && {p!{{["a", "b"]} == 20}}};
    println!("TEST3: {:?}", y);

}