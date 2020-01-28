use sp_domain::*;

#[test]
fn action_test() {
    let x = SPPath::from_string("x");
    let y = SPPath::from_string("x/y");
    let assign_5 = Compute::PredicateValue(
        PredicateValue::SPValue(5.to_spvalue()));
    let assign_true = Compute::PredicateValue(
        PredicateValue::SPValue(true.to_spvalue()));
    let assign_false = Compute::PredicateValue(
        PredicateValue::SPValue(false.to_spvalue()));
    assert_eq!(a!(x = 5), Action::new(x.clone(),
                                           assign_5.clone()));

    assert_eq!(a!("x" = 5), Action::new(x.clone(),
                                             assign_5.clone()));

    assert_eq!(a!(p:y = 5), Action::new(y.clone(),
                                             assign_5.clone()));

    assert_eq!(a!(p:y), Action::new(y.clone(),
                                         assign_true.clone()));

    assert_eq!(a!(!"x"), Action::new(x.clone(),
                                          assign_false.clone()));

}

#[test]
fn pred_test() {
    let x_true = Predicate::EQ(
        PredicateValue::SPPath(SPPath::from_string("x"), None),
        PredicateValue::SPValue(true.to_spvalue()));
    let not_x_true = Predicate::NOT(Box::new(x_true.clone()));
    assert_eq!(p!(x), x_true);
    assert_eq!(p!(!x), not_x_true);
    assert_eq!(p!(!(x)), not_x_true);

    let lp = SPPath::from_string("really/long/path");
    let lp_true = Predicate::EQ(
        PredicateValue::SPPath(lp.clone(), None),
        PredicateValue::SPValue(true.to_spvalue()));
    let not_lp_true = Predicate::NOT(Box::new(lp_true.clone()));
    assert_eq!(p!([!p:lp] && [!x]),
               Predicate::AND(vec![not_lp_true.clone(), not_x_true.clone()]));

    assert_eq!(p!([!p:lp] && [!x] && [([x] || [p:lp])]),
               Predicate::AND(
                   vec![not_lp_true.clone(), not_x_true.clone(),
                        Predicate::OR(vec![x_true.clone(), lp_true.clone()])]));

    assert_eq!(p!(x == 5),
               Predicate::EQ(PredicateValue::SPPath(SPPath::from_string("x"), None),
                             PredicateValue::SPValue(5.to_spvalue())));
    assert_eq!(p!(p:lp == 5),
               Predicate::EQ(PredicateValue::SPPath(lp.clone(), None),
                             PredicateValue::SPValue(5.to_spvalue())));

}
