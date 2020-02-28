use sp_domain::*;
use std::collections::HashMap;
use super::*;

#[test]
fn test_update_guards_empty_model() {
    let m = Model::new("test", vec![]);
    let ts_orig = TransitionSystemModel::from(&m);
    let mut ts = ts_orig.clone();
    update_guards(&mut ts, &HashMap::new());
    assert_eq!(ts, ts_orig);
}