/// A SOP is a structure that defines a specific plan of transitions / operations that can be used
/// by both operations and intentions. The operations/intentions that is used in the SOP will be copied into new
/// operations/intentions that will be part of the runner but only activated when the parent is activated.
/// When a planner picks an operation or ability that contains a SOP, the SOP will run based on its preconditions


use super::*;
use std::collections::HashMap;



#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SOP {
    Sequence(Vec<SOP>),
    Parallel(Vec<SOP>),
    Alternative(Vec<SOP>),
    Arbitrary(Vec<SOP>),
    Operation(SPPath), // if an inline operation is defined, create a new operation/ability with a path before including it here
}

impl Default for SOP {
    fn default() -> Self {
        SOP::Parallel(Vec::new())
    }
}

impl SOP {
    pub fn extract_guards(
        &self,
        activation_predicate: Predicate,
        initial_value: SPValue,
        _executing_value: SPValue,
        finished_value: SPValue,
    ) -> HashMap<SPPath, Predicate> {
        let all_paths = SOP::get_paths(self, false, false);
        let mut pred_map: HashMap<SPPath, Vec<Predicate>> = all_paths.into_iter().map(|p| {
            (p.clone(), vec!(activation_predicate.clone()))
        }).collect();
        SOP::insert_preds(self, &mut pred_map, initial_value, finished_value);
        pred_map.into_iter().map(|(p,xs)| {
            (p, Predicate::AND(xs))
        }).collect()
    }

    fn insert_preds(sop: &SOP, pred_map: &mut HashMap<SPPath, Vec<Predicate>>, initial_value: SPValue, finished_value: SPValue) {
        match sop {
            SOP::Operation(_) => {},
            SOP::Sequence(xs) => {
                xs.iter().for_each(|x| SOP::insert_preds(x, pred_map, initial_value.clone(), finished_value.clone()));
                xs.as_slice().windows(2).for_each(|ps| {
                    match ps {
                        [a, b] => {
                            let pred_from_a = SOP::get_post_cond(a, finished_value.clone());
                            SOP::insert_into_map(b, pred_from_a, pred_map);
                        },
                        _ => {}
                    }
                });
            }
            SOP::Parallel(xs) => {
                xs.iter().for_each(|x| SOP::insert_preds(x, pred_map, initial_value.clone(), finished_value.clone()));
            }
            SOP::Alternative(xs) => {
                let pred = SOP::get_pre_cond(sop, initial_value.clone());
                SOP::insert_into_map(sop, pred, pred_map);
                xs.iter().for_each(|x| SOP::insert_preds(x, pred_map, initial_value.clone(), finished_value.clone()));
            },
            SOP::Arbitrary(xs) => {
                let preds: Vec<Predicate> = xs.iter().map(|x| {
                    SOP::insert_preds(x, pred_map, initial_value.clone(), finished_value.clone());
                    let pre = SOP::get_pre_cond(x, initial_value.clone());
                    let post = SOP::get_post_cond(x, finished_value.clone());
                    Predicate::OR(vec!(pre, post))
                }).collect();
                SOP::insert_into_map(sop, Predicate::AND(preds), pred_map);
            },
        }
    }

    fn insert_into_map(sop: &SOP, pred: Predicate, pred_map:&mut HashMap<SPPath, Vec<Predicate>>) {
        let first_op = SOP::get_paths(sop, true, false);
        first_op.iter().for_each(|&p| {
            let x = pred_map.entry(p.clone()).or_insert_with(|| vec!());
            x.push(pred.clone());
        });
    }

    fn get_post_cond(sop: &SOP, finished_value: SPValue) -> Predicate {
        let last = SOP::get_paths(sop, false, true);
        let preds = SOP::make_op_pred(&last, &finished_value);
        if preds.len() <= 1 {
            preds.first().unwrap_or(&Predicate::TRUE).clone()
        } else {
            match sop {
                SOP::Parallel(_) | SOP::Arbitrary(_) => Predicate::AND(preds),
                SOP::Alternative(_) => Predicate::OR(preds),
                _ => Predicate::TRUE
            }
        }
    }
    fn get_pre_cond(sop: &SOP, initial_value: SPValue) -> Predicate {
        let first = SOP::get_paths(sop, true, false);
        let preds = SOP::make_op_pred(&first, &initial_value);
        if preds.len() <= 1 {
            preds.first().unwrap_or(&Predicate::TRUE).clone()
        } else {
            match sop {
                SOP::Parallel(_) | SOP::Arbitrary(_) | SOP::Alternative(_) => Predicate::AND(preds),
                _ => Predicate::TRUE
            }
        }
    }

    fn make_op_pred(paths: &[&SPPath], v: &SPValue) -> Vec<Predicate> {
        paths.iter().map(|&p| {
            Predicate::EQ(PredicateValue::path(p.clone()),PredicateValue::value(v.clone()))
        }).collect()
    }

    fn get_paths(sop: &SOP, only_first: bool, only_last: bool) -> Vec<&SPPath> {
        let xs = match sop {
            SOP::Operation(p) => return vec!(p),
            SOP::Sequence(xs) => {
                if only_first && !xs.is_empty() {
                    let (head, _) = xs.split_at(1);
                    head
                } else if only_last && !xs.is_empty() {
                    let (_, tail) = xs.split_at(xs.len()-1);
                    tail
                } else {
                    xs
                }
            },
            SOP::Parallel(xs) => xs,
            SOP::Alternative(xs) => xs,
            SOP::Arbitrary(xs) => xs,
        };

        xs.iter().flat_map(|x| {
            SOP::get_paths(x, only_first, only_last)
        }).collect()
    }

    //fn get_all_paths(sop: &SOP) -> Vec<&SPPath>
}

#[cfg(test)]
mod test_sops {
    use super::*;

    #[test]
    fn get_paths() {
        let p1 = SPPath::from_string("p1");
        let p2 = SPPath::from_string("p2");
        let p3 = SPPath::from_string("p3");
        let p4 = SPPath::from_string("p4");
        let p5 = SPPath::from_string("p5");

        let alt = SOP::Alternative(vec!(
            SOP::Operation(p3.clone()),
            SOP::Sequence(vec!(
                SOP::Operation(p4.clone()),
                SOP::Operation(p5.clone()),
            ))
        ));

        let sop =
            SOP::Parallel(vec!(
                SOP::Sequence(vec!(
                    SOP::Operation(p1.clone()),
                    SOP::Operation(p2.clone()),
                )),
                alt.clone()
            ));

        println!("{:?}", sop);
        let res = SOP::get_paths(&sop, true, false);
        println!("sop first {:?}", res);
        assert_eq!(vec!(&p1, &p3, &p4), res);

        let res = SOP::get_paths(&alt, true, false);
        println!("alt first {:?}", res);
        assert_eq!(vec!(&p3, &p4), res);

        let res = SOP::get_paths(&sop, false, true);
        println!("sop last {:?}", res);
        assert_eq!(vec!(&p2, &p3, &p5), res);

        let res = SOP::get_paths(&alt, false, true);
        println!("alt last {:?}", res);
        assert_eq!(vec!(&p3, &p5), res);

        let res = SOP::get_paths(&sop, false, false);
        println!("sop all {:?}", res);
        assert_eq!(vec!(&p1, &p2, &p3, &p4, &p5), res);

        let res = SOP::get_paths(&alt, false, false);
        println!("alt all {:?}", res);
        assert_eq!(vec!(&p3, &p4, &p5), res);
    }

    #[test]
    fn insert_preds() {
        let p1 = SPPath::from_string("p1");
        let p2 = SPPath::from_string("p2");
        let p3 = SPPath::from_string("p3");
        let p4 = SPPath::from_string("p4");
        let p5 = SPPath::from_string("p5");

        let seq = SOP::Sequence(vec!(
            SOP::Operation(p1.clone()),
            SOP::Operation(p2.clone()),
            SOP::Operation(p3.clone()),
            SOP::Operation(p4.clone()),
        ));
        let mut m = HashMap::new();
        let res = SOP::insert_preds(&seq, &mut m, "i".to_spvalue(), "f".to_spvalue());
        m.iter().for_each(|kv| println!("map {:?}", kv));
        assert_eq!(m.get(&p1), None);
        assert_eq!(m.get(&p2), Some(&SOP::make_op_pred(&[&p1], &"f".to_spvalue())));
        assert_eq!(m.get(&p3), Some(&SOP::make_op_pred(&[&p2], &"f".to_spvalue())));
        assert_eq!(m.get(&p4), Some(&SOP::make_op_pred(&[&p3], &"f".to_spvalue())));

        let alt = SOP::Alternative(vec!(
            SOP::Operation(p1.clone()),
            SOP::Operation(p2.clone()),
            SOP::Operation(p3.clone()),
            SOP::Operation(p4.clone()),
        ));
        let mut m = HashMap::new();
        let res = SOP::insert_preds(&alt, &mut m, "i".to_spvalue(), "f".to_spvalue());
        m.iter().for_each(|kv| println!("map {:?}", kv));
        assert_eq!(m.get(&p1), Some(&vec!(Predicate::AND(SOP::make_op_pred(&[&p1, &p2, &p3, &p4], &"i".to_spvalue())))));
        assert_eq!(m.get(&p2), Some(&vec!(Predicate::AND(SOP::make_op_pred(&[&p1, &p2, &p3, &p4], &"i".to_spvalue())))));
        assert_eq!(m.get(&p3), Some(&vec!(Predicate::AND(SOP::make_op_pred(&[&p1, &p2, &p3, &p4], &"i".to_spvalue())))));
        assert_eq!(m.get(&p4), Some(&vec!(Predicate::AND(SOP::make_op_pred(&[&p1, &p2, &p3, &p4], &"i".to_spvalue())))));

        let par = SOP::Parallel(vec!(
            SOP::Operation(p1.clone()),
            SOP::Operation(p2.clone()),
            SOP::Operation(p3.clone()),
            SOP::Operation(p4.clone()),
        ));
        let mut m = HashMap::new();
        let res = SOP::insert_preds(&par, &mut m, "i".to_spvalue(), "f".to_spvalue());
        assert_eq!(m.get(&p1), None);
        assert_eq!(m.get(&p2), None);
        assert_eq!(m.get(&p3), None);
        assert_eq!(m.get(&p4), None);

        let arb = SOP::Arbitrary(vec!(
            SOP::Operation(p1.clone()),
            SOP::Operation(p2.clone()),
            SOP::Operation(p3.clone()),
            SOP::Operation(p4.clone()),
        ));
        let mut m = HashMap::new();
        let res = SOP::insert_preds(&arb, &mut m, "i".to_spvalue(), "f".to_spvalue());
        m.iter().for_each(|kv| println!("map {:?}", kv));
        assert_eq!(m.get(&p1), Some(&vec!(
            Predicate::AND(vec!(
                Predicate::OR(vec!(
                    SOP::make_op_pred(&[&p1], &"i".to_spvalue()).first().unwrap().clone(),
                    SOP::make_op_pred(&[&p1], &"f".to_spvalue()).first().unwrap().clone(),
                )),
                Predicate::OR(vec!(
                    SOP::make_op_pred(&[&p2], &"i".to_spvalue()).first().unwrap().clone(),
                    SOP::make_op_pred(&[&p2], &"f".to_spvalue()).first().unwrap().clone(),
                )),
                Predicate::OR(vec!(
                    SOP::make_op_pred(&[&p3], &"i".to_spvalue()).first().unwrap().clone(),
                    SOP::make_op_pred(&[&p3], &"f".to_spvalue()).first().unwrap().clone(),
                )),
                Predicate::OR(vec!(
                    SOP::make_op_pred(&[&p4], &"i".to_spvalue()).first().unwrap().clone(),
                    SOP::make_op_pred(&[&p4], &"f".to_spvalue()).first().unwrap().clone(),
                )),
            )),
        )));

        let sop = SOP::Sequence(vec!(
            alt,
            SOP::Operation(p5.clone())
        ));
        let mut m = HashMap::new();
        let res = SOP::insert_preds(&sop, &mut m, "i".to_spvalue(), "f".to_spvalue());
        m.iter().for_each(|kv| println!("last map {:?}", kv));
        assert_eq!(m.get(&p5), Some(&vec!(Predicate::OR(SOP::make_op_pred(&[&p1, &p2, &p3, &p4], &"f".to_spvalue())))));

    }

    #[test]
    fn extract_guards() {
        let o = SPPath::from_string("o");
        let p1 = SPPath::from_string("p1");
        let p2 = SPPath::from_string("p2");
        let p3 = SPPath::from_string("p3");
        let p4 = SPPath::from_string("p4");
        let p5 = SPPath::from_string("p5");

        let alt = SOP::Alternative(vec!(
            SOP::Operation(p3.clone()),
            SOP::Sequence(vec!(
                SOP::Operation(p4.clone()),
                SOP::Operation(p5.clone()),
            ))
        ));

        let sop =
            SOP::Sequence(vec!(
                alt.clone(),
                SOP::Sequence(vec!(
                    SOP::Operation(p1.clone()),
                    SOP::Operation(p2.clone()),
                )),
            ));

        let res = sop.extract_guards(
            SOP::make_op_pred(&[&o], &"e".to_spvalue()).first().unwrap().clone(),
            "i".to_spvalue(),
            "e".to_spvalue(),
            "f".to_spvalue());
        res.iter().for_each(|kv| println!("res {:?}", kv));
    }
}
