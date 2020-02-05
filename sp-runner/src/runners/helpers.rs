use sp_domain::*;
use sp_runner_api::*;

use std::collections::HashMap;

use guard_extraction::*;

fn sp_pred_to_ex(p: &Predicate,
                 var_map: &HashMap<SPPath, (usize, Variable)>,
                 pred_map: &HashMap<SPPath, Predicate>) -> Ex {
    match p {
        Predicate::AND(v) => {
            let v = v.iter().map(|x| sp_pred_to_ex(x, var_map, pred_map)).collect();
            Ex::AND(v)
        },
        Predicate::OR(v) => {
            let v = v.iter().map(|x| sp_pred_to_ex(x, var_map, pred_map)).collect();
            Ex::OR(v)
        },
        Predicate::NOT(v) => {
            let v = sp_pred_to_ex(v, var_map, pred_map);
            Ex::NOT(Box::new(v))
        },
        Predicate::FALSE => Ex::FALSE,
        Predicate::TRUE => Ex::TRUE,
        Predicate::EQ(PredicateValue::SPPath(var, _),
                      PredicateValue::SPValue(value)) => {
            if pred_map.contains_key(var) {
                if value == &SPValue::Bool(false) {
                    Ex::NOT(Box::new(sp_pred_to_ex(pred_map.get(var).unwrap(), var_map, pred_map)))
                } else if value == &SPValue::Bool(true) {
                    sp_pred_to_ex(pred_map.get(var).unwrap(), var_map, pred_map)
                } else {
                    panic!("predicates must be boolean");
                }
            } else if var_map.contains_key(var) {
                let (index, var) = var_map.get(var).unwrap();

                let value = match value {
                    SPValue::Bool(b) => Value::Bool(*b),
                    v => {
                        // find index in domain.
                        let dom = var.domain();
                        Value::InDomain(dom.iter().position(|e| e == v).unwrap())
                    }
                };

                Ex::EQ(*index, value)
            } else {
                panic!("VAR {:?}, VALUE {:?}", var, value)
            }
        },
        Predicate::EQ(PredicateValue::SPPath(var, _),
                      PredicateValue::SPPath(other, _)) => {
            if var_map.contains_key(var) && var_map.contains_key(other) {
                let (index, var) = var_map.get(var).unwrap();
                let (index2, other) = var_map.get(other).unwrap();
                Ex::EQ(*index, Value::Var(*index2))
            } else {
                panic!("VAR {:?}, OTHER {:?}", var, other)
            }
        },
        Predicate::NEQ(PredicateValue::SPPath(var, _),
                       PredicateValue::SPPath(other, _)) => {
            if var_map.contains_key(var) && var_map.contains_key(other) {
                let (index, var) = var_map.get(var).unwrap();
                let (index2, other) = var_map.get(other).unwrap();
                Ex::NOT(Box::new(Ex::EQ(*index, Value::Var(*index2))))
            } else {
                panic!("VAR {:?}, OTHER {:?}", var, other)
            }
        },
        Predicate::NEQ(PredicateValue::SPPath(var, _),
                      PredicateValue::SPValue(value)) => {
            if pred_map.contains_key(var) {
                if value == &SPValue::Bool(false) {
                    sp_pred_to_ex(pred_map.get(var).unwrap(), var_map, pred_map)
                } else if value == &SPValue::Bool(true) {
                    Ex::NOT(Box::new(sp_pred_to_ex(pred_map.get(var).unwrap(), var_map, pred_map)))
                } else {
                    panic!("predicates must be boolean");
                }
            } else if var_map.contains_key(var) {
                let (index, var) = var_map.get(var).unwrap();

                let value = match value {
                    SPValue::Bool(b) => Value::Bool(*b),
                    v => {
                        // find index in domain.
                        let dom = var.domain();
                        Value::InDomain(dom.iter().position(|e| e == v).unwrap())
                    }
                };

                Ex::NOT(Box::new(Ex::EQ(*index, value)))
            } else {
                panic!("VAR {:?}, VALUE {:?}", var, value)
            }
        },
        x => panic!("NO X {:?}", x)
    }
}


fn ex_to_sp_pred(e: &Ex,
                 var_map: &HashMap<SPPath, (usize, Variable)>,
                 pred_map: &HashMap<SPPath, Predicate>) -> Predicate {
    match e {
        Ex::AND(v) => {
            let v = v.iter().map(|x| ex_to_sp_pred(x, var_map, pred_map)).collect();
            Predicate::AND(v)
        },
        Ex::OR(v) => {
            let v = v.iter().map(|x| ex_to_sp_pred(x, var_map, pred_map)).collect();
            Predicate::OR(v)
        },
        Ex::NOT(v) => {
            let v = ex_to_sp_pred(v, var_map, pred_map);
            Predicate::NOT(Box::new(v))
        },
        Ex::FALSE => Predicate::FALSE,
        Ex::TRUE => Predicate::TRUE,
        Ex::EQ(index, value) => {
            let var = var_map.values().find(|(idx, _)| idx == index).unwrap().1.clone();
            let val = match value {
                Value::Bool(b) => PredicateValue::SPValue(SPValue::Bool(*b)),
                Value::InDomain(vid) => PredicateValue::SPValue(var.domain().get(*vid).unwrap().clone()),
                Value::Var(other) => {
                    let other = var_map.values().find(|(idx, _)| idx == other).unwrap().1.clone();
                    PredicateValue::SPPath(other.path().clone(), None)
                },
                Value::Free => panic!("not supported")
            };
            Predicate::EQ(PredicateValue::SPPath(var.path().clone(), None), val)
        },
        Ex::VAR(index) => {
            let var = var_map.values().find(|(idx, _)| idx == index).unwrap().1.clone();
            let val = PredicateValue::SPValue(SPValue::Bool(true));
            Predicate::EQ(PredicateValue::SPPath(var.path().clone(), None), val)
        },
    }
}

fn sp_action_to_ex(a: &Action,
                   var_map: &HashMap<SPPath, (usize, Variable)>,
                   pred_map: &HashMap<SPPath, Predicate>) -> Ex {
    let (index, var) = var_map.get(&a.var).expect(&format!("variable not found! {}", a.var));
    match &a.value {
        Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
            // assign p to var
            let x = var_map.get(p);
            assert!(x.is_some());
            let (other, var) = var_map.get(p).unwrap();
            Ex::EQ(*index, Value::Var(*other))
        },
        Compute::PredicateValue(PredicateValue::SPValue(value)) => {
            // assign value to var
            let value = match value {
                SPValue::Bool(b) => Value::Bool(*b),
                v => {
                    // find index in domain.
                    let dom = var.domain();
                    Value::InDomain(dom.iter().position(|e| e == v).unwrap())
                }
            };

            Ex::EQ(*index, value)
        },
        x => panic!("TODO: {:?}", x)
    }
}

fn sp_action_to_ac(a: &Action,
                   var_map: &HashMap<SPPath, (usize, Variable)>,
                   pred_map: &HashMap<SPPath, Predicate>) -> Ac {
    let (index, var) = var_map.get(&a.var).expect(&format!("variable not found! {}", a.var));
    let val = match &a.value {
        Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
            // assign p to var
            let x = var_map.get(p);
            if x.is_some() {
                let (other, var) = var_map.get(p).unwrap();
                Value::Var(*other)
            } else {
                println!("ASSUMING FREE VARIABLE");
                Value::Free
            }
        },
        Compute::PredicateValue(PredicateValue::SPValue(value)) => {
            // assign value to var
            match value {
                SPValue::Bool(b) => Value::Bool(*b),
                v => {
                    // find index in domain.
                    let dom = var.domain();
                    Value::InDomain(dom.iter().position(|e| e == v).unwrap())
                }
            }
        },
        x => panic!("TODO: {:?}", x)
    };
    Ac {
        var: *index,
        val: val,
    }
}


pub fn extract_guards(model: &Model, init: &Predicate) -> (HashMap<String, Predicate>, Predicate) {
    let items = model.items();

    // collect all OFFLINE specs.
    let specs: Vec<Spec> = items.iter().flat_map(|i| match i {
        SPItem::Spec(s) => Some(s),
        _ => None,
    }).cloned().collect();

    // no specs? we're done.
    if specs.is_empty() {
        return (HashMap::new(), init.clone())
    }

    // find "ab" transitions from resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let trans: Vec<_> = resources.iter().flat_map(|r| r.make_global_transitions()).collect();

    let preds: Vec<_> = resources.iter().flat_map(|r| r.make_global_state_predicates()).collect();
    let pred_map: HashMap<_,_> = preds.iter().flat_map(|p| {
        match p.variable_type() {
            VariableType::Predicate(x) => Some((p.path().clone(), x.clone())),
            _ => None
        }
    }).collect();

    let mut vars: Vec<_> = resources.iter().flat_map(|r| r.get_variables()).collect();

    let sub_items: Vec<_> = resources.iter().flat_map(|r| r.sub_items()).collect();
    let sub_item_variables: Vec<_> = sub_items.iter().flat_map(|i| match i {
        SPItem::Variable(v) => Some(v.clone()),
        _ => None,
    }).collect();
    vars.extend(sub_item_variables);


    let mut c = Context::default(); // guard extraction context
    let mut var_map = HashMap::new();

    for v in &vars {
        println!("{} {:?} {}", v.path(), v.value_type(), v.domain().len());

        let index = if v.value_type() == SPValueType::Bool {
            c.add_bool(&v.path().to_string())
        } else {
            c.add_enum(&v.path().to_string(), v.domain().len())
        };
        var_map.insert(v.path().clone(), (index, v.clone()));
    }

    // that was all vars, now add the transitions...
    let b = buddy_rs::take_manager(10000, 10000);
    let (gm, new_initial) = {
    let mut bc = BDDContext::from(&c, &b);

    for t in &trans {
        let guard = sp_pred_to_ex(t.guard(), &var_map, &pred_map);
        println!("guard: {:?}", guard);

        let actions: Vec<_> = t.actions().iter().map(|a| sp_action_to_ex(a, &var_map, &pred_map) ).collect();
        // println!("action: {:?}", actions);

        let effects: Vec<_> = t.effects().iter().map(|a| sp_action_to_ex(a, &var_map, &pred_map) ).collect();
        //println!("effects: {:?}", effects);

        let mut a = Vec::new();
        a.extend(actions.iter().cloned());
        a.extend(effects.iter().cloned());
        let a = Ex::AND(a);
        println!("all a/effects: {:?}", a);

        if t.controlled() {
            bc.c_trans(&t.path().to_string(), guard, a);
        } else {
            bc.uc_trans(&t.path().to_string(), guard, a);
        }
    }

    // pull out all specs.
    let forbidden =  Ex::AND(specs.iter().map(|s| {
        // todo... for now we just care about the first expression in each spec
        let s = s.always().first().unwrap().clone();
        // forbidden = not always
        Ex::NOT(Box::new(sp_pred_to_ex(&s, &var_map, &pred_map)))
    }).collect());

    // let s = spec.always().first().unwrap().clone(); // lazy
    // let sex = sp_pred_to_ex(&s, &var_map, &pred_map);
    // println!("forbidden {:?}", sex);

    let forbidden = bc.from_expr(&forbidden);
    // println!("{:#?}", bc);


    let init = sp_pred_to_ex(&init, &var_map, &pred_map);
    println!("init {:?}", init);

    let init = bc.from_expr(&init);


    let initial = bc.from_expr(&Ex::TRUE); // all states
    println!("bdd one {:?}", initial);

    let (reachable, bad, controllable) = bc.controllable(&initial, &forbidden);

    let new_guards = bc.compute_guards(&controllable, &bad);

    for (trans, guard) in &new_guards {
        let s = c.pretty_print(&guard);
        // println!("NEW GUARD FOR {}: {}", trans, s);
        // println!("NEW GUARD FOR {}: {:?}", trans, guard);
        let sppred = ex_to_sp_pred(&guard, &var_map, &pred_map);
        println!("sppred guard {}: {:?}", trans, sppred);
    }

    let new_initial = bc.to_expr(&controllable);
    let new_initial = ex_to_sp_pred(&new_initial, &var_map, &pred_map);


    let gm = new_guards.iter().map(|(path,guard)| (path.clone(),
                                                   ex_to_sp_pred(&guard, &var_map, &pred_map))).collect();
        (gm, new_initial)
    };

    buddy_rs::return_manager(b);
    (gm, new_initial)
}

// refines an invariant according to the model
pub fn refine_invariant(model: &Model, invariant: &Predicate) -> Predicate {
    let items = model.items();

    // find "ab" transitions from resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let trans: Vec<_> = resources.iter().flat_map(|r| r.make_global_transitions()).collect();

    let preds: Vec<_> = resources.iter().flat_map(|r| r.make_global_state_predicates()).collect();
    let pred_map: HashMap<_,_> = preds.iter().flat_map(|p| {
        match p.variable_type() {
            VariableType::Predicate(x) => Some((p.path().clone(), x.clone())),
            _ => None
        }
    }).collect();

    let mut vars: Vec<_> = resources.iter().flat_map(|r| r.get_variables()).collect();

    let sub_items: Vec<_> = resources.iter().flat_map(|r| r.sub_items()).collect();
    let sub_item_variables: Vec<_> = sub_items.iter().flat_map(|i| match i {
        SPItem::Variable(v) => Some(v.clone()),
        _ => None,
    }).collect();
    vars.extend(sub_item_variables);

    let mut c = Context::default(); // guard extraction context
    let mut var_map = HashMap::new();

    for v in &vars {
        println!("{} {:?} {}", v.path(), v.value_type(), v.domain().len());

        let index = if v.value_type() == SPValueType::Bool {
            c.add_bool(&v.path().to_string())
        } else {
            c.add_enum(&v.path().to_string(), v.domain().len())
        };
        var_map.insert(v.path().clone(), (index, v.clone()));
    }

    // that was all vars, now add the transitions...
    let b = buddy_rs::take_manager(10000, 10000);
    let new_invariant = {

        let mut bc = BDDContext::from(&c, &b);

        for t in &trans {
            let guard = sp_pred_to_ex(t.guard(), &var_map, &pred_map);
            println!("guard: {:?}", guard);

            if t.controlled() {

                let actions: Vec<_> = t.actions().iter().map(|a| sp_action_to_ac(a, &var_map, &pred_map) ).collect();
                // println!("action: {:?}", actions);

                let effects: Vec<_> = t.effects().iter().map(|a| sp_action_to_ac(a, &var_map, &pred_map) ).collect();
                //println!("effects: {:?}", effects);

                let mut a = Vec::new();
                a.extend(actions.iter().cloned());
                a.extend(effects.iter().cloned());
                println!("all actions and effects: {:?}", a);

                bc.c_trans2(&t.path().to_string(), guard, &a);
            } else {

                let actions: Vec<_> = t.actions().iter().map(|a| sp_action_to_ex(a, &var_map, &pred_map) ).collect();
                // println!("action: {:?}", actions);

                let effects: Vec<_> = t.effects().iter().map(|a| sp_action_to_ex(a, &var_map, &pred_map) ).collect();
                //println!("effects: {:?}", effects);

                let mut a = Vec::new();
                a.extend(actions.iter().cloned());
                a.extend(effects.iter().cloned());
                let a = Ex::AND(a);
                println!("all a/effects: {:?}", a);



                bc.uc_trans(&t.path().to_string(), guard, a);
            }
        }

        // forbidden = all states NOT conforming to the invariant
        let forbidden = Ex::NOT(Box::new(sp_pred_to_ex(&invariant, &var_map, &pred_map)));

        let forbidden = bc.from_expr(&forbidden);
        let new_invariant = bc.extend_forbidden(&forbidden);
        let new_invariant = bc.b.not(&new_invariant);
        bc.to_expr(&new_invariant)
    };

    println!("RETURNING");
    buddy_rs::return_manager(b);
    println!("RETURNED");
    ex_to_sp_pred(&new_invariant, &var_map, &pred_map)
}

#[test]
fn test_guard_extraction() {
    use crate::testing::*;

    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1")));
    m.add_item(SPItem::Resource(make_dummy_robot("r2")));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();

    // (offline) Specifications
    let table_zone = p!(!( [p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
    m.add_item(SPItem::Spec(Spec::new("table_zone", vec![table_zone])));

    let (new_guards, new_initial) = extract_guards(&m, &Predicate::TRUE);

    assert_eq!(new_guards.len(), 4);
    assert_ne!(new_initial, Predicate::TRUE);

    assert!(false);
}

#[test]
fn test_invariant_refinement() {
    use crate::testing::*;

    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1")));
    m.add_item(SPItem::Resource(make_dummy_robot("r2")));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();

    // (offline) Specifications
    let table_zone = p!(!( [p:r1_p_a == "at"] && [p:r2_p_a == "at"]));

    let new_table_zone = refine_invariant(&m, &table_zone);
    // println!("new spec: {}", new_table_zone);

    m.add_item(SPItem::Spec(Spec::new("table_zone", vec![new_table_zone])));

    let rm = make_runner_model_no_ge(&m);

    crate::planning::generate_offline_nuxvm(&rm, &Predicate::TRUE);

    assert!(false);
}

pub fn make_runner_model_no_ge(model: &Model) -> RunnerModel {
    let items = model.items();

    // find "ab" transitions from resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let trans: Vec<_> = resources.iter().flat_map(|r| r.make_global_transitions()).collect();

    let ab_ctrl: Vec<_> = trans.iter().filter(|t|t.controlled()).cloned().collect();
    let ab_un_ctrl = trans.iter().filter(|t|!t.controlled()).cloned().collect();

    let preds: Vec<_> = resources.iter().flat_map(|r| r.make_global_state_predicates()).collect();

    // TODO: handle resource "sub items"

    // TODO: add global transitions.


    // TODO: add global state.?

    // add global op transitions (all automatic for now).
    let global_ops: Vec<&Operation> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Operation(o) => Some(o),
            _ => None,
        })
        .collect();

    let global_ops_ctrl: Vec<_> = global_ops.iter().flat_map(|o|o.start()).cloned().collect();
    let global_ops_un_ctrl: Vec<_> = global_ops.iter().flat_map(|o|o.finish()).cloned().collect();

    let global_goals: Vec<IfThen> = global_ops.iter().flat_map(|o|o.goal().as_ref()).cloned().collect();

    // println!("{:?}", global_ops);

    // println!("{:?}", resources);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions {
            ctrl: global_ops_ctrl,
            un_ctrl: global_ops_un_ctrl,
        },
        ab_transitions: RunnerTransitions {
            ctrl: ab_ctrl,
            un_ctrl: ab_un_ctrl,
        },
        plans: RunnerPlans::default(),
        state_predicates: preds,
        goals: global_goals,
        model: model.clone(), // TODO: borrow?
    };

    return rm;
}

pub fn make_runner_model(model: &Model) -> RunnerModel {

    let (new_guards, new_initial) = extract_guards(&model, &Predicate::TRUE);


    let items = model.items();

    // find "ab" transitions from resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let trans: Vec<_> = resources.iter().flat_map(|r| r.make_global_transitions()).collect();

    let mut ab_ctrl: Vec<_> = trans.iter().filter(|t|t.controlled()).cloned().collect();
    ab_ctrl.iter_mut().for_each(|ct| {
        match new_guards.get(&ct.path().to_string()) {
            Some(t) => // AND with new guards.
            {
                println!("UPDATING GUARD FOR TRANS {}", ct.path());
                *ct.mut_guard() = Predicate::AND(vec![ct.guard().clone(), t.clone()]);
            },
            None => {},
        }
    });
    let ab_un_ctrl = trans.iter().filter(|t|!t.controlled()).cloned().collect();

    let preds: Vec<_> = resources.iter().flat_map(|r| r.make_global_state_predicates()).collect();

    // TODO: handle resource "sub items"

    // TODO: add global transitions.


    // TODO: add global state.?

    // add global op transitions (all automatic for now).
    let global_ops: Vec<&Operation> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Operation(o) => Some(o),
            _ => None,
        })
        .collect();

    let global_ops_ctrl: Vec<_> = global_ops.iter().flat_map(|o|o.start()).cloned().collect();
    let global_ops_un_ctrl: Vec<_> = global_ops.iter().flat_map(|o|o.finish()).cloned().collect();

    let global_goals: Vec<IfThen> = global_ops.iter().flat_map(|o|o.goal().as_ref()).cloned().collect();

    // println!("{:?}", global_ops);

    // println!("{:?}", resources);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions {
            ctrl: global_ops_ctrl,
            un_ctrl: global_ops_un_ctrl,
        },
        ab_transitions: RunnerTransitions {
            ctrl: ab_ctrl,
            un_ctrl: ab_un_ctrl,
        },
        plans: RunnerPlans::default(),
        state_predicates: preds,
        goals: global_goals,
        model: model.clone(), // TODO: borrow?
    };

    crate::planning::generate_offline_nuxvm(&rm, &new_initial);

    return rm;
}

pub fn make_initial_state(model: &Model) -> SPState {
    let items = model.items();

    // find all variables in resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let mut s = SPState::default();

    for r in &resources {
        s.extend(r.make_initial_state());
    }

    // add global state
    let vars: Vec<&Variable> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Variable(v) => Some(v),
            _ => None,
        })
        .collect();

    let state: Vec<_> = vars.iter().map(|v| (v.node().path().clone(), v.initial_value())).collect();
    s.extend(SPState::new_from_values(&state[..]));

    return s;
}
