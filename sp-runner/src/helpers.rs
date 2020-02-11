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

fn sp_action_to_ac(a: &Action,
                   var_map: &HashMap<SPPath, (usize, Variable)>,
                   pred_map: &HashMap<SPPath, Predicate>) -> Ac {
    let (index, var) = var_map.get(&a.var).expect(&format!("variable not found! {}", a.var));
    let val = match &a.value {
        Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
            // assign p to var
            let (other, var) = var_map.get(p).expect("variable not found");
            Value::Var(*other)
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
        Compute::Any => Value::Free,
        x => panic!("TODO: {:?}", x)
    };
    Ac {
        var: *index,
        val: val,
    }
}

fn make_context<'a>(model: &TransitionSystemModel) ->
    (Context, HashMap<SPPath, (usize, Variable)>, HashMap<SPPath, Predicate>) {
    let mut c = Context::default();

    let mut var_map = HashMap::new();
    for v in &model.vars {
        let index = if v.value_type() == SPValueType::Bool {
            c.add_bool(&v.path().to_string())
        } else {
            c.add_enum(&v.path().to_string(), v.domain().len())
        };
        var_map.insert(v.path().clone(), (index, v.clone()));
    }
    let pred_map: HashMap<_,_> = model.state_predicates.iter().flat_map(|p| {
        match p.variable_type() {
            VariableType::Predicate(x) => Some((p.path().clone(), x.clone())),
            _ => None
        }
    }).collect();

    for t in &model.transitions {
        let guard = sp_pred_to_ex(t.guard(), &var_map, &pred_map);
        let actions: Vec<_> = t.actions().iter().map(|a| sp_action_to_ac(a, &var_map, &pred_map) ).collect();
        let effects: Vec<_> = t.effects().iter().map(|a| sp_action_to_ac(a, &var_map, &pred_map) ).collect();
        let mut a = Vec::new();
        a.extend(actions.iter().cloned());
        a.extend(effects.iter().cloned());

        if t.controlled() {
            c.add_c_trans(&t.path().to_string(), &guard, &a);
        } else {
            c.add_uc_trans(&t.path().to_string(), &guard, &a);
        }
    }

    (c, var_map, pred_map)
}

pub fn extract_guards(model: &TransitionSystemModel, initial: &Predicate) -> (HashMap<String, Predicate>, Predicate) {
    let (c, var_map, pred_map) = make_context(&model);

    // pull out all specs.
    let forbidden =  Ex::OR(model.specs.iter().map(|s| {
        // forbidden = not always
        Ex::NOT(Box::new(sp_pred_to_ex(s.invariant(), &var_map, &pred_map)))
    }).collect());

    let initial = sp_pred_to_ex(&initial, &var_map, &pred_map);

    let (ng, supervisor) = c.compute_guards(&initial, &forbidden);

    let ng = ng.into_iter().map(|(n,e)| (n, ex_to_sp_pred(&e, &var_map, &pred_map))).collect();
    let supervisor = ex_to_sp_pred(&supervisor, &var_map, &pred_map);
    (ng, supervisor)
}

// refines an invariant according to the model
pub fn refine_invariant(model: &Model, invariant: &Predicate) -> Predicate {
    let model = TransitionSystemModel::from(&model);

    let (c, var_map, pred_map) = make_context(&model);

    // forbidden = all states NOT conforming to the invariant
    let forbidden = Ex::NOT(Box::new(sp_pred_to_ex(&invariant, &var_map, &pred_map)));

    let forbidden = c.extend_forbidden(&forbidden);

    let new_invariant = Ex::NOT(Box::new(forbidden));

    ex_to_sp_pred(&new_invariant, &var_map, &pred_map)
}

fn update_guards(ts_model: &mut TransitionSystemModel, ng: &HashMap<String, Predicate>) {
    ts_model.transitions.iter_mut().for_each(|ot| {
        match ng.get(&ot.path().to_string()) {
            Some(nt) => {
                println!("UPDATING GUARD FOR TRANS {}:", ot.path());
                println!("{}", nt);
                *ot.mut_guard() = Predicate::AND(vec![ot.guard().clone(), nt.clone()]);
            },
            None => {},
        }
    });
}

pub fn make_runner_model(model: &Model) -> RunnerModel {
    let mut ts_model = TransitionSystemModel::from(&model);

    // TODO: initial conditions...
    let (new_guards, _new_initial) = extract_guards(&ts_model, &Predicate::TRUE);

    // TODO: this feel wrong... perhaps change their type once processed instead.
    ts_model.specs.clear();

    // TODO: right now its very cumbersome to update the original Model.
    update_guards(&mut ts_model, &new_guards);

    // runner model has everything from planning model + operations and their global state
    let items = model.items();

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

    let rm = RunnerModel {
        op_transitions: RunnerTransitions {
            ctrl: global_ops_ctrl,
            un_ctrl: global_ops_un_ctrl,
        },
        ab_transitions: RunnerTransitions {
            ctrl: ts_model.transitions.iter().filter(|t|t.controlled()).cloned().collect(),
            un_ctrl: ts_model.transitions.iter().filter(|t|!t.controlled()).cloned().collect(),
        },
        plans: RunnerPlans::default(),
        state_predicates: ts_model.state_predicates.clone(),
        goals: global_goals,
        model: ts_model.clone(), // TODO: borrow?
    };

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


#[test]
fn test_guard_extraction() {
    use crate::testing::*;

    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();

    // (offline) Specifications
    let table_zone = p!(!( [p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
    m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

    let mut ts_model = TransitionSystemModel::from(&m);
    let (new_guards, new_initial) = extract_guards(&ts_model, &Predicate::TRUE);
    update_guards(&mut ts_model, &new_guards);

    ts_model.specs.clear();
    crate::planning::generate_offline_nuxvm(&ts_model, &new_initial);

    assert_eq!(new_guards.len(), 4);
    assert_ne!(new_initial, Predicate::TRUE);
}

#[test]
fn test_invariant_refinement() {
    use crate::testing::*;

    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
    m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
    let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();

    // (offline) Specifications
    let table_zone = p!(!( [p:r1_p_a == "at"] && [p:r2_p_a == "at"]));

    let new_table_zone = refine_invariant(&m, &table_zone);
    // println!("new spec: {}", new_table_zone);

    m.add_item(SPItem::Spec(Spec::new("table_zone", new_table_zone)));

    let ts_model = TransitionSystemModel::from(&m);
    crate::planning::generate_offline_nuxvm(&ts_model, &Predicate::TRUE);

    assert!(false);
}
