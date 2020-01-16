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
        Predicate::EQ(PredicateValue::SPPath(var),
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
        Predicate::NEQ(PredicateValue::SPPath(var),
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
                    PredicateValue::SPPath(other.get_path())
                }
            };
            Predicate::EQ(PredicateValue::SPPath(var.get_path()), val)
        },
        Ex::VAR(index) => {
            let var = var_map.values().find(|(idx, _)| idx == index).unwrap().1.clone();
            let val = PredicateValue::SPValue(SPValue::Bool(true));
            Predicate::EQ(PredicateValue::SPPath(var.get_path()), val)
        },
    }
}

fn sp_action_to_ex(a: &Action,
                   var_map: &HashMap<SPPath, (usize, Variable)>,
                   pred_map: &HashMap<SPPath, Predicate>) -> Ex {
    let (index, var) = var_map.get(&a.var).unwrap();
    match &a.value {
        Compute::PredicateValue(PredicateValue::SPPath(p)) => {
            // assign p to var
            panic!("todo")
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


pub fn extract_guards(model: &Model, init: &Predicate) ->
    (HashMap<String, Predicate>, Predicate) {
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
            VariableType::Predicate(x) => Some((p.get_path(), x.clone())),
            _ => None
        }
    }).collect();

    let vars: Vec<_> = resources.iter().flat_map(|r| r.get_variables()).collect();

    let mut c = Context::new(); // guard extraction context
    let mut var_map = HashMap::new();

    for v in &vars {
        println!("{} {:?} {}", v.get_path(), v.value_type(), v.domain().len());

        let index = if v.value_type() == SPValueType::Bool {
            c.add_bool(&v.get_path().to_string())
        } else {
            c.add_enum(&v.get_path().to_string(), v.domain().len())
        };
        var_map.insert(v.get_path().clone(), (index, v.clone()));
    }

    // that was all vars, now add the transitions...
    let mut bc = BDDContext::from(&c);

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
            bc.c_trans(&t.get_path().to_string(), guard, a);
        } else {
            bc.uc_trans(&t.get_path().to_string(), guard, a);
        }
    }


    // pull out all specs.
    let forbidden =  Ex::AND(model.items().iter().flat_map(|i| match i {
        SPItem::Spec(s) => {
            // todo... now we just care about the first expression in each spec
            let s = s.always().first().unwrap().clone();
            let ex = Ex::NOT(Box::new(sp_pred_to_ex(&s, &var_map, &pred_map)));
            // forbidden = not always
            Some(ex)
        },
        _ => None,
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

    let (reachable, bad, controllable) = bc.controllable(initial, forbidden);

    let state_count = satcount(&mut bc.b, controllable, bc.num_vars);
    println!("Nbr of states in supervisor: {}\n", state_count);
    let reachable_state_count = satcount(&mut bc.b, reachable, bc.num_vars);
    println!("Nbr of reachable states: {}\n", reachable_state_count);


    let new_guards = bc.compute_guards(controllable, bad);

    for (trans, guard) in &new_guards {
        let s = c.pretty_print(&guard);
        // println!("NEW GUARD FOR {}: {}", trans, s);
        // println!("NEW GUARD FOR {}: {:?}", trans, guard);
        let sppred = ex_to_sp_pred(&guard, &var_map, &pred_map);
        println!("sppred guard {}: {:?}", trans, sppred);
    }

    let new_initial = bc.to_expr(controllable);
    let new_initial = ex_to_sp_pred(&new_initial, &var_map, &pred_map);


    let gm = new_guards.iter().map(|(path,guard)| (path.clone(),
                                                   ex_to_sp_pred(&guard, &var_map, &pred_map))).collect();
    (gm, new_initial)
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
        match new_guards.get(&ct.get_path().to_string()) {
            Some(t) => // AND with new guards.
            {
                println!("UPDATING GUARD FOR TRANS {}", ct.get_path());
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
        invariants: Vec::new(),
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

    for v in &vars {
        let _r = s.insert(&v.node().global_path().as_ref().unwrap().to_sp(),
                          AssignStateValue::SPValue(v.initial_value()));
    }

    return s;
}
