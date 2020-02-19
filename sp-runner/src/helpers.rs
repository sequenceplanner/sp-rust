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
                let (index, _var) = var_map.get(var).unwrap();
                let (index2, _other) = var_map.get(other).unwrap();
                Ex::EQ(*index, Value::Var(*index2))
            } else {
                panic!("VAR {:?}, OTHER {:?}", var, other)
            }
        },
        Predicate::NEQ(PredicateValue::SPPath(var, _),
                       PredicateValue::SPPath(other, _)) => {
            if var_map.contains_key(var) && var_map.contains_key(other) {
                let (index, _var) = var_map.get(var).unwrap();
                let (index2, _other) = var_map.get(other).unwrap();
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
                   _pred_map: &HashMap<SPPath, Predicate>) -> Ac {
    let (index, var) = var_map.get(&a.var).expect(&format!("variable not found! {}", a.var));
    let val = match &a.value {
        Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
            // assign p to var
            let (other, _var) = var_map.get(p).expect("variable not found");
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

use cryptominisat::*;
pub fn plan(model: &TransitionSystemModel, initial: &Predicate, goals: &[(Predicate,Predicate)]) -> i32 {
    let (c, var_map, pred_map) = make_context(&model);

    let initial = sp_pred_to_ex(&initial, &var_map, &pred_map);
    let goals: Vec<(Ex,Ex)> = goals
        .iter()
        .map(|(i,g)|
             (sp_pred_to_ex(&i, &var_map, &pred_map),
              sp_pred_to_ex(&g, &var_map, &pred_map))
             ).collect();

    let sat_model = c.model_as_sat_model(&initial, &goals);

    // println!("{:?}", sat_model);
    println!("start planning");

    let now = std::time::Instant::now();

    let mut s = Solver::new();

    type GLit = guard_extraction::Lit;
    type CLit = cryptominisat::Lit;

    // s.set_num_threads(4); play with this some day
    let nv = sat_model.norm_vars.len();
    let all_num_vars = sat_model.num_vars - nv;
    let mut vars: Vec<CLit> = (0 .. all_num_vars).map(|_v| s.new_var()).collect();

    let pairing: Vec<_> = sat_model.norm_vars
        .iter()
        .zip(sat_model.next_vars.iter())
        .map(|(x, y)| (*x, *y))
        .collect();

    let ci = |i| {
        sat_model.norm_vars.iter().position(|&r| r == i).unwrap()
    };

    let ti = |i| {
        i - sat_model.next_vars.len() // next does not exist...
    };

    let is_norm = |l: &GLit| {
        sat_model.norm_vars.contains(&l.var)
    };

    let is_next = |l: &guard_extraction::Lit| {
        sat_model.next_vars.contains(&l.var)
    };

    let is_ts = |l: &guard_extraction::Lit| {
        !is_norm(l) && !is_next(l)
    };

    // add clauses for the initial state
    for c in &sat_model.init_clauses {
        let clause: Vec<cryptominisat::Lit> = c.0.iter().map(|l| {
            if is_norm(l) {
                if l.neg { !vars[ci(l.var)] } else { vars[ci(l.var)] }
            } else if is_ts(l) {
                if l.neg { !vars[ti(l.var)] } else { vars[ti(l.var)] }
            } else {
                panic!("error");
            }
        }
        ).collect();
        s.add_clause(&clause);
    }

    // unroll transitions, goals, and later invariants up to n steps
    let mut steps = -1i32;
    for step in 0..20 {
        // println!("START STEP {}", step);
        let now = std::time::Instant::now();

        let goal_active = s.new_var();
        vars.push(goal_active);

        // println!("vars.len: {}", vars.len());

        // at each step we have  + 1 extra variables due to goal activation.
        let nv = all_num_vars + 1;

        if step > 0 {
            for c in &sat_model.model_clauses {
                // model clauses has cur and next. let cur refer to previous step.
                let clause: Vec<cryptominisat::Lit> = c.0.iter().map(|l| {
                    if is_norm(&l) {
                        if l.neg { !vars[ci(l.var)+(step-1)*nv] } else { vars[ci(l.var)+(step-1)*nv] }
                    } else if is_next(&l) {
                        let i = pairing.iter().find(|(_idx,jdx)| &l.var == jdx).unwrap().0;
                        if l.neg { !vars[ci(i)+step*nv] } else { vars[ci(i)+step*nv] }
                    } else {
                        if l.neg { !vars[ti(l.var)+(step-1)*nv] } else { vars[ti(l.var)+(step-1)*nv] }
                    }
                }).collect();
                s.add_clause(&clause);
            }
        }

        // add new goals
        for c in &sat_model.goal_clauses {
            let mut clause: Vec<cryptominisat::Lit> = c.0.iter().flat_map(|l| {
                if sat_model.norm_vars.contains(&l.var) {
                    if l.neg { Some(!vars[ci(l.var)+step*nv]) } else { Some(vars[ci(l.var)+step*nv]) }
                } else if is_ts(l) {
                    if l.neg { Some(!vars[ti(l.var)+step*nv]) } else { Some(vars[ti(l.var)+step*nv]) }
                } else {
                    panic!("error");
                    None
                }
            }).collect();

            s.add_clause(&clause);
        }

        // add all invar clauses
        for c in &sat_model.invar_clauses {
            let mut clause: Vec<cryptominisat::Lit> = c.0.iter().flat_map(|l| {
                if sat_model.norm_vars.contains(&l.var) {
                    if l.neg { Some(!vars[ci(l.var)+step*nv]) } else { Some(vars[ci(l.var)+step*nv]) }
                } else if is_ts(l) {
                    if l.neg { Some(!vars[ti(l.var)+step*nv]) } else { Some(vars[ti(l.var)+step*nv]) }
                } else {
                    panic!("error");
                    None
                }
            }).collect();

            s.add_clause(&clause);
        }

        // add a disjunction that allows the goal to be active in any time step
        // (so we can finish one goal early but then keep going)
        // for top in &sat_model.goal_tops {
        for (invar, top) in sat_model.invar_tops.iter().zip(sat_model.goal_tops.iter()) {
            let mut clause: Vec<cryptominisat::Lit> = Vec::new();
            for i in 0..(step+1) {
                let lit = if sat_model.norm_vars.contains(&top.var) {
                    panic!("unexpected");
                    if top.neg { Some(!vars[ci(top.var)+i*nv]) } else { Some(vars[ci(top.var)+i*nv]) }
                } else if is_ts(top) {
                    if top.neg { panic!("not good"); }
                    if top.neg { Some(!vars[ti(top.var)+i*nv]) } else { Some(vars[ti(top.var)+i*nv]) }
                } else {
                    panic!("error");
                    None
                };

                if let Some(l) = lit {
                    clause.push(l);
                } else { panic!("ho no"); }

            }

            // here we force only TOP to be true. which should propagate down...
            let ga = goal_active;
            // println!("goal clause: {:?}, activation: {:?}", clause, ga);
            clause.push(ga);
            s.add_clause(&clause);

            // we also need to make sure our invariants hold for all timesteps
            // until the goal holds.
            // ie. (i0 | g0) & (i1 | g0 | g1) & (i2 | g0 | g1 | g2) ...

            // this is the same semantics as the LTL UNTIL operator.


            for i in 0..(step) {
                let i = i as usize;
                let mut clause: Vec<cryptominisat::Lit> = Vec::new();

                let i_lit = if sat_model.norm_vars.contains(&invar.var) {
                    panic!("unexpected");
                    if invar.neg { Some(!vars[ci(invar.var)+i*nv]) } else { Some(vars[ci(invar.var)+i*nv]) }
                } else if is_ts(top) {
                    if invar.neg { panic!("not good"); }
                    if invar.neg { Some(!vars[ti(invar.var)+i*nv]) } else { Some(vars[ti(invar.var)+i*nv]) }
                } else {
                    panic!("error");
                    None
                };

                if let Some(l) = i_lit {
                    clause.push(l);
                } else { panic!("ho no"); }

                for j in 0..(i+1) {
                    let lit = if sat_model.norm_vars.contains(&top.var) {
                        panic!("unexpected");
                        if top.neg { Some(!vars[ci(top.var)+j*nv]) } else { Some(vars[ci(top.var)+j*nv]) }
                    } else if is_ts(top) {
                        if top.neg { panic!("not good"); }
                        if top.neg { Some(!vars[ti(top.var)+j*nv]) } else { Some(vars[ti(top.var)+j*nv]) }
                    } else {
                        panic!("error");
                        None
                    };

                    if let Some(l) = lit {
                        clause.push(l);
                    } else { panic!("ho no"); }
                }

                s.add_clause(&clause);
            }
        }

        let ga = !goal_active;
        let res = s.solve_with_assumptions(&[ga]);

        // println!("res is {:?}", res);


        let reached_goal = res == Lbool::True;

        println!("Step {} computed in: {}ms\n", step, now.elapsed().as_millis());
        if reached_goal {
            println!("Found plan after {} steps", step+1);
            steps = step as i32 + 1;
            // println!("{:#?}", s.get_model());
            break;
        } else {
            let new_vars: Vec<cryptominisat::Lit> = (0 .. all_num_vars).map(|_v| s.new_var()).collect();

            vars.extend(new_vars.iter());

        }
    }

    println!("Plan computed in: {}ms\n", now.elapsed().as_millis());

    return steps; // for now :)
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
    // each resource contains a supervisor defining its good states
    let inits: Vec<Predicate> = model.resources().iter().flat_map(|r| r.sub_items())
        .flat_map(|si| match si {
            SPItem::Spec(s) if s.name() == "supervisor" => Some(s.invariant().clone()),
            _ => None
        }).collect();

    // we need to assume that we are in a state that adheres to the resources
    let initial = Predicate::AND(inits);

    let mut ts_model = TransitionSystemModel::from(&model);

    let (new_guards, supervisor) = extract_guards(&ts_model, &initial);

    // The specs are converted into guards + a global supervisor
    ts_model.specs.clear();
    ts_model.specs.push(Spec::new("global_supervisor", supervisor));

    // TODO: right now its very cumbersome to update the original Model.
    // but it would be nice if we could.
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

    let global_ops_trans:Vec<_> = global_ops.iter().flat_map(|o|o.transitinos()).cloned().collect();
    let global_ops_ctrl: Vec<_> = global_ops_trans.iter().filter(|o|o.controlled).cloned().collect();
    let global_ops_un_ctrl: Vec<_> = global_ops_trans.iter().filter(|o|!o.controlled).cloned().collect();
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

#[cfg(test)]
mod tests {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
    fn test_guard_extraction() {
        use crate::testing::*;

        // Make model
        let mut m = Model::new_root("test_guard_extraction", Vec::new());

        // Make resoureces
        m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
        m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

        let inits: Vec<Predicate> = m.resources().iter().flat_map(|r| r.sub_items())
            .flat_map(|si| match si {
                SPItem::Spec(s) if s.name() == "supervisor" => Some(s.invariant().clone()),
                _ => None
            }).collect();

        // we need to assume that we are in a state that adheres to the resources
        let initial = Predicate::AND(inits);

        // Make some global stuff
        let r1_p_a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
        let r2_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();

        // (offline) Specifications
        let table_zone = p!(!( [p:r1_p_a == "at"] && [p:r2_p_a == "at"]));
        m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

        let mut ts_model = TransitionSystemModel::from(&m);
        let (new_guards, new_initial) = extract_guards(&ts_model, &initial);
        update_guards(&mut ts_model, &new_guards);

        ts_model.specs.clear();
        crate::planning::generate_offline_nuxvm(&ts_model, &new_initial);

        assert_eq!(new_guards.len(), 4);
        assert_ne!(new_initial, Predicate::TRUE);
    }

    #[test]
    #[serial]
    fn test_sat_planning_ge_len11() {
        use crate::testing::*;

        // Make model
        let mut m = Model::new_root("test_sat_planning", Vec::new());

        // Make resoureces
        m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
        m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

        // was 872
        let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
        let r1r = m.find_item("ref_pos", &["r1"]).expect("check spelling").path();
        let r1active = m.find_item("active", &["r1"]).expect("check spelling").path();
        let r1activate = m.find_item("activate", &["r1", "Control"]).expect("check spelling").path();

        let r2a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();
        let r2r = m.find_item("ref_pos", &["r2"]).expect("check spelling").path();
        let r2active = m.find_item("active", &["r2"]).expect("check spelling").path();
        let r2activate = m.find_item("activate", &["r2", "Control"]).expect("check spelling").path();


        // spec to make it take more steps
        let table_zone = p!(!( [p:r1a == "at"] && [p:r2a == "at"]));
        m.add_item(SPItem::Spec(Spec::new("table_zone", table_zone)));

        let mut ts_model = TransitionSystemModel::from(&m);
        let (new_guards, new_initial) = extract_guards(&ts_model, &Predicate::TRUE);
        update_guards(&mut ts_model, &new_guards);

        let init = p!([p:r1a == "away"] && [p:r1r == "away"] && [!p:r1active] && [!p:r1activate] && [p:r2a == "away"] && [p:r2r == "away"] && [!p:r2active] && [!p:r2activate]);

        let i = Predicate::AND(vec![new_initial, init.clone()]);
        crate::planning::generate_offline_nuxvm(&ts_model, &i);
        // crate::planning::generate_offline_nuxvm(&ts_model, &Predicate::TRUE);

        // start planning test

        let i1 = Predicate::TRUE;
        let g1 = p!(p:r1a == "at");
        //let i2 = p!(!p:r2active);
        let i2 = Predicate::TRUE;
        let g2 = p!(p:r2a == "at");
        //let g2 = p!(p:r2active);

        let goals = vec![(i1, g1), (i2,g2)];
        //let goal = p!(p:r1a == "at");

        let now = std::time::Instant::now();
        let len = plan(&ts_model, &init, &goals);
        println!("Planning test performed in: {}ms\n", now.elapsed().as_millis());

        assert_eq!(len, 11);
    }

    #[test]
    #[serial]
    fn test_sat_planning_invar_len11() {
        use crate::testing::*;

        // Make model
        let mut m = Model::new_root("tsi", Vec::new());

        // Make resoureces
        m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
        m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

        // was 872
        let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
        let r1r = m.find_item("ref_pos", &["r1"]).expect("check spelling").path();
        let r1active = m.find_item("active", &["r1"]).expect("check spelling").path();
        let r1activate = m.find_item("activate", &["r1", "Control"]).expect("check spelling").path();

        let r2a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();
        let r2r = m.find_item("ref_pos", &["r2"]).expect("check spelling").path();
        let r2active = m.find_item("active", &["r2"]).expect("check spelling").path();
        let r2activate = m.find_item("activate", &["r2", "Control"]).expect("check spelling").path();


        let table_zone = p!(!( [p:r1a == "at"] && [p:r2a == "at"]));
        let new_table_zone = refine_invariant(&m, &table_zone);
        println!("new table zone:\n{}", new_table_zone);

        // no guard extr.
        let ts_model = TransitionSystemModel::from(&m);

        let init = p!([p:r1a == "away"] && [p:r1r == "away"] && [!p:r1active] && [!p:r1activate] && [p:r2a == "away"] && [p:r2r == "away"] && [!p:r2active] && [!p:r2activate]);

        crate::planning::generate_offline_nuxvm(&ts_model, &init);

        // start planning test

        //let i1 = Predicate::TRUE;
        let i1 = table_zone.clone();
        //let i1 = new_table_zone.clone();
        let g1 = p!(p:r1a == "at");
        let g1 = Predicate::AND(vec![g1, table_zone.clone()]);


        //let i2 = Predicate::TRUE;
        let i2 = table_zone.clone();
        //let i2 = new_table_zone.clone();
        let g2 = p!(p:r2a == "at");
        let g2 = Predicate::AND(vec![g2, table_zone.clone()]);

        let goals = vec![(i1, g1), (i2,g2)];

        let now = std::time::Instant::now();
        let len = plan(&ts_model, &init, &goals);
        println!("Planning test performed in: {}ms\n", now.elapsed().as_millis());

        assert_eq!(len, 11);
    }

    #[test]
    #[serial]
    fn test_sat_planning_invar_len9() {
        use crate::testing::*;

        // Make model
        let mut m = Model::new_root("tsi", Vec::new());

        // Make resoureces
        m.add_item(SPItem::Resource(make_dummy_robot("r1", &["at", "away"])));
        m.add_item(SPItem::Resource(make_dummy_robot("r2", &["at", "away"])));

        // was 872
        let r1a = m.find_item("act_pos", &["r1"]).expect("check spelling").path();
        let r1r = m.find_item("ref_pos", &["r1"]).expect("check spelling").path();
        let r1active = m.find_item("active", &["r1"]).expect("check spelling").path();
        let r1activate = m.find_item("activate", &["r1", "Control"]).expect("check spelling").path();

        let r2a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();
        let r2r = m.find_item("ref_pos", &["r2"]).expect("check spelling").path();
        let r2active = m.find_item("active", &["r2"]).expect("check spelling").path();
        let r2activate = m.find_item("activate", &["r2", "Control"]).expect("check spelling").path();

        // no guard extr.
        let ts_model = TransitionSystemModel::from(&m);

        let init = p!([p:r1a == "away"] && [p:r1r == "away"] && [!p:r1active] && [!p:r1activate] && [p:r2a == "away"] && [p:r2r == "away"] && [!p:r2active] && [!p:r2activate]);

        let i1 = Predicate::TRUE;
        let g1 = p!(p:r1a == "at");

        let i2 = Predicate::TRUE;
        let g2 = p!(p:r2a == "at");

        let goals = vec![(i1, g1), (i2,g2)];

        let now = std::time::Instant::now();
        let len = plan(&ts_model, &init, &goals);
        println!("Planning test performed in: {}ms\n", now.elapsed().as_millis());

        assert_eq!(len, 9);
    }

    #[test]
    #[serial]
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
}
