use sp_domain::*;
use guard_extraction::*;
use std::collections::HashMap;

pub struct FormalContext {
    pub context: Context,
    var_map: HashMap<SPPath, (usize, Variable)>,
    pred_map: HashMap<SPPath, Predicate>,
}

impl FormalContext {
    pub fn from(model: &TransitionSystemModel) -> Self {
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

        let mut fc = FormalContext {
            context: c,
            var_map: var_map,
            pred_map: pred_map,
        };

        for t in &model.transitions {
            let guard = fc.sp_pred_to_ex(t.guard());
            let actions: Vec<_> = t.actions().iter().map(|a| fc.sp_action_to_ac(a) ).collect();
            let effects: Vec<_> = t.effects().iter().map(|a| fc.sp_action_to_ac(a) ).collect();
            let mut a = Vec::new();
            a.extend(actions.iter().cloned());
            a.extend(effects.iter().cloned());

            if t.controlled() {
                fc.context.add_c_trans(&t.path().to_string(), &guard, &a);
            } else {
                fc.context.add_uc_trans(&t.path().to_string(), &guard, &a);
            }
        }

        fc
    }


    pub fn sp_action_to_ac(&self, a: &Action) -> Ac {
        let (index, var) = self.var_map.get(&a.var).expect(&format!("variable not found! {}", a.var));
        let val = match &a.value {
            Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
                // assign p to var
                let (other, _var) = self.var_map.get(p).expect("variable not found");
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

    pub fn sp_pred_to_ex(&self, p: &Predicate) -> Ex {
        match p {
            Predicate::AND(v) => {
                let v = v.iter().map(|x| self.sp_pred_to_ex(x)).collect();
                Ex::AND(v)
            },
            Predicate::OR(v) => {
                let v = v.iter().map(|x| self.sp_pred_to_ex(x)).collect();
                Ex::OR(v)
            },
            Predicate::NOT(v) => {
                let v = self.sp_pred_to_ex(v);
                Ex::NOT(Box::new(v))
            },
            Predicate::FALSE => Ex::FALSE,
            Predicate::TRUE => Ex::TRUE,
            Predicate::EQ(PredicateValue::SPPath(var, _),
                          PredicateValue::SPValue(value)) => {
                if self.pred_map.contains_key(var) {
                    if value == &SPValue::Bool(false) {
                        Ex::NOT(Box::new(self.sp_pred_to_ex(self.pred_map.get(var).unwrap())))
                    } else if value == &SPValue::Bool(true) {
                        self.sp_pred_to_ex(self.pred_map.get(var).unwrap())
                    } else {
                        panic!("predicates must be boolean");
                    }
                } else if self.var_map.contains_key(var) {
                    let (index, var) = self.var_map.get(var).unwrap();

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
                if self.var_map.contains_key(var) && self.var_map.contains_key(other) {
                    let (index, _var) = self.var_map.get(var).unwrap();
                    let (index2, _other) = self.var_map.get(other).unwrap();
                    Ex::EQ(*index, Value::Var(*index2))
                } else {
                    panic!("VAR {:?}, OTHER {:?}", var, other)
                }
            },
            Predicate::NEQ(PredicateValue::SPPath(var, _),
                           PredicateValue::SPPath(other, _)) => {
                if self.var_map.contains_key(var) && self.var_map.contains_key(other) {
                    let (index, _var) = self.var_map.get(var).unwrap();
                    let (index2, _other) = self.var_map.get(other).unwrap();
                    Ex::NOT(Box::new(Ex::EQ(*index, Value::Var(*index2))))
                } else {
                    panic!("VAR {:?}, OTHER {:?}", var, other)
                }
            },
            Predicate::NEQ(PredicateValue::SPPath(var, _),
                           PredicateValue::SPValue(value)) => {
                if self.pred_map.contains_key(var) {
                    if value == &SPValue::Bool(false) {
                        self.sp_pred_to_ex(self.pred_map.get(var).unwrap())
                    } else if value == &SPValue::Bool(true) {
                        Ex::NOT(Box::new(self.sp_pred_to_ex(self.pred_map.get(var).unwrap())))
                    } else {
                        panic!("predicates must be boolean");
                    }
                } else if self.var_map.contains_key(var) {
                    let (index, var) = self.var_map.get(var).unwrap();

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

    pub fn ex_to_sp_pred(&self, e: &Ex) -> Predicate {
        match e {
            Ex::AND(v) => {
                let v = v.iter().map(|x| self.ex_to_sp_pred(x)).collect();
                Predicate::AND(v)
            },
            Ex::OR(v) => {
                let v = v.iter().map(|x| self.ex_to_sp_pred(x)).collect();
                Predicate::OR(v)
            },
            Ex::NOT(v) => {
                let v = self.ex_to_sp_pred(v);
                Predicate::NOT(Box::new(v))
            },
            Ex::FALSE => Predicate::FALSE,
            Ex::TRUE => Predicate::TRUE,
            Ex::EQ(index, value) => {
                let var = self.var_map.values().find(|(idx, _)| idx == index).unwrap().1.clone();
                let val = match value {
                    Value::Bool(b) => PredicateValue::SPValue(SPValue::Bool(*b)),
                    Value::InDomain(vid) => PredicateValue::SPValue(var.domain().get(*vid).unwrap().clone()),
                    Value::Var(other) => {
                        let other = self.var_map.values().find(|(idx, _)| idx == other).unwrap().1.clone();
                        PredicateValue::SPPath(other.path().clone(), None)
                    },
                    Value::Free => panic!("not supported")
                };
                Predicate::EQ(PredicateValue::SPPath(var.path().clone(), None), val)
            },
            Ex::VAR(index) => {
                let var = self.var_map.values().find(|(idx, _)| idx == index).unwrap().1.clone();
                let val = PredicateValue::SPValue(SPValue::Bool(true));
                Predicate::EQ(PredicateValue::SPPath(var.path().clone(), None), val)
            },
        }
    }

    pub fn sat_clause_to_spstate(&self, values: &[guard_extraction::Value]) -> SPState {
        let mut state = SPState::new();
        values.iter().enumerate().for_each(|(i,v)| {
            let var = self.var_map.values().find(|(idx, _)| idx == &i).unwrap().1.clone();
            let val = match v {
                Value::Bool(b) => SPValue::Bool(*b),
                Value::InDomain(vid) => var.domain()[*vid].clone(),
                _ => panic!("cannot happen")
            };

            state.add_variable(var.path().clone(), val);
        });
        state
    }
}


pub fn extract_guards(model: &TransitionSystemModel, initial: &Predicate) -> (HashMap<String, Predicate>, Predicate) {
    let c = FormalContext::from(model);

    // pull out all specs.
    let forbidden =  Ex::OR(model.specs.iter().map(|s| {
        // forbidden = not always
        Ex::NOT(Box::new(c.sp_pred_to_ex(s.invariant())))
    }).collect());

    let initial = c.sp_pred_to_ex(&initial);

    let (ng, supervisor) = c.context.compute_guards(&initial, &forbidden);

    let ng = ng.into_iter().map(|(n,e)| (n, c.ex_to_sp_pred(&e))).collect();
    let supervisor = c.ex_to_sp_pred(&supervisor);
    (ng, supervisor)
}

pub fn refine_invariant(model: &Model, invariant: &Predicate) -> Predicate {
    let model = TransitionSystemModel::from(&model);
    let c = FormalContext::from(&model);

    // forbidden = all states NOT conforming to the invariant
    let forbidden = Ex::NOT(Box::new(c.sp_pred_to_ex(&invariant)));

    let forbidden = c.context.extend_forbidden(&forbidden);

    let new_invariant = Ex::NOT(Box::new(forbidden));

    c.ex_to_sp_pred(&new_invariant)
}

pub fn update_guards(ts_model: &mut TransitionSystemModel, ng: &HashMap<String, Predicate>) {
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

#[cfg(test)]
mod tests;
