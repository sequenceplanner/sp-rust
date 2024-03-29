use serde::{Deserialize, Serialize};
use guard_extraction::*;
use sp_domain::*;
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
        let pred_map: HashMap<_, _> = model
            .state_predicates
            .iter()
            .flat_map(|p| match p.variable_type() {
                VariableType::Predicate(x) => Some((p.path().clone(), x.clone())),
                _ => None,
            })
            .collect();

        let mut fc = FormalContext {
            context: c,
            var_map,
            pred_map,
        };

        for t in &model.transitions {
            let guard = fc.sp_pred_to_ex(t.guard());
            let actions: Vec<_> = t.actions().iter().map(|a| fc.sp_action_to_ac(a)).collect();

            if t.type_ == TransitionType::Controlled {
                fc.context
                    .add_c_trans(&t.path().to_string(), &guard, &actions);
            } else {
                fc.context
                    .add_uc_trans(&t.path().to_string(), &guard, &actions);
            }
        }

        fc
    }

    pub fn sp_action_to_ac(&self, a: &Action) -> Ac {
        let (index, var) = self
            .var_map
            .get(&a.var)
            .expect(&format!("variable not found! {}", a.var));
        let val = match &a.value {
            Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
                // assign p to var
                let (other, _var) = self.var_map.get(p).expect(&format!(
                    "variable not found: {}, looked in {:?}\n",
                    p,
                    self.var_map.keys()
                ));
                Value::Var(*other)
            }
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
            }
            Compute::Any => Value::Free,
            x => panic!("TODO: {:?}", x),
        };
        Ac { var: *index, val }
    }

    pub fn sp_pred_to_ex(&self, p: &Predicate) -> Ex {
        match p {
            Predicate::AND(v) => {
                let v = v.iter().map(|x| self.sp_pred_to_ex(x)).collect();
                Ex::AND(v)
            }
            Predicate::OR(v) => {
                let v = v.iter().map(|x| self.sp_pred_to_ex(x)).collect();
                Ex::OR(v)
            }
            Predicate::NOT(v) => {
                let v = self.sp_pred_to_ex(v);
                Ex::NOT(Box::new(v))
            }
            Predicate::FALSE => Ex::FALSE,
            Predicate::TRUE => Ex::TRUE,
            Predicate::EQ(PredicateValue::SPPath(var, _), PredicateValue::SPValue(value)) => {
                if self.pred_map.contains_key(var) {
                    if value == &SPValue::Bool(false) {
                        Ex::NOT(Box::new(
                            self.sp_pred_to_ex(self.pred_map.get(var).unwrap()),
                        ))
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
                            if let Some(p) = dom.iter().position(|e| e == v) {
                                Value::InDomain(p)
                            } else {
                                panic!("value {} is not in domain: {:?} (expr: {:?})", v, dom, p);
                            }
                        }
                    };

                    Ex::EQ(*index, value)
                } else {
                    panic!(
                        "VAR MAP\n{:#?}\n\nPRED MAP\n{:#?}\n\nVAR {:?}, VALUE {:?}",
                        self.var_map, self.pred_map, var, value
                    )
                }
            }
            Predicate::EQ(PredicateValue::SPPath(var, _), PredicateValue::SPPath(other, _)) => {
                if self.var_map.contains_key(var) && self.var_map.contains_key(other) {
                    let (index, _var) = self.var_map.get(var).unwrap();
                    let (index2, _other) = self.var_map.get(other).unwrap();
                    Ex::EQ(*index, Value::Var(*index2))
                } else {
                    panic!("VAR {:?}, OTHER {:?}", var, other)
                }
            }
            Predicate::NEQ(PredicateValue::SPPath(var, _), PredicateValue::SPPath(other, _)) => {
                if self.var_map.contains_key(var) && self.var_map.contains_key(other) {
                    let (index, _var) = self.var_map.get(var).unwrap();
                    let (index2, _other) = self.var_map.get(other).unwrap();
                    Ex::NOT(Box::new(Ex::EQ(*index, Value::Var(*index2))))
                } else {
                    panic!("VAR {:?}, OTHER {:?}", var, other)
                }
            }
            Predicate::NEQ(PredicateValue::SPPath(var, _), PredicateValue::SPValue(value)) => {
                if self.pred_map.contains_key(var) {
                    if value == &SPValue::Bool(false) {
                        self.sp_pred_to_ex(self.pred_map.get(var).unwrap())
                    } else if value == &SPValue::Bool(true) {
                        Ex::NOT(Box::new(
                            self.sp_pred_to_ex(self.pred_map.get(var).unwrap()),
                        ))
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
                    panic!(
                        "VAR {:?}, VALUE {:?}, pred {:?}, var {:?}",
                        var, value, &self.pred_map, &self.var_map
                    );
                }
            }
            x => panic!("NO X {:?}", x),
        }
    }

    pub fn ex_to_sp_pred(&self, e: &Ex) -> Predicate {
        match e {
            Ex::AND(v) => {
                let v = v.iter().map(|x| self.ex_to_sp_pred(x)).collect();
                Predicate::AND(v)
            }
            Ex::OR(v) => {
                let v = v.iter().map(|x| self.ex_to_sp_pred(x)).collect();
                Predicate::OR(v)
            }
            Ex::NOT(v) => {
                let v = self.ex_to_sp_pred(v);
                Predicate::NOT(Box::new(v))
            }
            Ex::FALSE => Predicate::FALSE,
            Ex::TRUE => Predicate::TRUE,
            Ex::EQ(index, value) => {
                let var = self
                    .var_map
                    .values()
                    .find(|(idx, _)| idx == index)
                    .unwrap()
                    .1
                    .clone();
                let val = match value {
                    Value::Bool(b) => PredicateValue::SPValue(SPValue::Bool(*b)),
                    Value::InDomain(vid) => {
                        PredicateValue::SPValue(var.domain().get(*vid).unwrap().clone())
                    }
                    Value::Var(other) => {
                        let other = self
                            .var_map
                            .values()
                            .find(|(idx, _)| idx == other)
                            .unwrap()
                            .1
                            .clone();
                        PredicateValue::SPPath(other.path().clone(), None)
                    }
                    Value::Free => panic!("not supported"),
                };
                Predicate::EQ(PredicateValue::SPPath(var.path().clone(), None), val)
            }
            Ex::VAR(index) => {
                let var = self
                    .var_map
                    .values()
                    .find(|(idx, _)| idx == index)
                    .unwrap()
                    .1
                    .clone();
                let val = PredicateValue::SPValue(SPValue::Bool(true));
                Predicate::EQ(PredicateValue::SPPath(var.path().clone(), None), val)
            }
        }
    }
}

use std::collections::HashSet;
fn modified_by(t: &Transition) -> HashSet<SPPath> {
    let mut r = HashSet::new();

    r.extend(t.actions().iter().map(|a| a.var.clone()));

    r
}

fn support_flatten_predicates(p: &Predicate, preds: &[Variable]) -> Vec<SPPath> {
    let mut flattened: Vec<SPPath> = p
        .support()
        .iter()
        .map(|p| match preds.iter().find(|t| t.path() == p) {
            Some(v) => {
                if let VariableType::Predicate(pred) = v.variable_type() {
                    support_flatten_predicates(&pred, preds)
                } else {
                    panic!("cannot happen")
                }
            }
            _ => vec![p.clone()],
        })
        .flatten()
        .collect();
    flattened.dedup();
    return flattened;
}

pub fn refine_invariant(model: &TransitionSystemModel, invariant: &Predicate) -> Predicate {
    let mut model = model.clone();

    // only look at the uncontrollable transitions.
    model
        .transitions
        .retain(|t| t.type_ != TransitionType::Controlled);

    // check if there are no unc transitions that can modify the state defined
    // by the invariants.
    let mut support: HashSet<SPPath> = HashSet::new();
    support.extend(support_flatten_predicates(
        invariant,
        &model.state_predicates,
    ));
    if model.transitions.iter().all(|t| {
        let modifies = modified_by(t);
        let intersection: HashSet<_> = support.intersection(&modifies).collect();
        intersection.is_empty()
    }) {
        // nothing to process! invariant is safe to use as is.
        return invariant.clone();
    }

    // hack to remove un-needed variabables
    // think about wheter this is sound, and also if more transitions can be removed
    let mut support_all: HashSet<SPPath> = support.clone();
    model.transitions.iter().for_each(|t| {
        support_all.extend(t.guard().support().into_iter());
        let modifies = modified_by(t);
        support_all.extend(modifies);
    });
    let preds: Vec<_> = support_all
        .iter()
        .filter_map(|s| {
            if let Some(v) = model.state_predicates.iter().find(|t| t.path() == s) {
                if let VariableType::Predicate(p) = v.variable_type() {
                    return Some(p);
                }
            }
            return None;
        })
        .collect();

    let mut support_all: HashSet<_> = support_all
        .into_iter()
        .filter(|s| !model.state_predicates.iter().any(|t| t.path() == s))
        .collect();

    support_all.extend(preds.iter().flat_map(|p| p.support()));

    // finally filter out any variables left
    // model.vars.retain(|v| support_all.contains(v.path()));

    // TODO: also filter out all variables and state predicates which
    // are not included in any of our transitions/invariants
    let c = FormalContext::from(&model);

    // forbidden = all states NOT conforming to the invariant
    let forbidden = Ex::NOT(Box::new(c.sp_pred_to_ex(&invariant)));

    let forbidden = c.context.extend_forbidden(&forbidden);

    let new_invariant = Ex::NOT(Box::new(forbidden));

    c.ex_to_sp_pred(&new_invariant)
}

pub fn clean_pred(model: &TransitionSystemModel, p: &Predicate) -> Predicate {
    let mut model = model.clone();
    model.transitions.clear(); // dont need these
    model.invariants.clear(); // ...
    let support = support_flatten_predicates(p, &model.state_predicates);
    model.vars.retain(|v| support.contains(v.path()));

    let c = FormalContext::from(&model);
    let ex = c.sp_pred_to_ex(p);
    let ex = c.context.cycle_expression(&ex);
    c.ex_to_sp_pred(&ex)
}

use std::io::{self, Read, Write};

#[derive(Serialize, Deserialize)]
#[serde(tag = "type")]
enum Request {
    Refine { ts_model: TransitionSystemModel, pred: Predicate },
    Clean { ts_model: TransitionSystemModel, pred: Predicate },
}

fn main() -> io::Result<()> {
    let mut buffer = String::new();
    io::stdin().read_to_string(&mut buffer)?;
    let req: Request = serde_json::from_str(&buffer)?;
    match req {
        Request::Refine { ref ts_model, ref pred } => {
            let new_pred = refine_invariant(ts_model, pred);
            let response = serde_json::to_string(&new_pred)?;
            io::stdout().write_all(response.as_bytes())?;
            Ok(())
        },
        Request::Clean { ref ts_model, ref pred } => {
            let new_pred = clean_pred(ts_model, pred);
            let response = serde_json::to_string(&new_pred)?;
            io::stdout().write_all(response.as_bytes())?;
            Ok(())
        }
    }
}
