use sp_domain::*;
use std::ops::Index;

// helpers for creating the "global" model

#[derive(Debug, PartialEq, Clone)]
pub struct GResource {
    name: String,
    variables: Vec<Variable>,
    model: Model,
}

impl Index<&str> for GResource {
    type Output = SPPath;
    fn index(&self, name: &str) -> &SPPath {
        let result = self.variables.iter().find(|v| v.name() == name);
        result
            .expect(&format!("{}/{} not found", self.name, name))
            .path()
    }
}

impl GResource {
    pub fn find_item(&self, name: &str, path_sections: &[&str]) -> SPPath {
        let result = self.model.find_item(name, path_sections);
        result
            .expect(&format!("{} [{:?}] not found", name, path_sections))
            .path()
    }
}

#[derive(Debug, PartialEq, Clone)]
struct GOperation {
    name: String,
    resets: bool,
    pre: Predicate,
    post: Predicate,
    post_actions: Vec<Action>,
    invariant: Option<Predicate>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct GModel {
    model: Model,
    initial_state: SPState,
}

impl GModel {
    pub fn new(name: &str) -> Self {
        GModel {
            model: Model::new_root(name, Vec::new()),
            initial_state: SPState::new(),
        }
    }

    pub fn use_named_resource(&mut self, name: &str, resource: Resource) -> GResource {
        let m = match self.model.find_item_mut(name, &[]) {
            Some(SPMutItemRef::Model(m)) => m,
            Some(_) => panic!("..."),
            None => {
                let temp_model = Model::new(name, vec![]);
                let _temp_model_path = self.model.add_item(SPItem::Model(temp_model));
                match self.model.find_item_mut(name, &[]) {
                    Some(SPMutItemRef::Model(m)) => m,
                    Some(_) => panic!("..."),
                    _ => panic!("cannot happen"),
                }
            }
        };
        let _rp = m.add_item(SPItem::Resource(resource.clone()));
        let r = self.model.find_item(resource.name(), &[name]).unwrap().unwrap_resource();
        let vars = r.get_variables();
        let mut model = self.model.clone();
        model.items.clear();
        model.add_item(SPItem::Resource(resource));
        GResource {
            name: r.name().to_owned(),
            variables: vars,
            model,
        }
    }

    pub fn use_resource(&mut self, resource: Resource) -> GResource {
        let rp = self.model.add_item(SPItem::Resource(resource.clone()));
        let r = self.model.get(&rp).unwrap().unwrap_resource();
        let vars = r.get_variables();
        let mut model = self.model.clone();
        model.items.clear();
        model.add_item(SPItem::Resource(resource));
        GResource {
            name: r.name().to_owned(),
            variables: vars,
            model,
        }
    }

    pub fn add_estimated_domain(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
        let v = Variable::new(
            name,
            VariableType::Estimated,
            domain[0].has_type(),
            domain.to_vec(),
        );
        self.model.add_item(SPItem::Variable(v))
    }

    pub fn add_predicate_variable(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
        let v = Variable::new(
            name,
            VariableType::Estimated,
            domain[0].has_type(),
            domain.to_vec(),
        );
        self.model.add_item(SPItem::Variable(v))
    }

    pub fn add_auto(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![], // no effects
            false,  // auto!
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_delib(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(name, guard.clone(), actions.to_vec(), vec![], true);
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_effect(&mut self, name: &str, guard: &Predicate, effects: &[Action]) {
        let trans = Transition::new(name, guard.clone(), vec![], effects.to_vec(), false);
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_simulation_auto(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![], // no effects
            false,  // auto!
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn find(&mut self, name: &str, path_sections: &[&str]) -> SPPath {
        self.model
            .find_item(name, path_sections)
            .expect(&format!("cannot find {:?} / {}", path_sections, name))
            .path()
    }

    pub fn add_op(
        &mut self, name: &str, resets: bool, pre: &Predicate, post: &Predicate, effects: &[Action],
        invariant: Option<Predicate>,
    ) -> SPPath {
        let state = SPPath::from_slice(&[self.model.name(), "operations", name, "state"]);

        let op_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p: state == "i")]),
            vec![a!(p: state = "e")],
            vec![],
            true,
        );

        // effect for planning/checking
        let op_effect = Transition::new(
            "executing",
            Predicate::AND(vec![p!(p:state == "e")]),
            vec![],
            effects.to_vec(),
            false,
        );

        let f_actions = if resets {
            vec![a!(p: state = "i")]
        } else {
            vec![a!(p: state = "f")]
        };

        let op_finish = Transition::new(
            "finish",
            Predicate::AND(vec![p!(p: state == "e"), post.clone()]),
            f_actions,
            vec![],
            false,
        );
        let op_goal = IfThen::new("goal", p!(p: state == "e"), post.clone(), invariant);

        // in the planning model, we only have a single transition,
        // effects are immediate
        let op_planning = Transition::new(
            "planning",
            Predicate::AND(vec![pre.clone()]),
            vec![],
            effects.to_vec(),
            true,
        );

        let op = Operation::new(
            name,
            &[op_start, op_effect, op_finish, op_planning],
            Some(op_goal),
        );

        let m = match self.model.find_item_mut("operations", &[]) {
            Some(SPMutItemRef::Model(m)) => m,
            Some(_) => panic!("operations error..."),
            None => {
                let temp_model = Model::new("operations", vec![]);
                let _temp_model_path = self.model.add_item(SPItem::Model(temp_model));
                match self.model.find_item_mut("operations", &[]) {
                    Some(SPMutItemRef::Model(m)) => m,
                    Some(_) => panic!("operations error..."),
                    _ => panic!("cannot happen"),
                }
            }
        };
        m.add_item(SPItem::Operation(op))
        //self.model.add_item(SPItem::Operation(op))
    }

    pub fn add_hl_op(
        &mut self, name: &str, resets: bool, pre: &Predicate, post: &Predicate,
        post_actions: &[Action], invariant: Option<Predicate>,
    ) -> SPPath {
        let state = SPPath::from_slice(&[self.model.name(), name, "state"]);

        let op_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p: state == "i"), pre.clone()]),
            vec![a!(p: state = "e")],
            vec![],
            true,
        );
        let mut f_actions = if resets {
            vec![a!(p: state = "i")]
        } else {
            vec![a!(p: state = "f")]
        };
        f_actions.extend(post_actions.iter().cloned());
        let op_finish = Transition::new(
            "finish",
            Predicate::AND(vec![p!(p: state == "e"), post.clone()]),
            f_actions,
            vec![],
            false,
        );
        let op_goal = IfThen::new("goal", p!(p: state == "e"), post.clone(), invariant);

        let op = Operation::new_hl(name, &[op_start, op_finish], Some(op_goal));

        self.model.add_item(SPItem::Operation(op))
    }

    pub fn add_invar(&mut self, name: &str, invariant: &Predicate) {
        self.model
            .add_item(SPItem::Spec(Spec::new(name, invariant.clone())));
    }

    pub fn initial_state(&mut self, state: &[(&SPPath, SPValue)]) {
        let s: Vec<(SPPath, SPValue)> = state
            .iter()
            .map(|(p, v)| ((*p).clone(), v.clone()))
            .collect();
        self.initial_state.add_variables(s);
    }

    pub fn make_model(self) -> (Model, SPState) {
        let mut s = self.initial_state.clone();

        // add operation to the initial state here
        let global_ops_vars: Vec<Variable> = self.model.all_operations().iter().map(|o| o.state_variable().clone()).collect();

        let state: Vec<_> = global_ops_vars.iter()
            .map(|v| (v.node().path().clone(), "i".to_spvalue()))
            .collect();
        s.extend(SPState::new_from_values(&state[..]));

        // lastly, add all specifications.
        (self.model, s)
    }


    pub fn generate_operation_model(&mut self, products: &[SPPath]) {
        let tsm = TransitionSystemModel::from(&self.model);

        tsm.transitions.iter().for_each(|t| {
            let sup = t.guard().support();
            if products.iter().any(|p| sup.contains(p)) {
                let cleaned_guard = t.guard().keep_only(&products).unwrap();
                let acts: Vec<_> = t.actions().into_iter().filter(|p| products.contains(&p.var)).collect();

                // highlevel ops cannot deal with effects
                // let effs: Vec<_> = t.effects().into_iter().filter(|p| products.contains(&p.var)).collect();
                // flatten out "all in domain" assignments.
                let ad: Vec<Vec<(Predicate,Action)>> = acts.iter().flat_map(|a| {
                    if let Compute::PredicateValue(PredicateValue::SPPath(p,_)) = &a.value {
                        let v = tsm.vars.iter().find(|v|v.path() == p).unwrap();
                        let domain = if v.value_type() == SPValueType::Bool {
                            vec![false.to_spvalue(), true.to_spvalue()]
                        } else {
                            v.domain().to_vec()
                        };
                        Some(domain.iter().map(|e| {
                            (Predicate::EQ(PredicateValue::SPPath(p.clone(), None), PredicateValue::SPValue(e.clone())),
                            Action::new(a.var.clone(),
                                        Compute::PredicateValue(PredicateValue::SPValue(e.clone()))))
                        }).collect::<Vec<(Predicate,Action)>>())
                    } else {
                        None
                    }
                }).collect();

                // keep also the "normal" actions
                let norm: Vec<Action> = acts.iter().flat_map(|a| {
                    if let Compute::PredicateValue(PredicateValue::SPPath(_,_)) = &a.value {
                        None
                    } else {
                        Some((*a).clone())
                    }
                }).collect();

                if ad.is_empty() {
                    let post = Predicate::AND(norm.iter().map(|a|a.to_predicate().unwrap()).collect());
                    println!("Generating operation {}", t.name());
                    self.add_op(t.name(), true, &cleaned_guard, &post, &norm, None);
                } else if ad.len() == 1 {
                    let head = ad[0].clone();
                    head.iter().for_each(|(g,a)| {
                        let mut all = norm.clone();
                        all.push(a.clone());

                        let v = if let Compute::PredicateValue(PredicateValue::SPValue(v)) = &a.value {
                            v.to_string()
                        } else { "error".to_string() };

                        let name = format!("{}_{}", t.name(), v);
                        println!("Generating operation {}", name);
                        let pre = Predicate::AND(vec![cleaned_guard.clone(), g.clone()]);
                        let post = Predicate::AND(all.iter().map(|a|a.to_predicate().unwrap()).collect());
                        self.add_op(&name, true, &pre, &post, &all, None);
                    });
                } else {
                    panic!("TODO! model too complicated for now");
                }

            }
        });

    }
}
