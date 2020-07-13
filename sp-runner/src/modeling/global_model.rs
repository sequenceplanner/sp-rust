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
    products: Vec<SPPath>,
}

impl GModel {
    pub fn new(name: &str) -> Self {
        GModel {
            model: Model::new_root(name, Vec::new()),
            initial_state: SPState::new(),
            products: Vec::new(),
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

    fn add_product_var(&mut self, v: Variable) -> SPPath {
        let m = match self.model.find_item_mut("product_state", &[]) {
            Some(SPMutItemRef::Model(m)) => m,
            Some(_) => panic!("error..."),
            None => {
                let temp_model = Model::new("product_state", vec![]);
                let _temp_model_path = self.model.add_item(SPItem::Model(temp_model));
                match self.model.find_item_mut("product_state", &[]) {
                    Some(SPMutItemRef::Model(m)) => m,
                    Some(_) => panic!("error..."),
                    _ => panic!("cannot happen"),
                }
            }
        };
        m.add_item(SPItem::Variable(v))
    }

    pub fn add_estimated_domain(&mut self, name: &str, domain: &[SPValue], product: bool) -> SPPath {
        let v = Variable::new(
            name,
            VariableType::Estimated,
            domain[0].has_type(),
            domain.to_vec(),
        );
        if product {
            self.add_product_var(v)
        } else {
            self.model.add_item(SPItem::Variable(v))
        }
    }

    pub fn add_estimated_bool(&mut self, name: &str, product: bool) -> SPPath {
        let v = Variable::new_boolean(name, VariableType::Estimated);
        if product {
            self.add_product_var(v)
        } else {
            self.model.add_item(SPItem::Variable(v))
        }
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

    pub fn add_op(&mut self, name: &str, guard: &Predicate, effects: &[Action],
                  goal: &Predicate, post_actions: &[Action], resets: bool) -> SPPath {
        let op = Operation::new(name, guard, effects, goal, post_actions, resets);

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
    }

    pub fn add_op_alt(&mut self, name: &str, guard: &Predicate, effects: &[(&str, &[Action])],
                      goal: &Predicate, post_actions: &[Action], resets: bool) -> Vec<SPPath> {
        let mut paths = Vec::new();
        for e in effects {
            let name = format!("{}_{}", name, e.0);
            paths.push(self.add_op(&name, guard, e.1, goal, post_actions, resets));
        }
        paths
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

        let i = Intention::new(name, &[op_start, op_finish], Some(op_goal));

        self.model.add_item(SPItem::Intention(i))
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
        // let global_ops_vars: Vec<Variable> = self.model.all_operations().iter().map(|o| o.state_variable().clone()).collect();

        // let state: Vec<_> = global_ops_vars.iter()
        //     .map(|v| (v.node().path().clone(), "i".to_spvalue()))
        //     .collect();
        // s.extend(SPState::new_from_values(&state[..]));

        let op_state = self.model.all_operations().iter()
            .map(|o| (o.state_variable().node().path().clone(), "i".to_spvalue()))
            .collect::<Vec<_>>();

        let intention_state = self.model.all_intentions().iter()
            .map(|i| (i.state_variable().node().path().clone(), "i".to_spvalue()))
            .collect::<Vec<_>>();

        s.extend(SPState::new_from_values(op_state.as_slice()));
        s.extend(SPState::new_from_values(intention_state.as_slice()));

        // lastly, add all specifications.
        (self.model, s)
    }
}
