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
        let mut mc = m.clone();
        mc.items.clear();
        let _rp = m.add_item(SPItem::Resource(resource.clone()));
        let r = self.model.find_item(resource.name(), &[name]).unwrap().unwrap_resource();
        let vars = r.get_variables();
        let mut model = self.model.clone();
        model.items.clear();
        let _rp = mc.add_item(SPItem::Resource(resource.clone()));
        model.add_item(SPItem::Model(mc));
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

    pub fn add_estimated_domain(&mut self, name: &str, domain: &[SPValue], product: bool) -> SPPath {
        let v = Variable::new(
            name,
            VariableType::Estimated,
            domain[0].has_type(),
            domain.to_vec(),
        );
        if product {
            self.add_sub_item("product_state", SPItem::Variable(v))
        } else {
            self.model.add_item(SPItem::Variable(v))
        }
    }

    pub fn add_estimated_bool(&mut self, name: &str, product: bool) -> SPPath {
        let v = Variable::new_boolean(name, VariableType::Estimated);
        if product {
            self.add_sub_item("product_state", SPItem::Variable(v))
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

    pub fn add_runner_transition(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![], // no effects
            false,  // auto!
        );
        self.add_sub_item("runner_transitions", SPItem::Transition(trans));
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

    // handles the special case where auto transitions are directly mapped
    // onto the higher level.
    pub fn add_op_auto(&mut self, name: &str, pre: &Predicate, effects: &[Action]) -> SPPath {
        // in the planning model, we only have a single transition,
        // effects are immediate
        let t = Transition::new(
            &format!("{}_auto", name),
            Predicate::AND(vec![pre.clone()]),
            effects.to_vec(),
            vec![],
            false,
        );

        self.add_sub_item("operations", SPItem::Transition(t))
    }

    pub fn add_op(&mut self, name: &str, guard: &Predicate, effects: &[Action],
                  goal: &Predicate, post_actions: &[Action], resets: bool, auto: bool,
                  mc_constraint: Option<Predicate>) -> SPPath {
        let pre = Predicate::AND(vec![guard.clone(), goal.clone()]);
        let mut act = effects.to_vec();
        act.extend(post_actions.iter().cloned());
        // add a new low level transition. goal // effects
        if auto {

            // it is important to realize that we cannot freely change
            // the goals of the high level when we are in this state
            // or any other state from which this state can
            // uncontrollably be reached. so we also create a spec here
            let spec = Spec::new(name, Predicate::NOT(Box::new(pre.clone())));
            self.add_sub_item("replan_specs", SPItem::Spec(spec));

            self.add_auto(name, &pre, &act);
        } else {

            // in this case it's fine (compare spec above) as the
            // variables are not changed uncontrollably.

            self.add_delib(name, &pre, &act);
        }

        if goal == &Predicate::TRUE {
            self.add_op_auto(name, guard, effects)
        } else {
            let op = Operation::new(name, guard, effects, &goal, post_actions, resets, mc_constraint);
            self.add_sub_item("operations", SPItem::Operation(op))
        }
    }

    pub fn add_runner_op(&mut self, name: &str, guard: &Predicate, effects: &[Action],
                  goal: &Predicate, post_actions: &[Action], resets: bool, auto: bool,
                  mc_constraint: Option<Predicate>) -> SPPath {
        let pre = Predicate::AND(vec![guard.clone(), goal.clone()]);
        let mut act = effects.to_vec();
        act.extend(post_actions.iter().cloned());
        // add a new low level transition. goal // effects
        if auto {

            // it is important to realize that we cannot freely change
            // the goals of the high level when we are in this state
            // or any other state from which this state can
            // uncontrollably be reached. so we also create a spec here
            // let spec = Spec::new(name, Predicate::NOT(Box::new(pre.clone())));
            // self.add_sub_item("replan_specs", SPItem::Spec(spec));

            self.add_runner_transition(name, &pre, &act);
        } else {

            // in this case it's fine (compare spec above) as the
            // variables are not changed uncontrollably.

            self.add_runner_transition(name, &pre, &act);
        }

        let op = Operation::new(name, guard, effects, &goal, post_actions, resets, mc_constraint);
        self.add_sub_item("runner_ops", SPItem::Operation(op))
    }

    pub fn add_intention(
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
        let op_goal = IfThen::new("goal", p!(p: state == "e"), post.clone(), invariant, None);

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
        // operations start in init
        let op_state = self.model.all_operations().iter()
            .map(|o| (o.state_variable().node().path().clone(), "i".to_spvalue()))
            .collect::<Vec<_>>();

        let runner_op_state = self.model.all_runner_operations().iter()
            .map(|o| (o.state_variable().node().path().clone(), "i".to_spvalue()))
            .collect::<Vec<_>>();

        // intentions are initially "paused"
        let intention_state = self.model.all_intentions().iter()
            .map(|i| (i.state_variable().node().path().clone(), "paused".to_spvalue()))
            .collect::<Vec<_>>();

        let mut s = SPState::new_from_values(op_state.as_slice());
        s.extend(SPState::new_from_values(runner_op_state.as_slice()));
        s.extend(SPState::new_from_values(intention_state.as_slice()));

        // intention state can be set manually in the initial state
        s.extend(self.initial_state);

        (self.model, s)
    }

    fn add_sub_item(&mut self, subpath: &str, item: SPItem) -> SPPath {
        let m = match self.model.find_item_mut(subpath, &[]) {
            Some(SPMutItemRef::Model(m)) => m,
            Some(_) => panic!("error..."),
            None => {
                let temp_model = Model::new(subpath, vec![]);
                let _temp_model_path = self.model.add_item(SPItem::Model(temp_model));
                match self.model.find_item_mut(subpath, &[]) {
                    Some(SPMutItemRef::Model(m)) => m,
                    Some(_) => panic!("error..."),
                    _ => panic!("cannot happen"),
                }
            }
        };
        m.add_item(item)
    }
}
