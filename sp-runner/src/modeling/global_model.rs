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
    // When we synchronize with transitions, the original is removed
    synchronized_paths: Vec<SPPath>,
}

impl GModel {
    pub fn new(name: &str) -> Self {
        GModel {
            model: Model::new_root(name, Vec::new()),
            initial_state: SPState::new(),
            synchronized_paths: Vec::new(),
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
        let r = self
            .model
            .find_item(&(resource.name()), &[name])
            .unwrap()
            .unwrap_resource();
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

    pub fn add_estimated_domain(
        &mut self, name: &str, domain: &[SPValue], product: bool,
    ) -> SPPath {
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
            TransitionType::Auto, // auto!
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_delib(&mut self, name: &str, guard: &Predicate, actions: &[Action]) -> SPPath {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            TransitionType::Controlled,
        );
        self.model.add_item(SPItem::Transition(trans))
    }

    pub fn add_effect(&mut self, name: &str, guard: &Predicate, effects: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            effects.to_vec(),
            TransitionType::Effect,
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_runner_transition(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            TransitionType::Runner,
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
        &mut self, name: &str, guard: &Predicate, effects: &[Action], goal: &Predicate,
        post_actions: &[Action], auto: bool, mc_constraint: Option<Predicate>,
    ) -> SPPath {
        let effects_goals = vec![(effects, goal, post_actions)];
        let op = Operation::new(name, auto, guard, &effects_goals, mc_constraint);
        self.add_sub_item("operations", SPItem::Operation(op))
    }

    /// Add an operation that models a non-deteministic plant, ie multiple possible outcomes.
    pub fn add_op_alt(
        &mut self, name: &str, guard: &Predicate,
        effects_goals_actions: &[(&[Action], &Predicate, &[Action])], auto: bool,
        mc_constraint: Option<Predicate>,
    ) -> SPPath {
        let op = Operation::new(name, auto, guard, effects_goals_actions, mc_constraint);
        self.add_sub_item("operations", SPItem::Operation(op))
    }

    pub fn add_intention(
        &mut self, name: &str, resets: bool, pre: &Predicate, post: &Predicate,
        post_actions: &[Action], invariant: Option<Predicate>,
    ) -> SPPath {
        let i = Intention::new(name, resets, pre, post, post_actions, invariant);
        self.add_sub_item("intentions", SPItem::Intention(i))
    }

    pub fn add_invar(&mut self, name: &str, invariant: &Predicate) {
        self.model
            .add_item(SPItem::Spec(Spec::new(name, invariant.clone())));
    }

    pub fn add_product_invar(&mut self, name: &str, invariant: &Predicate) {
        self.model.add_item(SPItem::ProductSpec(ProductSpec::new(
            name,
            invariant.clone(),
        )));
    }

    pub fn initial_state(&mut self, state: &[(&SPPath, SPValue)]) {
        let s: Vec<(SPPath, SPValue)> = state
            .iter()
            .map(|(p, v)| ((*p).clone(), v.clone()))
            .collect();
        self.initial_state.add_variables(s);
    }

    /// Add new guard/actions to existing transition. The original
    /// transition will be removed from the model and a new one with
    /// the new name will be added. (we don't send in a new transition
    /// as we want to keep the transition type).
    pub fn synchronize(
        &mut self, sync_with: &SPPath, new_name: &str, guard: Predicate, actions: &[Action],
    ) -> SPPath {
        let sync_t = self
            .model
            .get(sync_with)
            .expect(&format!("cannot find transition {}", sync_with));
        if let SPItemRef::Transition(t) = sync_t {
            let mut new_t = t.clone();
            new_t.guard = Predicate::AND(vec![guard, t.guard.clone()]);
            new_t.actions.extend(actions.iter().cloned());
            new_t
                .node_mut()
                .update_name(&format!("{}_{}", t.name(), new_name));
            self.synchronized_paths.push(sync_with.clone());
            self.model.add_item(SPItem::Transition(new_t))
        } else {
            panic!(
                "syncronizing with {}, but {} is not a transition",
                sync_with, sync_with
            );
        }
    }

    pub fn make_model(mut self) -> (Model, SPState) {
        // disable all synchronized transitions (we keep only the new ones)
        for tp in &self.synchronized_paths {
            if let Some(SPMutItemRef::Transition(t)) = self.model.get_mut(tp) {
                t.guard = Predicate::FALSE;
            }
        }

        // operations start in init
        let op_state = self
            .model
            .all_operations()
            .iter()
            .map(|o| (o.path().clone(), "i".to_spvalue()))
            .collect::<Vec<_>>();

        // intentions are initially "paused"
        let intention_state = self
            .model
            .all_intentions()
            .iter()
            .map(|i| (i.path().clone(), "paused".to_spvalue()))
            .collect::<Vec<_>>();

        let mut s = SPState::new_from_values(op_state.as_slice());
        s.extend(SPState::new_from_values(intention_state.as_slice()));

        // intention state can be set manually in the initial state
        s.extend(self.initial_state);

        (self.model, s)
    }

    fn add_sub_item(&mut self, root: &str, item: SPItem) -> SPPath {
        let root_path = SPPath::from_string(root);
        let m = match self.model.get_mut(&root_path) {
            Some(SPMutItemRef::Model(m)) => m,
            Some(item) => panic!(
                "trying to add submodel {}, but that path is already a '{}'.",
                root,
                item.item_type_as_string()
            ),
            None => {
                let temp_model = Model::new(root, vec![]);
                let temp_model_path = self.model.add_item(SPItem::Model(temp_model));
                match self.model.get_mut(&temp_model_path) {
                    Some(SPMutItemRef::Model(m)) => m,
                    _ => panic!("cannot happen"),
                }
            }
        };
        m.add_item(item)
    }
}
