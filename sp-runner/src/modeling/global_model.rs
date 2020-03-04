use sp_domain::*;
use std::ops::Index;

// helpers for creating the "global" model

#[derive(Debug, PartialEq, Clone)]
pub struct GResource {
    name: String,
    variables: Vec<Variable>
}

impl Index<&str> for GResource {
    type Output = SPPath;
    fn index(&self, name: &str) -> &SPPath {
        let result = self.variables.iter().find(|v|v.name() == name);
        result.expect(&format!("{}/{} not found", self.name, name)).path()
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

    pub fn use_resource(&mut self, r: Resource) -> GResource {
        let rp = self.model.add_item(SPItem::Resource(r));
        let r = self.model.get(&rp).unwrap().unwrap_resource();
        let vars = r.get_variables();
        GResource { name: r.name().to_owned(), variables: vars }
    }

    pub fn add_estimated_domain(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
        let v = Variable::new(name, VariableType::Estimated, SPValueType::String, domain.to_vec());
        self.model.add_item(SPItem::Variable(v))
    }

    pub fn add_predicate_variable(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
        let v = Variable::new(name, VariableType::Estimated, SPValueType::String, domain.to_vec());
        self.model.add_item(SPItem::Variable(v))
    }

    pub fn add_auto(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![], // no effects
            false, // auto!
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_delib(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![],
            true,
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_effect(&mut self, name: &str, guard: &Predicate, effects: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            vec![],
            effects.to_vec(),
            false,
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_simulation_auto(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![], // no effects
            false, // auto!
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn find(&mut self, name: &str, path_sections: &[&str]) -> SPPath {
        self.model.find_item(name, path_sections)
            .expect(&format!("cannot find {:?} / {}", path_sections, name)).path()
    }

    pub fn add_op(&mut self, name: &str, resets: bool, pre: &Predicate,
                  post: &Predicate, effects: &[Action],
                  invariant: Option<Predicate>) -> SPPath {

        let state = SPPath::from_slice(&[self.model.name(), name, "state"]);

        let op_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p:state == "i"), pre.clone()]),
            vec![a!(p:state = "e")],
            vec![],
            true,
        );

        // effect for planning
        let op_effect = Transition::new(
            "executing",
            Predicate::AND(vec![p!(p:state == "e")]),
            vec![],
            effects.to_vec(),
            false,
        );

        let f_actions =
            if resets {
                vec![a!(p:state = "i")]
            } else {
                vec![a!(p:state = "f")]
            };
        let op_finish = Transition::new(
            "finish",
            Predicate::AND(vec![p!(p:state == "e"), post.clone()]),
            f_actions,
            vec![],
            false,
        );
        let op_goal = IfThen::new("goal", p!(p:state == "e"), post.clone(), invariant);

        let op = Operation::new(
            name,
            &[op_start, op_effect, op_finish],
            Some(op_goal),
        );

        self.model.add_item(SPItem::Operation(op))
    }

    pub fn add_invar(&mut self, name: &str, invariant: &Predicate) {
        self.model.add_item(SPItem::Spec(Spec::new(name, invariant.clone())));
    }

    pub fn initial_state(&mut self, state: &[(&SPPath, SPValue)]) {
        let s: Vec<(SPPath, SPValue)> = state.iter().map(|(p,v)| {
            ((*p).clone(), v.clone())
        }).collect();
        self.initial_state.add_variables(s);
    }

    pub fn make_model(self) -> (Model, SPState) {

        let mut s = self.initial_state.clone();

        // add operation to the initial state here
        let global_ops: Vec<&Operation> = self.model.items()
            .iter()
            .flat_map(|i| match i {
                SPItem::Operation(o) => Some(o),
                _ => None,
            })
            .collect();
        let global_ops_vars = global_ops.iter().map(|o|o.state_variable());

        let state: Vec<_> = global_ops_vars.map(|v| (v.node().path().clone(), "i".to_spvalue())).collect();
        s.extend(SPState::new_from_values(&state[..]));


        // lastly, add all specifications.
        (self.model, s)
    }
}
