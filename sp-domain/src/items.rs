//!
//!
use super::*;
use std::collections::HashSet;

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Model {
    pub path: SPPath,
    pub intentions: Vec<Intention>,
    pub operations: Vec<Operation>,
    pub resources: Vec<Resource>,

    pub global_specs: Vec<Specification>,
    pub global_variables: Vec<Variable>,
    pub global_transitions: Vec<Transition>,
}

impl Model {
    pub fn new(name: &str) -> Model {
        let path = SPPath::from_string(name);
        Model { path, .. Model::default() }
    }

    pub fn add_resource(&mut self, name: &str) -> SPPath {
        let mut resource = Resource::new(name);
        let path = resource.path.add_parent_path_mut(&self.path);
        self.resources.push(resource);
        path
    }

    pub fn get_resource(&mut self, resource_path: &SPPath) -> &mut Resource {
        self.resources.iter_mut().find(|r| r.path() == resource_path).expect("resource not found")
    }

    pub fn add_product_domain(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
        let mut v = Variable::new(
            name,
            VariableType::Product,
            domain[0].has_type(),
            domain.to_vec(),
        );
        let product_state_path = self.path().add_child("product_state"); // TODO: fixme
        let path = v.path.add_parent_path_mut(&product_state_path);
        self.global_variables.push(v);
        path
    }

    pub fn add_estimated_bool(&mut self, name: &str) -> SPPath {
        let mut v = Variable::new_boolean(name, VariableType::Estimated);
        let path = v.path.add_parent_path_mut(&self.path);
        self.global_variables.push(v);
        path
    }

    pub fn add_estimated_domain(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
        let mut v = Variable::new(
            name,
            VariableType::Estimated,
            domain[0].has_type(),
            domain.to_vec(),
        );
        let path = v.path.add_parent_path_mut(&self.path);
        self.global_variables.push(v);
        path
    }

    pub fn add_invar(&mut self, name: &str, invariant: &Predicate) -> SPPath {
        let mut spec = Specification::new_transition_invariant(name, invariant.clone());
        let path = spec.path.add_parent_path_mut(&self.path);
        self.global_specs.push(spec);
        path
    }

    pub fn add_product_invar(&mut self, name: &str, invariant: &Predicate) -> SPPath {
        let mut spec = Specification::new_operation_invariant(name, invariant.clone());
        let path = spec.path.add_parent_path_mut(&self.path);
        self.global_specs.push(spec);
        path
    }

    pub fn add_transition(&mut self, mut transition: Transition) -> SPPath {
        let path = transition.path.add_parent_path_mut(&self.path);
        self.global_transitions.push(transition);
        path
    }

    pub fn add_op(&mut self, name: &str, guard: &Predicate, effects: &[Action], goal: &Predicate,
        post_actions: &[Action], auto: bool, mc_constraint: Option<Predicate>,
    ) -> SPPath {
        let effects_goals = vec![(effects, goal, post_actions)];
        let mut op = Operation::new(name, auto, guard, &effects_goals, mc_constraint);
        let ops_path = self.path().add_child("operations");
        let path = op.path.add_parent_path_mut(&ops_path);
        self.operations.push(op);
        path
    }

    /// Add an operation that models a non-deteministic plant, ie multiple possible outcomes.
    pub fn add_op_alt(
        &mut self, name: &str, guard: &Predicate,
        effects_goals_actions: &[(&[Action], &Predicate, &[Action])], auto: bool,
        mc_constraint: Option<Predicate>,
    ) -> SPPath {
        let mut op = Operation::new(name, auto, guard, effects_goals_actions, mc_constraint);
        let ops_path = self.path().add_child("operations");
        let path = op.path.add_parent_path_mut(&ops_path);
        self.operations.push(op);
        path
    }

    pub fn add_intention(
        &mut self, name: &str, resets: bool, pre: &Predicate, post: &Predicate,
        post_actions: &[Action]
    ) -> SPPath {
        let mut i = Intention::new(name, resets, pre, post, post_actions);
        let ints_path = self.path().add_child("intentions");
        let path = i.path.add_parent_path_mut(&ints_path);
        self.intentions.push(i);
        path
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Resource {
    pub path: SPPath,
    pub transitions: Vec<Transition>,
    pub specifications: Vec<Specification>,
    pub variables: Vec<Variable>,
    pub messages: Vec<Message>,
}

impl Resource {
    pub fn new(name: &str) -> Resource {
        let path = SPPath::from_string(name);
        Resource {
            path,
            transitions: vec![],
            specifications: vec![],
            variables: vec![],
            messages: vec![],
        }
    }

    pub fn messages(&self) -> &[Message] {
        self.messages.as_slice()
    }

    pub fn add_specification(&mut self, mut spec: Specification) -> SPPath {
        let path = spec.path.add_parent_path_mut(&self.path);
        self.specifications.push(spec);
        path
    }

    pub fn add_transition(&mut self, mut transition: Transition) -> SPPath {
        let path = transition.path.add_parent_path_mut(&self.path);
        self.transitions.push(transition);
        path
    }

    pub fn add_variable(&mut self, mut variable: Variable) -> SPPath {
        let path = variable.path.add_parent_path_mut(&self.path);
        self.variables.push(variable);
        path
    }

    pub fn add_message(&mut self, mut message: Message) {
        self.messages.push(message);
    }

    pub fn get_variables(&self) -> Vec<Variable> {
        self.variables.clone()
    }

    pub fn get_transitions(&self) -> Vec<Transition> {
        self.transitions.clone()
    }

    pub fn get_state_predicates(&self) -> Vec<Variable> {
        self.variables
            .iter()
            .filter(|x| {
                if let VariableType::Predicate(_) = x.variable_type() {
                    true
                } else {
                    false
                }
            })
            .cloned()
            .collect()
    }

    /// Get the message in the command topic. For now this is hardcoded
    pub fn get_message_for_topic(&self, topic: &SPPath) -> Option<Message> {
        for m in &self.messages {
            if &m.topic == topic {
                return Some(m.clone());
            }
        }
        None
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }


    //Modeling helpers below. These will crash on failure

    pub fn get_predicate(&self, name: &str) -> SPPath {
        for v in &self.variables {
            if let VariableType::Predicate(_) = v.variable_type() {
                if v.path().leaf() == name {
                    return v.path().clone();
                }
            }
        }
        panic!("could not find predicate {} in resource {}", name, self.path());
    }

    pub fn get_variable(&self, name: &str) -> SPPath {
        let name_as_path = SPPath::from_string(name);
        for v in &self.variables {
            if let VariableType::Predicate(_) = v.variable_type() {
                continue;
            } else {
                let mut variable_path = v.path().clone();
                variable_path.drop_parent(self.path()).expect("could not drop parent");
                if variable_path == name_as_path {
                    return v.path().clone();
                }
            }
        }
        panic!("could not find variable {} in resource {}", name, self.path());
    }

    /// Setup ros outgoing communication
    pub fn setup_ros_outgoing(&mut self, name: &str, topic: &str, msg_type: &str, variables: &[MessageVariable]) {
        let command_msg = Message {
            name: SPPath::from_string(name).add_parent_path_mut(&self.path),
            topic: SPPath::from_string(topic),
            category: MessageCategory::OutGoing,
            message_type: MessageType::Ros(msg_type.to_string()),
            variables: variables.to_vec(),
            variables_response: vec!(),
            variables_feedback: vec!(),
            send_predicate: Predicate::TRUE // TODO: FIX predicates for outgoing
        };
        self.add_message(command_msg);
    }

    /// Setup ros incoming communication
    pub fn setup_ros_incoming(&mut self, name: &str, topic: &str, msg_type: &str, variables: &[MessageVariable]) {
        let incoming_msg = Message {
            name: SPPath::from_string(name).add_parent_path_mut(&self.path),
            topic: SPPath::from_string(topic),
            category: MessageCategory::Incoming,
            message_type: MessageType::Ros(msg_type.to_string()),
            variables: variables.to_vec(),
            variables_response: vec!(),
            variables_feedback: vec!(),
            send_predicate: Predicate::TRUE // TODO: FIX predicates for outgoing
        };
        self.add_message(incoming_msg);
    }


    /// Setup ros service communication
    pub fn setup_ros_service(
        &mut self,
        name: &str,
        topic: &str,
        msg_type: &str,
        send_predicate: Predicate,
        request: &[MessageVariable],
        response: &[MessageVariable]) -> SPPath {

        let service_state = self.add_variable(Variable::new(
            &format!("{}/service", name),
            VariableType::Measured,
            SPValueType::String,
            vec!("ok".to_spvalue(), "req".to_spvalue(), "done".to_spvalue(), "timeout".to_spvalue())
        ));

        self.add_transition(
            Transition::new(
                &format!("{}_send_effect", name),
                p!([pp: send_predicate] && [[p: service_state == "ok"] || [p: service_state == "req"]]),
                vec![ a!( p: service_state = "done")],
                TransitionType::Effect
            )
        );
        self.add_transition(
            Transition::new(
                &format!("{}_reset_effect", name),
                p!([!pp: send_predicate] && [p: service_state == "done"]),
                vec![ a!( p: service_state = "ok")],
                TransitionType::Effect
            )
        );



        let service_msg = Message {
            name: SPPath::from_string(name).add_parent_path_mut(&self.path),
            topic: SPPath::from_string(topic),
            category: MessageCategory::Service,
            message_type: MessageType::Ros(msg_type.to_string()),
            variables: request.to_vec(),
            variables_response: response.to_vec(),
            variables_feedback: vec!(),
            send_predicate: send_predicate,
        };
        self.add_message(service_msg);
        service_state
    }

    /// Setup ros action communication
    pub fn setup_ros_action(
        &mut self,
        name: &str,
        topic: &str,
        msg_type: &str,
        send_predicate: Predicate,
        request: &[MessageVariable],
        feedback: &[MessageVariable],
        response: &[MessageVariable]) -> SPPath {

        let action_state = self.add_variable(Variable::new(
            &format!("{}/action", name),
            VariableType::Measured,
            SPValueType::String,
            vec!("ok".to_spvalue(), "requesting".to_spvalue(),
                 "accepted".to_spvalue(), "rejected".to_spvalue(),
                 "succeeded".to_spvalue(), "aborted".to_spvalue(),
                 "requesting_cancel".to_spvalue(),
                 "cancelling".to_spvalue(),
                 "cancel_rejected".to_spvalue(),
                 "timeout".to_spvalue(),
            )
        ));

        self.add_transition(
            Transition::new(
                &format!("{}_send_effect", name),
                p!([pp: send_predicate] && [
                    [p: action_state == "ok"] ||
                        [p: action_state == "requesting"] ||
                        [p: action_state == "accepted"]]),
                vec![ a!( p: action_state = "succeeded")],
                TransitionType::Effect
            )
        );
        self.add_transition(
            Transition::new(
                &format!("{}_reset_effect", name),
                p!([!pp: send_predicate] && [p: action_state == "succeeded"]),
                vec![ a!( p: action_state = "ok")],
                TransitionType::Effect
            )
        );

        let action_msg = Message {
            name: SPPath::from_string(name).add_parent_path_mut(&self.path),
            topic: SPPath::from_string(topic),
            category: MessageCategory::Action,
            message_type: MessageType::Ros(msg_type.to_string()),
            variables: request.to_vec(),
            variables_response: response.to_vec(),
            variables_feedback: feedback.to_vec(),
            send_predicate: send_predicate,
        };
        self.add_message(action_msg);
        action_state
    }

    /// Take a transition out from the resource in order to synchronize it with something else.
    pub fn take_transition(&mut self, name: &str) -> Transition {
        if let Some(idx) = self.transitions.iter().position(|t| t.path().leaf() == name) {
            return self.transitions.swap_remove(idx);
        } else {
            panic!("could not find transition {} in resource {}", name, self.path());
        }
    }

    pub fn initial_state(&self) -> SPState {
        let x:Vec<(SPPath, SPValue)> = self.variables.iter().map(|v| {
            let x = v.domain.first().unwrap_or_else(|| &SPValue::Unknown);
            (v.path().clone(), x.clone())
        }).collect();
        SPState::new_from_values(&x)
    }
}

/// Defines a message for a resource. It can only inlcude variables local to the
/// resource, but since the resource can include parameters, any value from the state
/// can be included.
/// The topic is the topic that will be used for publishing or subscribing

#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct Message {
    pub name: SPPath,
    pub topic: SPPath,
    pub category: MessageCategory,
    pub message_type: MessageType,
    pub variables: Vec<MessageVariable>,
    pub variables_response: Vec<MessageVariable>,
    pub variables_feedback: Vec<MessageVariable>,
    pub send_predicate: Predicate,
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum MessageCategory {
    OutGoing,
    Incoming,
    Service,
    Action
}
impl Default for MessageCategory {
    fn default() -> Self {
        MessageCategory::OutGoing
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum MessageType {
    Ros(String),
    JsonFlat,
    Json,
}
impl Default for MessageType {
    fn default() -> Self {
        MessageType::Json
    }
}

#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct MessageVariable {
    pub ros_path: SPPath,
    pub path: SPPath,
}

impl MessageVariable {
    pub fn new(path: &SPPath, ros_path: &str) -> Self {
        MessageVariable {
            ros_path: SPPath::from_string(ros_path),
            path: path.clone(),
        }
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Variable {
    pub path: SPPath,
    pub type_: VariableType,
    pub value_type: SPValueType,
    pub domain: Vec<SPValue>,
}

impl Variable {
    pub fn new(
        name: &str, type_: VariableType, value_type: SPValueType, domain: Vec<SPValue>,
    ) -> Variable {
        let path = SPPath::from_string(name);
        Variable {
            path,
            type_,
            value_type,
            domain,
        }
    }
    pub fn new_boolean(name: &str, type_: VariableType) -> Variable {
        Variable::new(
            name,
            type_,
            SPValueType::Bool,
            vec![false.to_spvalue(), true.to_spvalue()],
        )
    }

    pub fn new_predicate(name: &str, p: Predicate) -> Variable {
        Variable::new(
            name,
            VariableType::Predicate(p),
            SPValueType::Bool,
            vec![false.to_spvalue(), true.to_spvalue()],
        )
    }

    pub fn variable_type(&self) -> VariableType {
        self.type_.clone()
    }
    pub fn value_type(&self) -> SPValueType {
        self.value_type
    }
    pub fn domain(&self) -> &[SPValue] {
        self.domain.as_slice()
    }
    pub fn is_predicate(&self) -> bool {
        match self.variable_type() {
            VariableType::Predicate(_) => true,
            _ => false,
        }
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }

    pub fn to_message_variable(&self, ros_path: SPPath) -> MessageVariable {
        MessageVariable {
            ros_path,
            path: self.path().clone()
        }
    }
}

/// The possible variable types used by operations to define parameters
/// Must be the same as Variable
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum VariableType {
    Measured,
    Estimated,
    Command,
    Parameter,
    Predicate(Predicate),
    Runner, // not available to the formal models
    Product, // estimated variable available to the operation planner
}

impl Default for VariableType {
    fn default() -> Self {
        VariableType::Estimated
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum TransitionType {
    Controlled,
    Auto,
    Effect,
    Runner,
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Transition {
    pub path: SPPath,
    pub guard: Predicate,
    pub actions: Vec<Action>,
    pub type_: TransitionType,
}

impl Transition {
    pub fn new(name: &str, guard: Predicate, actions: Vec<Action>, type_: TransitionType) -> Self {
        let path = SPPath::from_string(name);
        Transition {
            path,
            guard,
            actions,
            type_,
        }
    }

    pub fn mut_guard(&mut self) -> &mut Predicate {
        &mut self.guard
    }

    pub fn guard(&self) -> &Predicate {
        &self.guard
    }
    pub fn actions(&self) -> &[Action] {
        self.actions.as_slice()
    }
    pub fn upd_state_path(&mut self, state: &SPState) {
        self.guard.upd_state_path(state);
        self.actions
            .iter_mut()
            .for_each(|a| a.upd_state_path(state));
    }

    pub fn modifies(&self) -> HashSet<SPPath> {
        let mut r = HashSet::new();

        r.extend(self.actions().iter().map(|a| a.var.clone()));
        r
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }

    // modeling stuff
    pub fn synchronize(&self, new_name: &str, guard: Predicate, actions: &[Action]) -> Transition {
        let mut t = self.clone();
        let orig_name = t.path.drop_leaf();
        let new_name = format!("{}_{}", orig_name, new_name);
        let new_path = t.path.add_child(&new_name);
        t.path = new_path;
        t.guard = Predicate::AND(vec![guard, self.guard.clone()]);
        t.actions.extend(actions.iter().cloned());
        t
    }
}

impl fmt::Display for Transition {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        let k = match self.type_ {
            TransitionType::Auto => "a",
            TransitionType::Controlled => "c",
            TransitionType::Effect => "e",
            TransitionType::Runner => "r",
        };

        let s = format!("{}_{}: {}/{:?}", k, self.path(), self.guard, self.actions);

        write!(fmtr, "{}", &s)
    }
}

impl EvaluatePredicate for Transition {
    fn eval(&self, state: &SPState) -> bool {
        self.guard.eval(state) && self.actions.iter().all(|a| a.eval(state))
    }
}

impl NextAction for Transition {
    fn next(&self, state: &mut SPState) -> SPResult<()> {
        for a in self.actions.iter() {
            if let Err(e) = a.next(state) {
                return Err(e);
            }
        }
        Ok(())
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Intention {
    pub path: SPPath,
    resets: bool,
    pre: Predicate,
    post: Predicate,
    post_actions: Vec<Action>,
}

impl Intention {
    pub fn new(
        name: &str, resets: bool, pre: &Predicate, post: &Predicate, post_actions: &[Action]) -> Intention {
        let path = SPPath::from_string(name);

        Intention {
            path,
            resets,
            pre: pre.clone(),
            post: post.clone(),
            post_actions: post_actions.to_vec(),
        }
    }

    pub fn is_executing(&self, state: &SPState) -> bool {
        state.sp_value_from_path(self.path())
            .map(|v| v == &"e".to_spvalue()).unwrap_or(false)
    }

    pub fn is_error(&self, state: &SPState) -> bool {
        state.sp_value_from_path(self.path())
            .map(|v| v == &"error".to_spvalue()).unwrap_or(false)
    }

    pub fn get_goal(&self) -> Predicate {
        self.post.clone()
    }

    pub fn make_runner_transitions(&self) -> Vec<Transition> {
        let state = self.path();

        let mut runner_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p: state == "i"), self.pre.clone()]),
            vec![a!(p: state = "e")],
            TransitionType::Controlled,
        );
        runner_start.path.add_parent_path_mut(&self.path);
        let mut f_actions = if self.resets {
            vec![a!(p: state = "i")]
        } else {
            vec![a!(p: state = "f")]
        };
        f_actions.extend(self.post_actions.iter().cloned());
        let mut runner_finish = Transition::new(
            "finish",
            Predicate::AND(vec![p!(p: state == "e"), self.post.clone()]),
            f_actions,
            TransitionType::Auto,
        );
        runner_finish.path.add_parent_path_mut(&self.path);

        vec![runner_start, runner_finish]
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Operation {
    pub path: SPPath,

    pub auto: bool,
    pub guard: Predicate,
    pub effects_goals_actions: Vec<(Vec<Action>, Predicate, Vec<Action>)>,
    pub mc_constraint: Option<Predicate>,
}

impl Operation {
    pub fn new(
        name: &str, auto: bool, guard: &Predicate,
        effects_goals_actions: &[(&[Action], &Predicate, &[Action])],
        mc_constraint: Option<Predicate>,
    ) -> Operation {
        let path = SPPath::from_string(name);

        Operation {
            path,
            auto,
            guard: guard.clone(),
            effects_goals_actions: effects_goals_actions
                .iter()
                .map(|(e, p, a)| (e.to_vec(), (*p).clone(), a.to_vec()))
                .collect(),
            mc_constraint,
        }
    }

    pub fn make_replan_invariants(&self) -> Vec<Specification> {
        let mut specs = vec![];
        for (effects, goal, actions) in self.effects_goals_actions.iter() {
            let pre = Predicate::AND(vec![self.guard.clone(), (*goal).clone()]);
            let mut act = effects.to_vec();
            act.extend(actions.iter().cloned());
            if !act.is_empty() {
                if self.auto {
                    // it is important to realize that we cannot freely change
                    // the goals of the high level when we are in this state
                    // or any other state from which this state can
                    // uncontrollably be reached. so we also create a spec here
                    let mut spec = Specification::new_transition_invariant("replan_spec", Predicate::NOT(Box::new(pre.clone())));
                    spec.path.add_parent_path_mut(&self.path);
                    specs.push(spec);
                }
            }
        }
        specs
    }

    pub fn make_lowlevel_transitions(&self) -> Vec<Transition> {
        let mut trans = vec![];
        for (i, (effects, goal, actions)) in self.effects_goals_actions.iter().enumerate() {
            let pre = Predicate::AND(vec![self.guard.clone(), (*goal).clone()]);
            let mut act = effects.to_vec();
            act.extend(actions.iter().cloned());
            // add a new low level transition. goal // effects
            if !act.is_empty() {
                let name = i.to_string();
                let mut t = Transition::new(
                    &name,
                    pre.clone(),
                    act.clone(),
                    if self.auto {
                        TransitionType::Auto
                    } else {
                        TransitionType::Controlled
                    },
                );
                t.path.add_parent_path_mut(&self.path);
                trans.push(t);
            }
        }
        trans
    }

    // todo: define "e" somewhere...
    pub fn is_executing(&self, state: &SPState) -> bool {
        state.sp_value_from_path(self.path())
            .map(|v| v == &"e".to_spvalue()).unwrap_or(false)
    }

    pub fn is_error(&self, state: &SPState) -> bool {
        state.sp_value_from_path(self.path())
            .map(|v| v == &"error".to_spvalue()).unwrap_or(false)
    }

    pub fn make_runner_transitions(&self) -> Vec<Transition> {
        // auto ops are not actually operations...
        let is_auto = self
            .effects_goals_actions
            .iter()
            .all(|(_, g, _)| g == &Predicate::TRUE);
        if is_auto {
            return vec![];
        }

        let goal_state_paths = self.get_goal_state_paths();
        let mut actions: Vec<_> = goal_state_paths.iter().map(|p| {
            let new_path = self.path().add_child_path(p);
            Action::new(new_path, Compute::PredicateValue(PredicateValue::SPPath(p.clone(), None)))
        }).collect();
        let state = self.path();
        actions.push(a!(p: state = "e"));
        let mut runner_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p: state == "i"), self.guard.clone()]),
            actions,
            TransitionType::Controlled,
        );
        runner_start.path.add_parent_path_mut(&self.path);

        let post_cond = p!(p: state == "e");
        let post_cond = Predicate::AND(vec![post_cond, self.get_goal(None)]);
        let mut runner_finish = Transition::new(
            "finish",
            post_cond,
            vec![a!(p: state = "i")],
            TransitionType::Auto,
        );
        runner_finish.path.add_parent_path_mut(&self.path);
        //println!("FINISH TRANSITION FOR {}: {:#?}", runner_finish.path(), runner_finish);

        vec![runner_start, runner_finish]
    }

    pub fn make_planning_trans(&self) -> Vec<Transition> {
        self.effects_goals_actions
            .iter()
            .enumerate()
            .map(|(i, (e, g, _))| {
                let is_auto = g == &Predicate::TRUE;
                let auto = if is_auto {
                    "_auto".to_string()
                } else {
                    "".to_string()
                };
                let name = format!("{}{}", i, auto);
                let mut t = Transition::new(
                    &name,
                    self.guard.clone(),
                    e.to_vec(),
                    if is_auto {
                        TransitionType::Auto
                    } else {
                        TransitionType::Controlled
                    },
                );
                t.path.add_parent_path_mut(&self.path);
                t
            })
            .collect()
    }

    /// Returns a list of paths with the goal states
    /// this operation goal needs access to.
    pub fn get_goal_state_paths(&self) -> Vec<SPPath> {
        self.effects_goals_actions.
            iter().flat_map(|(e, _, _)| {
                e.iter().flat_map(|e| {
                    match &e.value {
                        Compute::PredicateValue(PredicateValue::SPPath(p, _)) => Some(p.clone()),
                        _ => None
                    }
                })
            }).collect()
    }

    /// Returns this operations goal state as a predicate
    /// Optionally we can get the concrete goal if we also provide a state
    pub fn get_goal(&self, state: Option<&SPState>) -> Predicate {
        let v: Vec<Predicate> = self.effects_goals_actions.
            iter().map(|(e, _, _)| {
                Predicate::AND(
                    e.iter().flat_map(|e| {
                        match &e.value {
                            Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
                                let new_path = self.path().add_child_path(p);
                                if let Some(state) = state {
                                    let concrete_value = state.sp_value_from_path(&new_path)
                                        .expect("error getting concrete value from state");
                                    Some(Predicate::EQ(
                                        PredicateValue::SPPath(e.var.clone(), None),
                                        PredicateValue::SPValue(concrete_value.clone()),
                                    ))
                                } else {
                                    Some(Predicate::EQ(
                                        PredicateValue::SPPath(e.var.clone(), None),
                                        PredicateValue::SPPath(new_path, None),
                                    ))
                                }
                            },
                            Compute::PredicateValue(p) => {
                                Some(Predicate::EQ(
                                    PredicateValue::SPPath(e.var.clone(), None),
                                    p.clone(),
                                ))
                            },
                            _ => None
                        }
                    }).collect())
            }).collect();
        Predicate::OR(v)
    }

    pub fn make_verification_goal(&self) -> Predicate {
        let goals = Predicate::OR(
            self.effects_goals_actions
                .iter()
                .map(|(_, g, _)| (*g).clone())
                .collect(),
        );
        Predicate::AND(vec![self.guard.clone(), goals.clone()])
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum SpecificationType {
    TransitionInvariant(Predicate),
    OperationInvariant(Predicate),
}

/// Specs are used to define global constraints
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Specification {
    pub path: SPPath,
    pub type_: SpecificationType,
}

impl Specification {
    pub fn new_transition_invariant(name: &str, invariant: Predicate) -> Self {
        let path = SPPath::from_string(name);
        Specification { path, type_: SpecificationType::TransitionInvariant(invariant) }
    }
    pub fn new_operation_invariant(name: &str, invariant: Predicate) -> Self {
        let path = SPPath::from_string(name);
        Specification { path, type_: SpecificationType::OperationInvariant(invariant) }
    }
    pub fn invariant(&self) -> &Predicate {
        match &self.type_ {
            SpecificationType::TransitionInvariant(inv) => inv,
            SpecificationType::OperationInvariant(inv) => inv,
        }
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct TransitionSpec {
    pub path: SPPath,
    pub spec_transition: Transition,
    pub syncronized_with: Vec<SPPath>,
}

impl TransitionSpec {
    pub fn new(
        name: &str, spec_transition: Transition, syncronized_with: Vec<SPPath>,
    ) -> TransitionSpec {
        let path = SPPath::from_string(name);
        TransitionSpec {
            path,
            spec_transition,
            syncronized_with,
        }
    }

    pub fn path(&self) -> &SPPath {
        &self.path
    }
}

#[cfg(test)]
mod test_items {
    use super::*;

    #[test]
    fn testing_transitions() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let xy = SPPath::from_slice(&["x", "y"]);

        let mut s = state!(ab => 2, ac => true, kl => 3, xy => false);
        let p = p!([!p: ac] && [!p: xy]);

        let a = a!(p: ac = false);
        let b = a!(p:ab <- p:kl);
        let c = a!(p:xy ? p);

        let mut t1 = Transition::new("t1", p!(p: ac), vec![a], TransitionType::Auto);
        let mut t2 = Transition::new("t2", p!(!p: ac), vec![b], TransitionType::Auto);
        let mut t3 = Transition::new("t3", Predicate::TRUE, vec![c], TransitionType::Auto);

        let res = t1.eval(&s);
        println!("t1.eval: {:?}", res);
        assert!(res);

        let res = t1.next(&mut s);
        println!("t1.next: {:?}", res);

        s.take_transition();
        assert_eq!(s.sp_value_from_path(&ac).unwrap(), &false.to_spvalue());

        let res = t2.eval(&s);
        println!("t2: {:?}", res);
        assert!(res);

        let res = t2.next(&mut s);
        println!("t2.next: {:?}", res);

        s.take_transition();
        assert_eq!(s.sp_value_from_path(&ab).unwrap(), &3.to_spvalue());

        s.take_transition();
        let res = t3.eval(&s);
        println!("t3: {:?}", res);
        assert!(res);
        t3.next(&mut s).unwrap();

        s.take_transition();
        assert_eq!(s.sp_value_from_path(&xy).unwrap(), &true.to_spvalue());
    }
}
