//!
//! 
use super::*;

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum SPItem {
    Model(Model),
    Resource(Resource),
    Message(Message),
    Topic(Topic),
    Variable(Variable),
    Operation(Operation),
    Ability(Ability),
    Transition(Transition),
    IfThen(IfThen),
    //SOP(SOP),
}

#[derive(Debug, PartialEq, Clone)]
pub enum SPItemRef<'a> {
    Model(&'a Model),
    Resource(&'a Resource),
    Message(&'a Message),
    Topic(&'a Topic),
    Variable(&'a Variable),
    Operation(&'a Operation),
    Ability(&'a Ability),
    Transition(&'a Transition),
    IfThen(&'a IfThen),
    //SOP(SOP),
}

impl Noder for SPItem {
    fn node(&self) -> &SPNode {
        match self {
            SPItem::Model(x) => x.node(),
            SPItem::Resource(x) => x.node(),
            SPItem::Message(x) => x.node(),
            SPItem::Topic(x) => x.node(),
            SPItem::Variable(x) => x.node(),
            SPItem::Operation(x) => x.node(),
            SPItem::Ability(x) => x.node(),
            SPItem::Transition(x) => x.node(),
            SPItem::IfThen(x) => x.node(),
        }
    }
    fn node_mut(&mut self) -> &mut SPNode {
        match self {
            SPItem::Model(ref mut x) => x.node_mut(),
            SPItem::Resource(ref mut x) => x.node_mut(),
            SPItem::Message(ref mut x) => x.node_mut(),
            SPItem::Topic(ref mut x) => x.node_mut(),
            SPItem::Variable(ref mut x) => x.node_mut(),
            SPItem::Operation(ref mut x) => x.node_mut(),
            SPItem::Ability(ref mut x) => x.node_mut(),
            SPItem::Transition(ref mut x) => x.node_mut(),
            SPItem::IfThen(ref mut x) => x.node_mut(),
        }
    }
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.find_child(next, path),
            SPItem::Resource(x) => x.find_child(next, path),
            SPItem::Message(x) => x.find_child(next, path),
            SPItem::Topic(x) => x.find_child(next, path),
            SPItem::Variable(x) => x.find_child(next, path),
            SPItem::Operation(x) => x.find_child(next, path),
            SPItem::Ability(x) => x.find_child(next, path),
            SPItem::Transition(x) => x.find_child(next, path),
            SPItem::IfThen(x) => x.find_child(next, path),
        }
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        match self {
            SPItem::Model(x) => x.update_path_children(paths),
            SPItem::Resource(x) => x.update_path_children(paths),
            SPItem::Message(x) => x.update_path_children(paths),
            SPItem::Topic(x) => x.update_path_children(paths),
            SPItem::Variable(x) => x.update_path_children(paths),
            SPItem::Operation(x) => x.update_path_children(paths),
            SPItem::Ability(x) => x.update_path_children(paths),
            SPItem::Transition(x) => x.update_path_children(paths),
            SPItem::IfThen(x) => x.update_path_children(paths),
        }
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        match self {
            SPItem::Model(x) => x.as_ref(),
            SPItem::Resource(x) => x.as_ref(),
            SPItem::Message(x) => x.as_ref(),
            SPItem::Topic(x) => x.as_ref(),
            SPItem::Variable(x) => x.as_ref(),
            SPItem::Operation(x) => x.as_ref(),
            SPItem::Ability(x) => x.as_ref(),
            SPItem::Transition(x) => x.as_ref(),
            SPItem::IfThen(x) => x.as_ref(),
        }
    }
}


impl<'a> SPItemRef<'a> {
    pub fn node(&self) -> &SPNode {
        match self {
            SPItemRef::Model(x) => &x.node,
            SPItemRef::Resource(x) => &x.node,
            SPItemRef::Message(x) => &x.node,
            SPItemRef::Topic(x) => &x.node,
            SPItemRef::Variable(x) => &x.node,
            SPItemRef::Operation(x) => &x.node,
            SPItemRef::Ability(x) => &x.node,
            SPItemRef::Transition(x) => &x.node,
            SPItemRef::IfThen(x) => &x.node,
        }
    }
    pub fn name(&self) -> &str {
        self.node().name()
    }
}



#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Model {
    node: SPNode,
    items: Vec<SPItem>
}

impl Noder for Model {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        find_in_list(self.items.as_slice(), next, path)
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        update_path_in_list(self.items.as_mut_slice(), paths);
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Model(self)
    }
}

impl Model {
    pub fn new(name: &str, items: Vec<SPItem>) -> Model {
        let node = SPNode::new(name);
        Model {
            node,
            items
        }
    }
    pub fn new_root(name: &str, items: Vec<SPItem>) -> Model {
        let mut m = Model::new(name, items);
        m.update_path(&SPPaths::new(None, Some(GlobalPath::new())));
        m
    }

    pub fn items(&self) -> &[SPItem] {
        self.items.as_slice()
    }

    pub fn add_item(&mut self, mut item: SPItem) -> SPPaths {
        let paths = item.update_path(self.node.paths());
        self.items.push(item);
        paths
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Resource {
    node: SPNode,
    abilities: Vec<Ability>,
    parameters: Vec<Variable>,
    messages: Vec<Topic>, // Also include estimated here on an estimated topic
                              //pub comm: ResourceComm,
}

impl Noder for Resource {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        let res = find_in_list(self.abilities.as_slice(), next, path);
        if res.is_some() {return res};
        let res = find_in_list(self.parameters.as_slice(), next, path);
        if res.is_some() {return res};
        let res = find_in_list(self.messages.as_slice(), next, path);
        
        return res
    }
    fn update_path_children(&mut self, _paths: &SPPaths) { 
        let mut local = LocalPath::from(vec!(self.node.name().to_string()));
        self.node.paths_mut().upd_local(Some(local));
        let paths = self.node.paths();
        update_path_in_list(self.abilities.as_mut_slice(), &paths);
        update_path_in_list(self.parameters.as_mut_slice(), &paths);
        update_path_in_list(self.messages.as_mut_slice(), &paths);
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Resource(self)
    }
}

impl Resource {
    pub fn new(name: &str) -> Resource {
        let mut node = SPNode::new(name);
        let mut local = LocalPath::from(vec!(name.to_string()));
        node.update_path(&SPPaths::new(Some(local), None));
        Resource {
            node,
            ..Resource::default()
        }
    }

    pub fn abilities(&self) -> &[Ability] {self.abilities.as_slice()}
    pub fn add_ability(&mut self, mut ability: Ability) -> SPPaths {
        let paths = ability.update_path(self.node.paths());
        self.abilities.push(ability);
        paths
    }

    pub fn parameters(&self) -> &[Variable] {self.parameters.as_slice()}
    pub fn add_parameter(&mut self, mut parameter: Variable) -> SPPaths {
        let paths = parameter.update_path(self.node.paths());
        self.parameters.push(parameter);
        paths
    }

    pub fn messages(&self) -> &[Topic] {self.messages.as_slice()}
    pub fn add_message(&mut self, mut message: Topic) -> SPPaths {
        let paths = message.update_path(self.node.paths());
        self.messages.push(message);
        paths
    }
}


#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Topic {
    node: SPNode,
    msg: MessageField,
}

impl Noder for Topic {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        if self.msg.name() != next {return None}
        self.msg.find(path)
    }
    fn update_path_children(&mut self, paths: &SPPaths) { 
        self.msg.update_path(paths);
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Topic(self)
    }
}

impl Topic {
    pub fn new(name: &str, msg: MessageField) -> Topic {
        let node = SPNode::new(name);
        Topic {
            node,
            msg
        }
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Message {
    node: SPNode,
    fields: Vec<MessageField>,  // note, the field name is in each node
}

impl Noder for Message {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        find_in_list(self.fields.as_slice(), next, path)
    }
    fn update_path_children(&mut self, paths: &SPPaths) { 
        update_path_in_list(self.fields.as_mut_slice(), paths);
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Message(self)
    }
}

impl Message {
    pub fn new(name: &str, fields: Vec<MessageField>) -> Message {
        let node = SPNode::new(name);
        Message {
            node,
            fields
        }
    }
    pub fn fields(&self) -> &[MessageField] {self.fields.as_slice()}
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum MessageField {
    Msg(Message),
    Var(Variable),
}

impl Noder for MessageField {
    fn node(&self) -> &SPNode {
        match self {
            MessageField::Msg(ref x) => x.node(),
            MessageField::Var(ref x) => x.node()
        }
    }
    fn node_mut(&mut self) -> &mut SPNode {
        match self {
            MessageField::Msg(ref mut x) => x.node_mut(),
            MessageField::Var(ref mut x) => x.node_mut()
        }
    }
    
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        match self {
            MessageField::Msg(ref x) => x.find_child(next, path),
            MessageField::Var(ref x) => x.find_child(next, path),
        }
    }
    fn update_path_children(&mut self, paths: &SPPaths) { 
        match self {
            MessageField::Msg(ref mut x) => x.update_path_children(paths),
            MessageField::Var(ref mut x) => x.update_path_children(paths),
        }
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        match self {
            MessageField::Msg(ref x) => x.as_ref(),
            MessageField::Var(ref x) => x.as_ref(),
        }
    }
}

impl Default for MessageField {
    fn default() -> Self {
        MessageField::Var(Variable::default())
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Variable {
    node: SPNode,
    type_: VariableType,
    value_type: SPValueType,
    initial_value: SPValue,
    domain: Vec<SPValue>,
}

impl Noder for Variable {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _paths: &SPPaths) {}
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Variable(self)
    }
}

impl Variable {
    pub fn new(name: &str, 
                type_: VariableType,
                value_type: SPValueType,
                initial_value: SPValue,
                domain: Vec<SPValue>,
    ) -> Variable {
        let node = SPNode::new(name);
        Variable {
            node,
            type_,
            value_type,
            initial_value,
            domain,
        }
    }
    pub fn new_boolean(name: &str, 
                type_: VariableType,
    ) -> Variable {
        Variable::new(
            name,
            type_,
            SPValueType::Bool,
            false.to_spvalue(),
            vec!(false.to_spvalue(), true.to_spvalue()),
        )
    }
    pub fn variable_type(&self) -> VariableType {
        self.type_.clone()
    }
    pub fn value_type(&self) -> SPValueType {
        self.value_type.clone()
    }
    pub fn initial_value(&self) -> SPValue {
        self.initial_value.clone()
    }
    pub fn domain(&self) -> &[SPValue] {
        self.domain.as_slice()
    }
}

/// The possible variable types used by operations to define parameters
/// Must be the same as Variable
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum VariableType {
    Measured,
    Estimated,
    Command,
    Parameter(Option<SPPath>),
    Predicate(Predicate)
}

impl Default for VariableType {
    fn default() -> Self {
        VariableType::Estimated
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Transition {
    node: SPNode,
    guard: Predicate,
    actions: Vec<Action>,
    effects: Vec<Action>,
}

impl Noder for Transition {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _paths: &SPPaths) { }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Transition(self)
    }
}

impl Transition {
    pub fn new(name: &str, 
               guard: Predicate,
               actions: Vec<Action>,
               effects: Vec<Action> ) -> Transition {
        let node = SPNode::new(name);
        Transition {
            node,
            guard,
            actions,
            effects
        }
    }
}

impl EvaluatePredicate for Transition {
    fn eval(&self, state: &SPState) -> bool {
        self.guard.eval(state) && self.actions.iter().all(|a| a.eval(state))
    }
}

impl NextAction for Transition {
    fn next(&self, state: &SPState) -> Result<AssignState> {
        let mut s: HashMap<SPPath, AssignStateValue> = HashMap::new();
        for a in self.actions.iter() {
            let next = a.next(state)?;
            s.extend(next.s);
        }
        Ok(AssignState{s})
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Ability {
    node: SPNode,
    controlled: Vec<Transition>,
    uncontrolled: Vec<Transition>,
    predicates: Vec<Variable>
}

impl Noder for Ability {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        let res = find_in_list(self.controlled.as_slice(), next, path);
        if res.is_some() {return res};
        let res = find_in_list(self.uncontrolled.as_slice(), next, path);
        if res.is_some() {return res};
        let res = find_in_list(self.predicates.as_slice(), next, path);
        return res;
    }
    fn update_path_children(&mut self, paths: &SPPaths) { 
        update_path_in_list(self.controlled.as_mut_slice(), paths);
        update_path_in_list(self.uncontrolled.as_mut_slice(), paths);
        update_path_in_list(self.predicates.as_mut_slice(), paths);
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Ability(self)
    }
}

impl Ability {
    pub fn new(name: &str,
                controlled: Vec<Transition>,
                uncontrolled: Vec<Transition>,
                predicates: Vec<Variable>
    ) -> Ability {
        let node = SPNode::new(name);
        Ability {
            node,
            controlled,
            uncontrolled,
            predicates
        }
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Operation {
    node: SPNode,
    precondition: Vec<Transition>,
    postcondition: Vec<Transition>,
    uncontrolled: Vec<Transition>,
    predicates: Vec<Variable>,
    goal: Option<IfThen>,
    invariant: Option<IfThen>,
}

impl Noder for Operation {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        let res = find_in_list(self.precondition.as_slice(), next, path);
        if res.is_some() {return res};
        let res = find_in_list(self.postcondition.as_slice(), next, path);
        if res.is_some() {return res};
        let res = find_in_list(self.uncontrolled.as_slice(), next, path);
        if res.is_some() {return res};
        let res = find_in_list(self.predicates.as_slice(), next, path);
        if res.is_some() {return res};
        let res = self.goal.as_ref().and_then(|x| x.find(path));
        if res.is_some() {return res};
        let res = self.invariant.as_ref().and_then(|ref x| x.find(path));
        return res;
    }
    fn update_path_children(&mut self, paths: &SPPaths) { 
        update_path_in_list(self.precondition.as_mut_slice(), paths);
        update_path_in_list(self.postcondition.as_mut_slice(), paths);
        update_path_in_list(self.uncontrolled.as_mut_slice(), paths);
        update_path_in_list(self.predicates.as_mut_slice(), paths);
        self.goal.as_mut().map(|mut x| x.update_path(paths));
        self.invariant.as_mut().map(|mut x| x.update_path(paths));
    }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::Operation(self)
    }
}

impl Operation {
    pub fn new( name: &str,
                precondition: Vec<Transition>,
                postcondition: Vec<Transition>,
                uncontrolled: Vec<Transition>,
                predicates: Vec<Variable>,
                goal: Option<IfThen>,
                invariant: Option<IfThen>
    ) -> Operation {
        let node = SPNode::new(name);
        Operation {
            node,
            precondition,
            postcondition,
            uncontrolled,
            predicates,
            goal,
            invariant,
        }
    }
}

/// An IfThen is used by operaitons to define goals or invariants. When the if_
/// predicate is true, then the then_ predicate is either a goal or an invariant
/// that the planner will use for planning. TODO: Maybe we should have a better name?
#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct IfThen {
    node: SPNode,
    if_: Predicate,
    then_: Predicate,
}

impl Noder for IfThen {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode { &mut self.node}
    fn find_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _paths: &SPPaths) { }
    fn as_ref<'a>(&'a self) -> SPItemRef<'a> {
        SPItemRef::IfThen(self)
    }
}

impl IfThen {
    pub fn new( name: &str,
                if_: Predicate,
                then_: Predicate
    ) -> IfThen {
        let node = SPNode::new(name);
        IfThen {
            node,
            if_,
            then_
        }
    }
}




#[cfg(test)]
mod test_items {
    use super::*;

  #[test]
    fn testing_transitions() {
        let ab = SPPath::from_array(&["a", "b"]);
        let ac = SPPath::from_array(&["a", "c"]);
        let kl = SPPath::from_array(&["k", "l"]);
        let xy = SPPath::from_array(&["x", "y"]);

        let mut s = state!(ab => 2, ac => true, kl => 3, xy => false);
        let p = pr!{{p!(!ac)} && {p!(!xy)}};

        let a = a!(ac = false);
        let b = a!(ab <- kl);
        let c = a!(xy ? p);

        let t1 = Transition::new("t1", p!(ac),vec!(a), vec!());
        let t2 = Transition::new("t2", p!(!ac),vec!(b), vec!());
        let t3 = Transition::new("t3", Predicate::TRUE,vec!(c), vec!());

        let res = t1.eval(&s);
        println!("t1.eval: {:?}", res);
        assert!(res);

        let res = t1.next(&s).unwrap();
        println!("t1.next: {:?}", res);

        s.insert_map(res).unwrap();
        println!("s after t1: {:?}", s);
        assert_eq!(s.get_value(&ac).unwrap(), &false.to_spvalue());

        let res = t2.eval(&s);
        println!("t2: {:?}", res);
        assert!(res);

        let res = t2.next(&s).unwrap();
        println!("t2.next: {:?}", res);

        s.insert_map(res).unwrap();
        println!("s after t2{:?}", s);
        assert_eq!(s.get_value(&ab).unwrap(), &3.to_spvalue());

        s.take_all_next();
        let res = t3.eval(&s);
        println!("t3: {:?}", res);
        assert!(res);
        s.insert_map(t3.next(&s).unwrap()).unwrap();
        assert_eq!(s.get_value(&xy).unwrap(), &true.to_spvalue());

    }

    // #[test]
    // fn transition_macros() {
    //     let ab = SPPath::from_array(&["a", "b"]);
    //     let ac = SPPath::from_array(&["a", "c"]);
    //     let kl = SPPath::from_array(&["k", "l"]);
    //     let xy = SPPath::from_array(&["x", "y"]);

    //     let t = transition!(SPPath::from_array(&["t"]), Predicate::TRUE);
    //     let res = Transition::new(
    //         "res",
    //         Predicate::TRUE,
    //         vec!(),
    //         vec!(),
    //     );
    //     assert_eq!(t, res);

    //     let t = transition!(SPPath::from_array(&["t"]), p!(ab), a!(ab), a!(!kl));
    //     let res = Transition {
    //         path: t.path.clone(),
    //         guard: p!(ab),
    //         actions: vec!(a!(ab), a!(!kl)),
    //         effects: vec!(),
    //     };
    //     assert_eq!(t, res);

    //     let t = transition!(SPPath::from_array(&["t"]), p!(ab), a!(ab) ; a!(!kl));
    //     let res = Transition {
    //         path: t.path.clone(),
    //         guard: p!(ab),
    //         actions: vec!(a!(ab)),
    //         effects: vec!(a!(!kl)),
    //     };
    //     assert_eq!(t, res);

    //     let t = transition!(SPPath::from_array(&["t"]), p!(ab), a!(ab), a!(ab); a!(!kl), a!(!kl));
    //     let res = Transition {
    //         path: t.path.clone(),
    //         guard: p!(ab),
    //         actions: vec!(a!(ab), a!(ab)),
    //         effects: vec!(a!(!kl), a!(!kl)),
    //     };
    //     assert_eq!(t, res);
    // }



}