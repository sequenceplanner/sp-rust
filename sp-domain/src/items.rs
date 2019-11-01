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
    Spec(Spec),
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
            SPItem::Spec(x) => x.node(),
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
            SPItem::Spec(ref mut x) => x.node_mut(),
        }
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.get_child(next, path),
            SPItem::Resource(x) => x.get_child(next, path),
            SPItem::Message(x) => x.get_child(next, path),
            SPItem::Topic(x) => x.get_child(next, path),
            SPItem::Variable(x) => x.get_child(next, path),
            SPItem::Operation(x) => x.get_child(next, path),
            SPItem::Ability(x) => x.get_child(next, path),
            SPItem::Transition(x) => x.get_child(next, path),
            SPItem::IfThen(x) => x.get_child(next, path),
            SPItem::Spec(x) => x.get_child(next, path),
        }
    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Resource(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Message(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Topic(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Variable(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Operation(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Ability(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Transition(x) => x.find_item_among_childs(name, path_sections),
            SPItem::IfThen(x) => x.find_item_among_childs(name, path_sections),
            SPItem::Spec(x) => x.find_item_among_childs(name, path_sections),
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
            SPItem::Spec(x) => x.update_path_children(paths),
        }
    }
    fn as_ref(&self) -> SPItemRef<'_> {
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
            SPItem::Spec(x) => x.as_ref(),
        }
    }
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
    Spec(&'a Spec),
    //SOP(SOP),
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
            SPItemRef::Spec(x) => &x.node,
        }
    }
    pub fn item(&self) -> SPItem {
        match self {
            SPItemRef::Model(x) => SPItem::Model({*x}.clone()),
            SPItemRef::Resource(x) => SPItem::Resource({*x}.clone()),
            SPItemRef::Message(x) => SPItem::Message({*x}.clone()),
            SPItemRef::Topic(x) => SPItem::Topic({*x}.clone()),
            SPItemRef::Variable(x) => SPItem::Variable({*x}.clone()),
            SPItemRef::Operation(x) => SPItem::Operation({*x}.clone()),
            SPItemRef::Ability(x) => SPItem::Ability({*x}.clone()),
            SPItemRef::Transition(x) => SPItem::Transition({*x}.clone()),
            SPItemRef::IfThen(x) => SPItem::IfThen({*x}.clone()),
            SPItemRef::Spec(x) => SPItem::Spec({*x}.clone()),
        }
    }
    pub fn name(&self) -> &str {
        self.node().name()
    }
}


/// A trait for unwrapping SPItemRefs and cloning the items
pub trait SPItemUnwrapper {
    fn item(&self) -> SPItem;
    fn local_path(&self) -> Option<LocalPath> {
        self.item().node().local_path().clone()
    }
    fn unwrap_local_path(&self) -> LocalPath {
        self.local_path().unwrap()
    }
    fn global_path(&self) -> Option<GlobalPath> {
        self.item().node().global_path().clone()
    }
    fn unwrap_global_path(&self) -> GlobalPath {
        self.global_path().unwrap()
    }
    fn as_variable(&self) -> Option<Variable> {
        if let SPItem::Variable(x) = self.item() { Some(x) } else { None }
    }
    fn unwrap_variable(&self) -> Variable {
        self.as_variable().unwrap()
    }
    fn as_resource(&self) -> Option<Resource> {
        if let SPItem::Resource(x) = self.item() { Some(x) } else { None }
    }
    fn unwrap_resource(&self) -> Resource {
        self.as_resource().unwrap()
    }
    // Add items here when needed
}

impl SPItemUnwrapper for SPItemRef<'_> {
    fn item(&self) -> SPItem {
        self.item()
    }
}

impl SPItemUnwrapper for Option<SPItemRef<'_>> {
    fn item(&self) -> SPItem {
        self.clone().unwrap().item()
    }
}

impl SPItemUnwrapper for SPItem {
    fn item(&self) -> SPItem {
        self.clone()
    }
}


// ####################
// Items below


#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Model {
    node: SPNode,
    items: Vec<SPItem>,
}

impl Noder for Model {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        get_from_list(self.items.as_slice(), next, path)
    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.items.as_slice(), name, path_sections)
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        update_path_in_list(self.items.as_mut_slice(), paths);
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Model(self)
    }
}

impl Model {
    pub fn new(name: &str, items: Vec<SPItem>) -> Model {
        let node = SPNode::new(name);
        Model { node, items }
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
    sub_items: Vec<SPItem>
}

impl Noder for Resource {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        get_from_list(self.abilities.as_slice(), next, path).or_else(||
        get_from_list(self.parameters.as_slice(), next, path)).or_else(||
        get_from_list(self.sub_items.as_slice(), next, path)).or_else(||
        get_from_list(self.messages.as_slice(), next, path))

    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.abilities.as_slice(), name, path_sections).or_else(||
        find_item_in_list(self.parameters.as_slice(), name, path_sections)).or_else(||
        find_item_in_list(self.sub_items.as_slice(), name, path_sections)).or_else(||
        find_item_in_list(self.messages.as_slice(), name, path_sections))
    }
    fn update_path_children(&mut self, _paths: &SPPaths) {
        let mut local = LocalPath::from(vec![self.node.name().to_string()]);
        self.node.paths_mut().upd_local(Some(local));
        let paths = self.node.paths();
        update_path_in_list(self.abilities.as_mut_slice(), &paths);
        update_path_in_list(self.parameters.as_mut_slice(), &paths);
        update_path_in_list(self.sub_items.as_mut_slice(), &paths);
        update_path_in_list(self.messages.as_mut_slice(), &paths);
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Resource(self)
    }
}

impl Resource {
    pub fn new(name: &str) -> Resource {
        let mut node = SPNode::new(name);
        node.update_path(&SPPaths::new(Some(LocalPath::new()), None));
        Resource {
            node,
            ..Resource::default()
        }
    }

    pub fn abilities(&self) -> &[Ability] {
        self.abilities.as_slice()
    }
    pub fn add_ability(&mut self, mut ability: Ability) -> SPPaths {
        ability.transitions.iter_mut().
            for_each(|t|t.
                     add_path_to_temps(TemporaryPathNS::AbilityPredicate,
                                       TemporaryPathNS::Local,
                                       self.name()));

        let paths = ability.update_path(self.node.paths());
        self.abilities.push(ability);
        paths
    }

    pub fn parameters(&self) -> &[Variable] {
        self.parameters.as_slice()
    }
    pub fn add_parameter(&mut self, mut parameter: Variable) -> SPPaths {
        let paths = parameter.update_path(self.node.paths());
        self.parameters.push(parameter);
        paths
    }

    pub fn messages(&self) -> &[Topic] {
        self.messages.as_slice()
    }
    pub fn add_message(&mut self, mut message: Topic) -> SPPaths {
        let paths = message.update_path(self.node.paths());
        self.messages.push(message);
        paths
    }

    pub fn sub_items(&self) -> &[SPItem] {
        self.sub_items.as_slice()
    }
    pub fn add_sub_item(&mut self, mut sub_item: SPItem) -> SPPaths {
        let paths = sub_item.update_path(self.node.paths());
        self.sub_items.push(sub_item);
        paths
    }

     pub fn make_initial_state(&self) -> SPState {
         fn r(m: &MessageField, acum: &mut SPState) {
             match m {
                 MessageField::Msg(msg) => {
                     for f in &msg.fields {
                         r(f, acum);
                     }
                 },
                 MessageField::Var(var) => {
                     let _res = acum.insert(&var.node().global_path().as_ref().unwrap().to_sp(),
                                            AssignStateValue::SPValue(var.initial_value.clone()));
                 },
             }
         }

         let mut s = SPState::default();
         for t in &self.messages {
             r(&t.msg, &mut s);
         }

         for a in &self.abilities {
             for v in &a.predicates {
                 let _res = s.insert(&v.node().global_path().as_ref().unwrap().to_sp(),
                                     AssignStateValue::SPValue(v.initial_value.clone()));
             }
         }

         return s;
    }

    // requires resource to have a global path
    pub fn make_global_transitions(&self) -> Vec<Transition> {
        // local transitions are defined per resource
        let rgp = self.node().global_path().as_ref().unwrap();
        // thus parent of the resource needs to be added to all paths
        let parent = rgp.parent();

        let mut r = Vec::new();

        for a in &self.abilities {
            for t in &a.transitions {
                let updt = t.clone_with_global_paths(&parent);
                r.push(updt);
            }
        }
        return r;
    }

    // requires resource to have a global path
    pub fn make_global_state_predicates(&self) -> Vec<Variable> {
        // local transitions are defined per resource
        let rgp = self.node().global_path().as_ref().unwrap();
        // thus parent of the resource needs to be added to all paths
        let parent = rgp.parent();

        let mut r = Vec::new();

        for a in &self.abilities {
            for p in &a.predicates {
                let updp = p.clone_with_global_paths(&parent);
                r.push(updp);
            }
        }
        return r;
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
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        if self.msg.name() != next {
            return None;
        }
        self.msg.get(path)
    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        self.msg.find_item(name, path_sections)
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        self.msg.update_path(paths);
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Topic(self)
    }
}

impl Topic {
    pub fn new(name: &str, msg: MessageField) -> Topic {
        let node = SPNode::new(name);
        Topic { node, msg }
    }

    pub fn msg(&self) -> &MessageField {
        &self.msg
    }

    pub fn is_subscriber(&self) -> bool {
        fn is_sub(f: &MessageField) -> bool {
            match f {
                MessageField::Msg(msg) => msg.fields.iter().all(|m| is_sub(m)),
                MessageField::Var(v) => v.type_ == VariableType::Measured,
            }
        }
        return is_sub(&self.msg);
    }

    pub fn is_publisher(&self) -> bool {
        fn is_pub(f: &MessageField) -> bool {
            match f {
                MessageField::Msg(msg) => msg.fields.iter().all(|m| is_pub(m)),
                MessageField::Var(v) => v.type_ == VariableType::Command ||
                    v.type_ == VariableType::Command,
            }
        }
        return is_pub(&self.msg);
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Message {
    node: SPNode,
    type_: String,
    fields: Vec<MessageField>, // note, the field name is in each node
}

impl Noder for Message {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        get_from_list(self.fields.as_slice(), next, path)
    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.fields.as_slice(), name, path_sections)
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        update_path_in_list(self.fields.as_mut_slice(), paths);
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Message(self)
    }
}

impl Message {
    pub fn new(name: &str, fields: Vec<MessageField>) -> Message {
        let node = SPNode::new(name);
        Message {
            node,
            type_: String::default(),
            fields,
        }
    }
    pub fn new_with_type(name: &str, type_: &str, fields: Vec<MessageField>) -> Message {
        let node = SPNode::new(name);
        Message {
            node,
            type_: type_.to_string(),
            fields,
        }
    }
    pub fn fields(&self) -> &[MessageField] {
        self.fields.as_slice()
    }
    pub fn msg_type(&self) -> &str {
        &self.type_
    }
    pub fn update_msg_type(&mut self, type_: String) {
        self.type_ = type_;
    }
    pub fn instantiate(&self, new_name: &str) -> Message {
        Message::new_with_type(new_name, &self.type_, self.fields.clone())
    }
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
            MessageField::Var(ref x) => x.node(),
        }
    }
    fn node_mut(&mut self) -> &mut SPNode {
        match self {
            MessageField::Msg(ref mut x) => x.node_mut(),
            MessageField::Var(ref mut x) => x.node_mut(),
        }
    }

    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        match self {
            MessageField::Msg(ref x) => x.get_child(next, path),
            MessageField::Var(ref x) => x.get_child(next, path),
        }
    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        match self {
            MessageField::Msg(ref x) => x.find_item(name, path_sections),
            MessageField::Var(ref x) => x.find_item(name, path_sections),
        }
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        match self {
            MessageField::Msg(ref mut x) => x.update_path_children(paths),
            MessageField::Var(ref mut x) => x.update_path_children(paths),
        }
    }
    fn as_ref(&self) -> SPItemRef<'_> {
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
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_among_childs<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _paths: &SPPaths) {}
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Variable(self)
    }
}

impl Variable {
    pub fn new(
        name: &str,
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
    pub fn new_boolean(name: &str, type_: VariableType) -> Variable {
        Variable::new(
            name,
            type_,
            SPValueType::Bool,
            false.to_spvalue(),
            vec![false.to_spvalue(), true.to_spvalue()],
        )
    }

    pub fn new_predicate(name: &str, p: Predicate) -> Variable {
        Variable::new(
            name,
            VariableType::Predicate(p),
            SPValueType::Bool,
            false.to_spvalue(),
            vec![false.to_spvalue(), true.to_spvalue()],
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

    pub fn clone_with_global_paths(&self, parent: &GlobalPath) -> Variable {
        match self.type_ {
            VariableType::Predicate(ref pred) => {
                let mut new = self.clone();
                new.type_ = VariableType::Predicate(pred.clone_with_global_paths(parent));
                new
            }
            _ => self.clone()
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
    Parameter(Option<SPPath>),
    Predicate(Predicate),
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
    controlled: bool,
}

impl Noder for Transition {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_among_childs<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _paths: &SPPaths) {}
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Transition(self)
    }
}

impl Transition {
    pub fn new(
        name: &str,
        guard: Predicate,
        actions: Vec<Action>,
        effects: Vec<Action>,
        controlled: bool,
    ) -> Transition {
        let node = SPNode::new(name);
        Transition {
            node,
            guard,
            actions,
            effects,
            controlled
        }
    }

    pub fn guard(&self) -> &Predicate {
        &self.guard
    }
    pub fn actions(&self) -> &[Action] {
        self.actions.as_slice()
    }
    pub fn effects(&self) -> &[Action] {
        self.effects.as_slice()
    }
    pub fn controlled(&self) -> bool {
        self.controlled
    }

    pub fn add_path_to_temps(&mut self, namespace_from: TemporaryPathNS,
                             namespace_to: TemporaryPathNS, name: &str) {
        self.guard.change_temps(namespace_from, namespace_to, name);
    }

    pub fn clone_with_global_paths(&self, parent: &GlobalPath) -> Transition {
        let mut clone = self.clone();

        clone.guard = self.guard.clone_with_global_paths(parent);
        clone.actions = self.actions.iter().map(|a|a.clone_with_global_paths(parent)).collect();
        clone.effects = self.effects.iter().map(|e|e.clone_with_global_paths(parent)).collect();


        return clone;
    }
}

impl EvaluatePredicate for Transition {
    fn eval(&self, state: &SPState) -> bool {
        self.guard.eval(state) && self.actions.iter().all(|a| a.eval(state))
    }
}

impl NextAction for Transition {
    fn next(&self, state: &SPState) -> SPResult<AssignState> {
        let mut s: HashMap<SPPath, AssignStateValue> = HashMap::new();
        for a in self.actions.iter() {
            let next = a.next(state)?;
            s.extend(next.s);
        }
        Ok(AssignState { s })
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Ability {
    node: SPNode,
    transitions: Vec<Transition>,
    predicates: Vec<Variable>,
}

impl Noder for Ability {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        get_from_list(self.transitions.as_slice(), next, path).or_else(||
        get_from_list(self.predicates.as_slice(), next, path))
    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.transitions.as_slice(), name, path_sections).or_else(||
        find_item_in_list(self.predicates.as_slice(), name, path_sections))
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        update_path_in_list(self.transitions.as_mut_slice(), paths);
        update_path_in_list(self.predicates.as_mut_slice(), paths);
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Ability(self)
    }
}

impl Ability {
    pub fn new(
        name: &str,
        mut transitions: Vec<Transition>,
        predicates: Vec<Variable>,
    ) -> Ability {
        let node = SPNode::new(name);
        transitions.iter_mut().
            for_each(|t|t.add_path_to_temps(
                TemporaryPathNS::Predicate, TemporaryPathNS::AbilityPredicate, name));
        Ability {
            node,
            transitions,
            predicates,
        }
    }

    pub fn predicates(&self) -> &[Variable] {
        self.predicates.as_slice()
    }

}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Operation {
    node: SPNode,
    start: Vec<Transition>, // now we talk about transitions and not conditions
    finish: Vec<Transition>,
    // skip these for now: finish trans are by default unctrl
    // uncontrolled: Vec<Transition>,
    // predicates: Vec<Variable>,
    pub goal: Option<IfThen>,
    invariant: Option<IfThen>,
}

impl Noder for Operation {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        get_from_list(self.start.as_slice(), next, path).or_else(||
        get_from_list(self.finish.as_slice(), next, path)).or_else(||
        self.goal.as_ref().and_then(|x| x.get(path))).or_else(||
        self.invariant.as_ref().and_then(|ref x| x.get(path)))
    }
    fn find_item_among_childs<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.start.as_slice(), name, path_sections).or_else(||
        find_item_in_list(self.finish.as_slice(), name, path_sections)).or_else(||
        self.goal.as_ref().and_then(|x| x.find_item(name, path_sections))).or_else(||
        self.invariant.as_ref().and_then(|ref x| x.find_item(name, path_sections)))
    }
    fn update_path_children(&mut self, paths: &SPPaths) {
        update_path_in_list(self.start.as_mut_slice(), paths);
        update_path_in_list(self.finish.as_mut_slice(), paths);
        self.goal.as_mut().map(|mut x| x.update_path(paths));
        self.invariant.as_mut().map(|mut x| x.update_path(paths));
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Operation(self)
    }
}

impl Operation {
    pub fn new(
        name: &str,
        start: Vec<Transition>,
        finish: Vec<Transition>,
        goal: Option<IfThen>,
        invariant: Option<IfThen>,
    ) -> Operation {
        let node = SPNode::new(name);
        Operation {
            node,
            start,
            finish,
            goal,
            invariant,
        }
    }

    pub fn goal(&self) -> &Option<IfThen> {
        &self.goal
    }

    pub fn start(&self) -> &Vec<Transition> {
        &self.start
    }

    pub fn finish(&self) -> &Vec<Transition> {
        &self.finish
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
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_among_childs<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _paths: &SPPaths) {}
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::IfThen(self)
    }
}

impl IfThen {
    pub fn new(name: &str, if_: Predicate, then_: Predicate) -> IfThen {
        let node = SPNode::new(name);
        IfThen { node, if_, then_ }
    }
    pub fn if_(&self) -> &Predicate {
        &self.if_
    }
    pub fn then_(&self) -> &Predicate {
        &self.then_
    }
}


/// Specs are used to define global constraints
/// TODO: should we allow ltl expressions?
/// For now its just simple forbidden states
#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Spec {
    node: SPNode,
    always: Vec<Predicate>,
}

impl Noder for Spec {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_among_childs<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _paths: &SPPaths) {}
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Spec(self)
    }
}

impl Spec {
    pub fn new(name: &str, always: Vec<Predicate>) -> Spec {
        let node = SPNode::new(name);
        Spec { node, always }
    }
    pub fn always(&self) -> &[Predicate] {
        self.always.as_slice()
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
        let p = pr! {{p!(!ac)} && {p!(!xy)}};

        let a = a!(ac = false);
        let b = a!(ab <- kl);
        let c = a!(xy ? p);

        let t1 = Transition::new("t1", p!(ac), vec![a], vec![], false);
        let t2 = Transition::new("t2", p!(!ac), vec![b], vec![], false);
        let t3 = Transition::new("t3", Predicate::TRUE, vec![c], vec![], false);

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
