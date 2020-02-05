//!
//!
use super::*;
use std::collections::HashMap;

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum SPItem {
    Model(Model),
    Resource(Resource),
//    Message(Message),
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
            SPItem::Topic(x) => x.get_child(next, path),
            SPItem::Variable(x) => x.get_child(next, path),
            SPItem::Operation(x) => x.get_child(next, path),
            SPItem::Ability(x) => x.get_child(next, path),
            SPItem::Transition(x) => x.get_child(next, path),
            SPItem::IfThen(x) => x.get_child(next, path),
            SPItem::Spec(x) => x.get_child(next, path),
        }
    }
    fn find_item_among_children<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.find_item_among_children(name, path_sections),
            SPItem::Resource(x) => x.find_item_among_children(name, path_sections),
            SPItem::Topic(x) => x.find_item_among_children(name, path_sections),
            SPItem::Variable(x) => x.find_item_among_children(name, path_sections),
            SPItem::Operation(x) => x.find_item_among_children(name, path_sections),
            SPItem::Ability(x) => x.find_item_among_children(name, path_sections),
            SPItem::Transition(x) => x.find_item_among_children(name, path_sections),
            SPItem::IfThen(x) => x.find_item_among_children(name, path_sections),
            SPItem::Spec(x) => x.find_item_among_children(name, path_sections),
        }
    }
    fn update_path_children(&mut self, path:&SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        match self {
            SPItem::Model(x) => x.update_path_children(path, changes),
            SPItem::Resource(x) => x.update_path_children(path, changes),
            SPItem::Topic(x) => x.update_path_children(path, changes),
            SPItem::Variable(x) => x.update_path_children(path, changes),
            SPItem::Operation(x) => x.update_path_children(path, changes),
            SPItem::Ability(x) => x.update_path_children(path, changes),
            SPItem::Transition(x) => x.update_path_children(path, changes),
            SPItem::IfThen(x) => x.update_path_children(path, changes),
            SPItem::Spec(x) => x.update_path_children(path, changes),
        }
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match self {
            SPItem::Model(x) => x.rewrite_expressions(mapping),
            SPItem::Resource(x) => x.rewrite_expressions(mapping),
            SPItem::Topic(x) => x.rewrite_expressions(mapping),
            SPItem::Variable(x) => x.rewrite_expressions(mapping),
            SPItem::Operation(x) => x.rewrite_expressions(mapping),
            SPItem::Ability(x) => x.rewrite_expressions(mapping),
            SPItem::Transition(x) => x.rewrite_expressions(mapping),
            SPItem::IfThen(x) => x.rewrite_expressions(mapping),
            SPItem::Spec(x) => x.rewrite_expressions(mapping),
        }
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        match self {
            SPItem::Model(x) => x.as_ref(),
            SPItem::Resource(x) => x.as_ref(),
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
    pub fn path(&self) -> SPPath {
        self.node().path().clone()
    }

    pub fn node(&self) -> &SPNode {
        match self {
            SPItemRef::Model(x) => &x.node,
            SPItemRef::Resource(x) => &x.node,
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
            SPItemRef::Model(x) => SPItem::Model({ *x }.clone()),
            SPItemRef::Resource(x) => SPItem::Resource({ *x }.clone()),
            SPItemRef::Topic(x) => SPItem::Topic({ *x }.clone()),
            SPItemRef::Variable(x) => SPItem::Variable({ *x }.clone()),
            SPItemRef::Operation(x) => SPItem::Operation({ *x }.clone()),
            SPItemRef::Ability(x) => SPItem::Ability({ *x }.clone()),
            SPItemRef::Transition(x) => SPItem::Transition({ *x }.clone()),
            SPItemRef::IfThen(x) => SPItem::IfThen({ *x }.clone()),
            SPItemRef::Spec(x) => SPItem::Spec({ *x }.clone()),
        }
    }
    pub fn name(&self) -> &str {
        self.node().name()
    }
}

/// A trait for unwrapping SPItemRefs and cloning the items
pub trait SPItemUnwrapper {
    fn item(&self) -> SPItem;
    fn as_variable(&self) -> Option<Variable> {
        if let SPItem::Variable(x) = self.item() {
            Some(x)
        } else {
            None
        }
    }
    fn unwrap_variable(&self) -> Variable {
        self.as_variable().unwrap()
    }
    fn as_resource(&self) -> Option<Resource> {
        if let SPItem::Resource(x) = self.item() {
            Some(x)
        } else {
            None
        }
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
    fn find_item_among_children<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.items.as_slice(), name, path_sections)
    }
    fn update_path_children(&mut self, path:&SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        update_path_in_list(self.items.as_mut_slice(), path, changes);
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.items.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
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
        let mut changes = HashMap::new();
        m.update_path(&SPPath::new(), &mut changes);
        m.rewrite_expressions(&changes); // propagate path changes
        m
    }

    pub fn items(&self) -> &[SPItem] {
        self.items.as_slice()
    }

    pub fn add_item(&mut self, mut item: SPItem) -> SPPath {
        let mut changes = HashMap::new();
        let path = item.update_path(self.node.path(), &mut changes);
        self.items.push(item);
        self.rewrite_expressions(&changes); // propagate path changes
        path
    }
}

#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Resource {
    node: SPNode,
    abilities: Vec<Ability>,
    parameters: Vec<Variable>,
    messages: Vec<Topic>, // Also include estimated here on an estimated topic
    sub_items: Vec<SPItem>,
}

impl Noder for Resource {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        get_from_list(self.abilities.as_slice(), next, path)
            .or_else(|| get_from_list(self.parameters.as_slice(), next, path))
            .or_else(|| get_from_list(self.sub_items.as_slice(), next, path))
            .or_else(|| get_from_list(self.messages.as_slice(), next, path))
    }
    fn find_item_among_children<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.abilities.as_slice(), name, path_sections)
            .or_else(|| find_item_in_list(self.parameters.as_slice(), name, path_sections))
            .or_else(|| find_item_in_list(self.sub_items.as_slice(), name, path_sections))
            .or_else(|| find_item_in_list(self.messages.as_slice(), name, path_sections))
    }
    fn update_path_children(&mut self, path:&SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        // let mut local = SPPath::from(vec![self.node.name().to_string()]);
        // let path = self.node.path();
        update_path_in_list(self.abilities.as_mut_slice(), &path, changes);
        update_path_in_list(self.parameters.as_mut_slice(), &path, changes);
        update_path_in_list(self.sub_items.as_mut_slice(), &path, changes);
        update_path_in_list(self.messages.as_mut_slice(), &path, changes);
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.abilities.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
        self.parameters.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
        self.sub_items.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
        self.messages.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Resource(self)
    }
}

impl Resource {
    pub fn new(name: &str) -> Resource {
        let mut node = SPNode::new(name);
        node.update_path(&SPPath::new());
        Resource {
            node,
            ..Resource::default()
        }
    }

    pub fn abilities(&self) -> &[Ability] {
        self.abilities.as_slice()
    }
    pub fn add_ability(&mut self, mut ability: Ability) -> SPPath {
        let mut changes = HashMap::new();
        let ab_path = ability.update_path(self.node.path(), &mut changes);
        self.abilities.push(ability);
        ab_path
    }

    pub fn parameters(&self) -> &[Variable] {
        self.parameters.as_slice()
    }
    pub fn add_parameter(&mut self, mut parameter: Variable) -> SPPath {
        let mut changes = HashMap::new();
        let path = parameter.update_path(self.node.path(), &mut changes);
        self.parameters.push(parameter);
        path
    }

    pub fn messages(&self) -> &[Topic] {
        self.messages.as_slice()
    }
    pub fn add_message(&mut self, mut message: Topic) -> SPPath {
        let mut changes = HashMap::new();
        let path = message.update_path(self.node.path(), &mut changes);
        self.messages.push(message);
        path
    }

    pub fn sub_items(&self) -> &[SPItem] {
        self.sub_items.as_slice()
    }
    pub fn add_sub_item(&mut self, mut sub_item: SPItem) -> SPPath {
        let mut changes = HashMap::new();
        let path = sub_item.update_path(self.node.path(), &mut changes);
        self.sub_items.push(sub_item);
        path
    }

    pub fn make_initial_state(&self) -> SPState {
        fn r(m: &MessageField, acum: &mut SPState) {
            match m {
                MessageField::Msg(msg) => {
                    for (n, f) in &msg.fields {
                        r(f, acum);
                    }
                }
                MessageField::Var(var) => {
                    let _res = acum.add_variable(
                        var.path().clone(),
                        var.initial_value.clone(),
                    );
                }
            }
        }

        let mut s = SPState::default();
        for t in &self.messages {
            r(&t.msg, &mut s);
        }

        for a in &self.abilities {
            for v in &a.predicates {
                let _res = s.next_from_path(
                    &v.path(),
                    v.initial_value.clone(),
                );
            }
        }

        println!("runner intial state: {:?}", s);

        return s;
    }
    pub fn get_variables(&self) -> Vec<Variable> {
        fn r(m: &MessageField, acum: &mut Vec<Variable>) {
            match m {
                MessageField::Msg(msg) => {
                    for (_n, f) in &msg.fields {
                        r(f, acum);
                    }
                },
                MessageField::Var(var) => {
                    let _res = acum.push(var.clone());
                },
            }
        }

        let mut vs = Vec::new();
        for t in &self.messages {
            r(&t.msg, &mut vs);
        }

        return vs;
    }

    // TODO
    pub fn make_global_transitions(&self) -> Vec<Transition> {
        // local transitions are defined per resource
        let rgp = self.node().path().clone();
        // thus parent of the resource needs to be added to all paths
        // let parent = rgp.parent();

        let mut r = Vec::new();

        for a in &self.abilities {
            for t in &a.transitions {
                // let mut updt = t.clone();
                // updt.add_parent_path(&parent);
                r.push(t.clone());
            }
        }
        return r;
    }

    // TODO
    pub fn make_global_state_predicates(&self) -> Vec<Variable> {
        // local transitions are defined per resource
        let rgp = self.node().path().clone();
        // thus parent of the resource needs to be added to all paths
        // let parent = rgp.parent();

        let mut r = Vec::new();

        for a in &self.abilities {
            for p in &a.predicates {
                // let updp = p.clone_with_global_paths(&parent);
                r.push(p.clone());
            }
        }
        return r;
    }

}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
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
        // I dont understand what next and path are, also, does not seem to be used for anything?
        let mut stack = vec![&self.msg];
        loop {
            match stack.pop() {
                Some(MessageField::Msg(msg)) => {
                    stack.extend(msg.fields.iter().map(|(_n, m)|m).collect::<Vec<_>>()); },
                Some(MessageField::Var(v)) => {
                    if v.name() == next && v.path() == path {
                        return Some(SPItemRef::Variable(v))
                    }
                    // if let Some(item) = v.get_child(next, path) {
                    //     return Some(item)
                    // }
                },
                None => { return None; } // all done
            }
        }
        // panic!("todo next {} path {}", next, path);
        // None
    }
    fn find_item_among_children<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        let mut stack = vec![&self.msg];
        loop {
            match stack.pop() {
                Some(MessageField::Msg(msg)) => {
                    stack.extend(msg.fields.iter().map(|(_n, m)|m).collect::<Vec<_>>()); },
                Some(MessageField::Var(v)) => {
                    if let Some(item) = v.find_item(name, path_sections) {
                        return Some(item)
                    }
                },
                None => { return None; } // all done
            }
        }
    }
    fn update_path_children(&mut self, path:&SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        let mut stack = vec![&mut self.msg];
        loop {
            match stack.pop() {
                Some(MessageField::Msg(msg)) => { stack.extend(msg.fields.iter_mut().map(|(_n, m)|m).collect::<Vec<_>>()); },
                Some(MessageField::Var(v)) => { v.update_path(path, changes); },
                None => { return; } // all done
            }
        }
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        // TODO: variables with expressions in the messages.
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
                MessageField::Msg(msg) => msg.fields.iter().all(|(_n,m)| is_sub(m)),
                MessageField::Var(v) => v.type_ == VariableType::Measured,
            }
        }
        return is_sub(&self.msg);
    }

    pub fn is_publisher(&self) -> bool {
        fn is_pub(f: &MessageField) -> bool {
            match f {
                MessageField::Msg(msg) => msg.fields.iter().all(|(_n,m)| is_pub(m)),
                MessageField::Var(v) => v.type_ == VariableType::Command,
            }
        }
        return is_pub(&self.msg);
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Message {
    pub type_: String, // ros type
    pub fields: HashMap<String, MessageField>,
}

impl Message {

    pub fn new(type_: &str, fields: HashMap<String, MessageField>) -> Message {
        Message {
            type_: type_.to_string(),
            fields,
        }
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum MessageField {
    Msg(Message),
    Var(Variable),
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
    fn find_item_among_children<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path:&SPPath, _changes: &mut HashMap<SPPath, SPPath>) {}
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match &mut self.type_ {
            VariableType::Predicate(p) => p.replace_variable_path(mapping),
            _ => (),
        };
    }
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
    pub guard: Predicate,
    pub actions: Vec<Action>,
    pub effects: Vec<Action>,
    pub controlled: bool,
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
    fn find_item_among_children<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path:&SPPath, _changes: &mut HashMap<SPPath, SPPath>) {
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.guard.replace_variable_path(mapping);
        self.actions.iter_mut().for_each(|i| i.replace_variable_path(mapping));
        self.effects.iter_mut().for_each(|i| i.replace_variable_path(mapping));
    }
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
            controlled,
        }
    }

    // hack
    pub fn mut_guard(&mut self) -> &mut Predicate {
        &mut self.guard
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
    pub fn upd_state_path(&mut self, state: &SPState) {
        self.guard.upd_state_path(state);
        self.actions.iter_mut().for_each(|a| a.upd_state_path(state));
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
        get_from_list(self.transitions.as_slice(), next, path)
            .or_else(|| get_from_list(self.predicates.as_slice(), next, path))
    }
    fn find_item_among_children<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.transitions.as_slice(), name, path_sections)
            .or_else(|| find_item_in_list(self.predicates.as_slice(), name, path_sections))
    }
    fn update_path_children(&mut self, path:&SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        update_path_in_list(self.transitions.as_mut_slice(), path, changes);
        update_path_in_list(self.predicates.as_mut_slice(), path, changes);
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        // TODO
        self.transitions.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
        self.predicates.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Ability(self)
    }
}

impl Ability {
    pub fn new(name: &str, mut transitions: Vec<Transition>, predicates: Vec<Variable>) -> Ability {
        let node = SPNode::new(name);
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
}

impl Noder for Operation {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        get_from_list(self.start.as_slice(), next, path)
            .or_else(|| get_from_list(self.finish.as_slice(), next, path))
            .or_else(|| self.goal.as_ref().and_then(|x| x.get(path)))
    }
    fn find_item_among_children<'a>(
        &'a self,
        name: &str,
        path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.start.as_slice(), name, path_sections)
            .or_else(|| find_item_in_list(self.finish.as_slice(), name, path_sections))
            .or_else(|| {
                self.goal
                    .as_ref()
                    .and_then(|x| x.find_item(name, path_sections))
            })
    }
    fn update_path_children(&mut self, path:&SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        update_path_in_list(self.start.as_mut_slice(), path, changes);
        update_path_in_list(self.finish.as_mut_slice(), path, changes);
        self.goal.as_mut().map(|mut x| x.update_path(path, changes));
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.start.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
        self.finish.iter_mut().for_each(|i| i.rewrite_expressions(mapping));
        if let Some(goal) = &mut self.goal { goal.rewrite_expressions(mapping) };
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
    ) -> Operation {
        let node = SPNode::new(name);
        Operation {
            node,
            start,
            finish,
            goal,
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
/// MD 2020-01-30: I have changed this to include both a goal and an optional invariant.
/// The reasoning is that I think active invariants will always be connected to some
/// running operation and from a planning perspective they must be connected in order to be
/// "deactivated" once the goal of that operation is reached.
#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct IfThen {
    node: SPNode,
    pub condition: Predicate,
    pub goal: Predicate,
    pub invariant: Option<Predicate>,
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
    fn find_item_among_children<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path:&SPPath, _changes: &mut HashMap<SPPath, SPPath>) {}
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.condition.replace_variable_path(mapping);
        self.goal.replace_variable_path(mapping);
        if let Some(invariant) = &mut self.invariant {
            invariant.replace_variable_path(mapping);
        }
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::IfThen(self)
    }
}

impl IfThen {
    pub fn new(name: &str, condition: Predicate, goal: Predicate, invariant: Option<Predicate>) -> IfThen {
        let node = SPNode::new(name);
        IfThen { node, condition, goal, invariant }
    }
    pub fn condition(&self) -> &Predicate {
        &self.condition
    }
    pub fn goal(&self) -> &Predicate {
        &self.goal
    }

    pub fn invariant(&self) -> &Option<Predicate> {
        &self.invariant
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
    fn find_item_among_children<'a>(
        &'a self,
        _name: &str,
        _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path:&SPPath, _changes: &mut HashMap<SPPath, SPPath>) {}
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.always.iter_mut().for_each(|i| i.replace_variable_path(mapping));
    }
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
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let xy = SPPath::from_slice(&["x", "y"]);

        let mut s = state!(ab => 2, ac => true, kl => 3, xy => false);
        let p = p!([!p:ac] && [!p:xy]);

        let a = a!(p:ac = false);
        let b = a!(p:ab <- p:kl);
        let c = a!(p:xy ? p);

        let mut t1 = Transition::new("t1", p!(p:ac), vec![a], vec![], false);
        let mut t2 = Transition::new("t2", p!(!p:ac), vec![b], vec![], false);
        let mut t3 = Transition::new("t3", Predicate::TRUE, vec![c], vec![], false);

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
        t3.next(&mut s);

        s.take_transition();
        assert_eq!(s.sp_value_from_path(&xy).unwrap(), &true.to_spvalue());
    }

}
