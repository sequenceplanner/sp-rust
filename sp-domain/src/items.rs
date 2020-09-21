//!
//!
use super::*;
use std::collections::{HashMap,HashSet};

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum SPItem {
    Model(Model),
    Resource(Resource),
    Message(Message),
    Topic(Topic),
    Variable(Variable),
    Intention(Intention),
    Operation(Operation),
    Transition(Transition),
    IfThen(IfThen),
    TransitionSpec(TransitionSpec),
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
            SPItem::Intention(x) => x.node(),
            SPItem::Operation(x) => x.node(),
            SPItem::Transition(x) => x.node(),
            SPItem::IfThen(x) => x.node(),
            SPItem::Spec(x) => x.node(),
            SPItem::TransitionSpec(x) => x.node(),
        }
    }
    fn node_mut(&mut self) -> &mut SPNode {
        match self {
            SPItem::Model(ref mut x) => x.node_mut(),
            SPItem::Resource(ref mut x) => x.node_mut(),
            SPItem::Message(ref mut x) => x.node_mut(),
            SPItem::Topic(ref mut x) => x.node_mut(),
            SPItem::Variable(ref mut x) => x.node_mut(),
            SPItem::Intention(ref mut x) => x.node_mut(),
            SPItem::Operation(ref mut x) => x.node_mut(),
            SPItem::Transition(ref mut x) => x.node_mut(),
            SPItem::IfThen(ref mut x) => x.node_mut(),
            SPItem::Spec(ref mut x) => x.node_mut(),
            SPItem::TransitionSpec(ref mut x) => x.node_mut(),
        }
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.get_child(next, path),
            SPItem::Resource(x) => x.get_child(next, path),
            SPItem::Message(x) => x.get_child(next, path),
            SPItem::Topic(x) => x.get_child(next, path),
            SPItem::Variable(x) => x.get_child(next, path),
            SPItem::Intention(x) => x.get_child(next, path),
            SPItem::Operation(x) => x.get_child(next, path),
            SPItem::Transition(x) => x.get_child(next, path),
            SPItem::IfThen(x) => x.get_child(next, path),
            SPItem::Spec(x) => x.get_child(next, path),
            SPItem::TransitionSpec(x) => x.get_child(next, path),
        }
    }
    fn get_child_mut<'a>(&'a mut self, next: &str, path: &SPPath) -> Option<SPMutItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.get_child_mut(next, path),
            SPItem::Resource(x) => x.get_child_mut(next, path),
            SPItem::Message(x) => x.get_child_mut(next, path),
            SPItem::Topic(x) => x.get_child_mut(next, path),
            SPItem::Variable(x) => x.get_child_mut(next, path),
            SPItem::Intention(x) => x.get_child_mut(next, path),
            SPItem::Operation(x) => x.get_child_mut(next, path),
            SPItem::Transition(x) => x.get_child_mut(next, path),
            SPItem::IfThen(x) => x.get_child_mut(next, path),
            SPItem::Spec(x) => x.get_child_mut(next, path),
            SPItem::TransitionSpec(x) => x.get_child_mut(next, path),
        }
    }
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.find_item_among_children(name, path_sections),
            SPItem::Resource(x) => x.find_item_among_children(name, path_sections),
            SPItem::Message(x) => x.find_item_among_children(name, path_sections),
            SPItem::Topic(x) => x.find_item_among_children(name, path_sections),
            SPItem::Variable(x) => x.find_item_among_children(name, path_sections),
            SPItem::Intention(x) => x.find_item_among_children(name, path_sections),
            SPItem::Operation(x) => x.find_item_among_children(name, path_sections),
            SPItem::Transition(x) => x.find_item_among_children(name, path_sections),
            SPItem::IfThen(x) => x.find_item_among_children(name, path_sections),
            SPItem::Spec(x) => x.find_item_among_children(name, path_sections),
            SPItem::TransitionSpec(x) => x.find_item_among_children(name, path_sections),
        }
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        match self {
            SPItem::Model(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Resource(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Message(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Topic(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Variable(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Intention(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Operation(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Transition(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::IfThen(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::Spec(x) => x.find_item_mut_among_children(name, path_sections),
            SPItem::TransitionSpec(x) => x.find_item_mut_among_children(name, path_sections),
        }
    }
    fn update_path_children(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        match self {
            SPItem::Model(x) => x.update_path_children(path, changes),
            SPItem::Resource(x) => x.update_path_children(path, changes),
            SPItem::Message(x) => x.update_path_children(path, changes),
            SPItem::Topic(x) => x.update_path_children(path, changes),
            SPItem::Variable(x) => x.update_path_children(path, changes),
            SPItem::Intention(x) => x.update_path_children(path, changes),
            SPItem::Operation(x) => x.update_path_children(path, changes),
            SPItem::Transition(x) => x.update_path_children(path, changes),
            SPItem::IfThen(x) => x.update_path_children(path, changes),
            SPItem::Spec(x) => x.update_path_children(path, changes),
            SPItem::TransitionSpec(x) => x.update_path_children(path, changes),
        }
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match self {
            SPItem::Model(x) => x.rewrite_expressions(mapping),
            SPItem::Resource(x) => x.rewrite_expressions(mapping),
            SPItem::Message(x) => x.rewrite_expressions(mapping),
            SPItem::Topic(x) => x.rewrite_expressions(mapping),
            SPItem::Variable(x) => x.rewrite_expressions(mapping),
            SPItem::Intention(x) => x.rewrite_expressions(mapping),
            SPItem::Operation(x) => x.rewrite_expressions(mapping),
            SPItem::Transition(x) => x.rewrite_expressions(mapping),
            SPItem::IfThen(x) => x.rewrite_expressions(mapping),
            SPItem::Spec(x) => x.rewrite_expressions(mapping),
            SPItem::TransitionSpec(x) => x.rewrite_expressions(mapping),
        }
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        match self {
            SPItem::Model(x) => x.as_ref(),
            SPItem::Resource(x) => x.as_ref(),
            SPItem::Message(x) => x.as_ref(),
            SPItem::Topic(x) => x.as_ref(),
            SPItem::Variable(x) => x.as_ref(),
            SPItem::Intention(x) => x.as_ref(),
            SPItem::Operation(x) => x.as_ref(),
            SPItem::Transition(x) => x.as_ref(),
            SPItem::IfThen(x) => x.as_ref(),
            SPItem::Spec(x) => x.as_ref(),
            SPItem::TransitionSpec(x) => x.as_ref(),
        }
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef {
        match self {
            SPItem::Model(x) => x.as_mut_ref(),
            SPItem::Resource(x) => x.as_mut_ref(),
            SPItem::Message(x) => x.as_mut_ref(),
            SPItem::Topic(x) => x.as_mut_ref(),
            SPItem::Variable(x) => x.as_mut_ref(),
            SPItem::Intention(x) => x.as_mut_ref(),
            SPItem::Operation(x) => x.as_mut_ref(),
            SPItem::Transition(x) => x.as_mut_ref(),
            SPItem::IfThen(x) => x.as_mut_ref(),
            SPItem::Spec(x) => x.as_mut_ref(),
            SPItem::TransitionSpec(x) => x.as_mut_ref(),
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
    Intention(&'a Intention),
    Operation(&'a Operation),
    Transition(&'a Transition),
    IfThen(&'a IfThen),
    TransitionSpec(&'a TransitionSpec),
    Spec(&'a Spec),
    //SOP(SOP),
}

#[derive(Debug, PartialEq)]
pub enum SPMutItemRef<'a> {
    Model(&'a mut Model),
    Resource(&'a mut Resource),
    Message(&'a mut Message),
    Topic(&'a mut Topic),
    Variable(&'a mut Variable),
    Intention(&'a mut Intention),
    Operation(&'a mut Operation),
    Transition(&'a mut Transition),
    IfThen(&'a mut IfThen),
    TransitionSpec(&'a mut TransitionSpec),
    Spec(&'a mut Spec),
    //SOP(SOP),
}

impl<'a> SPItemRef<'a> {
    pub fn from_mut(mref: SPMutItemRef<'a>) -> Self {
        match mref {
            SPMutItemRef::Model(x) => SPItemRef::Model(x),
            SPMutItemRef::Resource(x) => SPItemRef::Resource(x),
            SPMutItemRef::Message(x) => SPItemRef::Message(x),
            SPMutItemRef::Topic(x) => SPItemRef::Topic(x),
            SPMutItemRef::Variable(x) => SPItemRef::Variable(x),
            SPMutItemRef::Intention(x) => SPItemRef::Intention(x),
            SPMutItemRef::Operation(x) => SPItemRef::Operation(x),
            SPMutItemRef::Transition(x) => SPItemRef::Transition(x),
            SPMutItemRef::IfThen(x) => SPItemRef::IfThen(x),
            SPMutItemRef::Spec(x) => SPItemRef::Spec(x),
            SPMutItemRef::TransitionSpec(x) => SPItemRef::TransitionSpec(x),
        }
    }

    pub fn path(&self) -> SPPath {
        self.node().path().clone()
    }

    pub fn path_ref(&self) -> &SPPath {
        self.node().path()
    }

    pub fn node(&self) -> &SPNode {
        match self {
            SPItemRef::Model(x) => &x.node,
            SPItemRef::Resource(x) => &x.node,
            SPItemRef::Message(x) => &x.node,
            SPItemRef::Topic(x) => &x.node,
            SPItemRef::Variable(x) => &x.node,
            SPItemRef::Intention(x) => &x.node,
            SPItemRef::Operation(x) => &x.node,
            SPItemRef::Transition(x) => &x.node,
            SPItemRef::IfThen(x) => &x.node,
            SPItemRef::Spec(x) => &x.node,
            SPItemRef::TransitionSpec(x) => &x.node,
        }
    }
    pub fn item(&self) -> SPItem {
        match self {
            SPItemRef::Model(x) => SPItem::Model({ *x }.clone()),
            SPItemRef::Resource(x) => SPItem::Resource({ *x }.clone()),
            SPItemRef::Message(x) => SPItem::Message({ *x }.clone()),
            SPItemRef::Topic(x) => SPItem::Topic({ *x }.clone()),
            SPItemRef::Variable(x) => SPItem::Variable({ *x }.clone()),
            SPItemRef::Intention(x) => SPItem::Intention({ *x }.clone()),
            SPItemRef::Operation(x) => SPItem::Operation({ *x }.clone()),
            SPItemRef::Transition(x) => SPItem::Transition({ *x }.clone()),
            SPItemRef::IfThen(x) => SPItem::IfThen({ *x }.clone()),
            SPItemRef::Spec(x) => SPItem::Spec({ *x }.clone()),
            SPItemRef::TransitionSpec(x) => SPItem::TransitionSpec({ *x }.clone()),
        }
    }
    pub fn name(&self) -> &str {
        self.node().name()
    }
}

impl<'a> SPMutItemRef<'a> {
    pub fn path(&self) -> SPPath {
        self.node().path().clone()
    }

    pub fn node(&self) -> &SPNode {
        match self {
            SPMutItemRef::Model(x) => &x.node,
            SPMutItemRef::Resource(x) => &x.node,
            SPMutItemRef::Message(x) => &x.node,
            SPMutItemRef::Topic(x) => &x.node,
            SPMutItemRef::Variable(x) => &x.node,
            SPMutItemRef::Intention(x) => &x.node,
            SPMutItemRef::Operation(x) => &x.node,
            SPMutItemRef::Transition(x) => &x.node,
            SPMutItemRef::IfThen(x) => &x.node,
            SPMutItemRef::Spec(x) => &x.node,
            SPMutItemRef::TransitionSpec(x) => &x.node,
        }
    }
    pub fn as_variable(&'a mut self) -> &'a mut Variable {
        match self {
            SPMutItemRef::Variable(x) => x,
            _ => panic!(format!("trying to unwrap variable but we have {:?}", self)),
        }
    }
    pub fn name(&self) -> &str {
        self.node().name()
    }
    pub fn item_type_as_string(&self) -> String {
        match self {
            SPMutItemRef::Model(_) => "model".to_string(),
            SPMutItemRef::Resource(_) => "resource".to_string(),
            SPMutItemRef::Message(_) => "message".to_string(),
            SPMutItemRef::Topic(_) => "topic".to_string(),
            SPMutItemRef::Variable(_) => "variable".to_string(),
            SPMutItemRef::Intention(_) => "intention".to_string(),
            SPMutItemRef::Operation(_) => "operation".to_string(),
            SPMutItemRef::Transition(_) => "transition".to_string(),
            SPMutItemRef::IfThen(_) => "ifthen".to_string(),
            SPMutItemRef::Spec(_) => "spec".to_string(),
            SPMutItemRef::TransitionSpec(_) => "transitionspec".to_string(),
        }
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
    fn as_model(&self) -> Option<Model> {
        if let SPItem::Model(x) = self.item() {
            Some(x)
        } else {
            None
        }
    }
    fn unwrap_model(&self) -> Model {
        self.as_model().unwrap()
    }
    // Add items here when needed
}

impl SPItemUnwrapper for SPItemRef<'_> {
    fn item(&self) -> SPItem {
        self.item()
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
    pub items: Vec<SPItem>,
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
    fn get_child_mut<'a>(&'a mut self, next: &str, path: &SPPath) -> Option<SPMutItemRef<'a>> {
        get_from_list_mut(self.items.as_mut_slice(), next, path)
    }
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.items.as_slice(), name, path_sections)
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        find_item_mut_in_list(self.items.as_mut_slice(), name, path_sections)
    }
    fn update_path_children(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        update_path_in_list(self.items.as_mut_slice(), path, changes);
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.items
            .iter_mut()
            .for_each(|i| i.rewrite_expressions(mapping));
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Model(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Model(self)
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

    pub fn new_no_root(name: &str, items: Vec<SPItem>) -> Model {
        // hack to not update the item names for the model...
        Model::new(name, items)
    }

    pub fn items(&self) -> &[SPItem] {
        self.items.as_slice()
    }

    pub fn resources(&self) -> Vec<&Resource> {
        self.items
            .iter()
            .flat_map(|i| match i {
                SPItem::Resource(r) => Some(r),
                _ => None,
            })
            .collect()
    }

    // all resources including ones nested in layers of models
    // we should probably change all code to use this one in time...
    pub fn all_resources(&self) -> Vec<&Resource> {
        self.items
            .iter()
            .flat_map(|i| match i {
                SPItem::Model(m) => m.all_resources(),
                SPItem::Resource(r) => vec![r],
                _ => vec![],
            })
            .collect()
    }


    pub fn all_operations(&self) -> Vec<&Operation> {
        self.items
            .iter()
            .flat_map(|i| match i {
                SPItem::Model(m) => if m.name() != "runner_ops" { m.all_operations() } else { vec![] },
                SPItem::Operation(o) => vec![o],
                _ => vec![],
            })
            .collect()
    }

    pub fn all_runner_operations(&self) -> Vec<&Operation> {
        self.items
            .iter()
            .flat_map(|i| match i {
                SPItem::Model(m) => if m.name() == "runner_ops" { m.all_operations() } else { vec![] },
                SPItem::Operation(o) => vec![o],
                _ => vec![],
            })
            .collect()
    }

    pub fn all_intentions(&self) -> Vec<&Intention> {
        self.items
            .iter()
            .flat_map(|i| match i {
                SPItem::Model(m) => m.all_intentions(),
                SPItem::Intention(int) => vec![int],
                _ => vec![],
            })
            .collect()
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
    pub transitions: Vec<Transition>,
    pub predicates: Vec<Variable>,
    pub estimated: Vec<Variable>,
    pub messages: Vec<Topic>,
    pub specs: Vec<Spec>,
}

impl Noder for Resource {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, next: &str, path: &SPPath) -> Option<SPItemRef<'a>> {
        if let Some(x) = get_from_list(self.transitions.as_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list(self.predicates.as_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list(self.estimated.as_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list(self.messages.as_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list(self.specs.as_slice(), next, path) {
            return Some(x);
        }
        return None;
    }
    fn get_child_mut<'a>(&'a mut self, next: &str, path: &SPPath) -> Option<SPMutItemRef<'a>> {
        if let Some(x) = get_from_list_mut(self.transitions.as_mut_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list_mut(self.predicates.as_mut_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list_mut(self.estimated.as_mut_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list_mut(self.messages.as_mut_slice(), next, path) {
            return Some(x);
        }
        if let Some(x) = get_from_list_mut(self.specs.as_mut_slice(), next, path) {
            return Some(x);
        }
        return None;
    }
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.transitions.as_slice(), name, path_sections)
            .or_else(|| find_item_in_list(self.predicates.as_slice(), name, path_sections))
            .or_else(|| find_item_in_list(self.estimated.as_slice(), name, path_sections))
            .or_else(|| find_item_in_list(self.messages.as_slice(), name, path_sections))
            .or_else(|| find_item_in_list(self.specs.as_slice(), name, path_sections))
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        if let Some(x) = find_item_mut_in_list(self.transitions.as_mut_slice(), name, path_sections) {
            Some(x)
        } else if let Some(x) =
            find_item_mut_in_list(self.predicates.as_mut_slice(), name, path_sections)
        {
            Some(x)
        } else if let Some(x) =
            find_item_mut_in_list(self.estimated.as_mut_slice(), name, path_sections)
        {
            Some(x)
        } else if let Some(x) =
            find_item_mut_in_list(self.messages.as_mut_slice(), name, path_sections)
        {
            Some(x)
        } else {
            find_item_mut_in_list(self.specs.as_mut_slice(), name, path_sections)
        }
    }

    fn update_path_children(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        update_path_in_list(self.estimated.as_mut_slice(), &path, changes);
        update_path_in_list(self.messages.as_mut_slice(), &path, changes);
        update_path_in_list(self.transitions.as_mut_slice(), path, changes);
        update_path_in_list(self.predicates.as_mut_slice(), path, changes);
        update_path_in_list(self.specs.as_mut_slice(), path, changes);
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.messages
            .iter_mut()
            .for_each(|i| i.rewrite_expressions(mapping));
        self.transitions
            .iter_mut()
            .for_each(|i| i.rewrite_expressions(mapping));
        self.predicates
            .iter_mut()
            .for_each(|i| i.rewrite_expressions(mapping));
        self.estimated
            .iter_mut()
            .for_each(|i| i.rewrite_expressions(mapping));
        self.specs
            .iter_mut()
            .for_each(|i| i.rewrite_expressions(mapping));
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Resource(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Resource(self)
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

    pub fn messages(&self) -> &[Topic] {
        self.messages.as_slice()
    }

    pub fn add_message(&mut self, mut message: Topic) -> SPPath {
        let mut changes = HashMap::new();
        let path = message.update_path(self.node.path(), &mut changes);
        self.messages.push(message);
        path
    }

    pub fn add_spec(&mut self, mut spec: Spec) -> SPPath {
        let mut changes = HashMap::new();
        let path = spec.update_path(self.node.path(), &mut changes);
        self.specs.push(spec);
        path
    }

    pub fn add_estimated(&mut self, mut estimated: Variable) -> SPPath {
        let mut changes = HashMap::new();
        let path = estimated.update_path(self.node.path(), &mut changes);
        self.estimated.push(estimated);
        path
    }

    pub fn add_transition(&mut self, mut transition: Transition) -> SPPath {
        let mut changes = HashMap::new();
        let path = transition.update_path(self.node.path(), &mut changes);
        self.transitions.push(transition);
        path
    }

    pub fn add_predicate(&mut self, mut predicate: Variable) -> SPPath {
        let mut changes = HashMap::new();
        let path = predicate.update_path(self.node.path(), &mut changes);
        self.predicates.push(predicate);
        path
    }

    pub fn get_variables(&self) -> Vec<Variable> {
        fn r(
            m: &MessageField, acum: &mut Vec<Variable>) {
            match m {
                MessageField::Msg(msg) => {
                    for f in &msg.fields {
                        r(f, acum);
                    }
                }
                MessageField::Var(var) => {
                    acum.push(var.clone());
                }
            }
        }

        let mut vs = Vec::new();
        for t in &self.messages {
            r(&t.msg, &mut vs);
        }

        vs.extend(self.estimated.iter().cloned());
        vs
    }

    pub fn get_transitions(&self) -> Vec<Transition> {
        self.transitions.clone()
    }

    pub fn get_state_predicates(&self) -> Vec<Variable> {
        self.predicates.clone()
    }

    /// Get the message in the command topic. For now this is hardcoded
    pub fn get_message(&self, topic: &str) -> Option<MessageField> {
        let mut res = None;
        self.messages.iter().for_each(|t| {
            let t: &Topic = t;
            if t.node.name() == topic {
                res = Some(t.msg().clone());
            }
        });
        res
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
        if self.msg.name() != next {
            return None;
        }
        self.msg.get(path)
    }
    fn get_child_mut<'a>(&'a mut self, next: &str, path: &SPPath) -> Option<SPMutItemRef<'a>> {
        if self.msg.name() != next {
            return None;
        }
        self.msg.get_mut(path)
    }
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        self.msg.find_item(name, path_sections)
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        self.msg.find_item_mut(name, path_sections)
    }
    fn update_path_children(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        self.msg.update_path(path, changes);
    }
    fn rewrite_expressions(&mut self, _mapping: &HashMap<SPPath, SPPath>) {
        // TODO: variables with expressions in the messages.
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Topic(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Topic(self)
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
        is_sub(&self.msg)
    }

    pub fn is_publisher(&self) -> bool {
        fn is_pub(f: &MessageField) -> bool {
            match f {
                MessageField::Msg(msg) => msg.fields.iter().all(|m| is_pub(m)),
                MessageField::Var(v) => v.type_ == VariableType::Command,
            }
        }
        is_pub(&self.msg)
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Message {
    node: SPNode,
    pub type_: String, // ros type
    pub fields: Vec<MessageField>,
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
    fn get_child_mut<'a>(&'a mut self, next: &str, path: &SPPath) -> Option<SPMutItemRef<'a>> {
        get_from_list_mut(self.fields.as_mut_slice(), next, path)
    }
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        find_item_in_list(self.fields.as_slice(), name, path_sections)
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        find_item_mut_in_list(self.fields.as_mut_slice(), name, path_sections)
    }
    fn update_path_children(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        update_path_in_list(self.fields.as_mut_slice(), path, changes);
    }
    fn rewrite_expressions(&mut self, _mapping: &HashMap<SPPath, SPPath>) {
        // TODO: variables with expressions in the messages.
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Message(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Message(self)
    }
}

impl Message {
    pub fn new(name: &str, type_: &str, fields: &[MessageField]) -> Message {
        let node = SPNode::new(name);
        Message {
            node,
            type_: type_.to_string(),
            fields: fields.to_vec(),
        }
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
    fn get_child_mut<'a>(&'a mut self, next: &str, path: &SPPath) -> Option<SPMutItemRef<'a>> {
        match self {
            MessageField::Msg(ref mut x) => x.get_child_mut(next, path),
            MessageField::Var(ref mut x) => x.get_child_mut(next, path),
        }
    }
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        match self {
            MessageField::Msg(ref x) => x.find_item(name, path_sections),
            MessageField::Var(ref x) => x.find_item(name, path_sections),
        }
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        match self {
            MessageField::Msg(ref mut x) => x.find_item_mut(name, path_sections),
            MessageField::Var(ref mut x) => x.find_item_mut(name, path_sections),
        }
    }
    fn update_path_children(&mut self, path: &SPPath, changes: &mut HashMap<SPPath, SPPath>) {
        match self {
            MessageField::Msg(ref mut x) => x.update_path_children(path, changes),
            MessageField::Var(ref mut x) => x.update_path_children(path, changes),
        }
    }
    fn rewrite_expressions(&mut self, _mapping: &HashMap<SPPath, SPPath>) {
        // TODO: variables with expressions in the messages.
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        match self {
            MessageField::Msg(ref x) => x.as_ref(),
            MessageField::Var(ref x) => x.as_ref(),
        }
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        match self {
            MessageField::Msg(ref mut x) => x.as_mut_ref(),
            MessageField::Var(ref mut x) => x.as_mut_ref(),
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
    domain: Vec<SPValue>,
    pub planned: bool, // does it exist in the planner
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
    fn get_child_mut<'a>(&'a mut self, _: &str, _: &SPPath) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn find_item_among_children<'a>(
        &'a self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path: &SPPath, _changes: &mut HashMap<SPPath, SPPath>) {}
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match &mut self.type_ {
            VariableType::Predicate(p) => p.replace_variable_path(mapping),
            _ => (),
        };
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Variable(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Variable(self)
    }
}

impl Variable {
    pub fn new(
        name: &str, type_: VariableType, value_type: SPValueType, domain: Vec<SPValue>,
    ) -> Variable {
        let node = SPNode::new(name);
        Variable {
            node,
            type_,
            value_type,
            domain,
            planned: true,
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

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum TransitionType {
    Controlled,
    Auto,
    Effect,
    Runner,
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Transition {
    node: SPNode,
    pub guard: Predicate,
    pub actions: Vec<Action>,
    pub type_: TransitionType,
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
    fn get_child_mut<'a>(&'a mut self, _: &str, _: &SPPath) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn find_item_among_children<'a>(
        &'a self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path: &SPPath, _changes: &mut HashMap<SPPath, SPPath>) {}
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.guard.replace_variable_path(mapping);
        self.actions
            .iter_mut()
            .for_each(|i| i.replace_variable_path(mapping));

    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Transition(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Transition(self)
    }
}

impl Transition {
    pub fn new(name: &str, guard: Predicate, actions: Vec<Action>, type_: TransitionType) -> Self {
        let node = SPNode::new(name);
        Transition {
            node,
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
    pub fn controlled(&self) -> bool {
        self.type_ == TransitionType::Controlled
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
}

impl fmt::Display for Transition {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        let k = match self.type_ {
            TransitionType::Auto => "a",
            TransitionType::Controlled => "c",
            TransitionType::Effect => "e",
            TransitionType::Runner => "r",
        };

        let s = format!(
            "{}_{}: {}/{:?}",
            k,
            self.path(),
            self.guard,
            self.actions
        );

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
    node: SPNode,

    resets: bool,
    pre: Predicate,
    post: Predicate,
    post_actions: Vec<Action>,
    invariant: Option<Predicate>,
}

impl Noder for Intention {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, _next: &str, _path: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn get_child_mut<'a>(&'a mut self, _next: &str, _path: &SPPath) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn find_item_among_children<'a>(
        &'a self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path: &SPPath, _changes: &mut HashMap<SPPath, SPPath>) {
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.pre.replace_variable_path(mapping);
        self.post.replace_variable_path(mapping);
        self.post_actions
            .iter_mut()
            .for_each(|a| a.replace_variable_path(mapping));
        if let Some(invar) = &mut self.invariant {
            invar.replace_variable_path(mapping);
        };
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Intention(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Intention(self)
    }
}

impl Intention {
    pub fn new(name: &str, resets: bool, pre: &Predicate, post: &Predicate,
               post_actions: &[Action], invariant: Option<Predicate>) -> Intention {
        let node = SPNode::new(name);

        Intention {
            node,
            resets,
            pre: pre.clone(),
            post: post.clone(),
            post_actions: post_actions.to_vec(),
            invariant,
        }
    }

    pub fn make_goal(&self) -> IfThen {
        let state = self.path();
        let mut it = IfThen::new("goal", p!(p: state == "e"), self.post.clone(), self.invariant.clone(), None);
        it.node_mut().update_path(self.path());
        it
    }

    pub fn make_runner_transitions(&self) -> Vec<Transition> {
        let state = self.path();

        let mut runner_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p: state == "i"), self.pre.clone()]),
            vec![a!(p: state = "e")],
            TransitionType::Controlled,
        );
        runner_start.node_mut().update_path(self.path());
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
            TransitionType::Auto
        );
        runner_finish.node_mut().update_path(self.path());

        vec![runner_start, runner_finish]
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
    pub actions: Option<Vec<Action>>,
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
    fn get_child_mut<'a>(&'a mut self, _: &str, _: &SPPath) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn find_item_among_children<'a>(
        &'a self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path: &SPPath, _changes: &mut HashMap<SPPath, SPPath>) {}
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
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::IfThen(self)
    }
}

impl IfThen {
    pub fn new(
        name: &str, condition: Predicate, goal: Predicate, invariant: Option<Predicate>, actions: Option<Vec<Action>>
    ) -> IfThen {
        let node = SPNode::new(name);
        IfThen {
            node,
            condition,
            goal,
            invariant,
            actions
        }
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

    pub fn upd_state_path(&mut self, state: &SPState) {
        self.condition.upd_state_path(state);
        self.goal.upd_state_path(state);
        self.invariant.as_mut().map(|x| x.upd_state_path(state));
    }
}


#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Operation {
    node: SPNode,

    pub auto: bool,
    pub guard: Predicate,
    pub effects_goals: Vec<(Vec<Action>, Predicate)>,
    pub post_actions: Vec<Action>, // TODO: this should be synchronization only
    pub mc_constraint: Option<Predicate>,
}

impl Noder for Operation {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, _next: &str, _path: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }

    fn get_child_mut<'a>(&'a mut self, _next: &str, _path: &SPPath) -> Option<SPMutItemRef<'a>> {
        None
    }

    fn find_item_among_children<'a>(
        &'a self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path: &SPPath, _changes: &mut HashMap<SPPath, SPPath>) {
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.guard.replace_variable_path(mapping);
        self.effects_goals.iter_mut().for_each(|(e,g)| {
            e.iter_mut().for_each(|e| e.replace_variable_path(mapping));
            g.replace_variable_path(mapping);
        });
        self.post_actions.iter_mut().for_each(|e| e.replace_variable_path(mapping));
        self.mc_constraint.as_mut().map(|c| c.replace_variable_path(mapping));
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Operation(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Operation(self)
    }
}

impl Operation {

    pub fn new(name: &str, auto: bool, guard: &Predicate,
               effects_goals: &[(&[Action], &Predicate)],
               post_actions: &[Action],
               mc_constraint: Option<Predicate>) -> Operation {
        let node = SPNode::new(name);

        Operation {
            node,
            auto,
            guard: guard.clone(),
            effects_goals: effects_goals.iter().map(|(a,p)| (a.to_vec(), (*p).clone())).collect(),
            post_actions: post_actions.to_vec(),
            mc_constraint,
        }
    }

    pub fn make_replan_specs(&self) -> Vec<Spec> {
        let mut specs = vec![];
        for (effects, goal) in self.effects_goals.iter() {
            let pre = Predicate::AND(vec![self.guard.clone(), (*goal).clone()]);
            let mut act = effects.to_vec();
            act.extend(self.post_actions.iter().cloned());
            if !act.is_empty() {
                if self.auto {
                    // it is important to realize that we cannot freely change
                    // the goals of the high level when we are in this state
                    // or any other state from which this state can
                    // uncontrollably be reached. so we also create a spec here
                    let mut spec = Spec::new("replan_spec",
                                             Predicate::NOT(Box::new(pre.clone())));
                    spec.node_mut().update_path(self.path());
                    specs.push(spec);
                }
            }
        }
        specs
    }

    pub fn make_lowlevel_transitions(&self) -> Vec<Transition> {
        let mut trans = vec![];
        for (i, (effects, goal)) in self.effects_goals.iter().enumerate() {
            let pre = Predicate::AND(vec![self.guard.clone(), (*goal).clone()]);
            let mut act = effects.to_vec();
            act.extend(self.post_actions.iter().cloned());
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
                    }
                );
                t.node_mut().update_path(self.path());
                trans.push(t);
            }
        }
        trans
    }

    pub fn make_runner_transitions(&self) -> Vec<Transition> {
        // auto ops are not actually operations...
        let is_auto = self.effects_goals.iter()
            .all(|(_,g)| g == &Predicate::TRUE);
        if is_auto {
            return vec![]
        }

        let state = self.path();
        let mut runner_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p: state == "i"), self.guard.clone()]),
            vec![a!(p: state = "e")],
            TransitionType::Controlled
        );
        runner_start.node_mut().update_path(self.path());

        let mut runner_finish = Transition::new(
            "finish",
            p!(p: state == "e"),
            // note that the missing "goal" is added when running...
            vec![a!(p: state = "i")],
            TransitionType::Auto
        );
        runner_finish.node_mut().update_path(self.path());

        vec![runner_start, runner_finish]
    }

    pub fn make_planning_trans(&self) -> Vec<Transition> {
        self.effects_goals.iter().enumerate()
            .map(|(i,(e,g))| {
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
                    });
                t.node_mut().update_path(self.path());
                t
            }
        ).collect()
    }

    pub fn make_verification_goal(&self) -> Predicate {
        let goals = Predicate::OR(self.effects_goals.iter()
                                  .map(|(_,g)| (*g).clone())
                                  .collect());
        Predicate::AND(vec![self.guard.clone(), goals.clone()])
    }

}

/// Specs are used to define global constraints
/// TODO: should we allow ltl expressions?
/// For now its just simple forbidden states
#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct Spec {
    node: SPNode,
    pub invariant: Predicate,
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
    fn get_child_mut<'a>(&'a mut self, _: &str, _: &SPPath) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn find_item_among_children<'a>(
        &'a self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        None
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, _name: &str, _path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn update_path_children(&mut self, _path: &SPPath, _changes: &mut HashMap<SPPath, SPPath>) {
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.invariant.replace_variable_path(mapping);
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::Spec(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::Spec(self)
    }
}

impl Spec {
    pub fn new(name: &str, invariant: Predicate) -> Spec {
        let node = SPNode::new(name);
        Spec { node, invariant }
    }
    pub fn invariant(&self) -> &Predicate {
        &self.invariant
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct TransitionSpec {
    node: SPNode,
    pub spec_transition: Transition,
    pub syncronized_with: Vec<SPPath>,
}

impl Noder for TransitionSpec {
    fn node(&self) -> &SPNode {
        &self.node
    }
    fn node_mut(&mut self) -> &mut SPNode {
        &mut self.node
    }
    fn get_child<'a>(&'a self, _: &str, _: &SPPath) -> Option<SPItemRef<'a>> {
        None
    }
    fn get_child_mut<'a>(&'a mut self, _: &str, _: &SPPath) -> Option<SPMutItemRef<'a>> {
        None
    }
    fn find_item_among_children<'a>(
        &'a self, name: &str, path_sections: &[&str],
    ) -> Option<SPItemRef<'a>> {
        self.spec_transition.find_item(name, path_sections)
    }
    fn find_item_mut_among_children<'a>(
        &'a mut self, name: &str, path_sections: &[&str],
    ) -> Option<SPMutItemRef<'a>> {
        self.spec_transition.find_item_mut(name, path_sections)
    }
    fn update_path_children(&mut self, _path: &SPPath, _changes: &mut HashMap<SPPath, SPPath>) {
        self.spec_transition.update_path(_path, _changes);
    }
    fn rewrite_expressions(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        self.spec_transition.rewrite_expressions(mapping);
        self.syncronized_with.iter_mut().for_each(|p| {
            if let Some(np) = mapping.get(p) {
                *p = np.clone();
            }
        })
    }
    fn as_ref(&self) -> SPItemRef<'_> {
        SPItemRef::TransitionSpec(self)
    }
    fn as_mut_ref(&mut self) -> SPMutItemRef<'_> {
        SPMutItemRef::TransitionSpec(self)
    }
}

impl TransitionSpec {
    pub fn new(
        name: &str, spec_transition: Transition, syncronized_with: Vec<SPPath>,
    ) -> TransitionSpec {
        let node = SPNode::new(name);
        TransitionSpec {
            node,
            spec_transition,
            syncronized_with,
        }
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
