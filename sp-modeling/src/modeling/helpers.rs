use sp_domain::*;
use std::iter::FromIterator;

#[derive(Debug, PartialEq, Clone)]
pub struct Domain {
    pub domain: Option<Vec<SPValue>>, // none=domain is boolean
}

#[derive(Debug, PartialEq, Clone)]
pub struct CommandTopic {
    pub topic: String,
    pub ros_type: String,
    pub vars: Vec<(String, Domain)>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct MeasuredTopic {
    pub topic: String,
    pub ros_type: String,
    pub vars: Vec<(String, Domain)>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct EstimatedVars {
    pub vars: Vec<(String, Domain)>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct MTransition {
    pub controlled: bool,
    pub guard: Predicate,
    pub actions: Vec<Action>,
    pub effects: Vec<Action>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct MInvariant {
    pub name: String,
    pub prop: Predicate,
}

#[derive(Debug, PartialEq, Clone)]
pub enum ModelItem {
    CommandTopic(CommandTopic),
    MeasuredTopic(MeasuredTopic),
    EstimatedVars(EstimatedVars),
    MInvariant(MInvariant),
    Transitions(Vec<Transition>),
    Predicates(Vec<Variable>),
}

#[derive(Debug, PartialEq, Clone)]
pub struct MResource {
    pub name: String,
    pub items: Vec<ModelItem>,
}

pub fn build_resource(r: &MResource) -> Resource {
    use std::collections::HashMap;

    let mut transitions: Vec<_> = r
        .items
        .iter()
        .flat_map(|i| match i {
            ModelItem::Transitions(t) => Some(t.clone()),
            _ => None,
        })
        .flatten()
        .collect();

    let mut predicates: Vec<_> = r
        .items
        .iter()
        .flat_map(|i| match i {
            ModelItem::Predicates(p) => Some(p.clone()),
            _ => None,
        })
        .flatten()
        .collect();

    let out_topics: Vec<_> = r
        .items
        .iter()
        .flat_map(|i| match i {
            ModelItem::CommandTopic(ct) => Some(ct.clone()),
            _ => None,
        })
        .collect();
    let in_topics: Vec<_> = r
        .items
        .iter()
        .flat_map(|i| match i {
            ModelItem::MeasuredTopic(mt) => Some(mt.clone()),
            _ => None,
        })
        .collect();
    let estimated_vars: Vec<_> = r
        .items
        .iter()
        .flat_map(|i| match i {
            ModelItem::EstimatedVars(dv) => Some(dv.clone()),
            _ => None,
        })
        .collect();
    let mut invariants: Vec<_> = r
        .items
        .iter()
        .flat_map(|i| match i {
            ModelItem::MInvariant(i) => Some(i.clone()),
            _ => None,
        })
        .collect();

    let mut valid_remaps = Vec::new();

    for t in out_topics.iter() {
        for (name, _) in &t.vars {
            let path = SPPath::from_slice(&[r.name.clone(), "goal".to_string(), name.clone()]);
            let op = SPPath::from_string(name);
            valid_remaps.push((op, path));
        }
    }

    for t in in_topics.iter() {
        for (name, _) in &t.vars {
            let path = SPPath::from_slice(&[r.name.clone(), "measured".to_string(), name.clone()]);
            let op = SPPath::from_string(name);
            valid_remaps.push((op, path));
        }
    }

    for t in &estimated_vars {
        for (name, _) in &t.vars {
            let op = SPPath::from_string(name);
            let path = SPPath::from_slice(&[r.name.clone(), name.clone()]);
            valid_remaps.push((op, path));
        }
    }

    for p in &predicates {
        let name = p.name().to_string();
        let path = SPPath::from_slice(&[r.name.clone(), name.clone()]);
        let op = SPPath::from_slice(&[name]);
        valid_remaps.push((op, path));
    }

    println!("all valid remaps");
    valid_remaps.iter().for_each(|x| println!("{:?}", x));
    let valid_remaps = HashMap::from_iter(valid_remaps.into_iter());

    // fix predicate paths
    for p in &mut predicates {
        println!("original predicate: {:?}", p);
        p.rewrite_expressions(&valid_remaps);
        println!("new predicate: {:?}", p);
    }
    // fix transition paths
    for t in &mut transitions {
        println!("original guard: {:?}", t.guard);
        t.guard.replace_variable_path(&valid_remaps);
        println!("new guard: {:?}", t.guard);

        println!("original actions: {:?}", t.actions);
        t.actions
            .iter_mut()
            .for_each(|a| a.replace_variable_path(&valid_remaps));
        println!("new actions: {:?}", t.actions);
    }

    for i in &mut invariants {
        // println!("INVARIANT BEFORE: {}", i.prop);
        i.prop.replace_variable_path(&valid_remaps);
        // println!("INVARIANT AFTER: {}", i.prop);
    }

    // using all our fixed stuff, build a new resource
    let mut r = Resource::new(&r.name);

    for t in out_topics.iter() {
        let vars: Vec<Variable> = t
            .vars
            .iter()
            .map(|(name, domain)| {
                let node_name = format!("goal/{}", name);
                domain
                    .domain
                    .clone()
                    .and_then(|d| {
                        if !d.is_empty() {
                            Some(Variable::new(
                                &node_name,
                                VariableType::Command,
                                d.first().unwrap().has_type(),
                                d,
                            ))
                        } else {
                            None
                        }
                    })
                    .unwrap_or(Variable::new_boolean(&node_name, VariableType::Command))
            })
            .collect();

        let var_map = vars
            .into_iter()
            .map(|v| {
                let map = MessageVariable {
                    name: v.node().name_path().drop_root(),
                    path: v.node().name_path().clone(),
                    relative_path: true,
                };
                r.add_variable(v);
                map
            })
            .collect();

        let mess_type = if t.ros_type == "json" {
            MessageType::Json
        } else if t.ros_type == "flat" {
            MessageType::JsonFlat
        } else {
            MessageType::Ros(t.ros_type.clone())
        };

        let msg = NewMessage {
            topic: SPPath::from_string(&t.topic),
            relative_topic: true,
            category: MessageCategory::OutGoing,
            message_type: mess_type,
            variables: var_map,
        };

        r.add_messsage(msg);
    }

    for t in in_topics.iter() {
        let vars: Vec<Variable> = t
            .vars
            .iter()
            .map(|(name, domain)| {
                let node_name = format!("measured/{}", name);
                domain
                    .domain
                    .clone()
                    .and_then(|d| {
                        if !d.is_empty() {
                            Some(Variable::new(
                                &node_name,
                                VariableType::Measured,
                                d.first().unwrap().has_type(),
                                d,
                            ))
                        } else {
                            None
                        }
                    })
                    .unwrap_or(Variable::new_boolean(&node_name, VariableType::Measured))
            })
            .collect();

        let var_map = vars
            .into_iter()
            .map(|v| {
                let map = MessageVariable {
                    name: v.node().name_path().drop_root(),
                    path: v.node().name_path().clone(),
                    relative_path: true,
                };
                r.add_variable(v);
                map
            })
            .collect();

        let mess_type = if t.ros_type == "json" {
            MessageType::Json
        } else if t.ros_type == "flat" {
            MessageType::JsonFlat
        } else {
            MessageType::Ros(t.ros_type.clone())
        };

        let msg = NewMessage {
            topic: SPPath::from_string(&t.topic),
            relative_topic: true,
            category: MessageCategory::Incoming,
            message_type: mess_type,
            variables: var_map,
        };

        r.add_messsage(msg);
    }

    for t in &estimated_vars {
        t.vars.iter().for_each(|(name, domain)| {
            let node_name = format!("{}", name);
            let v = domain
                .domain
                .clone()
                .and_then(|d| {
                    if !d.is_empty() {
                        Some(Variable::new(
                            &node_name,
                            VariableType::Estimated,
                            d.first().unwrap().has_type(),
                            d,
                        ))
                    } else {
                        None
                    }
                })
                .unwrap_or(Variable::new_boolean(&node_name, VariableType::Estimated));
            r.add_variable(v);
        });
    }

    let resource_path = r.path().clone();
    let guard: Predicate = p!(p: resource_path);
    for mut t in transitions {
        t.guard = Predicate::AND(vec![guard.clone(), t.guard]);
        r.add_transition(t);
    }
    for p in predicates {
        r.add_variable(p);
    }
    for i in invariants {
        r.add_spec(Spec::new(&i.name, i.prop.clone()));
    }

    let temp_model = Model::new_no_root(&r.name(), vec![SPItem::Resource(r.clone())]);
    let temp_ts_model = TransitionSystemModel::from(&temp_model);
    crate::planning::generate_offline_nuxvm(&temp_ts_model, &Predicate::TRUE);
    return r;
}

#[macro_export]
macro_rules! command {
    (topic: $t:tt , $($rest:tt)*) => {{
        let mut cmd = crate::modeling::CommandTopic {
            topic: String::from($t),
            ros_type: String::new(),
            vars: Vec::new(),
        };
        command!(private cmd $($rest)*)
    }};

    // no domain defaults to boolean variable
    (private $cmd:ident msg_type: $msg_type:tt, $($rest:tt)*) => {{
        $cmd.ros_type = String::from($msg_type);
        command!(private $cmd $($rest)*)
    }};

    // no domain defaults to boolean variable
    (private $cmd:ident $var_name:tt : bool, $($rest:tt)*) => {{
        $cmd.vars.push((stringify!($var_name).to_string(),
                         crate::modeling::Domain { domain: None } ));
        command!(private $cmd $($rest)*)
    }};

    // same as above with no trailing comma
    (private $cmd:ident $first:tt : bool) => {{
        command!(private $cmd $first : bool ,)
    }};

    // variable with domain + trailing comma
    (private $cmd:ident $var_name:tt : $dom:expr, $($rest:tt)*) => {{
        {
            let domain: Vec<SPValue> = $dom.iter().map(|d| d.to_spvalue()).collect();
            $cmd.vars.push((stringify!($var_name).to_string(),
                crate::modeling::Domain { domain: Some(domain) } ));
        }
        command!(private $cmd $($rest)*)
    }};

    // same as above with no trailing comma
    (private $cmd:ident $first:tt : $second:tt) => {{
        command!(private $cmd $first : $second ,)
    }};

    (private $cmd:expr) => {{
        crate::modeling::ModelItem::CommandTopic($cmd)
    }};
}

#[macro_export]
macro_rules! measured {
    (topic: $t:tt , $($rest:tt)*) => {{
        let mut measured = crate::modeling::MeasuredTopic {
            topic: String::from($t),
            ros_type: String::new(),
            vars: Vec::new(),
        };
        measured!(private measured $($rest)*)
    }};

    (private $measured:ident msg_type: $msg_type:tt, $($rest:tt)*) => {{
        $measured.ros_type = String::from($msg_type);
        measured!(private $measured $($rest)*)
    }};

    (private $measured:ident $($var_name:ident $(/)?)+ : bool, $($rest:tt)*) => {{
        let name = vec![ $( stringify!($var_name) , )+ ];
        let name = name.join("/");
        $measured.vars.push((name,
            crate::modeling::Domain { domain: None } ));
        measured!(private $measured $($rest)*)
    }};

    // same as above with no trailing comma
    (private $measured:ident $($var_name:ident $(/)?)+ : bool) => {{
        measured!(private $measured $($var_name:ident $(/)?)+ : bool ,)
    }};

    // variable with domain + trailing comma
    (private $measured:ident $($var_name:ident $(/)?)+ : $dom:expr, $($rest:tt)*) => {{
        {
            let name = vec![ $( stringify!($var_name) , )+ ];
            let name = name.join("/");
            let domain: Vec<SPValue> = $dom.iter().map(|d| d.to_spvalue()).collect();
            $measured.vars.push((name,
                crate::modeling::Domain { domain: Some(domain) } ));
        }
        measured!(private $measured $($rest)*)
    }};

    // same as above with no trailing comma
    (private $measured:ident $($var_name:ident $(/)?)+ : $dom:expr) => {{
        measured!(private $measured $($var_name:ident $(/)?)+ : $dom:expr ,)
    }};

    (private $measured:expr) => {{
        crate::modeling::ModelItem::MeasuredTopic($measured)
    }};
}

#[macro_export]
macro_rules! estimated {
    // no domain defaults to boolean variable
    (private $estimated:ident $var_name:tt : bool, $($rest:tt)*) => {{
        $estimated.vars.push((stringify!($var_name).to_string(),
            crate::modeling::Domain { domain: None } ));
        estimated!(private $estimated $($rest)*)
    }};

    // same as above with no trailing comma
    (private $estimated:ident $first:tt : bool) => {{
        estimated!(private $estimated $first : bool ,)
    }};

    // variable with domain + trailing comma
    (private $estimated:ident $var_name:tt : $dom:expr, $($rest:tt)*) => {{
        {
            let domain: Vec<SPValue> = $dom.iter().map(|d| d.to_spvalue()).collect();
            $estimated.vars.push((stringify!($var_name).to_string(),
            crate::modeling::Domain { domain: Some(domain) } ));
        }
        estimated!(private $estimated $($rest)*)
    }};

    // same as above with no trailing comma
    (private $estimated:ident $first:tt : $second:tt) => {{
        estimated!(private $estimated $first : $second ,)
    }};

    (private $estimated:expr) => {{
        crate::modeling::ModelItem::EstimatedVars($estimated)
    }};

    // didnt match any private. start fresh
    ($($rest:tt)*) => {{
        let mut estimated = crate::modeling::EstimatedVars {
            vars: Vec::new(),
        };
        estimated!(private estimated $($rest)*)
    }};
}

#[macro_export]
macro_rules! always {
    (name: $n:tt , prop: $pred:expr) => {{
        let spec = MInvariant {
            name: String::from(stringify!($n)),
            prop: $pred.clone(),
        };
        ModelItem::MInvariant(spec)
    }};
}

#[macro_export]
macro_rules! never {
    (name: $n:tt , prop: $pred:expr) => {{
        let spec = crate::modeling::MInvariant {
            name: String::from(stringify!($n)),
            prop: Predicate::NOT(Box::new($pred.clone())),
        };
        crate::modeling::ModelItem::MInvariant(spec)
    }};
}

#[macro_export]
macro_rules! resource {
    (name: $n:expr , $($rest:tt)*) => {{
        let mut resource = crate::modeling::MResource {
            name: String::from($n), // String::from(stringify!($n)),
            items: Vec::new(),
        };
        resource!(private resource $($rest)*)
    }};

    // everything
    (private $resource:ident $item:expr, $($rest:tt)*) => {{
        $resource.items.push($item.clone());
        resource!(private $resource $($rest)*)
    }};

    // no trailing comma
    (private $resource:ident $item:expr) => {{
        resource!(private $resource $item,)
    }};

    // all done
    (private $resource:expr) => {{
        crate::modeling::build_resource(&$resource)
    }};
}

#[macro_export]
macro_rules! predicates {
    ($($name: tt: $p: expr),* $(,)?) => {{
        let mut preds = Vec::new();
        $(preds.push(Variable::new_predicate(&stringify!($name).to_string(), $p));)*
        crate::modeling::ModelItem::Predicates(preds)
    }};
}

#[macro_export]
macro_rules! transitions {
    ($($name: tt: $guard: expr , $actions: expr),* $(,)?) => {{
        let mut trans = Vec::new();
        $(
            let name = stringify!($name).to_string();
            if name.starts_with("c_") {
                trans.push(Transition::new(name.trim_start_matches("c_"), $guard, $actions, TransitionType::Controlled));
            } else if name.starts_with("a_") {
                trans.push(Transition::new(name.trim_start_matches("a_"), $guard, $actions, TransitionType::Auto));
            } else if name.starts_with("e_") {
                trans.push(Transition::new(name.trim_start_matches("e_"), $guard, $actions, TransitionType::Effect));
            } else if name.starts_with("r_") {
                trans.push(Transition::new(name.trim_start_matches("e_"), $guard, $actions, TransitionType::Runner));
            }
        )*
        crate::modeling::ModelItem::Transitions(trans)
    }};
}

#[test]
fn resource_test() {
    let domain = vec!["unknown", "at", "away"];
    let r = resource! {
        name: "dummy_robot",
        command!{
            topic: "hej",
            msg_type: "std_messages",

            activate : bool,
            ref_pos : domain,
        },
        measured!{
            topic: "hej2",
            msg_type: "std_messages2",

            active: bool,
            act_pos: domain,
        },
        estimated!{
            prev_pos: domain,
        },

        predicates!{
            to_away_enabled: p!([active] && [prev_pos == "at"] && [ref_pos != "at"] && [act_pos != "at"]),
            to_away_exeucting: p!([active] && [ref_pos == "at"] && [act_pos != "at"]),
            to_away_finished: p!([active] && [ref_pos == "at"] && [act_pos == "at"]),
        },

        transitions!{
            c_start: p!(to_away_enabled), vec![ a!(ref_pos = "at") ],
            e_finish: p!(to_away_exeucting), vec![a!(act_pos = "at") ],

            r_activate: p!(!active), vec![a!(activate) ],
            a_deactivate: p!(active), vec![a!(!activate) ],
        },

        never!{
            name: never_unknown,
            prop: p!(ref_pos == "unknown")
        },

        always!{
            name: always_away,
            prop: p!(act_pos == "away")
        },

    };
    println!("{:#?}", r);
}
