use sp_domain::*;
use std::collections::HashMap;
use std::iter::FromIterator;
use crate::formal_model::*;

#[derive(Debug, PartialEq, Clone)]
pub struct Domain {
    // pub name: String, // would it be possible to add this? z3 needs it
    pub domain: Option<Vec<SPValue>>, // none=domain is boolean
}

#[derive(Debug, PartialEq, Clone)]
pub struct CommandTopic {
    pub topic: String,
    pub ros_type: String,
    pub vars: HashMap<String, Domain>,
    pub initial_values: HashMap<String, SPValue>, // initial states
}

#[derive(Debug, PartialEq, Clone)]
pub struct MeasuredTopic {
    pub topic: String,
    pub ros_type: String,
    pub vars: HashMap<String, Domain>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct EstimatedVars {
    pub vars: HashMap<String, Domain>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct MTransition {
    pub controlled: bool,
    pub guard: Predicate,
    pub actions: Vec<Action>,
    pub effects: Vec<Action>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct MAbility {
    pub name: String,
    pub predicates: HashMap<String, Predicate>,
    pub transitions: HashMap<String, MTransition>,
}

#[derive(Debug, PartialEq, Clone)]
pub struct MInvariant {
    pub name: String,
    pub prop: Predicate
}

#[derive(Debug, PartialEq, Clone)]
pub struct MResource {
    pub name: String,
    pub items: Vec<ModelItem>
}

#[derive(Debug, PartialEq, Clone)]
pub enum ModelItem {
    CommandTopic(CommandTopic),
    MeasuredTopic(MeasuredTopic),
    EstimatedVars(EstimatedVars),
    MAbility(MAbility),
    MInvariant(MInvariant),
}

pub fn build_resource(r: &MResource) -> Resource {
    let mut abilities: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::MAbility(a) => Some(a.clone()), _ => None }).collect();
    let out_topics: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::CommandTopic(ct) => Some(ct.clone()), _ => None }).collect();
    let in_topics: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::MeasuredTopic(mt) => Some(mt.clone()), _ => None }).collect();
    let estimated_vars: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::EstimatedVars(dv) => Some(dv.clone()), _ => None }).collect();
    let mut invariants: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::MInvariant(i) => Some(i.clone()), _ => None }).collect();

    let mut valid_remaps = Vec::new();
    let mut echos = Vec::new();

    for (mt, t) in out_topics.iter().enumerate() {
        for name in t.vars.keys() {
            let path = SPPath::from_slice(&[r.name.clone(), t.topic.clone(), mt.to_string(), name.clone()]);
            let op = SPPath::from_string(name);
            valid_remaps.push((op,path));
        }
    }

    for (mt, t) in in_topics.iter().enumerate() {
        for name in t.vars.keys() {
            // take care of our special "echo" here.
            println!("measured: {}", name);
            let op = SPPath::from_string(name);
            let mut path = SPPath::from_slice(&[r.name.clone(), t.topic.clone(), mt.to_string()]);
            path.add_child_path(&op);
            let is_echo = op.path.iter().nth(0).map(|s| s=="echo").unwrap_or(false);
            if is_echo {
                println!("ECHO {}", path);
                echos.push(path);
            } else {
                valid_remaps.push((op,path));
            }
        }
    }

    for t in &estimated_vars {
        for name in t.vars.keys() {
            let path = SPPath::from_slice(&[r.name.clone(), name.clone()]);
            let op = SPPath::from_string(name);
            valid_remaps.push((op,path));
        }
    }

    for a in &abilities {
        for (name, _pred) in &a.predicates {
            let path = SPPath::from_slice(&[r.name.clone(), a.name.clone(), name.clone()]);
            let op = SPPath::from_slice(&[a.name.clone(), name.clone()]);
            valid_remaps.push((op, path));
        }
    }

    // println!("all valid remaps: {:?}", valid_remaps);
    let valid_remaps = HashMap::from_iter(valid_remaps.into_iter());

    // fix ability paths
    for a in &mut abilities {

        // first try to find a local remap (e.g. to our own predicates).
        let mut local_remaps = Vec::new();
        for (name, _pred) in &mut a.predicates {
            let path = SPPath::from_slice(&[r.name.clone(), a.name.clone(), name.clone()]);
            let op = SPPath::from_string(name);
            local_remaps.push((op, path));
        }

        let local_remaps = HashMap::from_iter(local_remaps.into_iter());


        for (_name, pred) in &mut a.predicates {
            // println!("original predicate: {:?}", pred);
            pred.replace_variable_path(&local_remaps);
            pred.replace_variable_path(&valid_remaps);
            // println!("new predicate: {:?}", pred);
        }
        for (_name, trans) in &mut a.transitions {
            // println!("original guard: {:?}", trans.guard);
            trans.guard.replace_variable_path(&local_remaps);
            trans.guard.replace_variable_path(&valid_remaps);
            // println!("new guard: {:?}", trans.guard);

            // println!("original actions: {:?}", trans.actions);
            trans.actions.iter_mut().for_each(|a| a.replace_variable_path(&valid_remaps));
            // println!("new actions: {:?}", trans.actions);

            // println!("original effects: {:?}", trans.effects);
            trans.effects.iter_mut().for_each(|a| a.replace_variable_path(&valid_remaps));
            // println!("new effects: {:?}", trans.effects);
        }
    }

    for i in &mut invariants {
        // println!("INVARIANT BEFORE: {}", i.prop);
        i.prop.replace_variable_path(&valid_remaps);
        // println!("INVARIANT AFTER: {}", i.prop);
    }

    // using all our fixed stuff, build a new resource
    let mut r = Resource::new(&r.name);

    for (mt, t) in out_topics.iter().enumerate() {
        let msg = Message::new(
            &mt.to_string(),
            &t.ros_type,
            t.vars.iter().map(|(name, d)| {

                let is_bool = d.domain.is_none();
                let var = if is_bool {
                    MessageField::Var(Variable::new(name,
                                                    VariableType::Command,
                                                    SPValueType::Bool,
                                                    SPValue::Bool(false), // replace with initial_values
                                                    vec![]))
                } else {
                    let dom: Vec<_> = d.domain.clone().unwrap().clone(); // fix
                    let sp_val_type = dom[0].has_type();
                    let initial = dom[0].clone(); // replace with initial_values
                    MessageField::Var(Variable::new(name,
                                                    VariableType::Command,
                                                    sp_val_type,
                                                    initial,
                                                    dom))
                };
                var
            }).collect::<Vec<_>>().as_slice());

        let topic = Topic::new(&t.topic, MessageField::Msg(msg));
        r.add_message(topic);
    }

    for (mt, t) in in_topics.iter().enumerate() {

        // collect all different set of subtopics
        let mut subs: Vec<_> = t.vars.iter().map(|(name, _d)| {
            let op = SPPath::from_string(name);
            op.parent()
        }).collect();

        subs.sort_by(|l,r| {
            if l.path.len() < r.path.len() {
                std::cmp::Ordering::Less
            } else if l.path.len() > r.path.len() {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });
        subs.dedup();

        // let mut belongs_to: HashMap<SPPath, Vec<SPPath>> = t.vars

        let sub_leafs: HashMap<SPPath, HashMap<String, MessageField>> = subs.iter().map(|s| {
            let msg = t.vars.iter().flat_map(|(name, d)| { // not on this "level"
                let op = SPPath::from_string(name);
                let name = op.leaf();
                if &op.parent() != s { None } else {
                    let is_bool = d.domain.is_none();
                    let var = if is_bool {
                        MessageField::Var(Variable::new(&name,
                                                        VariableType::Measured,
                                                        SPValueType::Bool,
                                                        SPValue::Bool(false), // replace with initial_values
                                                        vec![]))
                    } else {
                        let dom: Vec<_> = d.domain.clone().unwrap().clone(); // fix
                        let sp_val_type = dom[0].has_type();
                        let initial = dom[0].clone(); // replace with initial_values
                        MessageField::Var(Variable::new(&name,
                                                        VariableType::Measured,
                                                        sp_val_type,
                                                        initial,
                                                        dom))
                    };
                    Some((name.clone(), var))
                }
            }).collect();
            (s.clone(), msg)
        }).collect();

        // start from the top, recurse down.
        let mut children: HashMap<SPPath, HashMap<String,MessageField>> = HashMap::new();
        while !subs.is_empty() {
            let current = subs.pop().unwrap();
            let mut leafs = sub_leafs.get(&current).unwrap_or(&HashMap::new()).clone();

            // add children of already added
            let children_to_add = children.get(&current).unwrap_or(&HashMap::new()).clone();

            for (k, v) in children_to_add {
                leafs.insert(k.clone(),v.clone());
            }

            let node = current.leaf();
            let tt = if node == "" {
                t.ros_type.clone()
            } else {
                "anonymous".to_string()
            };
            let leafs: Vec<_> = leafs.into_iter().map(|(_k,v)|v).collect();
            let nn = if node == "" {
                mt.to_string()
            } else {
                node.clone()
            };
            let msg = Message::new(
                &nn,
                &tt,
                leafs.as_slice());

            let parent = current.parent();

            if node != "" {
                let mut old = children.get(&parent).unwrap_or(&HashMap::new()).clone();
                old.insert(node, MessageField::Msg(msg));
                children.insert(parent, old);
            } else {
                let mut hm = HashMap::new();
                hm.insert(node, MessageField::Msg(msg));
                // at the root, overwrite instead of add.
                children.insert(SPPath::new(), hm);
            }
        }

        let msg = children.get(&SPPath::new()).unwrap().clone();
        let msg = msg.get("").unwrap();

        let topic = Topic::new(&t.topic, msg.clone());

        r.add_message(topic);
    }

    // add echo mirrors
    for e in echos {
        let name = SPPath::from_string(&e.leaf());
        let command_var = valid_remaps.get(&name).expect("echo remap failed");
        r.add_command_mirror(&command_var, &e);
    }

    for t in &estimated_vars {
        t.vars.iter().for_each(|(name, d)| {
            let is_bool = d.domain.is_none();
            let var = if is_bool {
                Variable::new(name,
                              VariableType::Estimated,
                              SPValueType::Bool,
                              SPValue::Bool(false), // replace with initial_values
                              vec![])
            } else {
                let dom: Vec<_> = d.domain.clone().unwrap().clone(); // fix
                let sp_val_type = dom[0].has_type();
                let initial = dom[0].clone(); // replace with initial_values
                Variable::new(name,
                              VariableType::Estimated,
                              sp_val_type,
                              initial,
                              dom)
            };
            r.add_sub_item(SPItem::Variable(var));
        });
    }

    // fix ability paths
    for a in &abilities {
        let p = a.predicates.iter().map(|(n,p)| Variable::new_predicate(n, p.clone())).collect();

        // below is how I would like to do it => just keep the "Any"
        // vals. works great for planning and we can still spit out
        // invariants to the planning problem that works in the same
        // way as the guards. however its harder to understand what's
        // going on.

        // let t = a.transitions.iter().map(|(n, t)|
        //                                  Transition::new(n, t.guard.clone(),
        //                                                  t.actions.clone(), t.effects.clone(),
        //                                                  t.controlled)).collect();

        // below is how I have to do it for guard extraction to work,
        // flattening the "Any"s into new transitions.

        let t = a.transitions.iter().flat_map(|(n, t)| {
            let anys: Vec<_> = t.actions.iter().flat_map(|a| match a.value {
                Compute::Any => Some(a.clone()),
                _ => None
            }).collect();

            let not_anys: Vec<Action> = t.actions.iter().flat_map(|a| match a.value {
                Compute::Any => None,
                _ => Some(a.clone())
            }).collect();

            if anys.len() == 1 {
                // hack to expand into multiple trans
                let any_var = r.get(&anys[0].var).expect("variable not found");
                let new_actions: Vec<_> = any_var.as_variable()
                    .expect("error2")
                    .domain()
                    .iter()
                    .map(|d|
                         (Action::new(anys[0].var.clone(),
                                     Compute::PredicateValue(
                                         PredicateValue::SPValue(d.clone()))), d.to_string())).collect();

                new_actions.iter().map(|(a,v)| {
                    let name = format!("{}_with_{}", n, v);
                    let mut a = vec![a.clone()];
                    a.extend(not_anys.iter().cloned());
                    Transition::new(&name, t.guard.clone(),
                                    a, t.effects.clone(),
                                    t.controlled)
                }).collect()

            } else {
                vec![Transition::new(n, t.guard.clone(),
                                t.actions.clone(), t.effects.clone(),
                                t.controlled)]
            }
        }).collect();
        let a = Ability::new(&a.name, t, p);
        r.add_ability(a);
    }

    let mut supervisor = Predicate::AND(invariants.iter().map(|i|i.prop.clone()).collect());
    if !r.abilities.is_empty() && !invariants.is_empty() {
        // add invariants to model
        let mut to_add = Vec::new();
        to_add.push(SPItem::Resource(r.clone()));

        for i in &invariants {
            to_add.push(SPItem::Spec(Spec::new(&i.name, i.prop.clone())));
        }

        // lastly, we should preprocess the individual resource.
        let temp_model = Model::new_no_root("temp", to_add);
        let temp_ts_model = TransitionSystemModel::from(&temp_model);
        println!("GE FOR {}", r.name());
        let (new_guards, sv) = extract_guards(&temp_ts_model, &Predicate::TRUE);
        supervisor = sv;

        for a in &mut r.abilities {
            let mut to_remove = Vec::new();
            for t in &mut a.transitions {
                match new_guards.get(&t.path().to_string()) {
                    Some(Predicate::FALSE) => {
                        println!("guard is FALSE for {}, removing transition", t.path());
                        to_remove.push(t.path().clone());
                    }
                    Some(g) => {
                        println!("adding guard for {}: {}", t.path(), g);
                        *t.mut_guard() = Predicate::AND(vec![t.guard().clone(), g.clone()]);
                    },
                    None => {},
                }
            }
            a.transitions.retain(|t| !to_remove.contains(&t.path()));
        }
    }

    let temp_model = Model::new_no_root(r.name(), vec![SPItem::Resource(r.clone())]);
    let temp_ts_model = TransitionSystemModel::from(&temp_model);
    crate::planning::generate_offline_nuxvm(&temp_ts_model, &supervisor);

    let supervisor_spec = Spec::new("supervisor", supervisor);
    r.add_sub_item(SPItem::Spec(supervisor_spec));

    return r;
}

#[macro_export]
macro_rules! command {
    (topic: $t:tt , $($rest:tt)*) => {{
        let mut cmd = CommandTopic {
            topic: String::from($t),
            ros_type: String::new(),
            vars: HashMap::new(),
            initial_values: HashMap::new(),
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
        $cmd.vars.insert(stringify!($var_name).to_string(),
                         Domain { domain: None } );
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
            $cmd.vars.insert(stringify!($var_name).to_string(),
                             Domain { domain: Some(domain) } );
        }
        command!(private $cmd $($rest)*)
    }};

    // same as above with no trailing comma
    (private $cmd:ident $first:tt : $second:tt) => {{
        command!(private $cmd $first : $second ,)
    }};

    (private $cmd:expr) => {{
        ModelItem::CommandTopic($cmd)
    }};
}

#[macro_export]
macro_rules! measured {
    (topic: $t:tt , $($rest:tt)*) => {{
        let mut measured = MeasuredTopic {
            topic: String::from($t),
            ros_type: String::new(),
            vars: HashMap::new(),
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
        $measured.vars.insert(name,
                              Domain { domain: None } );
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
            $measured.vars.insert(name,
                                  Domain { domain: Some(domain) } );
        }
        measured!(private $measured $($rest)*)
    }};

    // same as above with no trailing comma
    (private $measured:ident $($var_name:ident $(/)?)+ : $dom:expr) => {{
        measured!(private $measured $($var_name:ident $(/)?)+ : $dom:expr ,)
    }};

    (private $measured:expr) => {{
        ModelItem::MeasuredTopic($measured)
    }};
}

#[macro_export]
macro_rules! estimated {
    // no domain defaults to boolean variable
    (private $estimated:ident $var_name:tt : bool, $($rest:tt)*) => {{
        $estimated.vars.insert(stringify!($var_name).to_string(),
                              Domain { domain: None } );
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
            $estimated.vars.insert(stringify!($var_name).to_string(),
                                  Domain { domain: Some(domain) } );
        }
        estimated!(private $estimated $($rest)*)
    }};

    // same as above with no trailing comma
    (private $estimated:ident $first:tt : $second:tt) => {{
        estimated!(private $estimated $first : $second ,)
    }};

    (private $estimated:expr) => {{
        ModelItem::EstimatedVars($estimated)
    }};

    // didnt match any private. start fresh
    ($($rest:tt)*) => {{
        let mut estimated = EstimatedVars {
            vars: HashMap::new(),
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
        let spec = MInvariant {
            name: String::from(stringify!($n)),
            prop: Predicate::NOT(Box::new($pred.clone())),
        };
        ModelItem::MInvariant(spec)
    }};
}

#[macro_export]
macro_rules! ability {
    (name: $n:tt , $($rest:tt)*) => {{
        let mut ability = MAbility {
            name: String::from(stringify!($n)),
            predicates: HashMap::new(),
            transitions: HashMap::new(),
        };
        ability!(private ability $($rest)*)
    }};

    // list of predicates
    (private $ability:ident $name:tt : $p:expr, $($rest:tt)*) => {{
        $ability.predicates.insert(stringify!($name).to_string(),
                                   $p.clone());
        ability!(private $ability $($rest)*)
    }};

    // no trailing comma...
    (private $ability:ident $name:tt : $p:expr) => {{
        ability!(private $ability $name : $p, )
    }};


    // list of transitions
    (private $ability:ident * $name:tt : $g:expr => [ $($a:expr $(,)?)* ] / [ $($e:expr $(,)?)*], $($rest:tt)*) => {{
        ability!(private $ability true $name : $g => [$($a,)*] / [$($e,)*], $($rest)*)
    }};

    (private $ability:ident $name:tt : $g:expr => [ $($a:expr $(,)?)* ] / [ $($e:expr $(,)?)*], $($rest:tt)*) => {{
        ability!(private $ability false $name : $g => [$($a,)*] / [$($e,)*], $($rest)*)
    }};

    // without trailing commas
    (private $ability:ident * $name:tt : $g:expr => [ $($a:expr $(,)?)* ] / [ $($e:expr $(,)?)*]) => {{
        ability!(private $ability true $name : $g => [$($a,)*] / [$($e,)*],)
    }};

    (private $ability:ident $name:tt : $g:expr => [ $($a:expr $(,)?)* ] / [ $($e:expr $(,)?)*]) => {{
        ability!(private $ability false $name : $g => [$($a,)*] / [$($e,)*],)
    }};


    (private $ability:ident $controlled:tt $name:tt : $g:expr => [ $($a:expr $(,)?)* ] / [ $($e:expr $(,)?)*], $($rest:tt)*) => {{
        let actions = vec![ $($a.clone() ,)* ];
        let effects = vec![ $($e.clone() ,)* ];

        let t = MTransition {
            controlled: $controlled,
            guard: $g.clone(),
            actions: actions,
            effects: effects,
        };
        $ability.transitions.insert(stringify!($name).to_string(), t);
        ability!(private $ability $($rest)*)
    }};

    // all done
    (private $ability:expr) => {{
        ModelItem::MAbility($ability)
    }};
}

#[macro_export]
macro_rules! resource {
    (name: $n:expr , $($rest:tt)*) => {{
        let mut resource = MResource {
            name: String::from($n), // String::from(stringify!($n)),
            items: Vec::new(),
        };
        resource!(private resource $($rest)*)
    }};

    // everything :)
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
        build_resource(&$resource)
    }};
}


#[test]
fn resource_test() {
    let rp_c = SPPath::from_string("ref_pos");
    let ap_m = SPPath::from_string("act_pos");
    let activate_c = SPPath::from_string("activate");
    let active_m = SPPath::from_string("active");
    let enabled = SPPath::from_string("enabled");
    let executing = SPPath::from_string("executing");
    let finished = SPPath::from_string("finished");


    let resource = resource!{
        name: "dummy_robot",
        command!{
            topic: "/hej",
            msg_type: "std_messages",

            act_pos : [1,2,3,4,5],
            active : bool,
            ref_pos : ["hej", "hoppe"],
        },
        measured!{
            topic: "hej2",
            msg_type: "std_messages2",

            act_pos : [1,2,3,4,5,6],
            active : bool,
            ref_pos : ["hej", "hoppe", "doppe"],
        },

        ability!{
            name: to_away,

            enabled : p!([active_m] && [rp_c != "at"] && [ap_m != "at"]),
            executing : p!([active_m] && [rp_c == "at"] && [ap_m != "at"]),
            finished : p!([active_m] && [rp_c == "at"] && [ap_m == "at"]),

            *start : p!(enabled) => [ a!(rp_c = "at"), a!(ap_m = "at") ] / [a!(ap_m = "unknown")],
            finish : p!(executing) => [a!(ap_m = "at") ] / [a!(ap_m = "at")]
        }
    };

    // println!("resource: {:#?}", resource);
    //assert!(false);
}
