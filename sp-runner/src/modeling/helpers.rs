use sp_domain::*;
use std::collections::HashMap;

#[derive(Debug, PartialEq, Clone)]
pub struct Domain {
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
pub struct MResource {
    pub name: String,
    pub items: Vec<ModelItem>
}

#[derive(Debug, PartialEq, Clone)]
pub enum ModelItem {
    CommandTopic(CommandTopic),
    MeasuredTopic(MeasuredTopic),
    MAbility(MAbility),
}


fn fix_predicate_val(pv: &mut PredicateValue, allowed_remaps: &[(SPPath, SPPath)]) {
    match pv {
        PredicateValue::SPValue(_) => {} ,
        PredicateValue::SPPath(op, _) => {
            allowed_remaps.iter().find(|(p, _np)| op == p).iter_mut().for_each(|(_p, np)| *op = np.clone());
        }
    }
}

fn fix_predicate(p: &mut Predicate, allowed_remaps: &[(SPPath,SPPath)]) {
    match p {
        Predicate::AND(v) => { v.iter_mut().for_each(|e| fix_predicate(e, allowed_remaps)); },
        Predicate::OR(v) => { v.iter_mut().for_each(|e| fix_predicate(e, allowed_remaps)); },
        Predicate::XOR(v) => { v.iter_mut().for_each(|e| fix_predicate(e, allowed_remaps)); },
        Predicate::NOT(b) => { fix_predicate(&mut *b, allowed_remaps);},
        Predicate::TRUE => {},
        Predicate::FALSE => {},
        Predicate::EQ(pv1, pv2) => {
            fix_predicate_val(pv1, allowed_remaps);
            fix_predicate_val(pv2, allowed_remaps);
        },
        Predicate::NEQ(pv1, pv2) => {
            fix_predicate_val(pv1, allowed_remaps);
            fix_predicate_val(pv2, allowed_remaps);
        },
    }
}

fn fix_action(a: &mut Action, allowed_remaps: &[(SPPath,SPPath)]) {
    allowed_remaps.iter().find(|(p, _np)| &a.var == p).iter_mut().for_each(|(_p, np)| a.var = np.clone());
    match &mut a.value {
        Compute::PredicateValue(pv) => { fix_predicate_val(pv, allowed_remaps); }
        Compute::Predicate(p) => { fix_predicate(p, allowed_remaps); }
    }
}

pub fn build_resource(r: &MResource) -> Resource {
    let mut abilities: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::MAbility(a) => Some(a.clone()), _ => None }).collect();
    let out_topics: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::CommandTopic(ct) => Some(ct.clone()), _ => None }).collect();
    let in_topics: Vec<_> = r.items.iter().flat_map(|i| match i { ModelItem::MeasuredTopic(mt) => Some(mt.clone()), _ => None }).collect();

    let mut valid_remaps = Vec::new();

    for t in &out_topics {
        println!("out_topic: {:?}", t);
        for name in t.vars.keys() {
            let path = SPPath::from_slice(&[r.name.clone(), t.topic.clone(), name.clone()]);
            let op = SPPath::from_string(name);
            valid_remaps.push((op,path));
        }
    }

    for t in &in_topics {
        println!("in_topic: {:?}", t);
        for name in t.vars.keys() {
            let path = SPPath::from_slice(&[r.name.clone(), t.topic.clone(), name.clone()]);
            let op = SPPath::from_string(name);
            valid_remaps.push((op,path));
        }
    }


    for a in &abilities {
        println!("ability: {}", a.name);

        for (name, _pred) in &a.predicates {
            let path = SPPath::from_slice(&[r.name.clone(), a.name.clone(), name.clone()]);
            // println!("predicate: {}", path);
            let op = SPPath::from_string(name);
            valid_remaps.push((op, path));
        }
    }

    println!("all valid remaps: {:?}", valid_remaps);

    // fix ability paths
    for a in &mut abilities {

        // first try to find a local remap (e.g. to our own predicates).
        let mut local_remaps = Vec::new();
        for (name, _pred) in &mut a.predicates {
            let path = SPPath::from_slice(&[r.name.clone(), a.name.clone(), name.clone()]);
            let op = SPPath::from_string(name);
            local_remaps.push((op, path));
        }


        for (_name, pred) in &mut a.predicates {
            println!("original predicate: {:?}", pred);
            fix_predicate(pred, &local_remaps);
            fix_predicate(pred, &valid_remaps);
            println!("new predicate: {:?}", pred);
        }
        for (_name, trans) in &mut a.transitions {
            println!("original guard: {:?}", trans.guard);
            fix_predicate(&mut trans.guard, &local_remaps);
            fix_predicate(&mut trans.guard, &valid_remaps);
            println!("new guard: {:?}", trans.guard);

            println!("original actions: {:?}", trans.actions);
            trans.actions.iter_mut().for_each(|a| fix_action(a, &valid_remaps));
            println!("new actions: {:?}", trans.actions);

            println!("original effects: {:?}", trans.effects);
            trans.effects.iter_mut().for_each(|e| fix_action(e, &valid_remaps));
            println!("new effects: {:?}", trans.effects);
        }
    }

    // using all our fixed stuff, build a new resource
    let mut r = Resource::new(&r.name);

    for t in &out_topics {
        let msg = Message::new(
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
                (name.clone(), var)
            }).collect());

        let topic = Topic::new(&t.topic, MessageField::Msg(msg));
        r.add_message(topic);
    }

    for t in &in_topics {
        let msg = Message::new(
            &t.ros_type,
            t.vars.iter().map(|(name, d)| {

                let is_bool = d.domain.is_none();
                let var = if is_bool {
                    MessageField::Var(Variable::new(name,
                                                    VariableType::Measured,
                                                    SPValueType::Bool,
                                                    SPValue::Bool(false), // replace with initial_values
                                                    vec![]))
                } else {
                    let dom: Vec<_> = d.domain.clone().unwrap().clone(); // fix
                    let sp_val_type = dom[0].has_type();
                    let initial = dom[0].clone(); // replace with initial_values
                    MessageField::Var(Variable::new(name,
                                                    VariableType::Measured,
                                                    sp_val_type,
                                                    initial,
                                                    dom))
                };
                (name.clone(), var)
            }).collect());

        let topic = Topic::new(&t.topic, MessageField::Msg(msg));
        r.add_message(topic);
    }

    // fix ability paths
    for a in &abilities {
        let p = a.predicates.iter().map(|(n,p)| Variable::new_predicate(n, p.clone())).collect();
        let t = a.transitions.iter().map(|(n, t)|
                                         Transition::new(n, t.guard.clone(),
                                                         t.actions.clone(), t.effects.clone(),
                                                         t.controlled)).collect();
        let a = Ability::new(&a.name, t, p);
        r.add_ability(a);
    }

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
    (private $cmd:ident $var_name:tt : [ $( $dom:tt ),+ ], $($rest:tt)*) => {{
        {
            let domain = vec![ $($dom.to_spvalue() ,)+ ];
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

    // no domain defaults to boolean variable
    (private $measured:ident msg_type: $msg_type:tt, $($rest:tt)*) => {{
        $measured.ros_type = String::from($msg_type);
        measured!(private $measured $($rest)*)
    }};

    // no domain defaults to boolean variable
    (private $measured:ident $var_name:tt : bool, $($rest:tt)*) => {{
        $measured.vars.insert(stringify!($var_name).to_string(),
                              Domain { domain: None } );
        measured!(private $measured $($rest)*)
    }};

    // same as above with no trailing comma
    (private $measured:ident $first:tt : bool) => {{
        measured!(private $measured $first : bool ,)
    }};

    // variable with domain + trailing comma
    (private $measured:ident $var_name:tt : [ $( $dom:tt ),+ ], $($rest:tt)*) => {{
        {
            let domain = vec![ $($dom.to_spvalue() ,)+ ];
            $measured.vars.insert(stringify!($var_name).to_string(),
                                  Domain { domain: Some(domain) } );
        }
        measured!(private $measured $($rest)*)
    }};

    // same as above with no trailing comma
    (private $measured:ident $first:tt : $second:tt) => {{
        measured!(private $measured $first : $second ,)
    }};

    (private $measured:expr) => {{
        ModelItem::MeasuredTopic($measured)
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
        println!("adding trans: {}", stringify!($name));

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

    println!("resource: {:#?}", resource);

    assert!(false);
}