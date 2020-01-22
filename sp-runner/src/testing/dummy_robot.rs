use crate::runners::*;
use sp_domain::*;
use sp_runner_api::*;
use std::collections::HashMap;

#[derive(Debug, PartialEq, Clone)]
struct Domain {
    domain: Option<Vec<SPValue>>, // none=domain is boolean
}

#[derive(Debug, PartialEq, Clone)]
struct CommandTopic {
    topic: String,
    ros_type: String,
    vars: HashMap<String, Domain>,
    initial_values: HashMap<String, SPValue>, // initial states
}

#[derive(Debug, PartialEq, Clone)]
struct MeasuredTopic {
    topic: String,
    ros_type: String,
    vars: HashMap<String, Domain>,
}

#[derive(Debug, PartialEq, Clone)]
struct MTransition {
    controlled: bool,
    guard: Predicate,
    actions: Vec<Action>,
    effects: Vec<Action>,
}

#[derive(Debug, PartialEq, Clone)]
struct MAbility {
    name: String,
    predicates: HashMap<String, Predicate>,
    transitions: HashMap<String, MTransition>,
}

#[derive(Debug, PartialEq, Clone)]
struct MResource {
    name: String,
    items: Vec<ModelItem>
}

#[derive(Debug, PartialEq, Clone)]
enum ModelItem {
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

fn build_resource(r: &MResource) -> Resource {
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
        for (_name, pred) in &mut a.predicates {
            println!("original predicate: {:?}", pred);
            fix_predicate(pred, &valid_remaps);
            println!("new predicate: {:?}", pred);
        }
        for (_name, trans) in &mut a.transitions {
            println!("original guard: {:?}", trans.guard);
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

pub fn make_dummy_robot(name: &str) -> Resource {
    let mut new_resource = Vec::new();
    let mut cmd_vars = HashMap::new();
    cmd_vars.insert("ref_pos".to_string(), Domain { domain: Some(vec!["unknown".to_spvalue(), "at".to_spvalue(), "away".to_spvalue()]) });
    cmd_vars.insert("activate".to_string(), Domain { domain: None });

    let command = CommandTopic {
        topic: "Control".into(),
        ros_type: "dummy_robot_messages/msg/Control".into(),
        vars: cmd_vars,
        initial_values: HashMap::new()
    };

    let mut measured_vars = HashMap::new();
    measured_vars.insert("act_pos".to_string(), Domain { domain: Some(vec!["unknown".to_spvalue(), "at".to_spvalue(), "away".to_spvalue()]) });
    measured_vars.insert("active".to_string(), Domain { domain: None });

    let measured = MeasuredTopic {
        topic: "State".into(),
        ros_type: "dummy_robot_messages/msg/State".into(),
        vars: measured_vars,
    };

    new_resource.push(ModelItem::CommandTopic(command));
    new_resource.push(ModelItem::MeasuredTopic(measured));

    let rp_c = SPPath::from_string("ref_pos");
    let ap_m = SPPath::from_string("act_pos");
    let activate_c = SPPath::from_string("activate");
    let active_m = SPPath::from_string("active");

    let mut predicates = HashMap::new();
    predicates.insert("enabled".to_string(), pr! {{p!(active_m)} && {p!(rp_c != "at")} && {p!(ap_m != "at")}});
    predicates.insert("executing".to_string(), pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m != "at")}});
    predicates.insert("finished".to_string(), pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m == "at")}});

    let enabled = SPPath::from_string("enabled");

    let mut transitions = HashMap::new();
    let start = MTransition {
        controlled: true,
        guard: pr! {{p!(enabled)} && {p!(rp_c == "at")} && {p!(ap_m == "at")}},
        actions: vec![ a!(rp_c = "at")],
        effects: vec![ a!(ap_m = "unknown")],
    };

    transitions.insert("start".to_string(), start);

    let to_table = MAbility {
        name: "to_table".to_string(),
        predicates : predicates,
        transitions: transitions,
    };

    new_resource.push(ModelItem::MAbility(to_table));

    println!("NEW RESOURCE:");
    println!("{:#?}", new_resource);

    // build resource by going through all transitions and predicates
    // and looking for names "upwards" in the hierarchy defined by the
    // resource.

    let resource = MResource { name: name.to_string(), items: new_resource };
    let r = build_resource(&resource);

    return r;
}

#[test]
fn test_dummy() {
    let r1 = make_dummy_robot("r1");
    // println!("{:#?}", r1);

    let r1_p_a = r1.find_item("act_pos", &["r1"]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let r1_p_a = m.find_item("act_pos", &["r2"]).expect("check spelling").path();
    println!("path: {}", r1_p_a);

    let rm = make_runner_model(&m);

    println!("{:#?}", rm);
    assert!(false);
}
