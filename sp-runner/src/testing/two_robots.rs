use sp_domain::*;
use crate::runners::*;
use std::collections::HashMap;

#[derive(Debug)]
pub struct TempModel {
    pub initial_state: SPState,
    pub variables: HashMap<SPPath, Variable>,
    pub runner_transitions: RunnerTransitions,
    pub opfs: Vec<IfThen>,
    pub resource: Resource,
}

fn make_robot(name: &str, upper: i32) -> Resource {
    let r = SPPath::from_array(&[name, "ref", "data"]);
    let a = SPPath::from_array(&[name, "act", "data"]);
    let activate = SPPath::from_array(&[name, "activate", "data"]);
    let activated = SPPath::from_array(&[name, "activated", "data"]);

    let r_c = Variable::new("data",
        VariableType::Command,
        SPValueType::Int32,
        0.to_spvalue(),
        (0..upper + 1).map(|v| v.to_spvalue()).collect(),
    );

    let a_m = Variable::new("data",
        VariableType::Measured,
        SPValueType::Int32,
        0.to_spvalue(),
        (0..upper + 1).map(|v| v.to_spvalue()).collect(),
    );

    let act_c = Variable::new("data",
        VariableType::Command,
        SPValueType::Bool,
        false.to_spvalue(),
        Vec::new(),
    );

    let act_m = Variable::new("data",
        VariableType::Measured,
        SPValueType::Bool,
        false.to_spvalue(),
        Vec::new(),
    );

    let to_upper = Transition::new("trans_to_upper",
        pr!{{p!{activated}} && {p!{a != upper}}},
        vec![a!(r = upper)],
        vec![a!(a = upper)],
    );
    let to_lower = Transition::new("trans_to_lower",
        pr!{{p!{activated}} && {p!{a != 0}}},
        vec![a!(r = 0)],
        vec![a!(a = 0)],
    );
    let t_activate = Transition::new("trans_activate",
        p!(!activated),
        vec![a!(activate)],
        vec![a!(activated)],
    );
    let t_deactivate = Transition::new(
        "trans_deactivate",
        p!(activated),
        vec![a!(!activate)],
        vec![a!(!activated)],
    );

    let to_upper_ab = Ability::new("to_upper", vec![], vec![to_upper], vec![]);
    let to_lower_ab = Ability::new("to_lower", vec![], vec![to_lower], vec![]);
    let activate_ab = Ability::new("activate", vec![t_activate], vec![], vec![]);
    let deactivate_ab = Ability::new("deactivate", vec![t_deactivate], vec![], vec![]);

    let opf = IfThen::new("to_upper", p!(a != upper), p!(a == upper));
    let opfs = vec!(opf);

    let s = state!(
        r => 0,
        a => 0,
        activate => false,
        activated => false
    );

    let ref_msg = Message::new_with_type("int".into(), "std_msgs/msg/Int32".into(), vec![MessageField::Var(r_c.clone())]);
    let ref_topic = Topic::new("ref", MessageField::Msg(ref_msg));

    let activate_msg = Message::new_with_type("bool".into(), "std_msgs/msg/Bool".into(), vec![MessageField::Var(act_c.clone())]);
    let activate_topic = Topic::new("activate", MessageField::Msg(activate_msg));

    let act_msg = Message::new_with_type("int".into(), "std_msgs/msg/Int32".into(), vec![MessageField::Var(a_m.clone())]);
    let act_topic = Topic::new("act", MessageField::Msg(act_msg));

    let activated_msg = Message::new_with_type("bool".into(), "std_msgs/msg/Bool".into(), vec![MessageField::Var(act_m.clone())]);
    let activated_topic = Topic::new("activated", MessageField::Msg(activated_msg));

    let mut r = Resource::new(name);
    r.add_message(ref_topic);
    r.add_message(activate_topic);
    r.add_message(act_topic);
    r.add_message(activated_topic);

    r.add_ability(to_upper_ab);
    r.add_ability(to_lower_ab);
    r.add_ability(activate_ab);
    r.add_ability(deactivate_ab);

    return r;
}

fn make_runner_model(model: &Model) -> RunnerModel {

    let items = model.items();

    // find "ab" transitions from resources
    let resources: Vec<&Resource> = items.iter().flat_map(|i| match i {
        SPItem::Resource(r) => Some(r),
        _ => None }).collect();
    let ab_ctrl_ts: Vec<&Transition> = resources.iter().
        flat_map(|r| r.abilities().iter().flat_map(|a|a.controlled.iter())).collect();
    let ab_unctrl_ts: Vec<&Transition> = resources.iter().
        flat_map(|r| r.abilities().iter().flat_map(|a|a.uncontrolled.iter())).collect();

    // TODO: add global transitions. currently impossible.

    // add global op transitions (all automatic for now).
    let global_ops: Vec<&Operation> = items.iter().flat_map(|i| match i {
        SPItem::Operation(o) => Some(o),
        _ => None }).collect();

    // todo: finish operations.

    println!("{:?}", resources);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions::default(),
        ab_transitions: RunnerTransitions {
            ctrl: ab_ctrl_ts.into_iter().cloned().collect(),
            un_ctrl: ab_unctrl_ts.into_iter().cloned().collect(),
        },
        plans: RunnerPlans::default(),
        state_predicates: Vec::new(),
        goals: Vec::new(),
        invariants: Vec::new(),
        model: model.clone(), // TODO: borrow?
    };

    return rm;
}

fn make_initial_state(model: &Model) -> SPState {
    let items = model.items();

    // find all variables in resources
    let resources: Vec<&Resource> = items.iter().flat_map(|i| match i {
        SPItem::Resource(r) => Some(r),
        _ => None }).collect();

    let mut s = SPState::default();

    for r in &resources {
        s.extend(r.make_initial_state());
    }

    return s;
}

pub fn one_robot() -> (RunnerModel, SPState) {
    let r1 = make_robot("r1", 10);
    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    println!("{}", s.external());

    (rm, s)
}

pub fn two_robots() -> (RunnerModel, SPState) {
    let r1 = make_robot("r1", 10);
    let r2 = make_robot("r2", 10);

    let m = Model::new_root("two_robot_model", vec![SPItem::Resource(r1), SPItem::Resource(r2)]);
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}

// pub fn two_robots() -> (RunnerModel, SPState, Vec<Resource>) {
//     let r1 = make_robot("r1", 10);
//     let r2 = make_robot("r2", 10);

//     let mut s = r1.initial_state;
//     s.extend(r2.initial_state);
//     let mut tr = r1.runner_transitions;
//     tr.extend(r2.runner_transitions);
//     let mut vars = r1.variables;
//     vars.extend(r2.variables.into_iter());
//     let mut opfs = r1.opfs;
//     opfs.extend(r2.opfs.into_iter());

//     let r = vec![r1.resource, r2.resource];

//     let rm = RunnerModel {
//         op_transitions: RunnerTransitions::default(),
//         ab_transitions: tr,
//         plans: RunnerPlans::default(),
//         state_predicates: Vec::new(),
//         goals: opfs,
//         invariants: Vec::new(),
//         vars: vars,
//     };

//     (rm, s, r)
// }
