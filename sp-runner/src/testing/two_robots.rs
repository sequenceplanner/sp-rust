use crate::runners::*;
use sp_domain::*;
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
    let r_c = Variable::new(
        "data",
        VariableType::Command,
        SPValueType::Int32,
        0.to_spvalue(),
        (0..upper + 1).map(|v| v.to_spvalue()).collect(),
    );

    let a_m = Variable::new(
        "data",
        VariableType::Measured,
        SPValueType::Int32,
        0.to_spvalue(),
        (0..upper + 1).map(|v| v.to_spvalue()).collect(),
    );

    let act_c = Variable::new(
        "data",
        VariableType::Command,
        SPValueType::Bool,
        false.to_spvalue(),
        Vec::new(),
    );

    let act_m = Variable::new(
        "data",
        VariableType::Measured,
        SPValueType::Bool,
        false.to_spvalue(),
        Vec::new(),
    );

    let ref_msg = Message::new_with_type(
        "int".into(),
        "std_msgs/msg/Int32".into(),
        vec![MessageField::Var(r_c)],
    );
    let ref_topic = Topic::new("ref", MessageField::Msg(ref_msg));

    let activate_msg = Message::new_with_type(
        "bool".into(),
        "std_msgs/msg/Bool".into(),
        vec![MessageField::Var(act_c)],
    );
    let activate_topic = Topic::new("activate", MessageField::Msg(activate_msg));

    let act_msg = Message::new_with_type(
        "int".into(),
        "std_msgs/msg/Int32".into(),
        vec![MessageField::Var(a_m)],
    );
    let act_topic = Topic::new("act", MessageField::Msg(act_msg));

    let activated_msg = Message::new_with_type(
        "bool".into(),
        "std_msgs/msg/Bool".into(),
        vec![MessageField::Var(act_m)],
    );
    let activated_topic = Topic::new("activated", MessageField::Msg(activated_msg));

    let mut r = Resource::new(name);
    r.add_message(ref_topic);
    r.add_message(activate_topic);
    r.add_message(act_topic);
    r.add_message(activated_topic);

    let p_r = r.find_item("data", &["ref"]).unwrap_local_path().to_sp();
    let p_a = r.find_item("data", &["act"]).unwrap_local_path().to_sp();
    let p_activate = r
        .find_item("data", &["activate"])
        .unwrap_local_path()
        .to_sp();
    let p_activated = r
        .find_item("data", &["activated"])
        .unwrap_local_path()
        .to_sp();

    let to_upper = Transition::new(
        "trans_to_upper",
        pr! {{p!{p_activated}} && {p!{p_a != upper}}},
        vec![a!(p_r = upper)],
        vec![a!(p_a = upper)],
        false
    );
    let to_lower = Transition::new(
        "trans_to_lower",
        pr! {{p!{p_activated}} && {p!{p_a != 0}}},
        vec![a!(p_r = 0)],
        vec![a!(p_a = 0)],
        false
    );
    let t_activate = Transition::new(
        "trans_activate",
        p!(!p_activated),
        vec![a!(p_activate)],
        vec![a!(p_activated)],
        true
    );
    let t_deactivate = Transition::new(
        "trans_deactivate",
        p!(p_activated),
        vec![a!(!p_activate)],
        vec![a!(!p_activated)],
        true
    );

    let to_upper_ab = Ability::new("to_upper", vec![to_upper], vec![]);
    let to_lower_ab = Ability::new("to_lower", vec![to_lower], vec![]);
    let activate_ab = Ability::new("activate", vec![t_activate], vec![]);
    let deactivate_ab = Ability::new("deactivate", vec![t_deactivate], vec![]);

    r.add_ability(to_upper_ab);
    r.add_ability(to_lower_ab);
    r.add_ability(activate_ab);
    r.add_ability(deactivate_ab);

    return r;
}

fn make_runner_model(model: &Model) -> RunnerModel {
    let items = model.items();

    // find "ab" transitions from resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let trans: Vec<_> = resources.iter().flat_map(|r| r.make_global_transitions()).collect();
    let ctrl = trans.iter().filter(|t|t.controlled()).cloned().collect();
    let unctrl = trans.iter().filter(|t|!t.controlled()).cloned().collect();

    // TODO: add global transitions.

    // add global op transitions (all automatic for now).
    let global_ops: Vec<&Operation> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Operation(o) => Some(o),
            _ => None,
        })
        .collect();

    let global_goals: Vec<IfThen> = global_ops.iter().flat_map(|o|o.goal.as_ref()).cloned().collect();

    // todo: finish operations.
    println!("{:?}", global_ops);

    // println!("{:?}", resources);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions::default(),
        ab_transitions: RunnerTransitions {
            ctrl: ctrl,
            un_ctrl: unctrl,
        },
        plans: RunnerPlans::default(),
        state_predicates: Vec::new(),
        goals: global_goals,
        invariants: Vec::new(),
        model: model.clone(), // TODO: borrow?
    };

    return rm;
}

fn make_initial_state(model: &Model) -> SPState {
    let items = model.items();

    // find all variables in resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

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
    // Make model
    let mut m = Model::new_root("two_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_robot("r1", 10)));
    m.add_item(SPItem::Resource(make_robot("r2", 10)));

    // Make some global stuff
    let r1_p_a = m.find_item("data", &["r1", "act"]).unwrap_global_path().to_sp();
    let r2_p_a = m.find_item("data", &["r2", "act"]).unwrap_global_path().to_sp();
    let r1_op1it = IfThen::new("goal", p!(r1_p_a != 10), p!(r1_p_a == 10));
    let op = Operation::new("some_robot_at_upper", Vec::new(), Vec::new(), Vec::new(), Vec::new(),
                            Some(r1_op1it), None);



    m.add_item(SPItem::Operation(op));

    // Make it runnable
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}
