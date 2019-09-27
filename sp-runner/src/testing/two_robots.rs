use sp_domain::*;
use crate::runners::*;
use std::collections::HashMap;

#[derive(Debug)]
pub struct TempModel {
    pub initial_state: SPState,
    pub variables: HashMap<SPPath, Variable>,
    pub runner_transitions: RunnerTransitions,
    pub opfs: Vec<OperationFunction>,
    pub resource: Resource,
}

fn make_robot(name: &str, upper: i32) -> TempModel {
    let r = SPPath::from_array(&[name, "ref", "data"]);
    let a = SPPath::from_array(&[name, "act", "data"]);
    let activate = SPPath::from_array(&[name, "activate", "data"]);
    let activated = SPPath::from_array(&[name, "activated", "data"]);

    let r_c = Variable::Command(VariableData {
        type_: SPValueType::Int32,
        initial_value: None,
        domain: (0..upper + 1).map(|v| v.to_spvalue()).collect(),
    });

    let a_m = Variable::Measured(VariableData {
        type_: SPValueType::Int32,
        initial_value: None,
        domain: (0..upper + 1).map(|v| v.to_spvalue()).collect(),
    });

    let act_c = Variable::Command(VariableData {
        type_: SPValueType::Bool,
        initial_value: None,
        domain: Vec::new(),
    });

    let act_m = Variable::Measured(VariableData {
        type_: SPValueType::Bool,
        initial_value: None,
        domain: Vec::new(),
    });

    let to_upper = Transition::new(
        SPPath::from_array(&[name, "trans", "to_upper"]),
        pr!{{p!{activated}} && {p!{a != upper}}},
        vec![a!(r = upper)],
        vec![a!(a = upper)],
    );
    let to_lower = Transition::new(
        SPPath::from_array(&[name, "trans", "to_lower"]),
        pr!{{p!{activated}} && {p!{a != 0}}},
        vec![a!(r = 0)],
        vec![a!(a = 0)],
    );
    let t_activate = Transition::new(
        SPPath::from_array(&[name, "trans", "activate"]),
        p!(!activated),
        vec![a!(activate)],
        vec![a!(activated)],
    );
    let t_deactivate = Transition::new(
        SPPath::from_array(&[name, "trans", "deactivate"]),
        p!(activated),
        vec![a!(!activate)],
        vec![a!(!activated)],
    );


    let opf = OperationFunction::Goal(p!(a != upper), p!(a == upper));
    let opfs = vec!(opf);

    let s = state!(
        r => 0,
        a => 0,
        activate => false,
        activated => false
    );

    let vars = hashmap![
        r => r_c.clone(),
        a => a_m.clone(),
        activate => act_c.clone(),
        activated => act_m.clone()
    ];

    // ros comm stuff
    let comm = ResourceComm::RosComm(RosComm {
        publishers: vec![RosPublisherDefinition {
            topic: format!("/{}/ref", name),
            qos: "".into(),
            definition: RosMsgDefinition::Message(
                "std_msgs/msg/Int32".into(),
                hashmap![
                    "data".into() => RosMsgDefinition::Field(r_c.clone())],
            ),
        },
                         RosPublisherDefinition {
                             topic: format!("/{}/activate", name),
                             qos: "".into(),
                             definition: RosMsgDefinition::Message(
                                 "std_msgs/msg/Bool".into(),
                                 hashmap![
                                     "data".into() => RosMsgDefinition::Field(act_c.clone())],
                             ),
                         }
        ],
        subscribers: vec![
            RosSubscriberDefinition {
                topic: format!("/{}/act", name),
                definition: RosMsgDefinition::Message(
                    "std_msgs/msg/Int32".into(),
                    hashmap![
                        "data".into() => RosMsgDefinition::Field(a_m.clone())
                    ],
                ),
            },
            RosSubscriberDefinition {
                topic: format!("/{}/activated", name),
                definition: RosMsgDefinition::Message(
                    "std_msgs/msg/Bool".into(),
                    hashmap![
                        "data".into() => RosMsgDefinition::Field(act_m.clone())
                    ],
                ),
            },
        ],
    });

    let r = Resource {
        abilities: Vec::new(),
        parameters: Vec::new(),
        comm: comm,
    };

    TempModel {
        initial_state: s,
        variables: vars,
        runner_transitions: RunnerTransitions {
            ctrl: vec![t_activate, t_deactivate],
            un_ctrl: vec![to_lower, to_upper],
        },
        opfs: opfs,
        resource: r,
    }
}


pub fn one_robot(name: &str, upper: i32) -> (RunnerModel, SPState, Vec<Resource>) {
    let r1 = make_robot(name, upper);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions::default(),
        ab_transitions: r1.runner_transitions,
        plans: RunnerPlans::default(),
        state_functions: vec![],
        op_functions: r1.opfs,
        vars: r1.variables,
    };

    (rm, r1.initial_state, vec!(r1.resource))
}

pub fn two_robots() -> (RunnerModel, SPState, Vec<Resource>) {
    let r1 = make_robot("r1", 10);
    let r2 = make_robot("r2", 10);

    let mut s = r1.initial_state;
    s.extend(r2.initial_state);
    let mut tr = r1.runner_transitions;
    tr.extend(r2.runner_transitions);
    let mut vars = r1.variables;
    vars.extend(r2.variables.into_iter());
    let mut opfs = r1.opfs;
    opfs.extend(r2.opfs.into_iter());

    let r = vec![r1.resource, r2.resource];

    let rm = RunnerModel {
        op_transitions: RunnerTransitions::default(),
        ab_transitions: tr,
        plans: RunnerPlans::default(),
        state_functions: vec![],
        op_functions: opfs,
        vars: vars,
    };

    (rm, s, r)
}
