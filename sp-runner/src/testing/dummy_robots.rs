use sp_domain::*;
use sp_runner_api::*;

fn make_dummy_robot(name: &str) -> Resource {
    let command_msg = Message::new_with_type(
        "dr_c".into(),
        "dummy_robot_messages/msg/Control".into(),
        vec![
            MessageField::Var(Variable::new(
                "ref_pos",
                VariableType::Command,
                SPValueType::String,
                "unknown".to_spvalue(),
                vec!["unknown", "at_table", "at_away"].iter()
                    .map(|v| v.to_spvalue()).collect(),
            )),
            MessageField::Var(Variable::new(
                "activate",
                VariableType::Command,
                SPValueType::Bool,
                false.to_spvalue(),
                Vec::new(),
            ))
        ],
    );
    let topic = format!("{}/Control", name);
    let command_topic = Topic::new(&topic, MessageField::Msg(command_msg));

    let state_msg = Message::new_with_type(
        "dr_m".into(),
        "dummy_robot_messages/msg/State".into(),
        vec![
            MessageField::Var(Variable::new(
                "act_pos",
                VariableType::Measured,
                SPValueType::String,
                "unknown".to_spvalue(),
                vec!["unknown", "at_table", "at_away"].iter()
                    .map(|v| v.to_spvalue()).collect(),
            )),
            MessageField::Var(Variable::new(
                "active",
                VariableType::Measured,
                SPValueType::Bool,
                false.to_spvalue(),
                Vec::new(),
            ))
        ],
    );
    let topic = format!("{}/State", name);
    let state_topic = Topic::new(&topic, MessageField::Msg(state_msg));

    let mut r = Resource::new(name);
    r.add_message(command_topic);
    r.add_message(state_topic);

    let rp = r.find_item("ref_pos", &[]).unwrap_local_path().to_sp();
    let ap = r.find_item("act_pos", &[]).unwrap_local_path().to_sp();
    let activate = r.find_item("activate", &[]).unwrap_local_path().to_sp();
    let active = r.find_item("active", &[]).unwrap_local_path().to_sp();

    let to_table_start = Transition::new(
        "to_table_start",
        Predicate::AND(vec![Predicate::EQ(PredicateValue::SPPath(active.clone()),
                                          PredicateValue::SPValue(true.to_spvalue())),
                            Predicate::NEQ(PredicateValue::SPPath(ap.clone()),
                                           PredicateValue::SPValue("at_table".to_spvalue()))]),
        vec![Action { var: rp.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue("at_table".to_spvalue())) }],
        vec![],
        true
    );

    let to_table_fini = Transition::new(
        "to_table_fini",
        Predicate::AND(vec![Predicate::EQ(PredicateValue::SPPath(active.clone()),
                                          PredicateValue::SPValue(true.to_spvalue())),
                            Predicate::NEQ(PredicateValue::SPPath(ap.clone()),
                                           PredicateValue::SPValue("at_table".to_spvalue())),
                            Predicate::EQ(PredicateValue::SPPath(rp.clone()),
                                          PredicateValue::SPValue("at_table".to_spvalue()))]),
        vec![],
        vec![Action { var: ap, value:
                      Compute::PredicateValue(PredicateValue::SPValue("at_table".to_spvalue())) }],
        false
    );

    let to_table = Ability::new("to_table", vec![to_table_start, to_table_fini], vec![]);

    let activate_start = Transition::new(
        "activate_start",
        Predicate::EQ(PredicateValue::SPPath(active.clone()),
                      PredicateValue::SPValue(false.to_spvalue())),
        vec![Action { var: activate.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())) }],
        vec![],
        true
    );

    let activate_fini = Transition::new(
        "activate_fini",
        Predicate::AND(vec![Predicate::EQ(PredicateValue::SPPath(active.clone()),
                                          PredicateValue::SPValue(false.to_spvalue())),
                            Predicate::EQ(PredicateValue::SPPath(activate.clone()),
                                          PredicateValue::SPValue(true.to_spvalue()))]),
        vec![],
        vec![Action { var: active.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())) }],
        false
    );

    let activate = Ability::new("activate", vec![activate_start, activate_fini], vec![]);

    r.add_ability(to_table);
    r.add_ability(activate);

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

    // println!("{:?}", global_ops);

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

pub fn one_dummy_robot() -> (RunnerModel, SPState) {
    let r1 = make_dummy_robot("r1");
    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}

pub fn two_dummy_robots() -> (RunnerModel, SPState) {
    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1")));
    m.add_item(SPItem::Resource(make_dummy_robot("r2")));

    // Make some global stuff
    // let r1_p_a = m.find_item("data", &["r1", "act"]).unwrap_global_path().to_sp();
    // let r2_p_a = m.find_item("data", &["r2", "act"]).unwrap_global_path().to_sp();
    // let r1_op1it = IfThen::new("goal", p!(r1_p_a != 10), p!(r1_p_a == 10));
    // let op = Operation::new("some_robot_at_upper", Vec::new(), Vec::new(), Vec::new(), Vec::new(),
    //                         Some(r1_op1it), None);



    // m.add_item(SPItem::Operation(op));

    // Make it runnable
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}
