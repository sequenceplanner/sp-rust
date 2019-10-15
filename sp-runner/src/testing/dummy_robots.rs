use crate::runners::*;
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
    let command_topic = Topic::new("Control", MessageField::Msg(command_msg));

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
    let state_topic = Topic::new("State", MessageField::Msg(state_msg));

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
        vec![Action { var: ap.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue("at_table".to_spvalue())) }],
        false
    );

    let to_away_start = Transition::new(
        "to_away_start",
        Predicate::AND(vec![Predicate::EQ(PredicateValue::SPPath(active.clone()),
                                          PredicateValue::SPValue(true.to_spvalue())),
                            Predicate::NEQ(PredicateValue::SPPath(ap.clone()),
                                           PredicateValue::SPValue("at_away".to_spvalue()))]),
        vec![Action { var: rp.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue("at_away".to_spvalue())) }],
        vec![],
        true
    );

    let to_away_fini = Transition::new(
        "to_away_fini",
        Predicate::AND(vec![Predicate::EQ(PredicateValue::SPPath(active.clone()),
                                          PredicateValue::SPValue(true.to_spvalue())),
                            Predicate::NEQ(PredicateValue::SPPath(ap.clone()),
                                           PredicateValue::SPValue("at_away".to_spvalue())),
                            Predicate::EQ(PredicateValue::SPPath(rp.clone()),
                                          PredicateValue::SPValue("at_away".to_spvalue()))]),
        vec![],
        vec![Action { var: ap.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue("at_away".to_spvalue())) }],
        false
    );

    // let activate_start = Transition::new(
    //     "activate_start",
    //     Predicate::EQ(PredicateValue::SPPath(active.clone()),
    //                   PredicateValue::SPValue(false.to_spvalue())),
    //     vec![Action { var: activate.clone(), value:
    //                   Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())) }],
    //     vec![],
    //     true
    // );

    let activate_start = Transition::new(
        "activate_start",
        Predicate::EQ(PredicateValue::SPPath(SPPath::from(&["enabled".into()])),
                      PredicateValue::SPValue(true.to_spvalue())),
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

    let deactivate_start = Transition::new(
        "deactivate_start",
        Predicate::EQ(PredicateValue::SPPath(active.clone()),
                      PredicateValue::SPValue(true.to_spvalue())),
        vec![Action { var: activate.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue(false.to_spvalue())) }],
        vec![],
        true
    );

    let deactivate_fini = Transition::new(
        "deactivate_fini",
        Predicate::AND(vec![Predicate::EQ(PredicateValue::SPPath(active.clone()),
                                          PredicateValue::SPValue(true.to_spvalue())),
                            Predicate::EQ(PredicateValue::SPPath(activate.clone()),
                                          PredicateValue::SPValue(false.to_spvalue()))]),
        vec![],
        vec![Action { var: active.clone(), value:
                      Compute::PredicateValue(PredicateValue::SPValue(false.to_spvalue())) }],
        false
    );

    let to_table = Ability::new("to_table", vec![to_table_start, to_table_fini], vec![]);
    let to_away = Ability::new("to_away", vec![to_away_start, to_away_fini], vec![]);

    let activate_enabled = Predicate::EQ(PredicateValue::SPPath(active.clone()), PredicateValue::SPValue(false.to_spvalue()));
    let activate_enabled = Variable::new_predicate("enabled", activate_enabled);

    let activate = Ability::new("activate", vec![activate_start, activate_fini], vec![activate_enabled]);
    let deactivate = Ability::new("deactivate", vec![deactivate_start, deactivate_fini], vec![]);


    r.add_ability(to_table);
    r.add_ability(to_away);
    r.add_ability(activate);
    r.add_ability(deactivate);

    return r;
}

#[test]
fn test_dummy() {
    let r1 = make_dummy_robot("r1");
    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);

    println!("{:#?}", m);
    assert!(false);
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
    let r1_p_a = m.find_item("act_pos", &["r1"]).unwrap_global_path().to_sp();
    let r1_op1it = IfThen::new("goal", p!(r1_p_a != "at_table"), p!(r1_p_a == "at_table"));
    let op = Operation::new("t1_to_table", Vec::new(), Vec::new(), Vec::new(), Vec::new(),
                            Some(r1_op1it), None);

    m.add_item(SPItem::Operation(op));

    // Make it runnable
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}
