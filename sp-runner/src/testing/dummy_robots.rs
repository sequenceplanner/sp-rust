use crate::runners::*;
use sp_domain::*;
use sp_runner_api::*;


// helpers
fn pred_path(name: &str) -> SPPath {
    TemporaryPath::from_array(TemporaryPathNS::Predicate, &[name]).to_sp()
}
fn pred_true(name: &str) -> Predicate {
    Predicate::EQ(
        PredicateValue::SPPath(pred_path(name)),
        PredicateValue::SPValue(true.to_spvalue()))
}

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
                vec!["unknown", "at", "away"]
                    .iter()
                    .map(|v| v.to_spvalue())
                    .collect(),
            )),
            MessageField::Var(Variable::new(
                "activate",
                VariableType::Command,
                SPValueType::Bool,
                false.to_spvalue(),
                Vec::new(),
            )),
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
                vec!["unknown", "at", "away"]
                    .iter()
                    .map(|v| v.to_spvalue())
                    .collect(),
            )),
            MessageField::Var(Variable::new(
                "active",
                VariableType::Measured,
                SPValueType::Bool,
                false.to_spvalue(),
                Vec::new(),
            )),
        ],
    );
    let state_topic = Topic::new("State", MessageField::Msg(state_msg));

    let mut r = Resource::new(name);
    r.add_message(command_topic);
    r.add_message(state_topic);

    let rp_c = r.find_item("ref_pos", &[]).unwrap_local_path().to_sp();
    let ap_m = r.find_item("act_pos", &[]).unwrap_local_path().to_sp();
    let activate_c = r.find_item("activate", &[]).unwrap_local_path().to_sp();
    let active_m = r.find_item("active", &[]).unwrap_local_path().to_sp();

    let to_table = {
        let enabled = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::NEQ(
                PredicateValue::SPPath(rp_c.clone()),
                PredicateValue::SPValue("at".to_spvalue()),
            ),
            Predicate::NEQ(
                PredicateValue::SPPath(ap_m.clone()),
                PredicateValue::SPValue("at".to_spvalue()),
            ),
        ]);
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(rp_c.clone()),
                PredicateValue::SPValue("at".to_spvalue()),
            ),
            Predicate::NEQ(
                PredicateValue::SPPath(ap_m.clone()),
                PredicateValue::SPValue("at".to_spvalue()),
            ),
        ]);
        let executing = Variable::new_predicate("executing", executing);

        let finished = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(rp_c.clone()),
                PredicateValue::SPValue("at".to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(ap_m.clone()),
                PredicateValue::SPValue("at".to_spvalue()),
            ),
        ]);
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![Action { var: rp_c.clone(), value:
                          Compute::PredicateValue(PredicateValue::SPValue("at".to_spvalue())) }],
            vec![],
            true
        );

        let finish = Transition::new(
            "finish",
            pred_true("exeucting"),
            vec![],
            vec![Action {
                var: ap_m.clone(),
                value: Compute::PredicateValue(PredicateValue::SPValue("at".to_spvalue())),
            }],
            false,
        );

        Ability::new("to_table", vec![start, finish], vec![enabled, executing, finished])
    };

    let to_away = {
        let enabled = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::NEQ(
                PredicateValue::SPPath(rp_c.clone()),
                PredicateValue::SPValue("away".to_spvalue()),
            ),
            Predicate::NEQ(
                PredicateValue::SPPath(ap_m.clone()),
                PredicateValue::SPValue("away".to_spvalue()),
            ),
        ]);
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(rp_c.clone()),
                PredicateValue::SPValue("away".to_spvalue()),
            ),
            Predicate::NEQ(
                PredicateValue::SPPath(ap_m.clone()),
                PredicateValue::SPValue("away".to_spvalue()),
            ),
        ]);
        let executing = Variable::new_predicate("executing", executing);

        let finished = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(rp_c.clone()),
                PredicateValue::SPValue("away".to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(ap_m.clone()),
                PredicateValue::SPValue("away".to_spvalue()),
            ),
        ]);
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![Action { var: rp_c.clone(), value:
                          Compute::PredicateValue(PredicateValue::SPValue("away".to_spvalue())) }],
            vec![],
            true
        );

        let finish = Transition::new(
            "finish",
            pred_true("exeucting"),
            vec![],
            vec![Action {
                var: ap_m.clone(),
                value: Compute::PredicateValue(PredicateValue::SPValue("away".to_spvalue())),
            }],
            false,
        );

        Ability::new("to_away", vec![start, finish], vec![enabled, executing, finished])
    };

    let activate = {
        let enabled = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(activate_c.clone()),
                PredicateValue::SPValue(false.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(false.to_spvalue()),
            ),
        ]);
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(activate_c.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(false.to_spvalue()),
            ),
        ]);
        let executing = Variable::new_predicate("executing", executing);

        let finished = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(activate_c.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
        ]);
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![Action { var: activate_c.clone(), value:
                          Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())) }],
            vec![],
            true
        );

        let finish = Transition::new(
            "finish",
            pred_true("exeucting"),
            vec![],
            vec![Action {
                var: active_m.clone(),
                value: Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())),
            }],
            false,
        );

        Ability::new("activate", vec![start, finish], vec![enabled, executing, finished])
    };

    let deactivate = {
        let enabled = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(activate_c.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
        ]);
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(activate_c.clone()),
                PredicateValue::SPValue(false.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(true.to_spvalue()),
            ),
        ]);
        let executing = Variable::new_predicate("executing", executing);

        let finished = Predicate::AND(vec![
            Predicate::EQ(
                PredicateValue::SPPath(activate_c.clone()),
                PredicateValue::SPValue(false.to_spvalue()),
            ),
            Predicate::EQ(
                PredicateValue::SPPath(active_m.clone()),
                PredicateValue::SPValue(false.to_spvalue()),
            ),
        ]);
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![Action { var: activate_c.clone(), value:
                          Compute::PredicateValue(PredicateValue::SPValue(false.to_spvalue())) }],
            vec![],
            true
        );

        let finish = Transition::new(
            "finish",
            pred_true("exeucting"),
            vec![],
            vec![Action {
                var: active_m.clone(),
                value: Compute::PredicateValue(PredicateValue::SPValue(false.to_spvalue())),
            }],
            false,
        );

        Ability::new("deactivate", vec![start, finish], vec![enabled, executing, finished])
    };

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
    let rm = make_runner_model(&m);

    println!("{:#?}", rm);
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
    let r1_op1it = IfThen::new("goal", p!(r1_p_a != "at"), p!(r1_p_a == "at"));
    let op = Operation::new(
        "t1_to_table",
        Vec::new(),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        Some(r1_op1it),
        None,
    );

    m.add_item(SPItem::Operation(op));

    // Make it runnable
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}
