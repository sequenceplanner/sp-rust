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
        PredicateValue::SPValue(true.to_spvalue()),
    )
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
        let enabled = pr! {{p!(active_m == true)} && {p!(rp_c != "at")} && {p!(ap_m != "at")}};
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing = pr! {{p!(active_m == true)} && {p!(rp_c == "at")} && {p!(ap_m != "at")}};
        let executing = Variable::new_predicate("executing", executing);

        let finished = pr! {{p!(active_m == true)} && {p!(rp_c == "at")} && {p!(ap_m == "at")}};
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![a!(rp_c = "at")],
            vec![],
            true,
        );

        let finish = Transition::new(
            "finish",
            pred_true("executing"),
            vec![],
            vec![a!(ap_m = "at")],
            false,
        );

        Ability::new(
            "to_table",
            vec![start, finish],
            vec![enabled, executing, finished],
        )
    };

    let to_away = {
        let enabled = pr! {{p!(active_m == true)} && {p!(rp_c != "away")} && {p!(ap_m != "away")}};
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing =
            pr! {{p!(active_m == true)} && {p!(rp_c == "away")} && {p!(ap_m != "away")}};
        let executing = Variable::new_predicate("executing", executing);

        let finished = pr! {{p!(active_m == true)} && {p!(rp_c == "away")} && {p!(ap_m == "away")}};
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![a!(rp_c = "away")],
            vec![],
            true,
        );

        let finish = Transition::new(
            "finish",
            pred_true("executing"),
            vec![],
            vec![a!(ap_m = "away")],
            false,
        );

        Ability::new(
            "to_away",
            vec![start, finish],
            vec![enabled, executing, finished],
        )
    };

    let activate = {
        let enabled = pr! {{p!(activate_c == false)} && {p!(active_m == false)}};
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing = pr! {{p!(activate_c == true)} && {p!(active_m == false)}};
        let executing = Variable::new_predicate("executing", executing);

        let finished = pr! {{p!(activate_c == true)} && {p!(active_m == true)}};
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![a!(activate_c = true)],
            vec![],
            true,
        );

        let finish = Transition::new(
            "finish",
            pred_true("executing"),
            vec![],
            vec![a!(active_m = true)],
            false,
        );

        Ability::new(
            "activate",
            vec![start, finish],
            vec![enabled, executing, finished],
        )
    };

    let deactivate = {
        let enabled = pr! {{p!(activate_c == true)} && {p!(active_m == true)}};
        let enabled = Variable::new_predicate("enabled", enabled);

        let executing = pr! {{p!(activate_c == false)} && {p!(active_m == true)}};
        let executing = Variable::new_predicate("executing", executing);

        let finished = pr! {{p!(activate_c == false)} && {p!(active_m == false)}};
        let finished = Variable::new_predicate("finished", finished);

        let start = Transition::new(
            "start",
            pred_true("enabled"),
            vec![a!(activate_c = false)],
            vec![],
            true,
        );

        let finish = Transition::new(
            "finish",
            pred_true("executing"),
            vec![],
            vec![a!(active_m = false)],
            false,
        );

        Ability::new(
            "deactivate",
            vec![start, finish],
            vec![enabled, executing, finished],
        )
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

fn add_op(m: &mut Model, name: &str, pre: Predicate, post: Predicate) -> SPPath {
    let op_state = Variable::new(
        name,
        VariableType::Estimated,
        SPValueType::String,
        "i".to_spvalue(),
        vec!["i", "e", "f"].iter().map(|v| v.to_spvalue()).collect(),
    );
    let op_state = m
        .add_item(SPItem::Variable(op_state))
        .global_path()
        .as_ref()
        .unwrap()
        .to_sp();

    let op_start = Transition::new(
        "start",
        Predicate::AND(vec![p!(op_state == "i"), pre.clone()]),
        vec![a!(op_state = "e")],
        vec![],
        true,
    );
    let op_finish = Transition::new(
        "finish",
        Predicate::AND(vec![p!(op_state == "e"), post.clone()]),
        vec![a!(op_state = "f")],
        vec![],
        false,
    );
    let op_goal = IfThen::new("goal", p!(op_state == "e"), post.clone());

    let op = Operation::new(
        name,
        vec![op_start],
        vec![op_finish],
        Some(op_goal),
        None,
    );

    m.add_item(SPItem::Operation(op)).global_path().as_ref().unwrap().to_sp()
}

pub fn two_dummy_robots() -> (RunnerModel, SPState) {
    // Make model
    let mut m = Model::new_root("dummy_robot_model", Vec::new());

    // Make resoureces
    m.add_item(SPItem::Resource(make_dummy_robot("r1")));
    m.add_item(SPItem::Resource(make_dummy_robot("r2")));

    // Make some global stuff
    let r1_p_a = m.find_item("act_pos", &["r1"]).unwrap_global_path().to_sp();
    let r2_p_a = m.find_item("act_pos", &["r2"]).unwrap_global_path().to_sp();
    let r1_to_at = add_op(&mut m, "r1_to_at", p!(r1_p_a != "at"), p!(r1_p_a == "at"));
    let r1_to_away = add_op(&mut m, "r1_to_away",
                            pr!{{p!(r1_p_a != "away")} && {p!(r1_to_at == "f")}},
                            p!(r1_p_a == "away"));

    let r2_to_at = add_op(&mut m, "r2_to_at",
                          pr!{{p!(r2_p_a != "at")} && {p!(r1_to_away == "f")}},
                          p!(r2_p_a == "at"));

    let r1_to_away = add_op(&mut m, "r2_to_away",
                            pr!{{p!(r2_p_a != "away")} && {p!(r2_to_at == "f")}},
                            p!(r2_p_a == "away"));

    // Make it runnable
    let s = make_initial_state(&m);
    let rm = make_runner_model(&m);

    (rm, s)
}

#[test]
fn test_two_dummy_robots() {
    let (rm, s) = two_dummy_robots();

    println!("{:#?}", rm);
    println!("=================================");
    println!("{:#?}", s);
    assert!(false);
}
