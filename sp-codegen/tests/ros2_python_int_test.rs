use sp_codegen::*;

use sp_domain::*;
use sp_runner_api::*;
use sp_runner::*;

fn pred_path(name: &str) -> SPPath {
    TemporaryPath::from_array(TemporaryPathNS::Predicate, &[name]).to_sp()
}
fn pred_true(name: &str) -> Predicate {
    Predicate::EQ(
        PredicateValue::SPPath(pred_path(name)),
        PredicateValue::SPValue(true.to_spvalue()),
    )
}

pub fn make_dummy_door(name: &str) -> Resource {
    let command_msg = Message::new(
        "dummy_door_messages/msg/DoorControl".into(),
        vec![
            MessageField::Var(Variable::new(
                "closed_c",
                VariableType::Command,
                SPValueType::Bool,
                false.to_spvalue(),
                vec![true, false]
                    .iter()
                    .map(|v| v.to_spvalue())
                    .collect(),
            )),
            MessageField::Var(Variable::new(
                "opened_c",
                VariableType::Command,
                SPValueType::Bool,
                false.to_spvalue(),
                vec![true, false]
                    .iter()
                    .map(|v| v.to_spvalue())
                    .collect(),
            ))
        ],
    );

    let command_topic = Topic::new("door_control", MessageField::Msg(command_msg));

    let state_msg = Message::new(
        "dummy_door_messages/msg/DoorState".into(),
        vec![
            MessageField::Var(Variable::new(
                "closed_m",
                VariableType::Measured,
                SPValueType::Bool,
                false.to_spvalue(),
                vec![true, false]
                    .iter()
                    .map(|v| v.to_spvalue())
                    .collect(),
            )),
            MessageField::Var(Variable::new(
                "opened_m",
                VariableType::Command,
                SPValueType::Bool,
                false.to_spvalue(),
                vec![true, false]
                    .iter()
                    .map(|v| v.to_spvalue())
                    .collect(),
            ))
        ],
    );

    let state_topic = Topic::new("door_state", MessageField::Msg(state_msg));

    let mut r = Resource::new(name);
    r.add_message(command_topic);
    r.add_message(state_topic);

    let opened_c = r.find_item("opened_c", &[]).unwrap_local_path().to_sp();
    let closed_c = r.find_item("closed_c", &[]).unwrap_local_path().to_sp();
    let opened_m = r.find_item("opened_m", &[]).unwrap_local_path().to_sp();
    let closed_m = r.find_item("closed_m", &[]).unwrap_local_path().to_sp();

    let open = {
        let open_enabled = pr! {{p!(opened_c == false)} && {p!(opened_m == false)}};
        let open_enabled = Variable::new_predicate("open_enabled", open_enabled);

        let open_executing = pr! {{p!(opened_c == true)} && {p!(opened_m == false)}};
        let open_executing = Variable::new_predicate("open_executing", open_executing);

        let open_finishing = pr! {{p!(opened_c == true)} && {p!(opened_m == true)}};
        let open_finishing = Variable::new_predicate("open_finishing", open_finishing);

        let open_done = pr! {{p!(opened_c == false)} && {p!(opened_m == true)}};
        let open_done = Variable::new_predicate("open_done", open_done);

        let open_start = Transition::new(
            "open_start",
            pred_true("open_enabled"),
            vec![a!(opened_c = true)],
            vec![],
            true,
        );

        let open_finish = Transition::new(
            "open_finish",
            pred_true("open_executing"),
            vec![],
            vec![a!(opened_m = true)],
            false,
        );

        let open_reset = Transition::new(
            "open_reset",
            pred_true("open_finishing"),
            vec![a!(opened_c = false)],
            vec![],
            true,
        );

        Ability::new(
            "open",
            vec![open_start, open_finish, open_reset],
            vec![open_enabled, open_executing, open_finishing, open_done],
        )
    };

    let close = {
        let close_enabled = pr! {{p!(closed_c == false)} && {p!(closed_m == false)}};
        let close_enabled = Variable::new_predicate("close_enabled", close_enabled);

        let close_executing = pr! {{p!(closed_c == true)} && {p!(closed_m == false)}};
        let close_executing = Variable::new_predicate("close_executing", close_executing);

        let close_finishing = pr! {{p!(closed_c == true)} && {p!(closed_m == true)}};
        let close_finishing = Variable::new_predicate("close_finishing", close_finishing);

        let close_done = pr! {{p!(closed_c == false)} && {p!(closed_m == true)}};
        let close_done = Variable::new_predicate("close_done", close_done);

        let close_start = Transition::new(
            "close_start",
            pred_true("close_enabled"),
            vec![a!(closed_c = true)],
            vec![],
            true,
        );

        let close_finish = Transition::new(
            "close_finish",
            pred_true("close_executing"),
            vec![],
            vec![a!(closed_m = true)],
            false,
        );

        let close_reset = Transition::new(
            "close_reset",
            pred_true("close_finishing"),
            vec![a!(closed_c = false)],
            vec![],
            true,
        );

        Ability::new(
            "close",
            vec![close_start, close_finish, close_reset],
            vec![close_enabled, close_executing, close_finishing, close_done],
        )
    };

    r.add_ability(open);
    r.add_ability(close);
    return r;
}

#[test]
fn test_dummy_door() {
    let r1 = make_dummy_door("door1");
    let m = Model::new_root("dummy_robot_model", vec![SPItem::Resource(r1)]);
    let rm = make_runner_model(&m);

    let s = make_initial_state(&m);

    let mut variables = Vec::new();
    let mut predicates = Vec::new();

    for (key, value) in &s.s {
        let mut st = key.to_string();

        st.drain(..st.rfind('/').unwrap_or(st.len()));
        st.retain(|c| c != '/');

        let v: Vec<&str> = st.rsplit('_').collect();

        if v[0] == "m" || v[0] == "c" {
            variables.push(st);
        } else {
            predicates.push(st);
        };      
    }
    println!("variables: {:#?}", variables);
    println!("predicates: {:#?}", predicates);
    assert!(false);
}


#[test]
fn stuff_gen_macro() {
    generate_python_common!("macro_test_gen1");
}

#[test]
fn stuff_gen() {
    Directories::new("random_package_name_4");
    ConfigurationFile::new("random_package_name_4");
    ReadmeFile::new("random_package_name_4");
    ResourceFile::new("random_package_name_4");
    TestCopyrightFile::new("random_package_name_4");
    TestPep257File::new("random_package_name_4");
    TestFlake8File::new("random_package_name_4");
    DescriptionFile::new("random_package_name_4", "somedescr", "e@e.com", "endre");
    SetupFile::new("random_package_name_4", 
        vec!["a", "b", "c"], 
        "e@e.com", 
        "endre",
        "somedescr", 
        vec!["a", "b", "c", "asdf", "asdf2"],)
}