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

pub fn make_dummy_door(name: &str) -> (Resource, Vec<String>, Vec<String>) {
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

    let mut py_emul_predicates: Vec<String> = Vec::new();
    let mut py_emul_transitions: Vec<String> = Vec::new();

    let open = {

        let open_enabled = pr! {{p!(opened_c == false)} && {p!(opened_m == false)}};
        let open_enabled = Variable::new_predicate("open_enabled", open_enabled);
        py_emul_predicates.push("self.open_enabled = not self.opened_c and not self.opened_m".to_string());

        let open_executing = pr! {{p!(opened_c == true)} && {p!(opened_m == false)}};
        let open_executing = Variable::new_predicate("open_executing", open_executing);
        py_emul_predicates.push("self.open_executing = self.opened_c and not self.opened_m".to_string());

        let open_finishing = pr! {{p!(opened_c == true)} && {p!(opened_m == true)}};
        let open_finishing = Variable::new_predicate("open_finishing", open_finishing);
        py_emul_predicates.push("self.open_finishing = self.opened_c and self.opened_m".to_string());

        let open_done = pr! {{p!(opened_c == false)} && {p!(opened_m == true)}};
        let open_done = Variable::new_predicate("open_done", open_done);
        py_emul_predicates.push("self.open_done = not self.opened_c and self.opened_m".to_string());

        let open_start = Transition::new(
            "open_start",
            pred_true("open_enabled"),
            vec![a!(opened_c = true)],
            vec![],
            true,
        );
        py_emul_transitions.push("(self.open_enabled, ['self.opened_c = True'], [])".to_string());

        // let open_finish_str = ("open_finish", "open_executing", [], )
        let open_finish = Transition::new(
            "open_finish",
            pred_true("open_executing"),
            vec![],
            vec![a!(opened_m = true)],
            false,
        );
        py_emul_transitions.push("(self.open_executing, [], ['self.opened_m = True'])".to_string());

        let open_reset = Transition::new(
            "open_reset",
            pred_true("open_finishing"),
            vec![a!(opened_c = false)],
            vec![],
            true,
        );
        py_emul_transitions.push("(self.open_finishing, ['self.opened_c = False'], [])".to_string());

        Ability::new(
            "open",
            vec![open_start, open_finish, open_reset],
            vec![open_enabled, open_executing, open_finishing, open_done],
        )
    };

    let close = {

        let close_enabled = pr! {{p!(closed_c == false)} && {p!(closed_m == false)}};
        let close_enabled = Variable::new_predicate("close_enabled", close_enabled);
        py_emul_predicates.push("self.close_enabled = not self.closed_c and not self.closed_m".to_string());

        let close_executing = pr! {{p!(closed_c == true)} && {p!(closed_m == false)}};
        let close_executing = Variable::new_predicate("close_executing", close_executing);
        py_emul_predicates.push("self.close_executing = self.closed_c and not self.closed_m".to_string());

        let close_finishing = pr! {{p!(closed_c == true)} && {p!(closed_m == true)}};
        let close_finishing = Variable::new_predicate("close_finishing", close_finishing);
        py_emul_predicates.push("self.close_finishing = self.closed_c and self.closed_m".to_string());

        let close_done = pr! {{p!(closed_c == false)} && {p!(closed_m == true)}};
        let close_done = Variable::new_predicate("close_done", close_done);
        py_emul_predicates.push("self.close_done = not self.closed_c and self.closed_m".to_string());

        let close_start = Transition::new(
            "close_start",
            pred_true("close_enabled"),
            vec![a!(closed_c = true)],
            vec![],
            true,
        );
        py_emul_transitions.push("(self.close_enabled, ['self.closed_c = True'], [])".to_string());

        let close_finish = Transition::new(
            "close_finish",
            pred_true("close_executing"),
            vec![],
            vec![a!(closed_m = true)],
            false,
        );
        py_emul_transitions.push("(self.close_executing, [], ['self.closed_m = True'])".to_string());

        let close_reset = Transition::new(
            "close_reset",
            pred_true("close_finishing"),
            vec![a!(closed_c = false)],
            vec![],
            true,
        );
        py_emul_transitions.push("(self.close_finishing, ['self.closed_c = False'], [])".to_string());

        Ability::new(
            "close",
            vec![close_start, close_finish, close_reset],
            vec![close_enabled, close_executing, close_finishing, close_done],
        )
    };

    r.add_ability(open);
    r.add_ability(close);
    return (r, py_emul_predicates, py_emul_transitions);
}

#[test]
fn test_dummy_door() {
    let r1 = make_dummy_door("door1");

    let measured = GetSPModelVariables::new(r1.0.clone(), "door1").0;
    let command = GetSPModelVariables::new(r1.0.clone(), "door1").1;
    let predicates = GetSPModelVariables::new(r1.0.clone(), "door1").2;

    println!("variables: {:#?}", measured);
    println!("variables: {:#?}", command);
    println!("predicates: {:#?}", predicates);

    generate_python_model_based!("macro_test_gen1", r1.0.clone(), r1.1, r1.2);

    assert!(false);
}


#[test]
fn stuff_gen_macro() {
    generate_python_common!("macro_test_gen1", "door1");
}

// #[test]
// fn stuff_gen() {
//     Directories::new("random_package_name_4");
//     ConfigurationFile::new("random_package_name_4");
//     ReadmeFile::new("random_package_name_4");
//     ResourceFile::new("random_package_name_4");
//     TestCopyrightFile::new("random_package_name_4");
//     TestPep257File::new("random_package_name_4");
//     TestFlake8File::new("random_package_name_4");
//     DescriptionFile::new("random_package_name_4", "somedescr", "e@e.com", "endre");
//     SetupFile::new("random_package_name_4", 
//         vec!["a", "b", "c"], 
//         "e@e.com", 
//         "endre",
//         "somedescr", 
//         vec!["a", "b", "c", "asdf", "asdf2"],)
// }