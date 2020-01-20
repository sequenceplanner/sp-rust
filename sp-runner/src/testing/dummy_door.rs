/*
use crate::runners::*;
use sp_domain::*;
use sp_runner_api::*;

fn pred_path(name: &str) -> SPPath {
    TemporaryPath::from_array(TemporaryPathNS::Predicate, &[name]).to_sp()
}
fn pred_true(name: &str) -> Predicate {
    Predicate::EQ(
        PredicateValue::SPPath(pred_path(name)),
        PredicateValue::SPValue(true.to_spvalue()),
    )
}

pub fn make_dummy_door(name: &str) -> (Resource, Vec<String>) {
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

    let open = {

        let open_enabled = pr! {{p!(opened_c == false)} && {p!(opened_m == false)}};
        let open_enabled = Variable::new_predicate("open_enabled", open_enabled);
        py_emul_predicates.push("open_enabled = not opened_c and not opened_m".to_string());

        let open_executing = pr! {{p!(opened_c == true)} && {p!(opened_m == false)}};
        let open_executing = Variable::new_predicate("open_executing", open_executing);
        py_emul_predicates.push("open_executing = opened_c and not opened_m".to_string());

        let open_finishing = pr! {{p!(opened_c == true)} && {p!(opened_m == true)}};
        let open_finishing = Variable::new_predicate("open_finishing", open_finishing);
        py_emul_predicates.push("open_executing = opened_c and opened_m".to_string());

        let open_done = pr! {{p!(opened_c == false)} && {p!(opened_m == true)}};
        let open_done = Variable::new_predicate("open_done", open_done);
        py_emul_predicates.push("open_executing = not opened_c and opened_m".to_string());

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
        py_emul_predicates.push("close_enabled = not closed_c and not closed_m".to_string());

        let close_executing = pr! {{p!(closed_c == true)} && {p!(closed_m == false)}};
        let close_executing = Variable::new_predicate("close_executing", close_executing);
        py_emul_predicates.push("close_executing = closed_c and not closed_m".to_string());

        let close_finishing = pr! {{p!(closed_c == true)} && {p!(closed_m == true)}};
        let close_finishing = Variable::new_predicate("close_finishing", close_finishing);
        py_emul_predicates.push("close_finishing = closed_c and closed_m".to_string());

        let close_done = pr! {{p!(closed_c == false)} && {p!(closed_m == true)}};
        let close_done = Variable::new_predicate("close_done", close_done);
        py_emul_predicates.push("close_done = not closed_c and closed_m".to_string());

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
    return (r, py_emul_predicates);
}

#[test]
fn test_dummy_door() {
    let r1 = make_dummy_door("door_asdf1");
    let m = Model::new_root("dummy_robot_model", vec![SPItem::Resource(r1.0.clone())]);
    let rm = make_runner_model(&m);

    let s = make_initial_state(&m);
    let mut node = r1.0.node().to_string();
    node.drain(node.find('<').unwrap_or(node.len())..);
    let asdf = node.find('_').unwrap_or(node.len());

    fn capitalize(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        None => String::new(),
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        }
    }

    let mut asdf = capitalize("asdf_asdf!");
    asdf.retain(|c| c != '_');

    println!("{:#?}", r1.1);

    for ab in r1.0.abilities(){
        // println!("{:#?}", ab.as_ref());
    //     // for predicate in ab {
    //     //     println!("{:#?}", predicate );
    //     //     println!("====================================");
    }
    // }

    // println!("{:#?}", r1);

    // println!("{}", &format!("{}InterfacerToEmulator", asdf));

    // let mut capitalized_resource_name = node;
    //     // while capitalized_resource_name.contains('_') {
    //         let mut index = capitalized_resource_name.find('_').unwrap_or(capitalized_resource_name.len());
    //         capitalized_resource_name.chars().nth(index).unwrap().to_uppercase();
    //     // };

    // println!("{:#?}", capitalized_resource_name);

    // let asdf = capitalize("asdfasdf");
    // capitalize(node.chars().nth(asdf+1).unwrap().as_str());

    // let mut c = node[..].chars();
    // match c.next() {
    //     None => String::new(),
    //     Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
    // }
    // let node2 = node[..].chars().nth(asdf+1).unwrap().to_uppercase().to_string();
    // node.retain(|c| c != '_');

    // node;

    // println!("{:#?}", asdf);

    // println!("{:#?}", node);

    // for (key, value) in &r1.node {
    //         let mut st = key.to_string();

    // println!("{:#?}", r1);

    // println!("{:#?}", s);

    assert!(false);
}
 */
