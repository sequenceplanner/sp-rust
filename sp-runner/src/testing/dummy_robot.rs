use crate::runners::*;
use sp_domain::*;
use sp_runner_api::*;
use std::collections::HashMap;

#[derive(Debug, PartialEq, Clone)]
struct Domain {
    domain: Option<Vec<SPValue>>, // none=domain is boolean
}

#[derive(Debug, PartialEq, Clone)]
struct CommandTopic {
    topic: String,
    short_name: String, // remove this later
    ros_type: String,
    vars: HashMap<String, Domain>,
    initial_values: HashMap<String, SPValue>, // initial states
}

#[derive(Debug, PartialEq, Clone)]
struct MeasuredTopic {
    topic: String,
    short_name: String, // remove this later
    ros_type: String,
    vars: HashMap<String, Domain>,
}

#[derive(Debug, PartialEq, Clone)]
struct MTransition {
    controlled: bool,
    guard: Predicate,
    actions: Vec<Action>,
    effects: Vec<Action>,
}

#[derive(Debug, PartialEq, Clone)]
struct Ability {
    name: String,
    predicates: HashMap<String, Predicate>,
    transitions: HashMap<String, MTransition>,
}

#[derive(Debug, PartialEq, Clone)]
enum ModelItem {
    CommandTopic(CommandTopic),
    MeasuredTopic(MeasuredTopic),
    Ability(Ability),
}

fn build_model(m: &[ModelItem]) {
    let abilities: Vec<_> = m.iter().flat_map(|i| match i { ModelItem::Ability(a) => Some(a), _ => None }).collect();

    println!("abilities: {}", abilities.iter().map(|a| a.name.clone()).collect::<Vec<_>>().join(", "));




}

pub fn make_dummy_robot(name: &str) -> Resource {
    let mut r = Resource::new(name);


    let mut new_resource = Vec::new();
    let mut cmd_vars = HashMap::new();
    cmd_vars.insert("ref_pos".to_string(), Domain { domain: Some(vec!["unknown".to_spvalue(), "at".to_spvalue(), "away".to_spvalue()]) });
    cmd_vars.insert("activate".to_string(), Domain { domain: None });

    let command = CommandTopic {
        topic: "Control".into(),
        short_name: "dr_c".into(),
        ros_type: "dummy_robot_messages/msg/Control".into(),
        vars: cmd_vars,
        initial_values: HashMap::new()
    };

    let mut measured_vars = HashMap::new();
    measured_vars.insert("act_pos".to_string(), Domain { domain: Some(vec!["unknown".to_spvalue(), "at".to_spvalue(), "away".to_spvalue()]) });
    measured_vars.insert("active".to_string(), Domain { domain: None });

    let measured = MeasuredTopic {
        topic: "State".into(),
        short_name: "dr_m".into(),
        ros_type: "dummy_robot_messages/msg/State".into(),
        vars: measured_vars,
    };

    new_resource.push(ModelItem::CommandTopic(command));
    new_resource.push(ModelItem::MeasuredTopic(measured));

    let rp_c = SPPath::from_string("ref_pos");
    let ap_m = SPPath::from_string("act_pos");
    let activate_c = SPPath::from_string("activate");
    let active_m = SPPath::from_string("active");

    let mut predicates = HashMap::new();
    predicates.insert("enabled".to_string(), pr! {{p!(active_m)} && {p!(rp_c != "at")} && {p!(ap_m != "at")}});
    predicates.insert("executing".to_string(), pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m != "at")}});
    predicates.insert("finished".to_string(), pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m == "at")}});

    let enabled = SPPath::from_string("enabled");

    let mut transitions = HashMap::new();
    let start = MTransition {
        controlled: true,
        guard: pr! {{p!(enabled)} && {p!(rp_c == "at")} && {p!(ap_m == "at")}},
        actions: Vec::new(),
        effects: Vec::new(),
    };

    transitions.insert("start".to_string(), start);

    let to_table = Ability {
        name: "to_table".to_string(),
        predicates : predicates,
        transitions: transitions,
    };

    new_resource.push(ModelItem::Ability(to_table));

    println!("NEW RESOURCE:");
    println!("{:#?}", new_resource);

    // build resource by going through all transitions and predicates
    // and looking for names "upwards" in the hierarchy defined by the
    // resource.

    build_model(&new_resource);


    let command = command_topic("Control", "dr_c", "dummy_robot_messages/msg/Control",
                                &[
                                    ( "ref_pos", Some(&["unknown", "at", "away"])),
                                    ( "activate", None )  // none is boolean
                                ]);
    r.add_message(command);

    let measured = measured_topic("State", "dr_m", "dummy_robot_messages/msg/State",
                                  &[
                                      ( "act_pos", Some(&["unknown", "at", "away"])),
                                      ( "active", None )
                                  ]);
    r.add_message(measured);

    let rp_c = r.find_item("ref_pos", &[]).expect("check spelling").path();
    let ap_m = r.find_item("act_pos", &[]).expect("check spelling").path();
    let activate_c = r.find_item("activate", &[]).expect("check spelling").path();
    let active_m = r.find_item("active", &[]).expect("check spelling").path();

    let to_table =
        ability("to_table",
                &[
                    ("enabled", pr! {{p!(active_m)} && {p!(rp_c != "at")} && {p!(ap_m != "at")}}),
                    ("executing", pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m != "at")}}),
                    ("finished", pr! {{p!(active_m)} && {p!(rp_c == "at")} && {p!(ap_m == "at")}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(rp_c = "at")], &[a!(ap_m = "unknown")]),

                    ("finish", false, pred_true("executing"), &[], &[a!(ap_m = "at")] ),
                ]);


    let to_away =
        ability("to_away",
                &[
                    ("enabled", pr! {{p!(active_m)} && {p!(rp_c != "away")} && {p!(ap_m != "away")}}),
                    ("executing", pr! {{p!(active_m)} && {p!(rp_c == "away")} && {p!(ap_m != "away")}}),
                    ("finished", pr! {{p!(active_m)} && {p!(rp_c == "away")} && {p!(ap_m == "away")}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(rp_c = "away")], &[a!(ap_m = "unknown")]),

                    ("finish", false, pred_true("executing"), &[], &[a!(ap_m = "away")] ),
                ]);

    let activate =
        ability("activate",
                &[
                    ("enabled", pr! {{p!(activate_c == false)} && {p!(active_m == false)}} ),
                    ("executing", pr! {{p!(activate_c == true)} && {p!(active_m == false)}} ),
                    ("finished", pr! {{p!(activate_c == true)} && {p!(active_m == true)}} ),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(activate_c = true)], &[] ),
                    ("finish", false, pred_true("executing"), &[], &[a!(active_m = true)]),
                ]);

    let deactivate =
        ability("deactivate",
                &[
                    ("enabled", pr! {{p!(activate_c == true)} && {p!(active_m == true)}}),
                    ("executing", pr! {{p!(activate_c == false)} && {p!(active_m == true)}}),
                    ("finished", pr! {{p!(activate_c == false)} && {p!(active_m == false)}}),
                ],
                &[
                    ("start", true, pred_true("enabled"), &[a!(activate_c = false)], &[]),
                    ("finish", false, pred_true("executing"), &[], &[a!(active_m = false)]),
                ]);

    r.add_ability(to_table);
    r.add_ability(to_away);
    r.add_ability(activate);
    r.add_ability(deactivate);

    return r;
}

#[test]
fn test_dummy() {
    let r1 = make_dummy_robot("r1");
//    println!("{:#?}", r1);
    assert!(false);

    let m = Model::new_root("one_robot_model", vec![SPItem::Resource(r1)]);
    let rm = make_runner_model(&m);

    println!("{:#?}", rm);
    assert!(false);
}
