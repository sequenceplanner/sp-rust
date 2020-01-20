//use sp_domain::*;
//use std::collections::HashMap;

// TODO: Update the test for the new domain

// Testing show we know that SPValues and support can be used outside
//

// #[test]
// fn sp_value_testing_external() {
//     assert_eq!(true.to_spvalue(), SPValue::Bool(true));
//     let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);

//     let v = SPPath::from_array(&["a", "b"]);

//     let eq = Predicate::EQ(
//         predicates::PredicateValue::SPValue(2.to_spvalue()),
//         predicates::PredicateValue::SPPath(v.clone()),
//     );

//     let p = p! {v == 2};
//     println!("TEST: {:?}", &p);
//     let p2 = p! {{["a", "b"]} == 2};
//     println!("TEST: {:?}", &p2);

//     let x = pr! {p2 && p && p && p};
//     println!("TEST2: {:?}", x);

//     let y = pr! {{p!{{["a", "b"]} == 10}} && {p!{{["a", "b"]} == 20}}};
//     println!("TEST3: {:?}", y);
// }

// // a simulated ros msg
// use serde::{Deserialize, Serialize};
// #[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
// struct std_string {
//     data: String,
// }

// #[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
// struct std_string2 {
//     data: String,
//     data2: bool,
//     data3: std_string
// }

// fn spval_from_json(json: &serde_json::Value, spv_t: SPValueType) -> SPValue {
//     // as we have more options than json we switch on the spval type
//     let tm = "type mismatch! re-generate ros sources.";
//     match spv_t {
//         SPValueType::Bool => json.as_bool().expect(tm).to_spvalue(),
//         SPValueType::Int32 => (json.as_i64().expect(tm) as i32).to_spvalue(),
//         SPValueType::Float32 => (json.as_f64().expect(tm) as f32).to_spvalue(),
//         SPValueType::String => json.as_str().expect(tm).to_spvalue(),
//         // todo: check is_array
//         _ => unimplemented!("TODO"),
//     }
// }

// fn spval_to_json(spval: &SPValue) -> serde_json::Value {
//     match spval {
//         SPValue::Bool(x) => serde_json::json!(*x),
//         SPValue::Int32(x) => serde_json::json!(*x),
//         SPValue::Float32(x) => serde_json::json!(*x),
//         SPValue::String(x) => serde_json::json!(x),
//         SPValue::Array(_, x) => {
//             let v: Vec<serde_json::Value> = x.iter().map(|spval| spval_to_json(spval)).collect();
//             serde_json::json!(v)
//         }
//         _ => unimplemented!("TODO"),
//     }
// }

// fn make_resource() -> Resource {
//     let state_var_data = Variable::new(
//         "state_var_data",
//         VariableType::Measured,
//         SPValueType::String,
//         "off".to_spvalue(),
//         vec!["off".to_spvalue(), "on".to_spvalue()],
//     );
//     let state_var_data2 = Variable::new_boolean(
//         "state_var_data2", VariableType::Measured,
//     );
//     let state_var_data3 = Variable::new(
//         "state_var_data3",
//         VariableType::Measured,
//         SPValueType::String,
//         "".to_spvalue(),
//         Vec::new(),
//     );

//     let command_var_data = Variable::new(
//         "command_var_data",
//         VariableType::Command,
//         "on".to_spvalue().has_type(),
//         "off".to_spvalue(),
//         vec!["off".to_spvalue(), "on".to_spvalue()],
//     );
//     let command_var_data2 = Variable::new_boolean(
//         "command_var_data2",
//         VariableType::Command,
//     );
//     let command_var_data3 = Variable::new(
//         "command_var_data3",
//         VariableType::Command,
//         33i32.to_spvalue().has_type(),
//         0.to_spvalue(),
//         vec![0.to_spvalue(), 33.to_spvalue()],
//     );

//     let comm = ResourceComm::RosComm(RosComm {
//         publishers: vec![RosPublisherDefinition {
//             topic: "/r1/command".into(),
//             qos: "deault".into(),
//             definition: RosMsgDefinition::Message(
//                 "std_msgs/msg/String".into(),
//                 hashmap![
//                     "data".into() => RosMsgDefinition::Field(command_var_data.clone()),
//                     "data2".into() => RosMsgDefinition::Field(command_var_data2.clone()),
//                     "data3".into() => RosMsgDefinition::Message(
//                         "std_msgs/msg/String2".into(),
//                         hashmap![
//                             "data".into() => RosMsgDefinition::Field(command_var_data3.clone())
//                             ])
//                 ],
//             ),
//         }],
//         subscribers: vec![
//             RosSubscriberDefinition {
//                 topic: "/r1/state".into(),
//                 definition: RosMsgDefinition::Message(
//                     "std_msgs/msg/String".into(),
//                     hashmap![
//                         "data".into() => RosMsgDefinition::Field(state_var_data.clone())
//                     ],
//                 ),
//             },
//             RosSubscriberDefinition {
//                 topic: "/r1/battery/state".into(),
//                 definition: RosMsgDefinition::Message(
//                     "std_msgs/msg/String".into(),
//                     hashmap![
//                         "data".into() => RosMsgDefinition::Field(state_var_data.clone()),
//                         "data2".into() => RosMsgDefinition::Field(state_var_data2.clone()),
//                         "data3".into() => RosMsgDefinition::Message(
//                             "std_msgs/msg/String2".into(),
//                             hashmap![
//                                 "data".into() => RosMsgDefinition::Field(state_var_data3.clone())
//                             ])
//                     ],
//                 ),
//             },
//         ],
//     });

//     // Resource {
//     //     abilities: Vec::new(),
//     //     parameters: Vec::new(),
//     //     comm: comm,
//     // }
// }

// #[test]
// fn test_resource() {
//     let r = make_resource();

//     // fake a ros msg
//     let rosinput = std_string2 {
//         data: "hello".into(),
//         data2: true,
//         data3: std_string { data: "123".into() },
//     };
//     let ros_untyped = serde_json::to_value(rosinput.clone()).unwrap();

//     // get ros message definitions
//     let rc = r.comm.as_ros_comm().unwrap();

//     let subs = r.comm.test_subs();

//     for s in &subs {
//         let state = (s)(&ros_untyped);
//         println!("got sp state\n===============");
//         println!("{}", state);
//     }

//     let p = SPPath::from_array(&["/r1/command", "std_msgs/msg/String", "data"]);
//     let p2 = SPPath::from_array(&["/r1/command", "std_msgs/msg/String", "data2"]);
//     let p3 = SPPath::from_array(&["/r1/command", "std_msgs/msg/String",
//                                 "data3", "std_msgs/msg/String2", "data"]);

//     let state = StateExternal {
//         s: hashmap![p => "hello1".to_spvalue(),
//                     p2 => true.to_spvalue(),
//                     p3 => "33".to_spvalue()
//         ],
//     };
//     let pubs = r.comm.test_pubs();
//     for p in &pubs {
//         let json = (p)(&state);
// //        println!("got json from state: {}", json.to_string());
// //        println!("deserialized: {:#?}", serde_json::from_value::<std_string2>(json));

//     }

//     // println!("resource variables: {:?}", r.comm.variables());
//     //    println!("resource variables: {:#?}", r.comm.variables());
//     //    println!("resource variables: {:#?}", rc);

//     let json = serde_json::to_string_pretty(&r).unwrap();
//     // println!("{}", json);
// }
