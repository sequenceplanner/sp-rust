
#[cfg(not(feature = "ros"))]
mod ros {
    // empty when no ros support compiled in.
    pub struct RosNode {}
    use crossbeam::channel;
    use failure::*;
    use sp_domain::*;

    pub fn start_node() -> Result<RosNode, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn roscomm_setup(
        _node: &mut RosNode,
        _model: &Model,
        _tx_in: channel::Sender<SPState>,
    ) -> Result<channel::Sender<SPState>, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn roscomm_setup_misc(
        _node: &mut RosNode,
        _tx_in: channel::Sender<sp_runner_api::RunnerCommand>,
    ) -> Result<channel::Sender<sp_runner_api::RunnerInfo>, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn spin(_node: &mut RosNode) {}
}

#[cfg(feature = "ros")]
mod ros {
    // the actual mod
    use crossbeam::channel;
    use failure::Error;

    use sp_domain::*;
    use sp_runner_api::*;

    use std::thread;

    pub struct RosNode(r2r::Node);

    pub fn json_to_state(
        json: &serde_json::Value,
        md: &MessageField,
    ) -> SPState {
        fn json_to_state_<'a>(
            json: &serde_json::Value,
            md: &'a MessageField,
            p: &mut Vec<&'a str>,
            a: &mut Vec<(Vec<&'a str>, SPValue)>,
        ) {
            match md {
                MessageField::Msg(msg) => {
                    // let path_name = msg.node().name();
                    // p.push(&path_name); // keep message type in path?
                    for field in &msg.fields {
                        if let Some(json_child) = json.get(field.name()) {
                            p.push(&field.name());
                            json_to_state_(json_child, field, p, a);
                            p.pop();
                        }
                    }
                    // p.pop();
                }
                MessageField::Var(var) => {
                    let sp_val = SPValue::from_json(json, var.value_type());
                    a.push((p.clone(), sp_val));
                }
            }
        }

        let mut p = Vec::new();
        let mut a = Vec::new();
        json_to_state_(json, md, &mut p, &mut a);
        let res: Vec<(SPPath, SPValue)> = a.iter().map(|(path, spval)| (SPPath::from_slice(path), spval.clone())).collect();
        SPState::new_from_values(&res)
    }


    pub fn state_to_json(
        state: &SPState,
        md: &MessageField,
    ) -> serde_json::Value {
        fn state_to_json_<'a>(
            state: &SPState,
            md: &'a MessageField,
            p: &mut Vec<&'a str>,
        ) -> serde_json::Value {
            match md {
                MessageField::Msg(msg) => {
                    let mut map = serde_json::Map::new();
                    // let path_name = msg.node().name();
                    // p.push(&path_name); // keep message type in path?
                    for field in &msg.fields {
                        p.push(field.name());
                        map.insert(field.name().to_string(), state_to_json_(state, field, p));
                        p.pop();
                    }
                    // p.pop();
                    serde_json::Value::Object(map)
                }
                MessageField::Var(_var) => {
                    if let Some(spval) = state.sp_value_from_path(&SPPath::from_slice(p)) {
                        // TODO use sp type
                        let json = spval.to_json(); // , var.variable_data().type_);
                        json
                    } else {
                        // TODO maybe panic here
                        serde_json::Value::Null
                    }
                }
            }
        }

        let mut p = Vec::new();
        state_to_json_(state, md, &mut p)
    }

    pub fn start_node() -> Result<RosNode, Error> {
        let ctx = r2r::Context::create()?;
        let node = r2r::Node::create(ctx, "spnode", "")?;
        Ok(RosNode(node))
    }

    pub fn spin(node: &mut RosNode) {
        node.0.spin_once(std::time::Duration::from_millis(100));
    }

    pub fn roscomm_setup(
        node: &mut RosNode,
        model: &Model,
        tx_in: channel::Sender<SPState>,
    ) -> Result<channel::Sender<SPState>, Error> {
        let mut ros_pubs = Vec::new();

        let rcs: Vec<_> = model
            .items()
            .iter()
            .flat_map(|i| match i {
                SPItem::Resource(r) => Some(r),
                _ => None,
            })
            .collect();

        for r in rcs {
            for t in r.messages() {
                if let MessageField::Msg(m) = t.msg() {
                    if t.is_subscriber() {
                        let topic = t.path();
                        let topic_str = topic.to_string();

                        let tx = tx_in.clone();
                        let msg_cb = t.msg().clone();
                        let cb = move |msg: r2r::Result<serde_json::Value>| {
                            let json = msg.unwrap();
                            let mut state = json_to_state(&json, &msg_cb);
                            state.prefix_paths(&msg_cb.path());
                            tx.send(state).unwrap();
                        };
                        println!("setting up subscription to topic: {}", topic);
                        let _subref = node.0.subscribe_untyped(&topic_str, &m.type_, Box::new(cb))?;
                    }

                    else if t.is_publisher() {
                        let topic = t.path();
                        let topic_str = topic.to_string();
                        println!("setting up publishing to topic: {}", topic);
                        let rp = node.0.create_publisher_untyped(&topic_str, &m.type_)?;
                        let msg_cb = t.msg().clone();
                        let cb = move |state: &SPState| {
                            let local_state = state.sub_state_projection(&msg_cb.path());
                            let mut dropped_local_state = local_state.clone_state();
                            dropped_local_state.unprefix_paths(&msg_cb.path());
                            let to_send = state_to_json(&dropped_local_state, &msg_cb);
                            rp.publish(to_send).unwrap();
                        };
                        ros_pubs.push(cb);
                    }
                    else {
                        panic!("topic is neither publisher nor subscriber. check variable types");
                    }

                } else { panic!("must have a message under a topic"); }
            }
        }

        let (tx_out, rx_out) = channel::unbounded();
        thread::spawn(move || loop {
            let state = rx_out.recv().unwrap();
            for rp in &ros_pubs {
                (rp)(&state);
            }
        });

        Ok(tx_out)
    }


    pub fn roscomm_setup_misc(
        node: &mut RosNode,
        tx_in: channel::Sender<sp_runner_api::RunnerCommand>,
    ) -> Result<channel::Sender<sp_runner_api::RunnerInfo>, Error> {
        let runner_cmd_topic = "sp/runner/command";

        let cb = {
            let tx_in = tx_in.clone();
            move |msg: r2r::sp_messages::msg::RunnerCommand| {
                let oat = msg.override_ability_transitions.iter().map(|s| {
                    let path = format!("G:{}", s);
                    SPPath::from_string(&path)
                }).collect();
                let oot = msg.override_operation_transitions.iter().map(|s| {
                    let path = format!("G:{}", s);
                    SPPath::from_string(&path)
                }).collect();
                let rc = RunnerCommand {
                    pause: msg.pause,
                    override_ability_transitions: oat,
                    override_operation_transitions: oot,
                };
                println!("SENDING TO RUNNER {:?}", rc);
                tx_in.send(rc).unwrap();
            }
        };
        println!("setting up subscription to topic: {}", runner_cmd_topic);
        let _subref = node.0.subscribe(runner_cmd_topic, Box::new(cb))?;


        let runner_info_topic = "sp/runner/info";
        println!("setting up publishing to topic: {}", runner_info_topic);
        let rp = node.0.create_publisher::<r2r::sp_messages::msg::RunnerInfo>(runner_info_topic)?;
        let info_cb = move |info: sp_runner_api::RunnerInfo| {
            let mut sorted_state = info.state.projection();
            sorted_state.sort();
            let ri = r2r::sp_messages::msg::RunnerInfo {
                state: sorted_state.clone_vec_value().into_iter().map(|(k,v)| {
                    let s = k.to_string();
                    let val = v.to_json().to_string();
                    r2r::sp_messages::msg::State {
                        path: s,
                        value_as_json: val,
                    }
                }).collect(),
                ability_plan: info.ability_plan.iter().map(|p|p.to_string()).collect(),
                enabled_ability_transitions: info.enabled_ability_transitions.iter().map(|p|p.to_string()).collect(),
                operation_plan: info.operation_plan.iter().map(|p|p.to_string()).collect(),
                enabled_operation_transitions: info.enabled_operation_transitions.iter().map(|p|p.to_string()).collect(),
            };
            rp.publish(&ri).unwrap();
        };

        let (tx_out, rx_out) = channel::unbounded();
        thread::spawn(move || loop {
            let info = rx_out.recv().unwrap();
            (info_cb)(info)
        });

        Ok(tx_out)
    }

    #[cfg(test)]
    mod ros_tests {
        use super::*;

        use sp_domain::*;
        use sp_runner_api::*;


        #[test]
        fn test_json_to_state() {
            let payload = "hej".to_string();
            let msg = r2r::std_msgs::msg::String { data: payload.clone() };
            let json = serde_json::to_value(msg).unwrap();

            let data = json.get("data");
            assert!(data.is_some());

            let v = Variable::new(
                "data",
                VariableType::Measured,
                SPValueType::String,
                "".to_spvalue(),
                Vec::new(),
            );

            let msg = Message::new(
                "str".into(),
                "std_msgs/msg/String".into(),
                &[MessageField::Var(v)],
            );

            let msg = MessageField::Msg(msg);

            let s = json_to_state(&json, &msg);
            assert_eq!(s.sp_value_from_path(&SPPath::from_slice(&["data"])), Some(&SPValue::String(payload.clone())));
        }

        #[test]
        fn test_state_to_json() {
            let payload = "hej".to_string();

            let mut state = SPState::default();
            state.add_variable(SPPath::from_slice(&["data"]),
                               SPValue::String(payload.clone()));

            let v = Variable::new(
                "data",
                VariableType::Measured,
                SPValueType::String,
                "".to_spvalue(),
                Vec::new(),
            );

            let msg = Message::new(
                "str".into(),
                "std_msgs/msg/String".into(),
                &[MessageField::Var(v)]
            );

            let msg = MessageField::Msg(msg);

            //let local_state = state.sub_state_projection(&SPPath::from_slice(&["data"]));
            //let json = state_to_json(&local_state.clone_state(), &msg);
            let json = state_to_json(&state, &msg);
            println!("raw state {:?}", state);
            println!("raw json {:?}", json);

            let data = json.get("data");
            assert!(data.is_some());
            assert_eq!(data, Some(&serde_json::Value::String(payload.clone())));
        }
    }

}

pub use ros::*;
