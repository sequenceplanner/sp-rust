#[cfg(not(feature = "ros"))]
mod ros {
    // empty when no ros support compiled in.
    pub struct RosNode {}
    #[derive(Debug, PartialEq, Clone)]
    pub struct RosMessage {
        pub state: SPState,
        pub resource: SPPath,
        pub time_stamp: std::time::Instant,
    }
    #[derive(Debug, PartialEq, Clone)]
    pub struct NodeCmd {
        pub resource: SPPath,
        pub mode: String,
        pub time_stamp: std::time::Instant,
    }
    #[derive(Debug, PartialEq, Clone)]
    pub struct NodeMode {
        pub resource: SPPath,
        pub mode: String,
        pub time_stamp: std::time::Instant,
        pub echo: SPState,
    }

    use crossbeam::channel;
    use failure::*;
    use sp_domain::*;

    pub fn start_node() -> Result<RosNode, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn roscomm_setup(
        _node: &mut RosNode, _model: &Model, _tx_in: channel::Sender<RosMessage>,
    ) -> Result<channel::Sender<SPState>, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn roscomm_setup_misc(
        _node: &mut RosNode, _tx_in: channel::Sender<sp_runner_api::RunnerCommand>,
    ) -> Result<channel::Sender<sp_runner_api::RunnerInfo>, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn ros_node_comm_setup(
        _node: &mut RosNode, _model: &Model, _tx_in: channel::Sender<NodeMode>,
    ) -> Result<channel::Sender<NodeCmd>, Error> {
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
    #[derive(Debug, PartialEq, Clone)]
    pub struct RosMessage {
        pub state: SPState,
        pub resource: SPPath,
        pub time_stamp: std::time::Instant,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct NodeCmd {
        pub resource: SPPath,
        pub mode: String,
        pub time_stamp: std::time::Instant,
    }
    #[derive(Debug, PartialEq, Clone)]
    pub struct NodeMode {
        pub resource: SPPath,
        pub mode: String,
        pub time_stamp: std::time::Instant,
        pub echo: SPState,
    }

    fn json_to_state(json: &serde_json::Value, md: &MessageField) -> SPState {
        fn json_to_state_<'a>(
            json: &serde_json::Value, md: &'a MessageField, p: &mut Vec<&'a str>,
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
        let res: Vec<(SPPath, SPValue)> = a
            .iter()
            .map(|(path, spval)| (SPPath::from_slice(path), spval.clone()))
            .collect();
        SPState::new_from_values(&res)
    }

    fn state_to_json(state: &SPState, md: &MessageField) -> serde_json::Value {
        fn state_to_json_<'a>(
            state: &SPState, md: &'a MessageField, p: &mut Vec<&'a str>,
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

    // TODO: Probably use i64 and f64 instead in SPValue
    use std::convert::TryFrom;
    fn json_to_sp_value(value: &serde_json::Value) -> SPValue {
        match value {
            serde_json::Value::Bool(x) => SPValue::Bool(*x),
            serde_json::Value::Number(x) if x.is_u64()  => SPValue::Int32(i32::try_from(x.as_u64().unwrap()).unwrap()), // will panic if overflow
            serde_json::Value::Number(x) if x.is_i64()  => SPValue::Int32(i32::try_from(x.as_i64().unwrap()).unwrap()), // will panic if overflow
            serde_json::Value::Number(x) if x.is_f64()  => SPValue::Float32(x.as_f64().unwrap() as f32), // will NOT panic if overflow, so may generate wrong result
            serde_json::Value::String(x) => SPValue::String(x.clone()),
            serde_json::Value::Array(xs) => {
                let res: Vec<SPValue> = xs.iter().map(|x| json_to_sp_value(x)).collect();
                let t = if res.is_empty() {SPValueType::Unknown} else {res.first().unwrap().has_type()};
                SPValue::Array(t, res)
            },
            _ => {
                println!("Hmm, no ros to SPValue for {:?}", value);
                SPValue::Unknown
            },
        }
    }

    fn ros_state_to_sp_state(ros_state: &[r2r::sp_messages::msg::State]) -> SPState {
        let xs: Vec<(SPPath, SPValue)> = ros_state.iter().flat_map(|s| {
            let s: &r2r::sp_messages::msg::State = s;
            let v: Option<serde_json::Value> = serde_json::from_str(&s.value_as_json).ok();
            v.map(|x| (SPPath::from_string(&s.path), json_to_sp_value(&x)))
        }).collect();
        SPState::new_from_values(&xs)
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
        node: &mut RosNode, model: &Model, tx_in: channel::Sender<RosMessage>,
    ) -> Result<channel::Sender<SPState>, Error> {
        let mut ros_pubs = Vec::new();

        let rcs = model.all_resources();

        for r in rcs {
            for t in r.messages() {
                if let MessageField::Msg(m) = t.msg() {
                    if t.is_subscriber() {
                        let topic = t.path().drop_root();
                        let topic_str = topic.to_string();

                        let tx = tx_in.clone();
                        let msg_cb = t.msg().clone();
                        let r_path = r.path().clone();
                        let cb = move |msg: r2r::Result<serde_json::Value>| {
                            let json = msg.unwrap();
                            let mut state = json_to_state(&json, &msg_cb);
                            state.prefix_paths(&msg_cb.path());
                            let time_stamp = std::time::Instant::now();
                            let m = RosMessage {
                                state,
                                resource: r_path.clone(),
                                time_stamp,
                            };

                            tx.send(m).unwrap();
                        };
                        println!("setting up subscription to topic: {}", topic);
                        let _subref =
                            node.0
                                .subscribe_untyped(&topic_str, &m.type_, Box::new(cb))?;
                    } else if t.is_publisher() {
                        let topic = t.path().drop_root();
                        let topic_str = topic.to_string();
                        println!("setting up publishing to topic: {}", topic);
                        let rp = node.0.create_publisher_untyped(&topic_str, &m.type_)?;
                        let msg_cb = t.msg().clone();
                        let cb = move |state: &SPState| {
                            let local_state = state.sub_state_projection(&msg_cb.path());
                            if local_state.state.is_empty() {
                                return;
                            }
                            let mut dropped_local_state = local_state.clone_state();
                            dropped_local_state.unprefix_paths(&msg_cb.path());
                            let to_send = state_to_json(&dropped_local_state, &msg_cb);
                            let res = rp.publish(to_send.clone());
                            if res.is_err() {
                                println!(
                                    "RosComm not working for {}, error: {:?}",
                                    &msg_cb.path(),
                                    res
                                );
                                println!("RosComm, local state: {}", dropped_local_state);
                                println!("RosComm, to_send: {:?}", to_send);
                                println!("");
                            }
                        };
                        ros_pubs.push(cb);
                    } else {
                        panic!("topic is neither publisher nor subscriber. check variable types");
                    }
                } else {
                    panic!("must have a message under a topic");
                }
            }
        }

        let (tx_out, rx_out) = channel::unbounded();
        thread::spawn(move || loop {
            match rx_out.recv() {
                Ok(state) => {
                    for rp in &ros_pubs {
                        (rp)(&state);
                    }
                }
                Err(e) => {
                    println!("RosComm out did not work: {:?}", e);
                    break;
                }
            }
        });

        Ok(tx_out)
    }

    pub fn roscomm_setup_misc(
        node: &mut RosNode, tx_in: channel::Sender<sp_runner_api::RunnerCommand>,
    ) -> Result<channel::Sender<sp_runner_api::RunnerInfo>, Error> {
        let runner_cmd_topic = "sp/runner/command";

        let _x = r2r::sp_messages::msg::ForcedGoal {
            level: "0".to_string(),
            goal: "".to_string(),
        };


        let cb = {
            let tx_in = tx_in.clone();
            move |msg: r2r::sp_messages::msg::RunnerCommand| {
                let possible_cmd = if msg.set_state {
                    let state = ros_state_to_sp_state(&msg.state);
                    Some(RunnerCommand::SetState(state))
                } else if msg.force_state {
                    let state = ros_state_to_sp_state(&msg.state);
                    Some(RunnerCommand::ForceState(state))
                } else if msg.set_mode{
                    Some(RunnerCommand::Mode(msg.mode))
                } else {
                    // TODO: handle goal
                    None
                };

                if let Some(cmd) = possible_cmd {
                    tx_in.send(cmd).expect("Can not send runner commmands. Threads are dead?");
                }
            }
        };
        println!("setting up subscription to topic: {}", runner_cmd_topic);
        let _subref = node.0.subscribe(runner_cmd_topic, Box::new(cb))?;

        let runner_info_topic = "sp/runner/info";
        println!("setting up publishing to topic: {}", runner_info_topic);
        let rp = node
            .0
            .create_publisher::<r2r::sp_messages::msg::RunnerInfo>(runner_info_topic)?;
        let info_cb = move |info: sp_runner_api::RunnerInfo| {
            let mut sorted_state = info.state.projection();
            sorted_state.sort();
            let ri = r2r::sp_messages::msg::RunnerInfo {
                state: sorted_state
                    .clone_vec_value()
                    .into_iter()
                    .map(|(k, v)| {
                        let s = k.to_string();
                        let val = v.to_json().to_string();
                        r2r::sp_messages::msg::State {
                            path: s,
                            value_as_json: val,
                        }
                    })
                    .collect(),
                plans: vec![],
                mode: "".to_string(),
                forced_state: vec![],
                forced_goal: vec![],
            };
            rp.publish(&ri).unwrap();
        };

        let (tx_out, rx_out) = channel::unbounded();
        thread::spawn(move || loop {
            match rx_out.recv() {
                Ok(x) => (info_cb)(x),
                Err(e) => {
                    println!("RosMisc out did not work: {:?}", e);
                    break;
                }
            }
        });

        Ok(tx_out)
    }

    pub fn ros_node_comm_setup(
        node: &mut RosNode, model: &Model, tx_in: channel::Sender<NodeMode>,
    ) -> Result<channel::Sender<NodeCmd>, Error> {
        let mut ros_pubs = Vec::new();

        let rcs = model.all_resources();

        for r in rcs {
            for t in r.messages() {
                if !t.is_publisher() {
                    continue;
                }
                if let MessageField::Msg(_m) = t.msg() {
                    let tx = tx_in.clone();
                    let r_path = r.path().clone();
                    let cmd_msg = t.msg().clone();

                    let cb = move |msg: r2r::sp_messages::msg::NodeMode| {
                        let json = serde_json::from_str(&msg.echo);
                        let echo = json.map_or(SPState::new(), |json| {
                            let mut cmd_state = json_to_state(&json, &cmd_msg);
                            cmd_state.prefix_paths(&cmd_msg.path());
                            cmd_state
                        });

                        let time_stamp = std::time::Instant::now();
                        let m = NodeMode {
                            mode: msg.mode.clone(),
                            resource: r_path.clone(),
                            time_stamp,
                            echo,
                        };

                        tx.send(m).unwrap();
                    };
                    let topic = r.path().drop_root().add_child("mode").to_string();
                    println!("setting up subscription to resource on topic: {}", topic);
                    let _subref = node.0.subscribe(&topic, Box::new(cb))?;
                }
            }

            // Outgoing
            let topic = r.path().drop_root().add_child("node_cmd").to_string();
            let msg_type = "sp_messages/msg/NodeCmd";
            let r_path = r.path().clone();
            let rp = node.0.create_publisher_untyped(&topic, msg_type)?;
            let cb = move |cmd: &NodeCmd| {
                if cmd.resource == r_path.clone() {
                    let mut map = serde_json::Map::new();
                    map.insert(
                        "mode".to_string(),
                        serde_json::Value::String(cmd.mode.clone()),
                    );
                    let to_send = serde_json::Value::Object(map);
                    rp.publish(to_send).unwrap();
                }
            };
            ros_pubs.push(cb);
        }

        let (tx_out, rx_out) = channel::unbounded();
        thread::spawn(move || loop {
            match rx_out.recv() {
                Ok(state) => {
                    for rp in &ros_pubs {
                        (rp)(&state);
                    }
                }
                Err(e) => {
                    println!("RosNodeComm out did not work: {:?}", e);
                    break;
                }
            }
        });

        Ok(tx_out)
    }

    #[cfg(test)]
    mod ros_tests {
        use super::*;

        #[test]
        fn test_json_to_state() {
            let payload = "hej".to_string();
            let msg = r2r::std_msgs::msg::String {
                data: payload.clone(),
            };
            let json = serde_json::to_value(msg).unwrap();

            let data = json.get("data");
            assert!(data.is_some());

            let v = Variable::new(
                "data",
                VariableType::Measured,
                SPValueType::String,
                Vec::new(),
            );

            let msg = Message::new(
                "str".into(),
                "std_msgs/msg/String".into(),
                &[MessageField::Var(v)],
            );

            let msg = MessageField::Msg(msg);

            let s = json_to_state(&json, &msg);
            assert_eq!(
                s.sp_value_from_path(&SPPath::from_slice(&["data"])),
                Some(&SPValue::String(payload.clone()))
            );
        }

        #[test]
        fn test_state_to_json() {
            let payload = "hej".to_string();

            let mut state = SPState::default();
            state.add_variable(
                SPPath::from_slice(&["data"]),
                SPValue::String(payload.clone()),
            );

            let v = Variable::new(
                "data",
                VariableType::Measured,
                SPValueType::String,
                Vec::new(),
            );

            let msg = Message::new(
                "str".into(),
                "std_msgs/msg/String".into(),
                &[MessageField::Var(v)],
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
