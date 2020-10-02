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
    pub struct ROSResource {
        pub path: SPPath,
        pub model: Option<SPItem>,
        pub last_goal_from_sp: Option<SPState>,
    }

    pub fn log(msg: &str, file: &str, line: u32, severity: u32) {
        println!("{}:{}:[{}] - {}", file, line, severity, msg);
    }

    pub fn log_debug(msg: &str, file: &str, line: u32) {
        log(msg, file, line, 1);
    }
    pub fn log_info(msg: &str, file: &str, line: u32) {
        log(msg, file, line, 2);
    }
    pub fn log_warn(msg: &str, file: &str, line: u32) {
        log(msg, file, line, 3);
    }
    pub fn log_error(msg: &str, file: &str, line: u32) {
        log(msg, file, line, 4);
    }
    pub fn log_fatal(msg: &str, file: &str, line: u32) {
        log(msg, file, line, 5);
    }

    #[macro_export]
    macro_rules! log_debug {
        ($($args:tt)*) => {{
            $crate::log(&std::fmt::format(format_args!($($args)*)), file!(), line!(), 1);
        }}
    }

    #[macro_export]
    macro_rules! log_info {
        ($($args:tt)*) => {{
            $crate::log(&std::fmt::format(format_args!($($args)*)), file!(), line!(), 2);
        }}
    }

    #[macro_export]
    macro_rules! log_warn {
        ($($args:tt)*) => {{
            $crate::log(&std::fmt::format(format_args!($($args)*)), file!(), line!(), 3);
        }}
    }

    #[macro_export]
    macro_rules! log_error {
        ($($args:tt)*) => {{
            $crate::log(&std::fmt::format(format_args!($($args)*)), file!(), line!(), 4);
        }}
    }

    #[macro_export]
    macro_rules! log_fatal {
        ($($args:tt)*) => {{
            $crate::log(&std::fmt::format(format_args!($($args)*)), file!(), line!(), 5);
        }}
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

    pub fn ros_resource_comm_setup(
        _node: &mut RosNode,
        _tx_to_runner: channel::Sender<ROSResource>,
        _prefix_path: &SPPath,
    ) -> Result<(), Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn spin(_node: &mut RosNode) {
    }
}

#[cfg(feature = "ros")]
mod ros {
    // the actual mod
    use crossbeam::channel;
    use failure::Error;
    use std::thread;

    use sp_domain::*;

    pub struct RosNode(r2r::Node);
    #[derive(Debug, PartialEq, Clone)]
    pub struct RosMessage {
        pub state: SPState,
        pub resource: SPPath,
        pub time_stamp: std::time::Instant,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct ROSResource {
        pub path: SPPath,
        pub model: Option<SPItem>,
        pub last_goal_from_sp: Option<SPState>,
    }

    const SP_NODE_NAME: &str = "sp";

    pub fn log_debug(msg: &str, file: &str, line: u32) {
        r2r::log(msg, SP_NODE_NAME, file, line, r2r::LogSeverity::Debug);
    }

    #[macro_export]
    macro_rules! log_debug {
        ($($args:tt)*) => {{
            $crate::log_debug(&std::fmt::format(format_args!($($args)*)), file!(), line!());
        }}
    }

    pub fn log_info(msg: &str, file: &str, line: u32) {
        println!("{}:{} - {}", file, line, msg);
        r2r::log(msg, SP_NODE_NAME, file, line, r2r::LogSeverity::Info);
    }

    #[macro_export]
    macro_rules! log_info {
        ($($args:tt)*) => {{
            $crate::log_info(&std::fmt::format(format_args!($($args)*)), file!(), line!());
        }}
    }

    pub fn log_warn(msg: &str, file: &str, line: u32) {
        r2r::log(msg, SP_NODE_NAME, file, line, r2r::LogSeverity::Warn);
    }

    #[macro_export]
    macro_rules! log_warn {
        ($($args:tt)*) => {{
            $crate::log_warn(&std::fmt::format(format_args!($($args)*)), file!(), line!());
        }}
    }

    pub fn log_error(msg: &str, file: &str, line: u32) {
        r2r::log(msg, SP_NODE_NAME, file, line, r2r::LogSeverity::Error);
    }

    #[macro_export]
    macro_rules! log_error {
        ($($args:tt)*) => {{
            $crate::log_error(&std::fmt::format(format_args!($($args)*)), file!(), line!());
        }}
    }

    pub fn log_fatal(msg: &str, file: &str, line: u32) {
        r2r::log(msg, SP_NODE_NAME, file, line, r2r::LogSeverity::Fatal);
    }

    #[macro_export]
    macro_rules! log_fatal {
        ($($args:tt)*) => {{
            $crate::log_fatal(&std::fmt::format(format_args!($($args)*)), file!(), line!());
        }}
    }

    fn json_to_state(json: &serde_json::Value, parent_path: &SPPath) -> Result<SPState, serde_json::Error> {
        let s_json = SPStateJson::from_json(json.clone())?;
        let mut s = s_json.to_state();
        s.prefix_paths(parent_path);
        Ok(s)
    }

    fn state_to_json(state: &SPState, parent_path: &SPPath, recursive: bool) -> serde_json::Value {
        let mut s = state.clone();
        s.unprefix_paths(parent_path);
        let s_json = if recursive {
            SPStateJson::from_state_recursive(&s)
        } else {
            SPStateJson::from_state_flat(&s)
        };
        s_json.to_json()
    }

    pub fn start_node() -> Result<RosNode, Error> {
        let ctx = r2r::Context::create()?;
        let node = r2r::Node::create(ctx, SP_NODE_NAME, "")?;
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
                for m in &r.new_messages {
                    println!("WE HAVE A NEW MESSAGE IN A RESOURCE: {:?}", m);
                    let topic = if m.relative_topic {
                        r.path().add_child_path(&m.topic).drop_root()  // TODO. Change the path of resources so that this is not needed. Or send it in as input?
                    } else {
                        m.topic.clone()
                    };
                    let topic_message_type = match &m.message_type {
                        MessageType::Ros(x) => x.clone(),
                        _ => "std_msgs/msg/String".to_string(),
                    };
                    let topic_str = topic.to_string();
                    let resource_path = r.path().clone();
                    match m.category {
                        MessageCategory::OutGoing => {
                            println!("setting up publishing to topic NEW: {}", topic);
                            
                            let rp = node.0.create_publisher_untyped(&topic_str, &topic_message_type)?;

                            let m = m.clone();
                            let cb = move |state: SPState| {
                                let res: Vec<(SPPath, Option<&SPValue>)> = m.variables.iter().map(|v| {
                                    let name = v.name.clone();
                                    let path = resource_path.add_child_path(&v.path.clone());
                                    let value = state.sp_value_from_path(&path);
                                    if value.is_none() {
                                        log_info!("Not in state: Name {}, resource: {}, path: {}, state: {}", &name, &resource_path, &path, SPStateJson::from_state_flat(&state).to_json());
                                    } 
                                    (name, value)
                                }).collect();

                                for (x, y) in res.iter() {
                                    if y.is_none() {
                                        return;
                                    }
                                }
                                let res:Vec<(SPPath, SPValue)> = res.into_iter().map(|(x, y)| (x, y.unwrap().clone())).collect();
                                 

                                let msg = match m.message_type {
                                    MessageType::JsonFlat => {
                                        let json = SPStateJson::from_state_flat(&SPState::new_from_values(&res));
                                        let json = serde_json::to_string(&json).unwrap();
                                        let mut map = serde_json::Map::new();
                                        map.insert("data".to_string(), serde_json::Value::String(json));
                                        serde_json::Value::Object(map)
                                    },
                                    MessageType::Json => {
                                        let json = SPStateJson::from_state_recursive(&SPState::new_from_values(&res));
                                        let json = serde_json::to_string(&json).unwrap();
                                        let mut map = serde_json::Map::new();
                                        map.insert("data".to_string(), serde_json::Value::String(json));
                                        serde_json::Value::Object(map)
                                    }
                                    MessageType::Ros(_) => {
                                        let json = SPStateJson::from_state_recursive(&SPState::new_from_values(&res));
                                        serde_json::to_value(&json).unwrap()
                                    }
                                }; 
                                println!("SENDING!!!: {}", &msg);
                                let res = rp.publish(msg.clone());
                                if res.is_err() {
                                    log_info!(
                                        "RosComm not working for {}, error: {:?}, msg: {}, state: {}",
                                        &topic_str,
                                        res,
                                        msg,
                                        SPStateJson::from_state_flat(&state).to_json()
                                    );
                                }
                            };
                            ros_pubs.push(cb);
                        },
                        MessageCategory::Incoming => {
                            let tx = tx_in.clone();
                            let r_path = r.path().clone();
                            let m = m.clone();
                            let topic_cb = topic_str.clone();
                            let cb = move |msg: r2r::Result<serde_json::Value>| {
                                let json = msg.unwrap();
                                let json_s = SPStateJson::from_json(json);
                                if let Err(e) = json_s {
                                    log_info!(
                                        "Could not convert incoming message on {}, error: {:?}",
                                        &topic_cb,
                                        e
                                    );
                                } else {
                                    let x = json_s.unwrap();
                                    let mut msg_state = x.to_state();
                                    if m.message_type == MessageType::Json || m.message_type == MessageType::JsonFlat {
                                        msg_state.unprefix_paths(&SPPath::from_string("data"));
                                    }

                                    let map:Vec<(SPPath, Option<&SPValue>)>  = m.variables.iter().map(|v| {
                                        let p = if v.relative_path {
                                            r_path.add_child_path(&v.path)
                                        } else {
                                            v.path.clone()
                                        };
                                        let value = msg_state.sp_value_from_path(&v.name);
                                        if value.is_none() {
                                            log_info!("Not in msg: Name {}, path: {}, state: {}", &v.name, &p, SPStateJson::from_state_flat(&msg_state).to_json());
                                        } 
                                        (p, value)
                                    }).collect();
                                    for (x, y) in map.iter() {
                                        if y.is_none() {
                                            return;
                                        }
                                    }
                                    let mut map:Vec<(SPPath, SPValue)> = map.into_iter().map(|(x, y)| (x, y.unwrap().clone())).collect();

                                    map.push((r_path.add_child("timestamp"), SPValue::now()));

                                    let state = SPState::new_from_values(&map);
                                    let time_stamp = std::time::Instant::now();
                                    let m = RosMessage {
                                        state,
                                        resource: r_path.clone(),
                                        time_stamp,
                                    };
                                    println!("INCOMING!!!: {:?}", &m);
                                    tx.send(m).unwrap();
                                }
                            };
                            println!("setting up subscription to topic: {}", topic);
                            let _subref =
                                node.0
                                    .subscribe_untyped(&topic_str, &topic_message_type, Box::new(cb))?;
                        }
                    }
                }

            
        }

        let (tx_out, rx_out): (channel::Sender<SPState>, channel::Receiver<SPState>) = channel::unbounded();
        thread::spawn(move || loop {
            match rx_out.recv() {
                Ok(state) => {
                    for rp in &ros_pubs {
                        (rp)(state.clone());
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
        node: &mut RosNode, tx_in: channel::Sender<SPState>,
    ) -> Result<channel::Sender<SPState>, Error> {
        let set_state_topic = format!{"{}/set_state", SP_NODE_NAME};

        let tx_in = tx_in.clone();
        let cb = move |msg: r2r::std_msgs::msg::String| {
            let json: Result<serde_json::Value, serde_json::Error> = serde_json::from_str(&msg.data);
            let json_s = json.and_then(|x| SPStateJson::from_json(x));
            if let Err(e) = json_s {
                log_info!(
                    "Could not convert incoming runner command, error: {:?}",
                    msg
                );
            } else {
                let new_state = json_s.unwrap().to_state();
                tx_in.send(new_state).expect("Can not send runner commmands. Threads are dead?");
            }
        };

        println!("setting up subscription to topic: {}", set_state_topic);
        let _subref = node.0.subscribe(&set_state_topic, Box::new(cb))?;

        let state_topic = &format!{"{}/state", SP_NODE_NAME};
        let state_flat_topic = &format!{"{}/state_flat", SP_NODE_NAME};
        println!("setting up publishing to topic: {}", state_topic);
        let rp = node
            .0
            .create_publisher::<r2r::std_msgs::msg::String>(state_topic)?;
        let rp_flat = node
            .0
            .create_publisher::<r2r::std_msgs::msg::String>(state_flat_topic)?;

        let info_cb = move |state: &SPState| {
            let state_json = SPStateJson::from_state_recursive(state);
            let state_flat = SPStateJson::from_state_flat(state);
            let json = state_json.to_json().to_string();
            let json_flat = state_flat.to_json().to_string();
            let msg = r2r::std_msgs::msg::String { data: json };
            let msg_flat = r2r::std_msgs::msg::String { data: json_flat };
            rp.publish(&msg).unwrap();
            rp_flat.publish(&msg_flat).unwrap();
        };

        let resource_topic = format!{"{}/resources", SP_NODE_NAME};
        let path = SPPath::from_string("registered_resources");
        let rp = node.0.create_publisher::<r2r::sp_messages::msg::Resources>(&resource_topic)?;
        let send_resource_list = move |s: &SPState| {
            if let Some(SPValue::Array(SPValueType::Path, xs)) = s.sp_value_from_path(&path) {
                let resources: Vec<String> = xs.iter().map(|x| {
                    if let SPValue::Path(p) = x {
                        p.drop_root().to_string()
                    } else {
                        x.to_json().to_string()
                    }
                }).collect();
                let msg =  r2r::sp_messages::msg::Resources {
                    resources
                };
                let res = rp.publish(&msg);
                if res.is_err() {
                    println!(
                        "RosComm resources not working for {}, error: {:?}",
                        &resource_topic,
                        res
                    );
                }
            }
        };

        let (tx_out, rx_out) = channel::unbounded();
        thread::spawn(move || loop {
            match rx_out.recv() {
                Ok(x) => {
                    (info_cb)(&x);
                    (send_resource_list)(&x);
                },
                Err(e) => {
                    println!("RosMisc out did not work: {:?}", e);
                    break;
                }
            }
        });

        Ok(tx_out)
    }


    pub fn ros_resource_comm_setup(
        node: &mut RosNode,
        tx_to_runner: channel::Sender<ROSResource>,
        prefix_path: &SPPath,
    ) -> Result<(), Error> {
        let resource_topic = "sp/resource";
        let prefix_path = prefix_path.clone();

        let tx_in = tx_to_runner.clone();
        let cb = move |msg: r2r::sp_messages::msg::RegisterResource| {
                let mut p = SPPath::from_string(&msg.path);
                p.add_parent_path_mut(&prefix_path);
                let model: Result<SPItem, _> = serde_json::from_str(&msg.model);
                let last_goal_from_sp: Result<SPStateJson, _> = serde_json::from_str(&msg.last_goal_from_sp);
                println!("GOT A RESOURCE: {:?}, {:?}", &msg, &last_goal_from_sp);
                let last = last_goal_from_sp.map(|x| {
                    let mut s = x.to_state();
                    let mut goal_path = p.clone();
                    goal_path.add_child_path_mut(&SPPath::from_string("goal"));
                    s.prefix_paths(&goal_path);
                    s
                });
                let resource = ROSResource{
                    path:  p,
                    model: model.ok(),
                    last_goal_from_sp: last.ok(),
                };

                tx_in.send(resource).expect("Can not send the ROSResource. Threads are dead?");

        };
        println!("setting up subscription to topic: {}", resource_topic);
        let _subref = node.0.subscribe(resource_topic, Box::new(cb))?;

        Ok(())
    }

    #[cfg(test)]
    mod ros_tests {
        use super::*;

    }
}

pub use ros::*;
