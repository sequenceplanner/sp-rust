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
        _node: &mut RosNode, _model: &Model, _tx_in: channel::Sender<SPState>,
    ) -> Result<channel::Sender<SPState>, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn roscomm_setup_misc(
        _node: &mut RosNode, _tx_in: channel::Sender<SPState>,
    ) -> Result<channel::Sender<SPState>, Error> {
        bail!(format_err!("ROS support not compiled in"));
    }


    pub fn ros_resource_comm_setup(
        _node: &mut RosNode, _tx_to_runner: channel::Sender<ROSResource>, _prefix_path: &SPPath,
    ) -> Result<(), Error> {
        bail!(format_err!("ROS support not compiled in"));
    }

    pub fn spin(_node: &mut RosNode) {}
}



#[cfg(feature = "ros")]
mod ros {
    use std::collections::{HashMap, HashSet};
    // the actual mod
    use std::sync::{Arc, Mutex};
    use std::time::Instant;
    use futures::{TryFutureExt, future};
    use futures::StreamExt;

    use sp_domain::*;

    const SP_NODE_NAME: &str = "sp";
    

    #[derive(Debug, PartialEq, Clone)]
    pub struct RosMessage {
        pub state: SPState,
        pub resource: SPPath,
        pub time_stamp: std::time::Instant,
    }

    pub async fn launch_ros_comm(
        receiver: tokio::sync::watch::Receiver<SPState>,
        sender: tokio::sync::mpsc::Sender<SPState>,
        sender_model: tokio::sync::mpsc::Sender<Model>,
    ) -> Result<(), SPError> {
            let ctx = r2r::Context::create().map_err(SPError::map)?;
            let node = r2r::Node::create(ctx, SP_NODE_NAME, "").map_err(SPError::map)?;
            
            let arc_node = Arc::new(Mutex::new(node));

            let rx_model_internal = sp_comm(arc_node.clone(), receiver.clone(), sender.clone(), sender_model.clone())?;
            
            resource_comm(arc_node.clone(), receiver.clone(), rx_model_internal, sender.clone());

            tokio::task::spawn_blocking( move || {
                loop {
                    let mut node = arc_node.lock().unwrap();
                    node.spin_once(std::time::Duration::from_millis(100));
                }
            });

            Ok(())
    }







    

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





    fn sp_comm(
        arc_node: Arc<Mutex<r2r::Node>>,
        receiver: tokio::sync::watch::Receiver<SPState>,
        sender: tokio::sync::mpsc::Sender<SPState>,
        sender_model: tokio::sync::mpsc::Sender<Model>
    ) -> Result<tokio::sync::watch::Receiver<Model>, SPError> {

        let mut node = arc_node.lock().unwrap();
        
        // Set state service
        let state_tx = sender.clone();
        let mut set_state_srv = 
            node
            .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/set_state", SP_NODE_NAME})
            .map_err(SPError::map)?;
        tokio::task::spawn(async move {
            loop {
                if let Some(request) = set_state_srv.next().await {
                    let state_json = serde_json::from_str(&request.message.json).and_then(SPStateJson::from_json);
                    let r = match state_json {
                        Ok(sj) => { 
                            let mut state = sj.to_state();
                            state.add_variable(SPPath::from_string("/set_state_timestamp"), SPValue::now());
                            state_tx.send(state).await.expect("The state channel should always work!");
                            "ok".to_string()
                            
                        },
                        Err(e) => {
                            let error = format!(
                                "The set state service request {} is not a json: {}",
                                &request.message.json,
                                &e
                            );
                            log_error!("{}", &error);
                            error
                        }
                    };
                    let resp = serde_json::json!({
                        "response": r,
                        "time_stamp": std::time::SystemTime::now(),
                    });
                    let msg = r2r::sp_messages::srv::Json::Response{json: resp.to_string()};
                    
                    request.respond(msg);
                }
            }
        });
        
        
        // Set model service
        let model_tx = sender_model.clone();
        let mut set_model_srv = 
            node
            .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/set_model", SP_NODE_NAME})
            .map_err(SPError::map)?;
        let (internal_model_tx, internal_model_rx) = tokio::sync::watch::channel(Model::new("empty_model", vec!()));
        tokio::task::spawn(async move {
            loop {
                if let Some(request) = set_model_srv.next().await {
                    let model: Result<Model, _> = serde_json::from_str(&request.message.json);
                    let r = match model {
                        Ok(m) => {
                            model_tx.send(m.clone()).await.expect("The model channel should always work!");
                            internal_model_tx.send(m).expect("The internal model channel should always work!");
                            "ok".to_string()
                        },
                        Err(e) => {
                            let error = format!(
                                "The set model service request {} can not be converted to a model: {}",
                                &request.message.json,
                                &e
                            );
                            log_error!("{}",&error);
                            error
                        }
                    };
                    let resp = serde_json::json!({
                        "response": r,
                        "time_stamp": std::time::SystemTime::now()
                    });
                    let msg = r2r::sp_messages::srv::Json::Response{json: resp.to_string()};
                    request.respond(msg);
                }
            }
        });



        // State out
        let mut state_rx = receiver.clone();
        let pub_state = node
            .create_publisher::<r2r::std_msgs::msg::String>(&format! {"{}/state", SP_NODE_NAME})
            .map_err(SPError::map)?;
        let pub_flat_state = node
            .create_publisher::<r2r::std_msgs::msg::String>(&format! {"{}/state_flat", SP_NODE_NAME})
            .map_err(SPError::map)?;
        tokio::task::spawn(async move {
            loop {
                state_rx.changed().await.expect("The state out channel from the runner should never fail");
                let s = state_rx.borrow();
                let s_json = SPStateJson::from_state_recursive(&s);
                let s_json_flat = SPStateJson::from_state_flat(&s);
                let msg = r2r::std_msgs::msg::String{data: serde_json::to_string_pretty(&s_json).unwrap()};
                let msg_flat = r2r::std_msgs::msg::String{data: serde_json::to_string_pretty(&s_json_flat).unwrap()};
                pub_state.publish(&msg).unwrap(); 
                pub_flat_state.publish(&msg_flat).unwrap(); 
            }
        });

        Ok(internal_model_rx)
    }
    


    fn resource_comm(
        arc_node: Arc<Mutex<r2r::Node>>,
        receiver: tokio::sync::watch::Receiver<SPState>,
        mut receiver_model: tokio::sync::watch::Receiver<Model>,
        sender: tokio::sync::mpsc::Sender<SPState>
    ) {
        tokio::task::spawn(async move {
            let mut channels: HashSet<SPPath> = HashSet::new();
            loop {
                receiver_model.changed().await.expect("The internal model channel should never fail");
                let model = receiver_model.borrow();

                // Check if a resource comm is not started, then start.
                // If the resource comm is already started, this does nothing
                // to restart a resource comm for now, restart sp
                for resource in model.all_resources() {
                    if !channels.contains(resource.path()) {
                        channels.insert(resource.path().clone());
                        for mess in &resource.messages {
                            let topic_message_type = match &mess.message_type {
                                MessageType::Ros(x) => x.clone(),
                                _ => "std_msgs/msg/String".to_string(),
                            };
                            let topic_str = mess.topic.to_string();
                            let m = mess.clone();
                            let resource_path = resource.path().clone();
                            match &mess.category {
                                MessageCategory::OutGoing => {
                                    let mut node = arc_node.lock().unwrap();
                                    let publisher = 
                                        node
                                        .create_publisher_untyped(
                                            &topic_str, 
                                            &topic_message_type)
                                            .expect("Could not create a publisher");
                                    let receiver = receiver.clone();
                                    
                                    tokio::task::spawn(async move { 
                                        outgoing(
                                            publisher, 
                                            m, 
                                            resource_path,
                                            receiver)
                                            .await
                                            .expect("The outgoing communication failed");
                                    });
                                },
                                MessageCategory::Incoming => {
                                    let mut node = arc_node.lock().unwrap();
                                    let subscriber = 
                                        node
                                        .subscribe_untyped(&topic_str, &topic_message_type)
                                        .expect("Could not create a subscriber");
                                    let sender = sender.clone();
                                    tokio::task::spawn(async move { 
                                        incoming(
                                            subscriber, 
                                            m,
                                            resource_path,
                                            sender,
                                        )
                                        .await
                                        .expect("The incoming communication failed");
                                    });
                                },
                                MessageCategory::Service => {},
                                MessageCategory::Action => {},
                            }
                        }
                    }
                }
            }
        });
    }


    async fn incoming(
        mut subscriber: impl futures::Stream<Item = r2r::Result<serde_json::Value>> + Unpin,
        mess: Message,
        resource_path: SPPath,
        sender: tokio::sync::mpsc::Sender<SPState>,
    ) -> Result<(), SPError> {
            loop {
                let msg = subscriber
                    .next()
                    .await
                    .ok_or(SPError::No(format!("The subscriber stream for {}, on incoming is getting None! SHOULD NOT HAPPEN!!",&resource_path)))?
                    .map_err(SPError::map)?;
    
                let rm = ros_to_state(msg, &resource_path, &mess).map_err(SPError::map)?;
                sender.send(rm).await.map_err(SPError::map)?;
            }
    }

    async fn outgoing(
        publisher: r2r::PublisherUntyped,
        m: Message,
        resource_path: SPPath,
        mut receiver: tokio::sync::watch::Receiver<SPState>,
    ) -> Result<(), SPError>  {
            loop {
                receiver.changed().await;
                let state = receiver.borrow();
                let msg = state_to_ros(
                    &m, 
                    state, 
                    &resource_path)?;

                publisher.publish(msg).map_err(SPError::map)?;
            }
    }




    fn ros_to_state(
        msg: serde_json::Value,
        resource_path: &SPPath,
        m: &Message,
    ) -> Result<SPState, SPError> {

        let msg = SPStateJson::from_json(msg)?;
        let mut msg_state = msg.to_state();
        if m.message_type == MessageType::Json
            || m.message_type == MessageType::JsonFlat 
            {
            msg_state.unprefix_paths(&SPPath::from_string("data"));
        };

        let map: Vec<(SPPath, Option<&SPValue>)> = m
            .variables
        .iter()
        .map(|v| {
            let p = if v.relative_path {
                resource_path.add_child_path(&v.path)
            } else {
                v.path.clone()
            };
            let value = msg_state.sp_value_from_path(&v.name);
            if value.is_none() {
                log_error!(
                    "Not in msg: Name {}, path: {}, state: {}",
                    &v.name,
                    &p,
                    SPStateJson::from_state_flat(&msg_state).to_json()
                );
            }
            (p, value)
        })
        .collect();
            
        let mut map: Vec<(SPPath, SPValue)> = map
            .into_iter()
            .filter(|(_, y)| y.is_some())
            .map(|(x, y)| (x, y.unwrap().clone()))
            .collect();

        map.push((resource_path.add_child("timestamp"), SPValue::now()));

        let state = SPState::new_from_values(&map);
        Ok(state)
            
    }

    fn state_to_ros(
        m: &Message,
        state: tokio::sync::watch::Ref<SPState>,
        resource_path: &SPPath,
    ) -> Result<serde_json::Value, SPError> {
        let res: Vec<(SPPath, SPValue)>  = 
            m.variables.iter().flat_map(|v| {
                let name = v.name.clone();
                let path = resource_path.add_child_path(&v.path.clone());
                let value = state.sp_value_from_path(&path);
                if value.is_none() {
                    log_info!("Not in state: Name {}, resource: {}, path: {}, state: {}", 
                        &name, &resource_path, &path, SPStateJson::from_state_flat(&state).to_json());
                } 
                value.map(|v| (name, v.clone()))
            }).collect();
        

        let msg = match m.message_type {
            MessageType::JsonFlat => {
                let json = SPStateJson::from_state_flat(
                    &SPState::new_from_values(&res),
                );
                let json = serde_json::to_string(&json).unwrap();
                let mut map = serde_json::Map::new();
                map.insert("data".to_string(), serde_json::Value::String(json));
                serde_json::Value::Object(map)
            }
            MessageType::Json => {
                let json = SPStateJson::from_state_recursive(
                    &SPState::new_from_values(&res),
                );
                let json = serde_json::to_string(&json).unwrap();
                let mut map = serde_json::Map::new();
                map.insert("data".to_string(), serde_json::Value::String(json));
                serde_json::Value::Object(map)
            }
            MessageType::Ros(_) => {
                let json = SPStateJson::from_state_recursive(
                    &SPState::new_from_values(&res),
                );
                serde_json::to_value(&json).unwrap()
            }
        };

        Ok(msg)

    }
    

    // fn handle_incoming(msg: r2r::Result<serde_json::Value>, tx: channel::Sender<SPState>,
    //                    r_path: SPPath, m: Message, topic_cb: String) {
    //     let json = msg.unwrap();
    //     let json_s = SPStateJson::from_json(json);
    //     if let Err(e) = json_s {
    //         log_info!(
    //             "Could not convert incoming message on {}, error: {:?}",
    //             &topic_cb,
    //             e
    //         );
    //     } else {
    //         let x = json_s.unwrap();
    //         let mut msg_state = x.to_state();
    //         if m.message_type == MessageType::Json
    //             || m.message_type == MessageType::JsonFlat
    //         {
    //             msg_state.unprefix_paths(&SPPath::from_string("data"));
    //         }

    //         let map: Vec<(SPPath, Option<&SPValue>)> = m
    //             .variables
    //             .iter()
    //             .map(|v| {
    //                 let p = if v.relative_path {
    //                     r_path.add_child_path(&v.path)
    //                 } else {
    //                     v.path.clone()
    //                 };
    //                 let value = msg_state.sp_value_from_path(&v.name);
    //                 if value.is_none() {
    //                     log_info!(
    //                         "Not in msg: Name {}, path: {}, state: {}",
    //                         &v.name,
    //                         &p,
    //                         SPStateJson::from_state_flat(&msg_state).to_json()
    //                     );
    //                 }
    //                 (p, value)
    //             })
    //             .collect();
    //         for (_, y) in map.iter() {
    //             if y.is_none() {
    //                 return;
    //             }
    //         }
    //         let mut map: Vec<(SPPath, SPValue)> = map
    //             .into_iter()
    //             .map(|(x, y)| (x, y.unwrap().clone()))
    //             .collect();

    //         map.push((r_path.add_child("timestamp"), SPValue::now()));

    //         let state = SPState::new_from_values(&map);
    //         let time_stamp = std::time::Instant::now();
    //         let m = RosMessage {
    //             state,
    //             resource: r_path.clone(),
    //             time_stamp,
    //         };
    //         tx.send(m).unwrap();
    //     }
    // }

    // pub fn roscomm_setup(
    //     node: &mut RosNode, model: &Model, tx_in: channel::Sender<SPState>,
    // ) -> Result<channel::Sender<SPState>, Error> {
    //     let mut ros_pubs = Vec::new();

    //     let rcs = model.all_resources();

    //     for r in rcs {
    //         for m in &r.messages {
    //             println!("WE HAVE A NEW MESSAGE IN A RESOURCE: {:?}", m);
    //             let topic = if m.relative_topic {
    //                 r.path().add_child_path(&m.topic).drop_root() // TODO. Change the path of resources so that this is not needed. Or send it in as input?
    //             } else {
    //                 m.topic.clone()
    //             };
    //             let topic_message_type = match &m.message_type {
    //                 MessageType::Ros(x) => x.clone(),
    //                 _ => "std_msgs/msg/String".to_string(),
    //             };
    //             let topic_str = topic.to_string();
    //             let resource_path = r.path().clone();
    //             match m.category {
    //                 MessageCategory::OutGoing => {
    //                     println!("setting up publishing to topic NEW: {}", topic);

    //                     let rp = node
    //                         .0
    //                         .create_publisher_untyped(&topic_str, &topic_message_type)?;

    //                     let m = m.clone();
    //                     let cb = move |state: SPState| {
    //                         let res: Vec<(SPPath, Option<&SPValue>)> = m.variables.iter().map(|v| {
    //                                 let name = v.name.clone();
    //                                 let path = resource_path.add_child_path(&v.path.clone());
    //                                 let value = state.sp_value_from_path(&path);
    //                                 // if value.is_none() {
    //                                 //     log_info!("Not in state: Name {}, resource: {}, path: {}, state: {}", &name, &resource_path, &path, SPStateJson::from_state_flat(&state).to_json());
    //                                 // }
    //                                 (name, value)
    //                             }).collect();

    //                         for (_, y) in res.iter() {
    //                             if y.is_none() {
    //                                 return;
    //                             }
    //                         }
    //                         let res: Vec<(SPPath, SPValue)> = res
    //                             .into_iter()
    //                             .map(|(x, y)| (x, y.unwrap().clone()))
    //                             .collect();

    //                         let msg = match m.message_type {
    //                             MessageType::JsonFlat => {
    //                                 let json = SPStateJson::from_state_flat(
    //                                     &SPState::new_from_values(&res),
    //                                 );
    //                                 let json = serde_json::to_string(&json).unwrap();
    //                                 let mut map = serde_json::Map::new();
    //                                 map.insert("data".to_string(), serde_json::Value::String(json));
    //                                 serde_json::Value::Object(map)
    //                             }
    //                             MessageType::Json => {
    //                                 let json = SPStateJson::from_state_recursive(
    //                                     &SPState::new_from_values(&res),
    //                                 );
    //                                 let json = serde_json::to_string(&json).unwrap();
    //                                 let mut map = serde_json::Map::new();
    //                                 map.insert("data".to_string(), serde_json::Value::String(json));
    //                                 serde_json::Value::Object(map)
    //                             }
    //                             MessageType::Ros(_) => {
    //                                 let json = SPStateJson::from_state_recursive(
    //                                     &SPState::new_from_values(&res),
    //                                 );
    //                                 serde_json::to_value(&json).unwrap()
    //                             }
    //                         };
    //                         let res = rp.publish(msg.clone());
    //                         if res.is_err() {
    //                             log_info!(
    //                                 "RosComm not working for {}, error: {:?}, msg: {}, state: {}",
    //                                 &topic_str,
    //                                 res,
    //                                 msg,
    //                                 SPStateJson::from_state_flat(&state).to_json()
    //                             );
    //                         }
    //                     };
    //                     ros_pubs.push(cb);
    //                 }
    //                 MessageCategory::Incoming => {
    //                     let tx = tx_in.clone();
    //                     let r_path = r.path().clone();
    //                     let m = m.clone();
    //                     let topic_cb = topic_str.clone();

    //                     println!("setting up subscription to topic: {}", topic);
    //                     let sub = node.0.subscribe_untyped(&topic_str,&topic_message_type)?;

    //                     tokio::task::spawn(async move {
    //                         sub.for_each(move |msg| {
    //                             let tx = tx.clone();
    //                             let r_path = r_path.clone();
    //                             let m = m.clone();
    //                             let topic_cb = topic_cb.clone();
    //                             handle_incoming(msg, tx, r_path, m, topic_cb);
    //                             future::ready(())
    //                         }).await
    //                     });
    //                 },
    //                 _ => panic!("NOT IMPLEMENTED SERVICES AND ACTIONS YET")
    //             }
    //         }
    //     }

    //     let (tx_out, rx_out): (channel::Sender<SPState>, channel::Receiver<SPState>) =
    //         channel::unbounded();
    //     thread::spawn(move || loop {
    //         match rx_out.recv() {
    //             Ok(state) => {
    //                 for rp in &ros_pubs {
    //                     (rp)(state.clone());
    //                 }
    //             }
    //             Err(e) => {
    //                 println!("RosComm out did not work: {:?}", e);
    //                 break;
    //             }
    //         }
    //     });

    //     Ok(tx_out)
    // }

    // pub fn roscomm_setup_misc(
    //     node: &mut RosNode, tx_in: channel::Sender<SPState>,
    // ) -> Result<channel::Sender<SPState>, Error> {
    //     let set_state_topic = format! {"{}/set_state", SP_NODE_NAME};


    //     println!("setting up subscription to topic: {}", set_state_topic);
    //     let sub = node.0.subscribe(&set_state_topic)?;

    //     let tx_in = tx_in.clone();
    //     tokio::task::spawn(async move {
    //         sub.for_each(|msg: r2r::std_msgs::msg::String| {
    //             let json: Result<serde_json::Value, serde_json::Error> =
    //                 serde_json::from_str(&msg.data);
    //             let json_s = json.and_then(|x| SPStateJson::from_json(x));
    //             if let Err(e) = json_s {
    //                 log_info!(
    //                     "Could not convert incoming runner command, msg: {:?}, error: {}",
    //                     msg, e
    //                 );
    //             } else {
    //                 let new_state = json_s.unwrap().to_state();
    //                 tx_in
    //                     .send(new_state)
    //                     .expect("Can not send runner commmands. Threads are dead?");
    //             }
    //             future::ready(())
    //         }).await
    //     });

    //     let state_topic = &format! {"{}/state", SP_NODE_NAME};
    //     let state_flat_topic = &format! {"{}/state_flat", SP_NODE_NAME};
    //     println!("setting up publishing to topic: {}", state_topic);
    //     let rp = node
    //         .0
    //         .create_publisher::<r2r::std_msgs::msg::String>(state_topic)?;
    //     let rp_flat = node
    //         .0
    //         .create_publisher::<r2r::std_msgs::msg::String>(state_flat_topic)?;

    //     let info_cb = move |state: &SPState| {
    //         let state_json = SPStateJson::from_state_recursive(state);
    //         let state_flat = SPStateJson::from_state_flat(state);
    //         let json = state_json.to_json().to_string();
    //         let json_flat = state_flat.to_json().to_string();
    //         let msg = r2r::std_msgs::msg::String { data: json };
    //         let msg_flat = r2r::std_msgs::msg::String { data: json_flat };
    //         rp.publish(&msg).unwrap();
    //         rp_flat.publish(&msg_flat).unwrap();
    //     };

    //     let resource_topic = format! {"{}/resources", SP_NODE_NAME};
    //     let path = SPPath::from_string("registered_resources");
    //     let rp = node
    //         .0
    //         .create_publisher::<r2r::sp_messages::msg::Resources>(&resource_topic)?;
    //     let send_resource_list = move |s: &SPState| {
    //         if let Some(SPValue::Array(SPValueType::Path, xs)) = s.sp_value_from_path(&path) {
    //             let resources: Vec<String> = xs
    //                 .iter()
    //                 .map(|x| {
    //                     if let SPValue::Path(p) = x {
    //                         p.drop_root().to_string()
    //                     } else {
    //                         x.to_json().to_string()
    //                     }
    //                 })
    //                 .collect();
    //             let msg = r2r::sp_messages::msg::Resources { resources };
    //             let res = rp.publish(&msg);
    //             if res.is_err() {
    //                 println!(
    //                     "RosComm resources not working for {}, error: {:?}",
    //                     &resource_topic, res
    //                 );
    //             }
    //         }
    //     };

    //     let (tx_out, rx_out) = channel::unbounded();
    //     thread::spawn(move || loop {
    //         match rx_out.recv() {
    //             Ok(x) => {
    //                 (info_cb)(&x);
    //                 (send_resource_list)(&x);
    //             }
    //             Err(e) => {
    //                 println!("RosMisc out did not work: {:?}", e);
    //                 break;
    //             }
    //         }
    //     });

    //     Ok(tx_out)
    // }

    // pub fn ros_resource_comm_setup(
    //     node: &mut RosNode, tx_to_runner: channel::Sender<ROSResource>, prefix_path: &SPPath,
    // ) -> Result<(), Error> {
    //     let resource_topic = "sp/resource";
    //     let prefix_path = prefix_path.clone();

    //     println!("setting up subscription to topic: {}", resource_topic);
    //     let sub = node.0.subscribe(resource_topic)?;
    //     let tx_in = tx_to_runner.clone();
    //     tokio::task::spawn(async move {
    //         sub.for_each(|msg: r2r::sp_messages::msg::RegisterResource| {
    //             let mut p = SPPath::from_string(&msg.path);
    //             p.add_parent_path_mut(&prefix_path);
    //             let model: Result<SPItem, _> = serde_json::from_str(&msg.model);
    //             let last_goal_from_sp: Result<SPStateJson, _> =
    //                 serde_json::from_str(&msg.last_goal_from_sp);
    //             println!("GOT A RESOURCE: {:?}, {:?}", &msg, &last_goal_from_sp);
    //             let last = last_goal_from_sp.map(|x| {
    //                 let mut s = x.to_state();
    //                 let mut goal_path = p.clone();
    //                 goal_path.add_child_path_mut(&SPPath::from_string("goal"));
    //                 s.prefix_paths(&goal_path);
    //                 s
    //             });
    //             let resource = ROSResource {
    //                 path: p,
    //                 model: model.ok(),
    //                 last_goal_from_sp: last.ok(),
    //             };

    //             tx_in
    //                 .send(resource)
    //                 .expect("Can not send the ROSResource. Threads are dead?");
    //             future::ready(())
    //         }).await
    //     });

    //     Ok(())
    // }

    #[cfg(test)]
    mod ros_tests {
        use super::*;
    }
}

pub use ros::*;
