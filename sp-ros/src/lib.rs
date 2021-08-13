#[cfg(not(feature = "ros"))]
mod ros {
    use sp_domain::SPError;

    // empty when no ros support compiled in.

    #[derive(Debug, PartialEq, Clone)]
    pub struct RosMessage {
        pub state: SPState,
        pub resource: SPPath,
        pub time_stamp: std::time::Instant,
    }

    pub fn launch_ros_comm(
        receiver: tokio::sync::watch::Receiver<SPState>,
        sender: tokio::sync::mpsc::Sender<SPState>,
        sender_model: tokio::sync::mpsc::Sender<Model>,
    ) -> Result<(), SPError> {
        SPError::No("You can not start the ROS communication without the ros feature flag!")
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

    
}



#[cfg(feature = "ros")]
mod ros {
    use std::collections::{HashSet};
    use std::sync::{Arc, Mutex};
    use r2r::ServiceRequest;
    use sp_domain::*;
    use futures::*;

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


    pub fn launch_ros_comm(
        receiver: tokio::sync::watch::Receiver<SPState>,
        sender: tokio::sync::mpsc::Sender<SPState>,
        sender_model: tokio::sync::watch::Sender<Model>,
        receiver_model: tokio::sync::watch::Receiver<Model>
    ) -> Result<(), SPError> {
            let ctx = r2r::Context::create().map_err(SPError::from_any)?;
            let node = r2r::Node::create(ctx, SP_NODE_NAME, "").map_err(SPError::from_any)?;
            
            let arc_node = Arc::new(Mutex::new(node));

            sp_comm(arc_node.clone(), receiver.clone(), sender.clone(), sender_model)?;
            
            resource_comm(arc_node.clone(), receiver.clone(), receiver_model, sender.clone());

            tokio::task::spawn_blocking( move || {
                loop {
                    let mut node = arc_node.lock().unwrap();
                    node.spin_once(std::time::Duration::from_millis(100));
                }
            });

            Ok(())
    }

    pub struct RosComm {
        spin_handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
        sp_comm: SPComm,
        resources: Vec<ResourceComm>,
    }

    struct ResourceComm {
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
        resource: Resource,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        comms: Vec<Comm>,
    }

    enum Comm {
        Subscriber(SubscriberComm),
        Publisher(PublisherComm),
        ServiceClient(ServiceClientComm),
        ActionClient(ActionClientComm)
    }
    struct SubscriberComm{
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
    }
    struct PublisherComm{
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,

    }
    struct ServiceClientComm{
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
    }
    struct ActionClientComm{
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
    }

    

    struct SPComm {
        state_service: SPStateService,
        model_service: SPModelService,
        control_service: SPControlService,
    }

    pub struct SPStateService {
        arc_node: Arc<Mutex<r2r::Node>>,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        handle: Option<tokio::task::JoinHandle<()>>,
    }

    struct SPModelService {
        arc_node: Arc<Mutex<r2r::Node>>,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        current_model: tokio::sync::watch::Sender<Model>,
        model_watcher: tokio::sync::watch::Receiver<Model>,
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
    }

    struct SPControlService {
        arc_node: Arc<Mutex<r2r::Node>>,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
    }


    impl SPStateService {
        pub async fn new(
            arc_node: Arc<Mutex<r2r::Node>>,
            state_from_runner: tokio::sync::watch::Receiver<SPState>,
            state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        ) -> Result<SPStateService, SPError> {
                let mut x = SPStateService{
                    arc_node,
                    state_from_runner,
                    state_to_runner,
                    handle: None,
                };
                x.launch_services().await?;
                Ok(x)
        }

        async fn launch_services(&mut self) -> Result<(), SPError> {
            let mut node = self.arc_node.lock().unwrap();
            let set_state_srv = 
                node
                .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/set_state", SP_NODE_NAME})
                .map_err(SPError::from_any)?;
            let get_state_srv = 
                node
                .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/get_state", SP_NODE_NAME})
                .map_err(SPError::from_any)?;
            
            let tx = self.state_to_runner.clone();
            let rx = self.state_from_runner.clone();
            let handle = tokio::spawn(async move {
                tokio::spawn(async move {
                    SPStateService::set_service(set_state_srv, tx).await;
                });
                tokio::spawn(async move {
                    SPStateService::get_service(get_state_srv, rx).await;
                });
            });

            self.handle = Some(handle);

            Ok(())
        }

        pub async fn abort(self) -> Result<(), tokio::task::JoinError>  {
            if let Some(h) = self.handle {
                h.abort();
                h.await?;
            }
            Ok(())
        }

        async fn set_service(
            mut service: impl Stream<Item = ServiceRequest<r2r::sp_messages::srv::Json::Service>> + Unpin,
            state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        ) {
            loop {
                if let Some(request) = service.next().await {
                    let state_json = serde_json::from_str(&request.message.json).and_then(SPStateJson::from_json);
                    let r = match state_json {
                        Ok(sj) => { 
                            let mut state = sj.to_state();
                            state.add_variable(SPPath::from_string("/sp/set-state/timestamp"), SPValue::now());
                            state_to_runner.send(state).await.expect("The state channel should always work!");
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
        }

        async fn get_service(
            mut service: impl Stream<Item = ServiceRequest<r2r::sp_messages::srv::Json::Service>> + Unpin,
            state_from_runner: tokio::sync::watch::Receiver<SPState>,
        ) {
            loop {
                if let Some(request) = service.next().await {
                   let s = SPStateJson::from_state_recursive(&state_from_runner.borrow());
                    let resp = serde_json::json!({
                        "response": serde_json::to_string(&s).unwrap(),
                        "time_stamp": std::time::SystemTime::now(),
                    });
                    let msg = r2r::sp_messages::srv::Json::Response{json: resp.to_string()};
                    
                    request.respond(msg);
                }
            }
        }
    }


    

   





    fn sp_comm(
        arc_node: Arc<Mutex<r2r::Node>>,
        receiver: tokio::sync::watch::Receiver<SPState>,
        sender: tokio::sync::mpsc::Sender<SPState>,
        sender_model: tokio::sync::watch::Sender<Model>
    ) -> Result<tokio::sync::watch::Receiver<Model>, SPError> {

        let mut node = arc_node.lock().unwrap();
        
        // Set state service
        let state_tx = sender.clone();
        let mut set_state_srv = 
            node
            .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/set_state", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
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
        let model_tx = sender_model;
        let mut set_model_srv = 
            node
            .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/set_model", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
        let (internal_model_tx, internal_model_rx) = tokio::sync::watch::channel(Model::new("empty_model"));
        tokio::task::spawn(async move {
            loop {
                if let Some(request) = set_model_srv.next().await {
                    let model: Result<Model, _> = serde_json::from_str(&request.message.json);
                    let r = match model {
                        Ok(m) => {
                            model_tx.send(m.clone()).expect("The model channel should always work!");
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
            .map_err(SPError::from_any)?;
        let pub_flat_state = node
            .create_publisher::<r2r::std_msgs::msg::String>(&format! {"{}/state_flat", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
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
                for resource in model.resources.iter() {
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
                    .map_err(SPError::from_any)?;
    
                let rm = ros_to_state(msg, &resource_path, &mess).map_err(SPError::from_any)?;
                sender.send(rm).await.map_err(SPError::from_any)?;
            }
    }

    async fn outgoing(
        publisher: r2r::PublisherUntyped,
        m: Message,
        resource_path: SPPath,
        mut receiver: tokio::sync::watch::Receiver<SPState>,
    ) -> Result<(), SPError>  {
            loop {
                receiver.changed().await.expect("The receiver in outgoing should always work!");
                let state = receiver.borrow();
                let msg = state_to_ros(
                    &m, 
                    state, 
                    &resource_path)?;

                publisher.publish(msg).map_err(SPError::from_any)?;
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

}

#[cfg(test)]
mod ros_tests {
    use super::*;
    use sp_domain::*;
    use tokio::sync::{mpsc, watch};
    use std::{sync::{Arc, Mutex}};

    fn create_node(name: &str) -> (Arc<Mutex<r2r::Node>>, Arc<Mutex<bool>>) {
        let ctx = r2r::Context::create().map_err(SPError::from_any).unwrap();
        let node = r2r::Node::create(ctx, name, "").map_err(SPError::from_any).unwrap();
        let arc = Arc::new(Mutex::new(node));
        let kill = Arc::new(Mutex::new(false));

        let a = arc.clone();
        let k = kill.clone();
        tokio::task::spawn_blocking( move || {
            loop {
                println!("Spin");
                {
                    let mut node = a.lock().unwrap();
                    node.spin_once(std::time::Duration::from_millis(1000));
                }
                {
                    if *k.lock().unwrap() {
                        break;
                    }
                }
            }
        });


        (arc, kill)
    }

    fn init_state() -> SPState {
        state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true)
    }


    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn sp_state_services() {

            let (mut tx_watch, rx_watch) = tokio::sync::watch::channel(init_state());
            let (tx_mpsc, mut rx_mpsc) = mpsc::channel(2);

            let (arc_node, kill) = create_node("test_service");

            let x = SPStateService::new(
                arc_node.clone(), 
                rx_watch.clone(), 
                tx_mpsc.clone(),
            ).await.unwrap();

            let reply = tokio::spawn(async move {
                let k = rx_mpsc.recv().await.unwrap();
                println!("We got {}", &k);
                //assert!(k.projection() == init_state().projection());
            });

                println!("in client");
                let client = {
                    let mut node = arc_node.lock().unwrap();
                    let c = node.create_client::<r2r::sp_messages::srv::Json::Service>("/sp/set_state").unwrap();
                    println!("waiting for service...");

                    while !node.service_available(&c).unwrap() {
                        std::thread::sleep(std::time::Duration::from_millis(1000));
                    };
                    println!("service available.");
                    c
                };

                let req = r2r::sp_messages::srv::Json::Request { json: serde_json::to_string_pretty(&init_state()).unwrap() };
                let res = client.request(&req).unwrap();
                let res = res.await.unwrap();

                println!("I GOT: {:?}", res);


                *kill.lock().unwrap() = false;
                reply.await.unwrap();
        
    }

}

pub use ros::*;
    

   


