use std::sync::{Arc, Mutex};
use sp_domain::*;
use futures::*;

use crate::ros::*;


pub(crate) struct SPStateService {
    arc_node: Arc<Mutex<r2r::Node>>,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    handle_set: Option<tokio::task::JoinHandle<()>>,
    handle_get: Option<tokio::task::JoinHandle<()>>,
    handle_pub: Option<tokio::task::JoinHandle<()>>,
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
                handle_set: None,
                handle_get: None,
                handle_pub: None,
            };
            x.launch_services().await?;
            Ok(x)
    }

    pub fn abort(&self)  {
        if let Some(h) = &self.handle_set {
            h.abort();
        }
        if let Some(h) = &self.handle_get {
            h.abort();
        }
        if let Some(h) = &self.handle_pub {
            h.abort();
        }
    }

    pub async fn abort_and_await(&mut self) -> Result<(), SPError> {
        self.abort();
        if let Some(h) = self.handle_set.take() {
            h.await.map_err(SPError::from_any)?
        }
        if let Some(h) = self.handle_get.take() {
            h.await.map_err(SPError::from_any)?
        }
        if let Some(h) = self.handle_pub.take() {
            h.await.map_err(SPError::from_any)?
        }
        Ok(())
    }

    async fn launch_services(&mut self) -> Result<(), SPError> {
        let mut node = self.arc_node.lock().unwrap();
        let set_state_srv =
            node
            .create_service::<r2r::sp_msgs::srv::Json::Service>(&format! {"{}/set_state", super::ros::SP_NODE_NAME})
            .map_err(SPError::from_any)?;
        let get_state_srv =
            node
            .create_service::<r2r::sp_msgs::srv::Json::Service>(&format! {"{}/get_state", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
        let pub_state =
            node
            .create_publisher::<r2r::std_msgs::msg::String>(&format! {"{}/state", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
        let pub_state_flat =
            node
            .create_publisher::<r2r::std_msgs::msg::String>(&format! {"{}/state_flat", SP_NODE_NAME})
            .map_err(SPError::from_any)?;

        let tx = self.state_to_runner.clone();
        let rx = self.state_from_runner.clone();
        let handle_set = tokio::spawn(async move {
            SPStateService::set_service(set_state_srv, tx).await;
        });
        let rx = self.state_from_runner.clone();
        let handle_get = tokio::spawn(async move {
            SPStateService::get_service(get_state_srv, rx).await;
        });
        let rx = self.state_from_runner.clone();
        let handle_pub = tokio::spawn(async move {
            SPStateService::publish_state(rx,pub_state, pub_state_flat).await;
        });

        self.handle_set = Some(handle_set);
        self.handle_get = Some(handle_get);
        self.handle_pub = Some(handle_pub);

        Ok(())
    }



    async fn set_service(
        mut service: impl Stream<Item = r2r::ServiceRequest<r2r::sp_msgs::srv::Json::Service>> + Unpin,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    ) {
        // log_info!("SET SERVICE STARTING");
        loop {
            // log_info!("SET SERVICE Waiting");
            if let Some(request) = service.next().await {
                let state_json: Result<SPStateJson, _> = serde_json::from_str(&request.message.json);
                // log_info!("SET SERVICE GOT: {:?}", &state_json);
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
                let msg = r2r::sp_msgs::srv::Json::Response{json: r};
                request.respond(msg);
            }
        }
    }

    async fn get_service(
        mut service: impl Stream<Item = r2r::ServiceRequest<r2r::sp_msgs::srv::Json::Service>> + Unpin,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
    ) {
        loop {
            if let Some(request) = service.next().await {
               let s = SPStateJson::from_state_recursive(&state_from_runner.borrow());
                let resp = serde_json::to_string(&s).unwrap();
                let msg = r2r::sp_msgs::srv::Json::Response{json: resp};

                request.respond(msg);
            }
        }
    }

    async fn publish_state(
        mut state_from_runner: tokio::sync::watch::Receiver<SPState>,
        pub_state: r2r::Publisher<r2r::std_msgs::msg::String>,
        pub_flat_state: r2r::Publisher<r2r::std_msgs::msg::String>,
    ) {
        loop {
            if let Ok(request) = state_from_runner.changed().await {
                let s = state_from_runner.borrow();
                let s_json = SPStateJson::from_state_recursive(&s);
                let s_json_flat = SPStateJson::from_state_flat(&s);
                let msg = r2r::std_msgs::msg::String{data: serde_json::to_string(&s_json).unwrap()};
                let msg_flat = r2r::std_msgs::msg::String{data: serde_json::to_string(&s_json_flat).unwrap()};
                pub_state.publish(&msg).unwrap();
                pub_flat_state.publish(&msg_flat).unwrap();
            }
        }
    }
}

impl Drop for SPStateService {
    fn drop(&mut self) {
        self.abort();
    }
}


#[cfg(test)]
mod sp_comm_tests {
    use super::*;
    use serial_test::serial;
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
            for i in 1..20 {
                println!("Spin {}", i);
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



    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    #[serial]
    async fn set_state_services() {
        let p1 = SPPath::from_slice(&["a", "b"]);
        let p2 = SPPath::from_slice(&["a", "c"]);
        let init_state = state!(p1 => 1, p2 => false);

        let (mut tx_watch, rx_watch) = tokio::sync::watch::channel(init_state.clone());
        let (tx_mpsc, mut rx_mpsc) = mpsc::channel(2);

        let (arc_node, kill) = create_node("test_service");

        let mut state_service = SPStateService::new(
            arc_node.clone(),
            rx_watch.clone(),
            tx_mpsc.clone(),
        ).await.unwrap();

        let s = init_state.clone();
        let reply = tokio::spawn(async move {
            let k = rx_mpsc.recv().await.unwrap();
            println!("We got {}", &k.projection());
            assert!(k.sp_value_from_path(&p1) == s.sp_value_from_path(&p1));
        });

        let client = {
            let mut node = arc_node.lock().unwrap();
            let c = node.create_client::<r2r::sp_msgs::srv::Json::Service>("/sp/set_state").unwrap();
            println!("waiting for service...");

            //node.is_available(&c).unwrap().await;
            println!("service available.");
            c
        };

        let state_json = SPStateJson::from_state_recursive(&init_state);
        let req = r2r::sp_msgs::srv::Json::Request { json: serde_json::to_string_pretty(&state_json).unwrap() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        println!("response from server: {:?}", &res.json);
        assert!(res.json == "ok");

        // bad request
        let req = r2r::sp_msgs::srv::Json::Request { json: "no state".to_string() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        println!("error response from server: {:?}", &res.json);
        assert!(res.json != "ok");


        {
            let mut k = kill.lock().unwrap();
            *k = true;
        }

        reply.await.unwrap();

        let res = state_service.abort_and_await().await;
        println!("I got when abort and await: {:?}", res);

    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    #[serial]
    async fn get_state_services() {
        let p1 = SPPath::from_slice(&["a", "b"]);
        let p2 = SPPath::from_slice(&["a", "c"]);
        let init_state = state!(p1 => 1, p2 => false);

        let (mut tx_watch, rx_watch) = tokio::sync::watch::channel(init_state.clone());
        let (tx_mpsc, mut rx_mpsc) = mpsc::channel(2);

        let (arc_node, kill) = create_node("test_service");

        let mut state_service = SPStateService::new(
            arc_node.clone(),
            rx_watch.clone(),
            tx_mpsc.clone(),
        ).await.unwrap();

        let client = {
            let mut node = arc_node.lock().unwrap();
            let c = node.create_client::<r2r::sp_msgs::srv::Json::Service>("/sp/get_state").unwrap();
            println!("waiting for service...");

            node.is_available(&c).unwrap().await;
            println!("service available.");
            c
        };

        let state_json = SPStateJson::from_state_recursive(&init_state);
        let req = r2r::sp_msgs::srv::Json::Request { json: "{}".to_string() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        let json: SPStateJson = serde_json::from_str(&res.json).unwrap();


        println!("response from server: {}", json.to_state().projection());

        let p = SPPath::from_string("a/b");
        assert!(init_state.sp_value_from_path(&p1) == json.to_state().sp_value_from_path(&p));


        {
            let mut k = kill.lock().unwrap();
            *k = true;
        }

        let res = state_service.abort_and_await().await;
        println!("I got when abort and await: {:?}", res);

    }

}
