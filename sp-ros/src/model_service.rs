use std::{borrow::Borrow, sync::{Arc, Mutex}};
use sp_domain::*;
use futures::*;

use crate::ros::*;

/// The SPModelService wrapps the set and get model services in SP. By calling the /sp/set_model, a new model can be
/// loaded by SP. The /sp/get_model just returns the current loaded model.
/// Maybe TODO: use the state to return ok from the set_model service when the new model have been loaded. 
pub(crate) struct SPModelService {
    arc_node: Arc<Mutex<r2r::Node>>,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    model_watcher: tokio::sync::watch::Receiver<Model>,
    handle: Option<tokio::task::JoinHandle<()>>,
}

impl SPModelService {
    pub async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        initial_model: Model,
    ) -> Result<SPModelService, SPError> { 
        let (current_model, model_watcher) = tokio::sync::watch::channel(initial_model);
        let mut ms = SPModelService {
            arc_node,
            state_from_runner,
            state_to_runner,
            model_watcher,
            handle: None
        };

        ms.launch_services(current_model).await?;

        Ok(ms)
    }

    pub async fn abort(self) -> Result<(), tokio::task::JoinError>  {
        if let Some(h) = self.handle {
            h.abort();
            h.await?;
        }
        Ok(())
    }

    pub fn model_watcher(&self) -> tokio::sync::watch::Receiver<Model> {
        self.model_watcher.clone()
    }

    async fn launch_services(
        &mut self, 
        current_model: tokio::sync::watch::Sender<Model>
    ) -> Result<(), SPError> {
        let mut node = self.arc_node.lock().unwrap();
        let set_srv = 
            node
            .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/set_model", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
        let get_srv = 
            node
            .create_service::<r2r::sp_messages::srv::Json::Service>(&format! {"{}/get_model", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
        
        
        let rx = self.model_watcher.clone();
        let handle = tokio::spawn(async move {
            let x = tokio::spawn(async move {
                SPModelService::set_service(set_srv, current_model).await;
            });
            let y = tokio::spawn(async move {
                SPModelService::get_service(get_srv, rx).await;
            });

            let err = try_join! {x, y};
            log_error!("The sp model service has stopped working. Should never happen!: {:?}", &err);
            panic!("The sp model service has stopped working. Should never happen!: {:?}", err);

        });

        self.handle = Some(handle);

        Ok(())
    }

    async fn set_service(
        mut service: impl Stream<Item = r2r::ServiceRequest<r2r::sp_messages::srv::Json::Service>> + Unpin,
        mut current_model: tokio::sync::watch::Sender<Model>,
    ) {
        loop {
            if let Some(request) = service.next().await {
                let state_json: Result<Model, _> = serde_json::from_str(&request.message.json);
                let r = match state_json {
                    Ok(m) => { 
                        match current_model.send(m) {
                            Ok(_) => "ok".to_string(),
                            Err(e) => e.to_string(),
                        }
                    },
                    Err(e) => {
                        let error = format!(
                            "The set model service request {} is not a model: {}",
                            &request.message.json,
                            &e
                        );
                        log_error!("{}", &error);
                        error
                    }
                };
                let msg = r2r::sp_messages::srv::Json::Response{json: r};
                request.respond(msg);
            }
        }
    }

    async fn get_service(
        mut service: impl Stream<Item = r2r::ServiceRequest<r2r::sp_messages::srv::Json::Service>> + Unpin,
        watch_model: tokio::sync::watch::Receiver<Model>,
    ) {
        loop {
            if let Some(request) = service.next().await {
                let x = watch_model.borrow().clone();
                 let resp = serde_json::to_string(&x).unwrap();
                 let msg = r2r::sp_messages::srv::Json::Response{json: resp};
                 
                 request.respond(msg);
             }
        }
    }
}



#[cfg(test)]
mod sp_comm_tests {
    use super::*;
    use sp_domain::*;
    use tokio::sync::{mpsc, watch};
    use std::{sync::{Arc, Mutex}};
    use serial_test::serial;

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
    async fn set_model_services() {
        let mut m = Model::new("test");

        let (mut tx_watch, rx_watch) = tokio::sync::watch::channel(SPState::new());
        let (tx_mpsc, mut rx_mpsc) = mpsc::channel(2);

        let (arc_node, kill) = create_node("test_model_set");

        let model_service = SPModelService::new(
            arc_node.clone(), 
            rx_watch.clone(), 
            tx_mpsc.clone(),
            Model::new("hej")
        ).await.unwrap();

        let mut watch = model_service.model_watcher();
        let reply = tokio::spawn(async move {
            loop {
                watch.changed().await.unwrap();
                let m = watch.borrow();
                println!("We got model: {:?}", m);
                if m.path() == &SPPath::from_string("new") {
                    break;
                }
            }
        });

        let client = {
            let mut node = arc_node.lock().unwrap();
            let c = node.create_client::<r2r::sp_messages::srv::Json::Service>("/sp/set_model").unwrap();
            println!("waiting for service...");

            while !node.service_available(&c).unwrap() {
                std::thread::sleep(std::time::Duration::from_millis(100));
            };
            println!("service available.");
            c
        };

        let model = Model::new("new");
        let req = r2r::sp_messages::srv::Json::Request { json: serde_json::to_string_pretty(&model).unwrap() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        println!("response from server: {:?}", &res.json);
        assert!(res.json == "ok");

        // bad request
        let req = r2r::sp_messages::srv::Json::Request { json: "no model".to_string() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        println!("error response from server: {:?}", &res.json);
        assert!(res.json != "ok");

        {
            let mut k = kill.lock().unwrap();
            *k = true;
        }

        reply.await.unwrap();
        model_service.abort().await;
        
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    #[serial]
    async fn get_model_services() {
        let (mut tx_watch, rx_watch) = tokio::sync::watch::channel(SPState::new());
        let (tx_mpsc, mut rx_mpsc) = mpsc::channel(2);

        let (arc_node, kill) = create_node("test_service");

        let model_service = SPModelService::new(
            arc_node.clone(), 
            rx_watch.clone(), 
            tx_mpsc.clone(),
            Model::new("hej")
        ).await.unwrap();

        let client = {
            let mut node = arc_node.lock().unwrap();
            let c = node.create_client::<r2r::sp_messages::srv::Json::Service>("/sp/get_model").unwrap();
            println!("waiting for service...");

            while !node.service_available(&c).unwrap() {
                std::thread::sleep(std::time::Duration::from_millis(1000));
            };
            println!("service available.");
            c
        };

        let req = r2r::sp_messages::srv::Json::Request { json: "{}".to_string() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        let m: Model = serde_json::from_str(&res.json).unwrap();
        

        println!("response from server: {:?}", m);

        assert!(m.path() == &SPPath::from_string("hej"));


        {
            let mut k = kill.lock().unwrap();
            *k = true;
        }

        model_service.abort().await;
        
    }

}