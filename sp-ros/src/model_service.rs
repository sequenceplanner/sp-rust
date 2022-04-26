use std::sync::{Arc, Mutex};
use sp_domain::*;
use sp_formal::CompiledModel;
use super::RunnerModel;
use futures::*;

use crate::ros::*;

/// The SPModelService wrapps the set and get model services in SP. By calling the /sp/set_model, a new model can be
/// loaded by SP. The /sp/get_model just returns the current loaded model.
/// Maybe TODO: use the state to return ok from the set_model service when the new model have been loaded.
pub(crate) struct SPModelService {
    arc_node: Arc<Mutex<r2r::Node>>,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    model_watcher: tokio::sync::watch::Receiver<RunnerModel>,
    handle_set: Option<tokio::task::JoinHandle<()>>,
    handle_get: Option<tokio::task::JoinHandle<()>>,
    handle_change: Option<tokio::task::JoinHandle<()>>,
}

impl SPModelService {
    pub async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        initial_model: sp_formal::CompiledModel,
    ) -> Result<SPModelService, SPError> {
        let init = RunnerModel::new(initial_model);
        let (current_model, model_watcher) = tokio::sync::watch::channel(init);
        let mut ms = SPModelService {
            arc_node,
            state_from_runner,
            state_to_runner,
            model_watcher,
            handle_set: None,
            handle_get: None,
            handle_change: None
        };

        ms.launch_services(current_model).await?;

        Ok(ms)
    }

    pub fn abort(&self)  {
        if let Some(h) = &self.handle_set {
            h.abort();
        }
        if let Some(h) = &self.handle_get {
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
        Ok(())
    }

    pub fn model_watcher(&self) -> tokio::sync::watch::Receiver<RunnerModel> {
        self.model_watcher.clone()
    }

    async fn launch_services(
        &mut self,
        current_model: tokio::sync::watch::Sender<RunnerModel>
    ) -> Result<(), SPError> {
        let mut node = self.arc_node.lock().unwrap();
        let set_srv =
            node
            .create_service::<r2r::sp_msgs::srv::Json::Service>(&format! {"{}/set_model", SP_NODE_NAME})
            .map_err(SPError::from_any)?;
        let get_srv =
            node
            .create_service::<r2r::sp_msgs::srv::Json::Service>(&format! {"{}/get_model", SP_NODE_NAME})
            .map_err(SPError::from_any)?;


        let state_to_runner = self.state_to_runner.clone();
        let handle_set = tokio::spawn(async move {
            SPModelService::set_service(set_srv, current_model,
                                        state_to_runner).await;
        });

        let rx = self.model_watcher.clone();
        let handle_get = tokio::spawn(async move {
            SPModelService::get_service(get_srv, rx).await;
        });

        self.handle_set = Some(handle_set);
        self.handle_get = Some(handle_get);

        Ok(())
    }

    async fn set_service(
        mut service: impl Stream<Item = r2r::ServiceRequest<r2r::sp_msgs::srv::Json::Service>> + Unpin,
        current_model: tokio::sync::watch::Sender<RunnerModel>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    ) {
        loop {
            if let Some(request) = service.next().await {
                let compiled_model: Result<CompiledModel, _> = serde_json::from_str(&request.message.json);
                let model_change: Result<Model, _> = serde_json::from_str(&request.message.json);
                let r = match (compiled_model, model_change) {
                    (Ok(m), _) => {
                        let rm = RunnerModel::new(m);
                        match current_model.send(rm) {
                            Ok(_) => "ok".to_string(),
                            Err(e) => e.to_string(),
                        }
                    },
                    (_ , Ok(m)) => {
                        let mut cm = current_model.borrow().clone();
                        // update any modified intentions
                        cm.compiled_model.model.intentions.retain(|i| {
                            m.intentions.iter().find(|ii| ii.path() == i.path()).is_none()
                        });
                        cm.compiled_model.model.intentions.extend(m.intentions.clone());
                        // reset the state of the new/updated intentions
                        let new_values: Vec<_> = m.intentions.iter().map(|i| {
                                (i.path().clone(), "i".to_spvalue())
                            }).collect();
                        let new_state = SPState::new_from_values(&new_values);
                        if let Err(_) = state_to_runner.try_send(new_state) {
                            println!("could not update intention state");
                        }

                        cm.changes = Some(m);
                        match current_model.send(cm) {
                            Ok(_) => "ok".to_string(),
                            Err(e) => e.to_string(),
                        }
                    }
                    (Err(e1), Err(e2)) => {
                        let error = format!(
                            "The set model service request {} is not a compiled model: {}, or a change: {}",
                            &request.message.json,
                            &e1,
                            &e2
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
        watch_model: tokio::sync::watch::Receiver<RunnerModel>,
    ) {
        loop {
            if let Some(request) = service.next().await {
                let x = watch_model.borrow().clone();
                 let resp = serde_json::to_string(&x).unwrap();
                 let msg = r2r::sp_msgs::srv::Json::Response{json: resp};

                 request.respond(msg);
             }
        }
    }
}

impl Drop for SPModelService {
    fn drop(&mut self) {
        self.abort();
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
            let mut i = 0;
            loop {
                i+=1;
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

        let mut model_service = SPModelService::new(
            arc_node.clone(),
            rx_watch.clone(),
            tx_mpsc.clone(),
            CompiledModel::from(Model::new("hej"))
        ).await.unwrap();

        let mut watch = model_service.model_watcher();
        let reply = tokio::spawn(async move {
            loop {
                watch.changed().await.unwrap();
                let m = watch.borrow();
                println!("We got model: {:?}", m);
                if m.compiled_model.model.path() == &SPPath::from_string("new") {
                    break;
                }
            }
        });

        let (client, ready) = {
            let mut node = arc_node.lock().unwrap();
            let c = node.create_client::<r2r::sp_msgs::srv::Json::Service>("/sp/set_model").unwrap();

            let ready = node.is_available(&c).unwrap();
            (c, ready)
        };
        println!("waiting for service...");
        ready.await.expect("could wait for service");
        println!("service available.");

        let model = CompiledModel::from(Model::new("new"));
        let req = r2r::sp_msgs::srv::Json::Request { json: serde_json::to_string_pretty(&model).unwrap() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        println!("response from server: {:?}", &res.json);
        assert!(res.json == "ok");

        // bad request
        let req = r2r::sp_msgs::srv::Json::Request { json: "no model".to_string() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        println!("error response from server: {:?}", &res.json);
        assert!(res.json != "ok");

        {
            let mut k = kill.lock().unwrap();
            *k = true;
        }

        reply.await.unwrap();
        let res = model_service.abort_and_await().await;
        println!("I got when abort and await: {:?}", res);

    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    #[serial]
    async fn get_model_services() {
        let (mut tx_watch, rx_watch) = tokio::sync::watch::channel(SPState::new());
        let (tx_mpsc, mut rx_mpsc) = mpsc::channel(2);

        let (arc_node, kill) = create_node("test_service");

        let mut model_service = SPModelService::new(
            arc_node.clone(),
            rx_watch.clone(),
            tx_mpsc.clone(),
            CompiledModel::from(Model::new("hej"))
        ).await.unwrap();

        let (client, ready) = {
            let mut node = arc_node.lock().unwrap();
            let c = node.create_client::<r2r::sp_msgs::srv::Json::Service>("/sp/get_model").unwrap();

            let ready = node.is_available(&c).unwrap();
            (c, ready)
        };

        println!("waiting for service...");
        ready.await.expect("failed to complete waiting");
        println!("service available.");

        let req = r2r::sp_msgs::srv::Json::Request { json: "{}".to_string() };
        let res = client.request(&req).unwrap();
        let res = res.await.unwrap();
        let m: RunnerModel = serde_json::from_str(&res.json).unwrap();


        println!("response from server: {:?}", m);

        assert!(m.compiled_model.model.path() == &SPPath::from_string("hej"));


        {
            let mut k = kill.lock().unwrap();
            *k = true;
        }

        let res = model_service.abort_and_await().await;
        println!("I got when abort and await: {:?}", res);

    }

}
