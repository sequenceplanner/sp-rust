use std::{sync::{Arc, Mutex}, thread::JoinHandle};
use sp_domain::*;
use futures::*;


pub struct ResourceComm {
    arc_node: Arc<Mutex<r2r::Node>>,
    resource: Resource,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    comms: Vec<Comm>,
}

impl ResourceComm {
    pub async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        resource: Resource,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    ) -> Result<ResourceComm, SPError> {

        let mut comms = vec!();
        for mess in resource.messages() {
            let x = Comm::new(
                arc_node.clone(), 
                &resource,
                mess.clone(), 
                state_from_runner.clone(), 
                state_to_runner.clone(), 
                None)
                .await?;
            comms.push(x);
        }

        let rc = ResourceComm {
            arc_node,
            resource,
            state_from_runner,
            state_to_runner,
            comms
        };

        Ok(rc)
    }

    pub fn abort(&self)   {
        for c in &self.comms {
            c.abort();
        }
    }
    pub async fn abort_and_await(&mut self) -> Result<(), SPError>  {
        for c in &mut self.comms {
            c.abort_and_await().await.map_err(SPError::from_any)?;
        }
        Ok(())
    }

    pub fn resource(&self) -> &Resource {
        &self.resource
    }
}

impl Drop for ResourceComm {
    fn drop(&mut self) {
        self.abort();
    }
}



enum Comm {
    Subscriber(SubscriberComm),
    Publisher(PublisherComm),
    ServiceClient(ServiceClientComm),
    ActionClient(ActionClientComm)
}

impl Comm {
    async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        resource: &Resource,
        mess: Message,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
        handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
    ) -> Result<Comm, SPError> {
        let resource_path = resource.path().clone();
        let res = match mess.category {
            MessageCategory::OutGoing => {
                let x = PublisherComm::new(arc_node, mess, resource_path, state_from_runner).await;
                x.map(Comm::Publisher)
            },
            MessageCategory::Incoming => {
                let x = SubscriberComm::new(arc_node, mess, resource_path, state_to_runner).await;
                x.map(Comm::Subscriber)
            },
            MessageCategory::Service => {
                let x = ServiceClientComm::new(arc_node, mess, resource_path, state_from_runner, state_to_runner).await;
                x.map(Comm::ServiceClient)
            },
            MessageCategory::Action => {
                let x = ActionClientComm::new(arc_node, mess, resource_path, state_from_runner, state_to_runner).await;
                x.map(Comm::ActionClient)
            }  
        };
        match res {
            Ok(x) => Ok(x),
            Err(e) => {
                log_warn!("The resource communiation couln't be created: {:?}", &e);
                println!("The resource communiation couln't be created: {:?}", &e);
                Err(e)
            }
        }
    }

    pub fn abort(&self) {
        match self {
            Comm::Subscriber(x) => {
                x.abort();
            },
            Comm::Publisher(x) => {
                x.abort();
            },
            Comm::ActionClient(x) => {
                x.abort();
            },
            Comm::ServiceClient(x) => {
                x.abort();
            },
        }
    }

    pub async fn abort_and_await(&mut self) -> Result<(), tokio::task::JoinError>  {
        match self {
            Comm::Subscriber(x) => {
                Ok(x.abort_and_await().await?)
            },
            Comm::Publisher(x) => {
                Ok(x.abort_and_await().await?)
            },
            Comm::ActionClient(x) => {
                Ok(x.abort_and_await().await?)
            },
            Comm::ServiceClient(x) => {
                Ok(x.abort_and_await().await?)
            },
        }
    }
}

struct SubscriberComm{
    arc_node: Arc<Mutex<r2r::Node>>,
    mess: Message,
    resource_path: SPPath,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
}

impl SubscriberComm {
    async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        mess: Message,
        resource_path: SPPath,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    ) -> Result<SubscriberComm, SPError> { 
        let mut sc = SubscriberComm{
            arc_node,
            mess,
            resource_path,
            state_to_runner,
            handle: None
        };

        sc.launch().await?;

        Ok(sc)
    }

    pub fn abort(&self)  {
        if let Some(h) = &self.handle {
            h.abort();
        }
    }
    pub async fn abort_and_await(&mut self) -> Result<(), tokio::task::JoinError>  {
        self.abort();
        if let Some(h) = self.handle.take() {
            h.await?;
        }
        Ok(())
    }

    async fn launch(&mut self) -> Result<(), SPError> {
        let mut node = self.arc_node.lock().unwrap();
        let mut subscriber = 
            node
            .subscribe_untyped(&self.mess.topic.to_string(), &topic_message_type(&self.mess))
            .expect("Could not create a subscriber");
        
        let sender = self.state_to_runner.clone();
        let resource_path = self.resource_path.clone();
        let mess = self.mess.clone();
        let handle = tokio::task::spawn(async move { 
            loop {
                let msg = subscriber
                    .next()
                    .await
                    .ok_or(SPError::No(format!("The subscriber stream for {}, on incoming is getting None! SHOULD NOT HAPPEN!!",&resource_path)))?
                    .map_err(SPError::from_any)?;
    
                let rm = ros_to_state(msg, &resource_path, &mess).map_err(SPError::from_any)?;
                sender.send(rm).await.map_err(SPError::from_any)?;
            }
        });

        self.handle = Some(handle);

        Ok(())
    }
}

impl Drop for SubscriberComm {
    fn drop(&mut self) {
        self.abort();
    }
}





struct PublisherComm{
    arc_node: Arc<Mutex<r2r::Node>>,
    mess: Message,
    resource_path: SPPath,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
}

impl PublisherComm {
    async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        mess: Message,
        resource_path: SPPath,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
    ) -> Result<PublisherComm, SPError> { 
        let mut sc = PublisherComm {
            arc_node,
            mess,
            resource_path,
            state_from_runner,
            handle: None
        };

        sc.launch().await?;

        Ok(sc)
    }

    pub fn abort(&self)  {
        if let Some(h) = &self.handle {
            h.abort();
        }
    }
    pub async fn abort_and_await(&mut self) -> Result<(), tokio::task::JoinError>  {
        self.abort();
        if let Some(h) = self.handle.take() {
            h.await?;
        }
        Ok(())
    }

    async fn launch(&mut self) -> Result<(), SPError> {
        let mut node = self.arc_node.lock().unwrap();
        let publisher = 
            node
            .create_publisher_untyped(
                &self.mess.topic.to_string(), 
                &topic_message_type(&self.mess)
            ).map_err(SPError::from_any)?;

        let mut receiver = self.state_from_runner.clone();
        let resource_path = self.resource_path.clone();
        let m = self.mess.clone();
        let handle = tokio::task::spawn(async move { 
            loop {
                receiver.changed().await;
                let state = receiver.borrow();
                let msg = state_to_ros(
                    &m, 
                    state, 
                    &resource_path)?;

                publisher.publish(msg).map_err(SPError::from_any)?;
            }
        });

        self.handle = Some(handle);

        Ok(())
    }
}

impl Drop for PublisherComm {
    fn drop(&mut self) {
        self.abort();
    }
}









struct ServiceClientComm{
    arc_node: Arc<Mutex<r2r::Node>>,
    mess: Message,
    resource_path: SPPath,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
}

impl ServiceClientComm {
    async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        mess: Message,
        resource_path: SPPath,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    ) -> Result<ServiceClientComm, SPError> { 
        let mut sc = ServiceClientComm {
            arc_node,
            mess,
            resource_path,
            state_from_runner,
            state_to_runner,
            handle: None
        };

        sc.launch().await?;

        Ok(sc)
    }

    pub fn abort(&self)  {
        if let Some(h) = &self.handle {
            h.abort();
        }
    }
    pub async fn abort_and_await(&mut self) -> Result<(), tokio::task::JoinError>  {
        self.abort();
        if let Some(h) = self.handle.take() {
            h.await?;
        }
        Ok(())
    }

    async fn launch(&mut self) -> Result<(), SPError> {
        let mut node = self.arc_node.clone();
        let mut state_from_runner = self.state_from_runner.clone();
        let mess = self.mess.clone();

        let handle = tokio::task::spawn(async move { 
            loop {
                state_from_runner.changed().await;
                let x = state_from_runner.borrow();
                let client = {
                    let mut n = node.lock().unwrap();
                    n.create_client_untyped(
                    &mess.topic.to_string(), 
                    &topic_message_type(&mess)
                    ).map_err(SPError::from_any)?
                };



            }
        });

        self.handle = Some(handle);

        Ok(())
    }
}

impl Drop for ServiceClientComm {
    fn drop(&mut self) {
        self.abort();
    }
}







struct ActionClientComm{
    arc_node: Arc<Mutex<r2r::Node>>,
    mess: Message,
    resource_path: SPPath,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
}

impl ActionClientComm {
    async fn new(
        arc_node: Arc<Mutex<r2r::Node>>,
        mess: Message,
        resource_path: SPPath,
        state_from_runner: tokio::sync::watch::Receiver<SPState>,
        state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    ) -> Result<ActionClientComm, SPError> { 
        let mut sc = ActionClientComm {
            arc_node,
            mess,
            resource_path,
            state_from_runner,
            state_to_runner,
            handle: None
        };

        sc.launch().await?;

        Ok(sc)
    }

    pub fn abort(&self)  {
        if let Some(h) = &self.handle {
            h.abort();
        }
    }
    pub async fn abort_and_await(&mut self) -> Result<(), tokio::task::JoinError>  {
        self.abort();
        if let Some(h) = self.handle.take() {
            h.await?;
        }
        Ok(())
    }

    async fn launch(&mut self) -> Result<(), SPError> {
        let mut node = self.arc_node.clone();
        let mut state_from_runner = self.state_from_runner.clone();
        let mess = self.mess.clone();

        let handle = tokio::task::spawn(async move { 
            loop {
                state_from_runner.changed().await;
                let x = state_from_runner.borrow();
                let client = {
                    let mut n = node.lock().unwrap();
                    n.create_client_untyped(
                    &mess.topic.to_string(), 
                    &topic_message_type(&mess)
                    ).map_err(SPError::from_any)?
                };

                

            }
        });

        self.handle = Some(handle);

        Ok(())
    }
}

impl Drop for ActionClientComm {
    fn drop(&mut self) {
        self.abort();
    }
}







fn topic_message_type(mess: &Message) -> String {
    match &mess.message_type {
        MessageType::Ros(x) => x.clone(),
        _ => "std_msgs/msg/String".to_string(),
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
        let p =  v.path.clone();
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
            let value = state.sp_value_from_path(&v.path.clone());
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

    println!("HEJ HEJ: {:?}", &msg);

    Ok(msg)

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

        
        
        
    }


}

