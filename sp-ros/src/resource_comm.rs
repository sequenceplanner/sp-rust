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
                log_warn!("The resource communiation couldn't be created: {:?}", &e);
                println!("The resource communiation couldn't be created: {:?}", &e);
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
        let subscriber = {
            let mut node = self.arc_node.lock().unwrap();
            node
            .subscribe_untyped(&self.mess.topic.to_string(), &topic_message_type(&self.mess))
        };

        match subscriber {
            Err(e) => {
                log_error!("the subscriber {} did not start: {:?}", &self.mess.topic.to_string(), e);
                Err(SPError::from_any(e))
            },
            Ok(mut sub) => {
                let sender = self.state_to_runner.clone();
                let resource_path = self.resource_path.clone();
                let mess = self.mess.clone();
                let handle = tokio::task::spawn(async move { 
                    loop {
                        let msg = sub
                            .next()
                            .await;
                            
                        match msg {
                            Some(Ok(v)) => {
                                match ros_to_state(v, &resource_path, &mess, &mess.variables).map_err(SPError::from_any) {
                                    Ok(s) => {sender.send(s).await;},
                                    Err(e) => {
                                        log_warn!("ros_to_state didnt work: {:?}", e);
                                    } 
                                }
                            },
                            Some(Err(e)) => {
                                log_warn!("subscriber didnt work: {:?}", e);
                            }
                            _ => {
                                log_warn!("subscriber is none");
                            }
                        }
        
                        
                    }
                });
        
                self.handle = Some(handle);
        
                Ok(())
            }
        }

        
        
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
        let publisher = {
            let mut node = self.arc_node.lock().unwrap();
            node
            .create_publisher_untyped(
                &self.mess.topic.to_string(), 
                &topic_message_type(&self.mess)
            ).map_err(SPError::from_any)
        };

        if let Err(e) = publisher {
            log_error!("the publisher {} did not start: {:?}", &self.mess.topic.to_string(), e);
            return Err(SPError::from_any(e))
        }
        let publisher = publisher.unwrap();

        let mut state_from_runner = self.state_from_runner.clone();
        let resource_path = self.resource_path.clone();
        let mess = self.mess.clone();
        let handle = tokio::task::spawn(async move { 
            loop {
                state_from_runner.changed().await;
                let state = state_from_runner.borrow();

                if !mess.send_predicate.eval(&state_from_runner.borrow()) {
                    println!("YYY Send_predicate says no: {:?}", &mess);
                    continue;
                }

                let msg = state_to_ros(
                    &mess, 
                    state, 
                    &mess.variables);
                let x = msg.and_then(|m| {
                    Ok(publisher.publish(m).map_err(SPError::from_any)?)
                });
                if let Err(e) = x {
                    log_warn!("The publisher {} failed: {:?}", mess.topic.to_string(), e);
                };

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

        println!("YYY");

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
        let mess = self.mess.clone();
        let client = {
            let mut n = self.arc_node.lock().unwrap();
            let c = n.create_client_untyped(
                &mess.topic.to_string(), 
                &topic_message_type(&mess)
            );
            if let Err(e) = c {
                log_error!("the service client {} did not start: {:?}", &self.mess.topic.to_string(), e);
                return Err(SPError::from_any(e))
            }
            let c = c.unwrap();
            println!("YYY STARTAR SERVICE: {:?}", &mess);
            // Did not work??
            // while !n.service_available_untyped(&c).unwrap() {
            //     tokio::time::sleep(std::time::Duration::from_millis(100));
            // }
            println!("YYY Startade: {:?}", &mess);
            c
        };
        
        let mut state_from_runner = self.state_from_runner.clone();
        let state_to_runner = self.state_to_runner.clone();
        let resource_path = self.resource_path.clone();
        let handle = tokio::task::spawn(async move { 
            loop {
                println!("YYY Waiting: {:?}", &mess);
                state_from_runner.changed().await;

                if !mess.send_predicate.eval(&state_from_runner.borrow()) {
                    println!("YYY Send_predicate says no: {:?}", &mess);
                    continue;
                }
                

                let msg = state_to_ros(
                    &mess, 
                    state_from_runner.borrow(), 
                    &mess.variables
                );

                println!("YYY VI FICK state: {:?}", &msg);

                if let Err(e) = msg {
                    log_warn!("The service client {} failed: {:?}", mess.topic.to_string(), e);
                    continue;
                };

                let msg = msg.unwrap();
                let request = client.request(msg);
                if let Err(e) = request {
                    log_warn!("The service client request {} failed: {:?}", mess.topic.to_string(), e);
                    // TODO: Maybe write to state?
                    continue;
                };
                let result = request.unwrap().await;
                println!("YYY VI FICK response: {:?}", &result);
                let result = result.and_then(|x| x).and_then(|x| Ok(x));
                if let Err(e) = result {
                    log_warn!("The service client response {} failed: {:?}", mess.topic.to_string(), e);
                    // TODO: Maybe write to state?
                    continue;
                };
                let result = result.unwrap();
                let x = ros_to_state(
                    result, 
                    &resource_path, 
                    &mess, 
                    &mess.variables_response
                );
                match x {
                    Ok(s) => {
                        state_to_runner.send(s).await.unwrap();
                    },
                    Err(e) => {
                        println!("ros_to_state didnt work: {:?}", e);
                        log_warn!("ros_to_state didnt work: {:?}", e);
                    } 
                }   
            }
        });

        self.handle = Some(handle);

        Ok(())
    }

    //fn extract_request(mess: &Message, state: &SPState) -> Vec<SPPath>
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
    vars: &Vec<MessageVariable>,
) -> Result<SPState, SPError> {

    let msg = SPStateJson::from_json(msg)?;
    let mut msg_state = msg.to_state();
    if m.message_type == MessageType::Json
        || m.message_type == MessageType::JsonFlat 
        {
        msg_state.unprefix_paths(&SPPath::from_string("data"));
    };

    let mut map: Vec<(SPPath, SPValue)> = vars
        .iter()
        .flat_map(|v| {
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
        value.map(|v|(p, v.clone()))
    })
    .collect();
        
    map.push((resource_path.add_child("timestamp"), SPValue::now()));

    let state = SPState::new_from_values(&map);
    Ok(state)
        
}

fn state_to_ros(
    m: &Message,
    state: tokio::sync::watch::Ref<SPState>,
    vars: &Vec<MessageVariable>,
) -> Result<serde_json::Value, SPError> {
    let res: Vec<(SPPath, SPValue)>  = 
        m.variables.iter().flat_map(|v| {
            let name = v.name.clone();
            let value = state.sp_value_from_path(&v.path);
            if value.is_none() {
                log_info!("Not in state: Name {}, path: {}, state: {}", 
                    &name,  &v.path, SPStateJson::from_state_flat(&state).to_json());
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


fn send_predicate (
    m: &Message,
    state: tokio::sync::watch::Ref<SPState>,
) -> bool {
    m.send_predicate.eval(&state)
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

