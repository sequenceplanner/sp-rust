//! Stream support to communicate with resources with a sink and source resource
//! 

use sp_domain::*;

use tokio::prelude::*;
use tokio::sync::mpsc;
use futures::try_ready;
use std::collections::HashMap;

use serde::{Serialize, Deserialize};

/// The resource sink must be implemented by each type of outgoing interface we need in SP.
/// It is initialize with a receiver channel that the implementer must store and use.
/// TODO: Maybe we should send out StateExternal instead
pub trait ResourceSink {
    fn sink_channel(&mut self, channel: mpsc::Receiver<SPState>);
}

/// The resource source must be implemented by each type of incoming interfaces used in SP.
/// It is initialized with a sender channel into the runner buffer.
pub trait ResourceSource {
    fn source_channel(&mut self, channel: mpsc::Sender<AssignState>);
}




/// The Mockresource is used for testing and maybe also internal SP resources that connects
/// the output states with the input states. Define a struct and impl MockResourcePoll, then
/// includit here to make a internal resource.
#[derive(Debug)]
pub struct MockResource<T> {
    resource: T,
    source: Option<mpsc::Sender<AssignState>>,
    sink: Option<mpsc::Receiver<SPState>>,
}

pub trait MockResourcePoll {
    fn upd_state(&mut self, state: &SPState) -> AssignState;
}

impl<T> ResourceSink for MockResource<T> {
    fn sink_channel(&mut self, channel: mpsc::Receiver<SPState>) {
        self.sink = Some(channel);
    }
}

impl<T> ResourceSource for MockResource<T> {
    fn source_channel(&mut self, channel: mpsc::Sender<AssignState>) {
        self.source = Some(channel);
    }
}

impl<T: MockResourcePoll> Future for MockResource<T> {
    type Item = Option<()>;
    type Error = SPError;
    fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
        match (self.sink.as_mut(), self.source.as_ref()) {
            (Some(sink), Some(source)) => {
                match try_ready!(sink.poll().map_err(|e| SPError::No(format!("{:?}", e)))) {
                    Some(s) => {
                        let upd_s = self.resource.upd_state(&s);
                        let send = source
                            .clone()
                            .send(upd_s)
                            .map(move |_| ())
                            .map_err(|e| eprintln!("error = {:?}", e));
                        tokio::spawn(send);
                        task::current().notify();
                        Ok(Async::NotReady)
                    },
                    None => Ok(Async::Ready(None)),
                }
            },
            _ => return Err(SPError::No("The resource has not been initialized".to_string()))
        }
    }
}





