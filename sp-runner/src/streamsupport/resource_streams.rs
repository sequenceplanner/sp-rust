//! Stream support to communicate with resources with a sink and source resource
//!
#![allow(dead_code)]

use sp_domain::*;

use tokio::prelude::*;
use tokio::sync::mpsc;
use std::collections::HashMap;

use serde::{Serialize, Deserialize};
use std::sync::{Arc, Mutex};


/// The Mockresource is used for testing and maybe also internal SP resources that connects
/// the output states with the input states. Define a struct and impl MockResourcePoll, then
/// includit here to make a internal resource.
#[derive(Debug)]
pub struct MockResource<T> {
    send: MockSend,
    recv: MockReceive<T>,
}

impl<T> MockResource<T> where T: MockTransform + Clone + Sync + Send {
    pub fn new(t: T) -> MockResource<T> {
        let s = Arc::new(Mutex::new(SPState::default()));
        let send = MockSend{
            internal_state: s.clone()
        };
        let recv = MockReceive{
            resource: t,
            internal_state: s.clone()
        };
        MockResource {
            send, recv
        }
    }
}

#[derive(Debug)]
pub struct MockSend {
    internal_state: Arc<Mutex<SPState>>,
}

impl MockSend {
    pub fn make_future(self, sink: mpsc::Receiver<SPState>) -> impl Future<Item = (), Error = ()> {
        sink
        .for_each(move |s| {
            //println!("I am in send: {:?}", s);
            let mut x = self.internal_state.lock().unwrap();
            *x = s;
            Ok(())
        })
        .map_err(|e| eprintln!("error = {:?}", e))
    }
}



#[derive(Debug)]
pub struct MockReceive<T> {
    resource: T,
    internal_state: Arc<Mutex<SPState>>,
}

use std::time::{Duration, Instant};
use tokio::timer::Interval;

impl<T> MockReceive<T> where T: MockTransform + Clone + Sync + Send {
    pub fn make_future(mut self, source: mpsc::Sender<AssignState>) -> impl Future<Item = (), Error = ()> {
        let task = Interval::new(Instant::now(), Duration::from_secs(1))
            .map_err(|_| ())
            .for_each(move |_x| {
                let x = self.internal_state.lock().unwrap();
                let upd = self.resource.upd_state(&x);

                let send = source
                    .clone()
                    .send(upd)
                    .map_err(|_| ())
                    .map(|_| ());
                tokio::spawn(send)
            });
        task
    }
}


pub trait MockTransform {
    fn upd_state(&mut self, state: &SPState) -> AssignState;
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
struct DummyRobot {
    pub ref_path: SPPath,
    pub act_path: SPPath,
    pub act: SPValue
}

impl MockTransform for DummyRobot {
    fn upd_state(&mut self, state: &SPState) -> AssignState {

        let ref_value = match state.get_value(&self.ref_path) {
            Some(SPValue::Int32(x)) => *x,
            _ => 0
        };
        let mut act_value = match self.act {
            SPValue::Int32(x) => x,
            _ => 0
        };
        println!("ref {:?}, act {:?}", ref_value, act_value);
        if ref_value > act_value {
            act_value = act_value + 1;
        } else if ref_value < act_value {
            act_value = act_value - 1;
        };

        let mut s = HashMap::new();
        let act_v = act_value.to_spvalue();
        s.insert(self.act_path.clone(), AssignStateValue::SPValue(act_v.clone()));
        self.act = act_v;
        AssignState{s}
    }
}


/// ********** TESTS ***************
///


#[cfg(test)]
mod mock_resource_test {
    use super::*;
    #[test]
    fn test_me() {
        let r = MockResource::new(DummyRobot{
            ref_path: SPPath::from_array(&["ref"]),
            act_path: SPPath::from_array(&["act"]),
            act: 0.to_spvalue(),
        });
        println!("{:?}", r);

        let (to_buf, into_buf) = tokio::sync::mpsc::channel::<SPState>(1);
        let (outof_buf, from_buf) = tokio::sync::mpsc::channel::<AssignState>(1);

        let init_state = state!(
            ["ref"] => 10,
            ["act"] => 0
        );


        tokio::run(future::lazy(move || {
            tokio::spawn(r.send.make_future(into_buf));
            tokio::spawn(r.recv.make_future(outof_buf));

            let res = from_buf
                .for_each(|x| {
                    println!("{:?}", x);
                    Ok(())
                })
                .map_err(|_| ())
                .map(|_| ());
            tokio::spawn(res);

            let send = to_buf
                    .clone()
                    .send(init_state)
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
            tokio::spawn(send)

        }));
    }

}
