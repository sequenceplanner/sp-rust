//! The incoming handler is the one that polls all incoming streams and forward them into the
//! runner buffer. This is automatically set up by the model
#![allow(dead_code)]

use super::*;
use sp_domain::*;
use tokio::sync::mpsc;

pub(crate) struct IncomingHandler {
    pub buffer: MessageBuffer<AssignState>,
    pub channel: mpsc::Sender<AssignState>,
    pub runner_channel: mpsc::Receiver<AssignState>
}

impl IncomingHandler {
    pub fn new() -> IncomingHandler {
        let (buffer, channel, runner_channel) = MessageBuffer::new(3, IncomingHandler::merge);
        IncomingHandler { buffer, channel, runner_channel }
    }

    pub fn extract(self) -> (MessageBuffer<AssignState>, mpsc::Sender<AssignState>, mpsc::Receiver<AssignState>)  {
        (self.buffer, self.channel, self.runner_channel)
    }


    fn merge(prev: &mut AssignState, next: &AssignState) -> bool {
        if next.will_overwrite(prev){
            false
        } else {
            prev.merge(next.clone());
            true
        }
    }
}




/// ********** TESTS ***************

#[cfg(test)]
mod incoming_test {
    use super::*;
    use std::collections::HashMap;
    use tokio::prelude::*;

    fn ab() -> SPPath { SPPath::from_array(&["a", "b"])}
    fn ac() -> SPPath { SPPath::from_array(&["a", "c"])}
    fn initial_ab() -> AssignState {
        let s: HashMap<SPPath, AssignStateValue> = [
            (ab(), AssignStateValue::SPValue(false.to_spvalue())),
        ].into_iter().cloned().collect();
        AssignState {
           s
        }
    }
    fn next_ab() -> AssignState {
        let s: HashMap<SPPath, AssignStateValue> = [
            (ab(), AssignStateValue::SPValue(true.to_spvalue())),
        ].into_iter().cloned().collect();
        AssignState {
           s
        }
    }
    fn initial_ac() -> AssignState {
        let s: HashMap<SPPath, AssignStateValue> = [
            (ac(), AssignStateValue::SPValue(false.to_spvalue())),
        ].into_iter().cloned().collect();
        AssignState {
           s
        }
    }
    fn next_ac() -> AssignState {
        let s: HashMap<SPPath, AssignStateValue> = [
            (ac(), AssignStateValue::SPValue(true.to_spvalue())),
        ].into_iter().cloned().collect();
        AssignState {
           s
        }
    }





    #[test]
    fn test_incoming() {

        tokio::run(future::lazy(move || {
            let incoming = IncomingHandler::new();
            let (buffer, channel, runner_channel) = incoming.extract();



            let range = vec![initial_ab(), next_ab(), initial_ab(), initial_ab(), next_ab(), initial_ab(), next_ab(), initial_ab(), initial_ab(), next_ab()];
            let ch = channel.clone();
            let sending = stream::iter_ok(range).for_each(move |s| {
                let send = ch
                    .clone()
                    .send(s)
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send)
            });
            tokio::spawn(sending);

            let range = vec![next_ac(), initial_ac(), initial_ac(), next_ac(), next_ac(), initial_ac(), initial_ac(), next_ac()];
            let ch = channel.clone();
            let sending = stream::iter_ok(range).for_each(move |s| {
                let send = ch
                    .clone()
                    .send(s)
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send)
            });
            tokio::spawn(sending);

            let getting = runner_channel
                .map_err(|_| ())
                .for_each(|result| {
                    println!("Yes: {:?}", result);
                    Ok(())
                });


            tokio::spawn(getting);
            tokio::spawn(buffer)

         }));


    }


}
