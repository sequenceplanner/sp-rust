//! The incoming handler is the one that polls all incoming streams and forward them into the
//! runner buffer. This is automatically set up by the model
#![allow(dead_code)]

use super::*;
use sp_domain::*;
use tokio::sync::mpsc;

pub(crate) struct IncomingHandler {
    pub buffer: MessageBuffer<SPState>,
    pub channel: mpsc::Sender<SPState>,
    pub runner_channel: mpsc::Receiver<SPState>
}

impl IncomingHandler {
    pub fn new() -> IncomingHandler {
        let (buffer, channel, runner_channel) = MessageBuffer::new(3, IncomingHandler::merge);
        IncomingHandler { buffer, channel, runner_channel }
    }

    pub fn extract(self) -> (MessageBuffer<SPState>, mpsc::Sender<SPState>, mpsc::Receiver<SPState>)  {
        (self.buffer, self.channel, self.runner_channel)
    }


    fn merge(prev: &mut SPState, next: &SPState) -> bool {
        let p = prev.projection().projection;
        let n = next.projection();
        let merge_me = n.projection.iter().all( |(n_p, n_v)| {
            p.iter().all(|(p_p, p_v)|{
                p_p != n_p || (p_v.current_value() == n_v.current_value())
            })
            
        });
        if merge_me {
            prev.extend(n.clone_state());
        }
        merge_me
    }
}




/// ********** TESTS ***************

#[cfg(test)]
mod incoming_test {
    use super::*;
    use tokio::prelude::*;

    fn ab() -> SPPath { SPPath::from_slice(&["a", "b"])}
    fn ac() -> SPPath { SPPath::from_slice(&["a", "c"])}
    fn initial_ab() -> SPState {
        state!(["a", "b"] => false)
    }
    fn next_ab() -> SPState {
        state!(["a", "b"] => true)
    }
    fn initial_ac() -> SPState {
        state!(["a", "c"] => false)
    }
    fn next_ac() -> SPState {
        state!(["a", "c"] => false)
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
