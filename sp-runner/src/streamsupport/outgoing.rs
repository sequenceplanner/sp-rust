//! The incoming handler is the one that polls all incoming streams and forward them into the 
//! runner buffer. This is automatically set up by the model

use sp_domain::*;
use tokio::sync::mpsc;
use tokio::prelude::*;
use futures::try_ready;

pub(crate) struct OutGoingHandler {
    incoming_channel: mpsc::Receiver<SPState>,
    outgoing: Vec<OutgoingChannel>,
    prev_state: SPState,
    // add ticker
}

struct OutgoingChannel {
    c: mpsc::Sender<SPState>,  // Maybe add a merging buffer here
    filter: SPPath,
}

impl OutGoingHandler {
    pub fn new(channel: mpsc::Receiver<SPState>) -> OutGoingHandler {
        OutGoingHandler {
            incoming_channel: channel,
            outgoing: vec!(),
            prev_state: SPState::default()
        }
    }

    pub fn add_outgoing(&mut self, channel: mpsc::Sender<SPState>, filter: &SPPath) {
        let oc = OutgoingChannel {
            c: channel,
            filter: filter.clone()
        };
        self.outgoing.push(oc);
    }
}

impl Future for OutGoingHandler {
    type Item = ();
    type Error = ();
    fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
        match try_ready!(self.incoming_channel.poll().map_err(|_| ())) {
            Some(state) => {
                for oc in self.outgoing.iter_mut() {
                    if !state.is_sub_state_the_same(&self.prev_state, &oc.filter) {
                        oc.c.try_send(state.sub_state(&oc.filter)).expect("The resource channel is full. SHOULD NEVER HAPPEN. CHECK IT")
                    }
                }
                self.prev_state = state;
                task::current().notify();
                Ok(Async::NotReady)
            },
            None => Ok(Async::Ready(()))
        }
    }
}




/// ********** TESTS ***************

#[cfg(test)]
mod outgoing_test {
    use super::*;
    use std::collections::HashMap;
    use tokio::prelude::*;


    #[test]
    fn test_outgoing() {

        tokio::run(future::lazy(move || {
            let (ch, in_out) = mpsc::channel::<SPState>(2);
            let (to_r1, r1) = mpsc::channel::<SPState>(10);  
            let (to_r2, r2) = mpsc::channel::<SPState>(10);

            let a = SPPath::from_str(&["a"]);
            let ab = SPPath::from_str(&["a", "b"]);
            let ax = SPPath::from_str(&["a", "x"]);
            let abc = SPPath::from_str(&["a", "b", "c"]);
            let abx = SPPath::from_str(&["a", "b", "x"]);
            let c = SPPath::from_str(&["c"]);

            let mut out = OutGoingHandler::new(in_out);
            out.add_outgoing(to_r1, &ab);
            out.add_outgoing(to_r2, &ax);

            

            let range = vec![
                state!(abc => false, abx => false, ax => false, c => 0),
                state!(abc => false, abx => false, ax => true, c => 1),
                state!(abc => true,  abx => false, ax => true, c => 2),
                state!(abc => true,  abx => true, ax => true, c => 3),
                state!(abc => true, abx => true, ax => true, c => 4),
                state!(abc => true, abx => true, ax => false, c => 5),
                state!(abc => false, abx => false, ax => false, c => 6),
                state!(abc => false, abx => false, ax => false, c => 7),
                state!(abc => false, abx => false, ax => false, c => 8),

            ];

            let sending = stream::iter_ok(range).for_each(move |s| {
                let send = ch
                    .clone()
                    .send(s)
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send)
            });
            tokio::spawn(sending);


            let getting_r1 = r1
                .map_err(|_| ())
                .for_each(|result| {
                    println!("R1 got: {:?}", result);
                    Ok(())
                });
            let getting_r2 = r2
                .map_err(|_| ())
                .for_each(|result| {
                    println!("R2 got: {:?}", result);
                    Ok(())
                });


            tokio::spawn(getting_r1);
            tokio::spawn(getting_r2);
            tokio::spawn(out)

         }));
        

    }


}