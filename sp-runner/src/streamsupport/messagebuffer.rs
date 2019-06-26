use tokio::*;
use tokio::prelude::*;

use std::collections::*;
use std::sync::{Arc, Mutex};
use sync::mpsc;
use futures::try_ready;

/// Buffers and merges messages to handle slower consumers
/// 
/// If the buffer is full, the input stream will no be polled -> back pressure
/// (Maybe we instead should always poll and merge aggressive if buffer is full)


pub struct MessageBuffer<T> {
    in_R: mpsc::Receiver<T>, //Incoming messages from ROS
    out_S: mpsc::Sender<T>, // Outgoing to runner
    buffer_size: usize,                // the length of the buffer. Will pack-pressure incoming
    q: VecDeque<T>,            // the buffer for states
    merge: fn(&mut T, &T) -> bool  // Mutate last T if new T should be merged. Return true if merged
}

impl<T> MessageBuffer<T> {

    pub fn new (
        in_R: mpsc::Receiver<T>, 
        out_S: mpsc::Sender<T>,
        buffer_size: usize,
        merge: fn(&mut T, &T) -> bool
    ) -> MessageBuffer<T> {
        MessageBuffer{in_R, out_S, buffer_size, q: (VecDeque::new()), merge}
    }

    /// In this method the message should be merged if possible, else added into the queue
    /// if queue becomes to big if the consumer is always slower than producer, maybe
    /// handle that here by an aggressive merge
    fn new_message(&mut self, m: T) {
        if self.q.is_empty()  {
            self.q.push_back(m);
        } else {
            let last = self.q.back_mut().unwrap();
            let res = (self.merge)(last, &m);
            if !res {
                self.q.push_back(m);
            }
        }
    }
    
    fn has_message_to_send(& self) -> bool {
        !self.q.is_empty()
    }
    fn message_to_send(&mut self) -> Option<T> {
        self.q.pop_front()
    }

    /// This method tries to read all messages that are waiting
    fn get_message(&mut self) -> Result<Async<()>, sync::mpsc::error::RecvError> {
        loop {
            if self.q.len() >= self.buffer_size {
                return Ok(Async::NotReady);
            }
            let x = try_ready!(self.in_R.poll());
            x.map(|mess| self.new_message(mess));
        }
    }


}

/// A long living future that never returns
impl<T> Future for MessageBuffer<T> {
    type Item = (); 
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        let x = self.get_message();

        match self.out_S.poll_ready() {
            Ok(Async::NotReady) => {
                Ok(Async::NotReady)
            }
            Ok(Async::Ready(_)) => {
                if self.has_message_to_send() {
                    let mess = self.message_to_send().unwrap();
                    self.out_S.try_send(mess);
                }

                if self.has_message_to_send() {task::current().notify();}
                Ok(Async::NotReady)
            }
            Err(_) => panic!("outgoing message channel terminated")
        }
    }
}

#[cfg(test)]
mod messagebuffer_test {
    use super::*;
    #[test]
    fn create() {
        let (toBuf, incomingBuf) = tokio::sync::mpsc::channel::<String>(2);
        let (fromBuf, mut output) = tokio::sync::mpsc::channel::<String>(2);

        let newBuf = MessageBuffer::new(incomingBuf, fromBuf, 5, |last, next| {
            false // new merge
        });

        

        println!("test: {:?}", newBuf.in_R)
    }
}


            // let new_count = m.get("counter").unwrap();
            // let b = self.q.back_mut().unwrap();
            // let prev = b.get("tjo").unwrap_or(&1) + new_count;
            // b.insert("tjo".to_string(), prev);
            // b.insert("counter".to_string(), *new_count);