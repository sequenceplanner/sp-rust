#![allow(dead_code)]

/// Buffers and merges messages to handle slower consumers
///
/// If the buffer is full, the input stream will no be polled -> back pressure
/// (Maybe we instead should always poll and merge aggressive if buffer is full)

use tokio::prelude::*;
use tokio::*;

use futures::try_ready;
use std::collections::*;
use sync::mpsc;



pub struct MessageBuffer<T>
where
    T: std::fmt::Debug,
{
    in_r: mpsc::Receiver<T>,       //Incoming messages from ROS
    out_s: mpsc::Sender<T>,        // Outgoing to runner
    buffer_size: usize,            // the length of the buffer. Will pack-pressure incoming
    q: VecDeque<T>,                // the buffer for states
    merge: fn(&mut T, &T) -> bool, // Mutate last T if new T should be merged. Return true if merged
}

impl<T> MessageBuffer<T>
where
    T: std::fmt::Debug,
{
    pub fn new(
        buffer_size: usize,
        merge: fn(&mut T, &T) -> bool,
    ) -> (MessageBuffer<T>, mpsc::Sender<T>, mpsc::Receiver<T>) {
        let (to_buf, into_buf) = tokio::sync::mpsc::channel::<T>(2);
        let (outof_buf, from_buf) = tokio::sync::mpsc::channel::<T>(2);

        (
            MessageBuffer {
                in_r: into_buf,
                out_s: outof_buf,
                buffer_size,
                q: (VecDeque::new()),
                merge,
            },
            to_buf,
            from_buf,
        )
    }

    pub fn new_with_channel(
        in_r: mpsc::Receiver<T>,
        out_s: mpsc::Sender<T>,
        buffer_size: usize,
        merge: fn(&mut T, &T) -> bool,
    ) -> MessageBuffer<T> {
        MessageBuffer {
            in_r,
            out_s,
            buffer_size,
            q: (VecDeque::new()),
            merge,
        }
    }

    /// In this method the message should be merged if possible, else added into the queue
    /// if queue becomes to big if the consumer is always slower than producer, maybe
    /// handle that here by an aggressive merge
    fn new_message(&mut self, m: T) {
        //println!("message to que: {:?}", m);
        if self.q.is_empty() {
            self.q.push_back(m);
        } else {
            let last = self.q.back_mut().unwrap();
            let res = (self.merge)(last, &m);
            if !res {
                self.q.push_back(m);
            }
        }
    }

    fn has_message_to_send(&self) -> bool {
        !self.q.is_empty()
    }
    fn message_to_send(&mut self) -> Option<T> {
        self.q.pop_front()
    }

    /// This method tries to read all messages that are waiting
    fn get_message(&mut self) -> Result<Async<Option<()>>, sync::mpsc::error::RecvError> {
        loop {
            if self.q.len() >= self.buffer_size {
                return Ok(Async::NotReady);
            }
            let res = try_ready!(self.in_r.poll());
            //println!("We got: {:?}", res);
            match res {
                Some(mess) => self.new_message(mess),
                None => return Ok(Async::Ready(None)),
            }
        }
    }
}

/// A long living future that never returns
impl<T> Future for MessageBuffer<T>
where
    T: std::fmt::Debug,
{
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        let getting = self.get_message();

        let is_out_ready = self.out_s.poll_ready();
        if let Ok(Async::Ready(_)) = is_out_ready {
            //println!("kö Ready: {:?}", self.has_message_to_send());
            if self.has_message_to_send() {
                let mess = self.message_to_send().unwrap();
                self.out_s.try_send(mess); // should always work
            }
            if self.has_message_to_send() {
                task::current().notify();
            }
        };

        if let Err(e) = is_out_ready {
            eprintln!("MessageBuffer: We have an error in outgoing {:?}", e)
        };

        match getting {
            Ok(Async::Ready(None)) if self.q.is_empty() => Ok(Async::Ready(())),
            Err(e) => {
                eprintln!("MessageBuffer: We have an error in incoming {:?}", e);
                panic!("o no")
            }
            _ => Ok(Async::NotReady),
        }
    }
}

/// ********** TESTS ***************

#[cfg(test)]
mod messagebuffer_test {
    use super::*;
    #[test]
    fn create() {
        tokio::run(future::lazy(move || {
            let (buf, to_buf, from_buf) =
                MessageBuffer::new(2, |last: &mut String, next: &String| {
                    false // no merge
                });

            let range = vec!["one", "one", "two", "three", "four"];
            let sending = stream::iter_ok(range).for_each(move |s| {
                let send = to_buf
                    .clone()
                    .send(s.to_string())
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send)
            });
            tokio::spawn(sending);

            let getting = from_buf
                .collect()
                .map(|result| {
                    println!("Yes: {:?}", result);
                    assert_eq!(result.len(), 5);
                })
                .map(move |_| ())
                .map_err(|e| eprintln!("error = {:?}", e));
            tokio::spawn(getting);

            tokio::spawn(buf)
        }));
    }

    #[test]
    fn new_with_channel() {
        tokio::run(future::lazy(move || {
            let (to_buf, into_buf) = tokio::sync::mpsc::channel::<String>(1);
            let (outof_buf, from_buf) = tokio::sync::mpsc::channel::<String>(1);

            let buf = MessageBuffer::new_with_channel(into_buf, outof_buf, 2, |_, _| {
                false // no merge
            });

            let range = vec!["one", "one", "two", "three", "four"];
            let sending = stream::iter_ok(range).for_each(move |s| {
                let send = to_buf
                    .clone()
                    .send(s.to_string())
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send)
            });
            tokio::spawn(sending);

            let getting = from_buf
                .collect()
                .map(|result| {
                    println!("Yes: {:?}", result);
                    assert_eq!(result, vec!("one", "one", "two", "three", "four"));
                })
                .map(move |_| ())
                .map_err(|e| eprintln!("error = {:?}", e));
            tokio::spawn(getting);

            tokio::spawn(buf)
        }));
    }

    type State = HashMap<String, usize>;
    #[macro_export]
    macro_rules! state {
        ($( $key: expr => $val: expr ),*) => {{
            let mut map = ::std::collections::HashMap::new();
            $( map.insert($key, $val); )*
            map
        }}
    }

    #[test]
    fn slow_consumer_test_with_merge() {
        let (buf, to_buf, from_buf) = MessageBuffer::new(5, |last: &mut State, next: &State| {
            let new_count = next.get("counter").unwrap();
            let prev = last.get("tjo").unwrap_or(&1) + new_count;
            last.insert("tjo".to_string(), prev);
            last.insert("counter".to_string(), *new_count);
            true
        });

        use std::time::{Duration, Instant};
        use tokio::timer::Interval;

        let task = Interval::new(Instant::now(), Duration::from_millis(100))
            .map_err(|_| ())
            .zip(from_buf.map_err(|_| ()))
            .collect()
            .map(|x| {
                println!("We got: {:?}", x);
                assert!(x.len() < 5); // should merge
            });

        println!("Starting.....");

        tokio::run(future::lazy(move || {
            tokio::spawn(buf);

            (0..20).for_each(|x| {
                let s = state!("counter".to_string() => x);
                let send = to_buf
                    .clone()
                    .send(s)
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send);
            });

            tokio::spawn(task)
        }));
    }

    #[test]
    fn slow_consumer_test_no_merge() {
        let (buf, to_buf, from_buf) = MessageBuffer::new(10, |_: &mut State, _: &State| false);

        use std::time::{Duration, Instant};
        use tokio::timer::Interval;

        let task = Interval::new(Instant::now(), Duration::from_millis(50))
            .map_err(|_| ())
            .zip(from_buf.map_err(|_| ()))
            .collect()
            .map(|x| {
                println!("We got: {:?}", x.len());
                assert!(x.len() == 20); // should merge
            });

        println!("Starting.....");

        tokio::run(future::lazy(move || {
            tokio::spawn(buf);

            (0..20).for_each(|x| {
                let s = state!("counter".to_string() => x);
                let send = to_buf
                    .clone()
                    .send(s)
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send);
            });

            tokio::spawn(task)
        }));
    }

}
