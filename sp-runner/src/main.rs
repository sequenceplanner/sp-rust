extern crate tokio;

use tokio::*;
use tokio::prelude::*;

use std::collections::*;
use std::sync::{Arc, Mutex};
use sync::mpsc;
use futures::try_ready;

mod streamsupport;

type State = HashMap<String, usize>;

#[derive(Debug)]
struct InputMessages {
    input: mpsc::Receiver<State>, //Incoming messages from ROS
    output: mpsc::Sender<State>, // Outgoing to runner
    length: usize,                // the length of the buffer. Will pack-pressure incoming
    q: VecDeque<State>,            // the buffer for states
}



impl InputMessages {
    /// In this method the state should be merged if possible, else added into the queue
    /// if queue becomes to big if the consumer is always slower than producer, 
    /// handle that here by an aggressive merge
    fn new_message(&mut self, m: State) {

        // if you do not want to merge, uncomment and comment the merge
         self.q.push_back(m);

        // if you want to merge
        // if self.q.is_empty() || (self.q.back().unwrap().get("counter").unwrap() % 10) == 0 {
        //     self.q.push_back(m);
        // } else {
        //     let new_count = m.get("counter").unwrap();
        //     let b = self.q.back_mut().unwrap();
        //     let prev = b.get("tjo").unwrap_or(&1) + new_count;
        //     b.insert("tjo".to_string(), prev);
        //     b.insert("counter".to_string(), *new_count);
        // }

        
        println!("new message current que {:?}", self.q)
    }
    
    fn has_message_to_send(& self) -> bool {
        !self.q.is_empty()
    }
    fn message_to_send(&mut self) -> Option<State> {
        self.q.pop_front()
    }

    /// This method tries to read all messages that are waiting
    fn get_message(&mut self) -> Result<Async<()>, sync::mpsc::error::RecvError> {
        loop {
            if (self.q.len() >= self.length) {
                //task::current().notify();
                return Ok(Async::NotReady);
            }
            let x = try_ready!(self.input.poll());
            x.map(|mess| self.new_message(mess));
        }
    }


}

impl Future for InputMessages {
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        
        println!();
        println!("*******************");


        let x = self.get_message();
        println!("fetch {:?}", x);

        match self.output.poll_ready() {
            Ok(Async::NotReady) => {
                println!("NotReady");
                return Ok(Async::NotReady);
            }
            Ok(Async::Ready(_)) => {
                println!("Ready");

                if self.has_message_to_send() {
                    let mess = self.message_to_send().unwrap();
                    println!();
                    println!("I am SENDING: {:?}", mess);
                    println!();
                    self.output.try_send(mess).unwrap();
                }

                if self.has_message_to_send() {task::current().notify();}
                return Ok(Async::NotReady);

            }
            Err(_) => panic!("outgoing message channel terminated")
        }

        println!();

    }
}

#[macro_export]
macro_rules! state {
    ($( $key: expr => $val: expr ),*) => {{
         let mut map = ::std::collections::HashMap::new();
         $( map.insert($key, $val); )*
         map
    }}
}


fn main() {
    let byte_stream = codec::FramedRead::new(tokio::io::stdin(), codec::LinesCodec::new());
    let (toBuf, incomingBuf) = tokio::sync::mpsc::channel(2);
    let (fromBuf, mut output) = tokio::sync::mpsc::channel(2);
    let buf = InputMessages {
        input: incomingBuf,
        output: fromBuf,
        length: 5,
        q: VecDeque::new(),
    };

    use tokio::timer::Interval;
    use std::time::{Duration, Instant};

    let task = Interval::new(Instant::now(), Duration::from_millis(500))
        .map(|x|{println!("hej"); x})
        .map_err(|_| ())
        .zip(output.map_err(|_| ()))
        .for_each(|(_, res)| {
            println!("fire; yes={:?}", res);
            Ok(())
        });



    let mut aState: State = state!("kalle".to_string() => 1, "pelle".to_string() => 2);
    let mut counter = 0;

    let inputStream = byte_stream
        .map(move  |line| {
            match line.as_ref() {
                "a" => {
                    for x in 0..100 {
                        let s = state!("counter".to_string() => x);
                        let send = toBuf
                            .clone()
                            .send(s)
                            .map(move |_| ())
                            .map_err(|e| eprintln!("error = {:?}", e));
                        tokio::spawn(send);
                    };
                    println!{"Yes A"};
                }
                "b" => {
                    println!{"Yes B"};
                }
                x => {println!{"You wrote: {:?}", x}}
            }
            line
        });

    println!("Starting.....");

    tokio::run( future::lazy(|| {
        tokio::spawn(buf);

        tokio::spawn(
            task
        );

        tokio::spawn(
            inputStream
            .take_while(|x| Ok(x != "exit"))
            .for_each(|x| {
                Ok(())
            })
            .map_err(|e| println!("Error {:?}", e))
        )
        
    }));
}
