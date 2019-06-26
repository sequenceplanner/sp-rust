extern crate tokio;

use tokio::*;
use tokio::io;
use tokio::prelude::*;

use std::collections::*;
use sync::mpsc;

type State = HashMap<String, String>;

#[derive(Debug)]
struct InputMessages {
    input: mpsc::Receiver<State>, //Incoming messages from ROS
    output: mpsc::Sender<State>, // Outgoing to runner
    length: usize,
    q: VecDeque<State>
}


impl InputMessages {
    fn new_message(&mut self, m: State) {
        // check and merge
        self.q.push_back(m);
        println!("new message current que {:?}", self.q)
    }
    fn message_to_send(&mut self) -> Option<State> {
        let mess = self.q.pop_front();
        println!("taken from que {:?}", self.q);
        mess
    }

    fn get_message(&mut self) -> bool {
        match self.input.poll() {
            Ok(Async::Ready(Some(mess))) => { self.new_message(mess); true}
            Ok(Async::NotReady) => { false }
            _ => panic!("Incoming message channel terminated")
        }
    }
}

impl Future for InputMessages {
    type Item = ();
    type Error = ();



    fn poll(&mut self) -> Poll<(), ()> {
        let mut sender = self.output.clone();
        println!("A poll on  InputMessage: {:?}", self.q.len());
        match sender.poll_ready() {
            Ok(Async::NotReady) => {
                println!("in que, not ready");
                if self.q.len() < self.length {
                    self.get_message();
                } else {
                    println!("START BLOCKING");
                }
                if (!self.q.is_empty()){
                    task::current().notify();
                }
                Ok(Async::NotReady)
                
            }
            Ok(Async::Ready(_)) => {
                println!("in que, ready");
                let res = self.get_message();
                if let Some(m) = self.message_to_send() {
                    // let x = sender.try_send(m); // should work since we got Ready
                    // if let Err(e) = x {
                    //     println!("Got an error when sending in buffer {:?}", e);
                    // }
                    let send = sender
                        .send(m)
                        .map(move |_| ())
                        .map_err(|e| eprintln!("error = {:?}", e));
                    tokio::spawn(send); 
                }
                if !res && self.q.is_empty() {Ok(Async::NotReady)}
                else {
                    task::current().notify();
                    {Ok(Async::NotReady)}
                }
            }
            Err(_) => panic!("outgoing message channel terminated")
        }

        // if self.q.len() < self.length {

        //     self.input.poll().then(|mess|{
        //         self.newMessage(mess);
        //         if let Some(m) = self.messageToSend() {
        //             self.output.send(m);
        //         };
        //         Ok(())               
        //     })
        // } else  {
        //     Ok(Async::NotReady)
        // }

        // let result = self.input.for_each(|s| {
        //     if self.q.len() >= length {
        //         self.output.poll_ready()
        //     } else {
        //         self.newMessage(s);
        //         self.output.poll_ready().a

        //}
            

        // })

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
    let (toBuf, incomingBuf) = tokio::sync::mpsc::channel(1);
    let (fromBuf, mut output) = tokio::sync::mpsc::channel(1);
    let buf = InputMessages {
        input: incomingBuf,
        output: fromBuf,
        length: 5,
        q: VecDeque::new(),
    };

    use tokio::timer::Interval;
    use std::time::{Duration, Instant};

    let task = Interval::new(Instant::now(), Duration::from_secs(5))
        .map(|x|{println!("hej"); x})
        .map_err(|_| ())
        .zip(output.map_err(|_| ()))
        .for_each(|(_, res)| {
            println!("fire; yes={:?}", res);
            Ok(())
        });



    let mut aState: State = state!("kalle".to_string() => "a".to_string(), "pelle".to_string() => "b".to_string());
    let mut counter = 0;

    let inputStream = byte_stream
        .map(move  |line| {
            match line.as_ref() {
                "a" => {
                    counter += 1;
                    aState.insert("counter".to_string(), counter.to_string());
                    let send = toBuf
                        .clone()
                        .send(aState.clone())
                        .map(move |_| ())
                        .map_err(|e| eprintln!("error = {:?}", e));
                    tokio::spawn(send);
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




//fn main() {
    // let t_in = tokio::io::stdin();
    // let mut t_out = tokio::io::stdout();

    // let reader = std::io::BufReader::new(t_in);
    // //let buffer = Vec::new();

    // let byte_stream = codec::FramedRead::new(io::stdin(), codec::LinesCodec::new());

    // let f = io::read_until(reader, b'x', buffer)
    //     .and_then(move | (_, buffer) | {
    //         t_out.write_all(&buffer)
    //     }).map_err(|e| panic!(e));



    // fn launchF<F>() -> impl Future<Item=String, Error = F::Error> where F: Future<Item = String>{
    //      Box::new(String::new("done"))
    // }



// use tokio::timer::Delay;
// use std::time::{Duration, Instant};

// let mut agg: String = String::new();
// let long: Vec<i32> = (0..100).collect();

//     tokio::run(
//         byte_stream
//         .take_while(|x| Ok(x != "exit"))
//         .map(move |s|{
//             agg = format!{"{}-{}", agg, s};
//             agg.clone()
//         })
//         .map(|s|{
//             let long2 = (0..1_000_000).into_iter().map(|i| i +1);
//             println!("hep {:?}", long2.len());
//             s
//         })
//         // .and_then(|s: Result<String>| Delay::new(Instant::now() + Duration::from_millis(5000))
//         //    .and_then(|_| {println!("hej"); Ok(s)})
//         //    .map_err(|e| panic!("delay errored; err={:?}", e))
//         // )
//         .for_each(|x| {
//             println!("GOT: {:?}", x);
//             Ok(())
//         })
//         .map_err(|e| println!("Error {:?}", e))
//     );

//}