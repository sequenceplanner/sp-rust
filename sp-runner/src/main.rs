extern crate tokio;

use std::collections::*;
use tokio::prelude::*;
use tokio::*;

mod streamsupport;
use streamsupport::messagebuffer::*;

mod runners;

fn main() {

    // some example how to fetch from terminal
    //let byte_stream = codec::FramedRead::new(tokio::io::stdin(), codec::LinesCodec::new());

    // use tokio::timer::Interval;
    // use std::time::{Duration, Instant};

    // let task = Interval::new(Instant::now(), Duration::from_millis(500))
    //     .map(|x|{println!("hej"); x})
    //     .map_err(|_| ())
    //     .zip(from_buf.map_err(|_| ()))
    //     .for_each(|(_, res)| {
    //         println!("fire; yes={:?}", res);
    //         Ok(())
    //     });

    //         let inputStream = byte_stream
    //     .map(move  |line| {
    //         match line.as_ref() {
    //             "a" => {
    //                 for x in 0..20 {
    //                     let s = state!("counter".to_string() => x);
    //                     let send = to_buf
    //                         .clone()
    //                         .send(s)
    //                         .map(move |_| ())
    //                         .map_err(|e| eprintln!("error = {:?}", e));
    //                     tokio::spawn(send);
    //                 };
    //                 println!{"Yes A"};
    //             }
    //             "b" => {
    //                 println!{"Yes B"};
    //             }
    //             x => {println!{"You wrote: {:?}", x}}
    //         }
    //         line
    //     });

    //     tokio::run( future::lazy(|| {
    //         tokio::spawn(buf);

    //         tokio::spawn(
    //             task
    //         );

    //         tokio::spawn(
    //             inputStream
    //             .take_while(|x| Ok(x != "exit"))
    //             .for_each(|x| {
    //                 Ok(())
    //             })
    //             .map_err(|e| println!("Error {:?}", e))
    //         )

    //     }));

}
