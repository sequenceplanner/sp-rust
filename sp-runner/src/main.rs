extern crate tokio;

use tokio::prelude::*;
use tokio::*;

mod streamsupport;
use streamsupport::messagebuffer::*;

mod runners;

fn main() {

    //some example how to fetch from terminal
    let byte_stream = codec::FramedRead::new(tokio::io::stdin(), codec::LinesCodec::new());

    use tokio::timer::Interval;
    use std::time::{Duration, Instant};

    let (b, in_c, mut out_c) = MessageBuffer::new(10, |_, _| {false});

    let task = Interval::new(Instant::now(), Duration::from_millis(1000))
        .map_err(|_| ())
        .for_each(move |i| {
            // println!("Yes {:?}", &i);

            let res = in_c.clone().try_send(format!("{:?}", i));
            println!("Res {:?}", &res);


            Ok(())

            // let send = inC
            //     .clone()
            //     .send(format!("{:?}", i))
            //     .map(|_| ())
            //     .map_err(|e| eprintln!("error = {:?}", e));

            // tokio::spawn(send)
        });

            let input_stream = byte_stream
        .map(move  |line| {
            match line.as_ref() {
                "a" => {
                    let try_me = out_c.poll();
                    println!("WE GOT: {:?}", try_me)
                }
                "b" => {
                    loop {
                        match out_c.poll() {
                            Ok(Async::Ready(x)) => println!("WE GOT loop: {:?}", x),
                            Ok(Async::NotReady) => {
                                println!("done");
                                break;
                            },
                            x => {
                                println!("Hmm, b got {:?}", x);
                                break;
                            }
                        }
                    }
                }
                x => {println!{"You wrote: {:?}", x}}
            }
            line
        });

        tokio::run( future::lazy(|| {
            tokio::spawn(
                b
            );

            tokio::spawn(
                task
            );

            tokio::spawn(
                input_stream
                .take_while(|x| Ok(x != "exit"))
                .for_each(|_| {
                    Ok(())
                })
                .map_err(|e| println!("Error {:?}", e))
            )

        }));

}
