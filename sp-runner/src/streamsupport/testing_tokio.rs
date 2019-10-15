#![allow(dead_code)]

/// Buffers and merges messages to handle slower consumers
///
/// If the buffer is full, the input stream will no be polled -> back pressure
/// (Maybe we instead should always poll and merge aggressive if buffer is full)

use tokio::prelude::*;
use tokio::*;

use std::time;



pub struct TestMe {
    c: usize,
    delay_1: timer::DelayQueue<usize>,
}

impl TestMe {
    pub fn new() -> TestMe  {
        TestMe {
            c: 0,
            delay_1: timer::DelayQueue::new(),
        }
    }


}

/// A long living future that never returns
impl Future for TestMe {
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        self.c += 1;

        if self.delay_1.is_empty() {
            self.delay_1.insert(self.c, time::Duration::from_millis(100));
        }

        println!("c: {:?}", self.c);

        match self.delay_1.poll().unwrap() {
            Async::NotReady => return Ok(Async::NotReady),
            Async::Ready(Some(x)) => {
                if self.c < 10 {
                    self.delay_1.insert(self.c+5000, time::Duration::from_millis(5000));
                    self.delay_1.insert(self.c+1000, time::Duration::from_millis(1000));
                    self.delay_1.insert(self.c+500, time::Duration::from_millis(500));
                    self.delay_1.insert(self.c, time::Duration::from_millis(1));
                    self.delay_1.insert(self.c+1000, time::Duration::from_millis(1000));
                    self.delay_1.insert(self.c+1100, time::Duration::from_millis(1100));
                    self.delay_1.insert(self.c+1200, time::Duration::from_millis(1200));
                    self.delay_1.insert(self.c, time::Duration::from_millis(1));
                }
                println!("We got: {:?}", x);
                if self.delay_1.is_empty() {
                    return Ok(Async::Ready(()))
                };
                task::current().notify();
                return Ok(Async::NotReady)
            },
            Async::Ready(None) => return Ok(Async::Ready(()))
        }

        // while let Some(res) = try_ready!(self.delay_1.poll().map_err(|_| ())) {
        //     if self.c < 10 {
        //         self.delay_1.insert(self.c+5000, time::Duration::from_millis(5000));
        //         self.delay_1.insert(self.c+1000, time::Duration::from_millis(1000));
        //         self.delay_1.insert(self.c+500, time::Duration::from_millis(500));
        //         self.delay_1.insert(self.c, time::Duration::from_millis(1));
        //         self.delay_1.insert(self.c+1000, time::Duration::from_millis(1000));
        //         self.delay_1.insert(self.c+1100, time::Duration::from_millis(1100));
        //         self.delay_1.insert(self.c+1200, time::Duration::from_millis(1200));
        //         self.delay_1.insert(self.c, time::Duration::from_millis(1));
        //     }
        //     println!("We got: {:?}", res);
        // }
        // println!("NEVER THIS");
        // Ok(Async::Ready(()))


    }
}

/// ********** TESTS ***************

#[cfg(test)]
mod messagebuffer_test {
    use super::*;

    #[test]
    fn slow_consumer_test_with_merge() {
        let x = TestMe::new();

        tokio::run(future::lazy(move || {
            tokio::spawn(x)
        }));


    }

}
