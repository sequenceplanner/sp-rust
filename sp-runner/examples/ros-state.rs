use crossbeam::channel;
use failure::Error;
use futures::Future;
use r2r;
use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap;
use std::thread;
use tokio::prelude::*;
use tokio_threadpool;

fn main() -> Result<(), Error> {
    let (runner_model, initial_state, resources) = two_robots();

    // start ros node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "spnode", "")?;

    // data from resources to runner
    let (tx_in, rx_in) = channel::unbounded();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out = roscomm_setup(&mut node, resources, tx_in.clone())?;

    // "runner"
    thread::spawn(move || {
        let (runner, comm) = Runner::new(runner_model, initial_state);
        launch_tokio(runner, comm, rx_in, tx_out);
    });

    loop {
        // blocking ros spinning
        node.spin_once(std::time::Duration::from_millis(100));
    }
}

fn launch_tokio(
    runner: Runner,
    comm: RunnerComm,
    rx: channel::Receiver<StateExternal>,
    tx: channel::Sender<StateExternal>,
) {
    let (buf, mut to_buf, from_buf) =
        MessageBuffer::new(2, |_: &mut StateExternal, _: &StateExternal| {
            false // no merge
        });

    let pool = tokio_threadpool::ThreadPool::new();

    #[allow(unreachable_code)]
    pool.spawn(future::lazy(move || {
        loop {
            let _res = tokio_threadpool::blocking(|| {
                let msg = rx.recv().unwrap();
                let _res = to_buf.try_send(msg);
            })
            .map_err(|_| panic!("the threadpool shut down"));
        }
        Ok(())
    }));

    #[allow(unreachable_code)]
    tokio::run(future::lazy(move || {
        tokio::spawn(runner);

        let mut runner_in = comm.state_input.clone();
        let getting = from_buf
            .for_each(move |result| {
                println!("INTO RUNNER");
                println!("{}", result);
                runner_in.try_send(result.to_assignstate()).unwrap(); // if we fail, the runner do not keep up
                Ok(())
            })
            .map(move |_| ())
            .map_err(|e| eprintln!("error = {:?}", e));
        tokio::spawn(getting);

        let from_runner = comm
            .state_output
            //.take(5)
            .for_each(move |result| {
                println!("***************************");
                println!("FROM RUNNER");
                println!("{}", result.external());
                tx.clone().try_send(result.external()).unwrap(); // if we fail the ROS side to do not keep up
                Ok(())
            })
            .map(move |_| ())
            .map_err(|e| eprintln!("error = {:?}", e));
        tokio::spawn(from_runner);

        tokio::spawn(buf);

        Ok(())
    }));
}

fn roscomm_setup(
    node: &mut r2r::Node,
    rcs: Vec<Resource>,
    tx_in: channel::Sender<StateExternal>,
) -> Result<channel::Sender<StateExternal>, Error> {
    let mut all_ros_pubs = Vec::new();

    for r in rcs {
        if let Some(rc) = r.comm.as_ros_comm() {
            // setup ros subscribers
            for s in &rc.subscribers {
                // todo: fix lifetime issue when R is not static...
                let msg_type = s.definition.toplevel_msg_type().unwrap();
                let tx = tx_in.clone();
                let def = s.definition.clone();
                let topic = s.topic.to_owned();
                let cb = move |msg: r2r::Result<serde_json::Value>| {
                    let json = msg.unwrap();
                    let state = json_to_state(&json, &def, &topic);
                    tx.send(state).unwrap();
                };
                let _subref = node.subscribe_untyped(&s.topic, &msg_type, Box::new(cb))?;
            }

            // setup ros publishers
            let ros_pubs: Vec<_> = rc
                .publishers
                .iter()
                .map(|p| {
                    let msg_type = p.definition.toplevel_msg_type().unwrap();
                    let rp = node.create_publisher_untyped(&p.topic, &msg_type).unwrap();
                    let def = p.definition.clone();
                    let topic = p.topic.to_owned();
                    move |state: &StateExternal| {
                        let to_send = state_to_json(state, &def, &topic);
                        rp.publish(to_send).unwrap();
                    }
                })
                .collect();

            all_ros_pubs.extend(ros_pubs);
        }
    }

    let (tx_out, rx_out) = channel::unbounded();
    thread::spawn(move || loop {
        let state = rx_out.recv().unwrap();
        for rp in &all_ros_pubs {
            (rp)(&state);
        }
    });

    Ok(tx_out)
}
