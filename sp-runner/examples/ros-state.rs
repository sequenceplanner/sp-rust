use failure::Error;
use r2r;
use sp_domain::*;
use std::collections::HashMap;
use std::env;
use std::sync::mpsc;
use std::thread;

#[macro_use]
extern crate lazy_static;

fn make_resource() -> Resource {
    let command_var_data = Variable::Command(VariableData {
        type_: 0.to_spvalue().has_type(),
        initial_value: Some(0.to_spvalue()),
        domain: vec![0.to_spvalue(), 10.to_spvalue()],
    });

    let state_var_data = Variable::Measured(VariableData {
        type_: SPValueType::Int32,
        initial_value: None,
        domain: Vec::new(),
    });

    let comm = ResourceComm::RosComm(RosComm {
        node_name: "resource".into(),
        node_namespace: "".into(),
        publishers: vec![RosPublisherDefinition {
            topic: "/out".into(),
            qos: "".into(),
            definition: RosMsgDefinition::Message(
                "std_msgs/msg/Int32".into(),
                hashmap![
                        "data".into() => RosMsgDefinition::Field(command_var_data.clone())],
            ),
        }],
        subscribers: vec![RosSubscriberDefinition {
            topic: "/in".into(),
            definition: RosMsgDefinition::Message(
                "std_msgs/msg/Int32".into(),
                hashmap![
                    "data".into() => RosMsgDefinition::Field(state_var_data.clone())
                ],
            ),
        }],
    });

    Resource {
        abilities: Vec::new(),
        parameters: Vec::new(),
        comm: comm,
    }
}

lazy_static! {
    static ref R: Resource = make_resource();
}

fn roscomm_setup(
    node: &mut r2r::Node,
    rc: &'static RosComm,
    tx_in: mpsc::Sender<StateExternal>,
) -> Result<mpsc::Sender<StateExternal>, Error> {
    // setup ros subscribers
    for s in &rc.subscribers {
        // todo: fix lifetime issue when R is not static...
        let msg_type = s.definition.toplevel_msg_type().unwrap();
        let tx = tx_in.clone();
        let cb = move |msg: r2r::Result<serde_json::Value>| {
            let json = msg.unwrap();
            let state = json_to_state(&json, &s.definition, &s.topic);
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
            move |state: &StateExternal| {
                let to_send = state_to_json(state, &p.definition, &p.topic);
                rp.publish(to_send).unwrap();
            }
        })
        .collect();

    let (tx_out, rx_out) = mpsc::channel::<StateExternal>();
    thread::spawn(move || loop {
        let state = rx_out.recv().unwrap();
        for rp in &ros_pubs {
            (rp)(&state);
        }
    });

    Ok(tx_out)
}

fn main() -> Result<(), Error> {
    let rc = R.comm.as_ros_comm().unwrap();

    // start ros node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, &rc.node_name, &rc.node_namespace)?;

    // data from resources to runner
    let (tx_in, rx_in) = mpsc::channel::<StateExternal>();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out = roscomm_setup(&mut node, &rc, tx_in)?;

    thread::spawn(move || loop {
        // "runner"
        let state = rx_in.recv().unwrap();
        println!("got sp state\n===============");
        println!("{}", state);
    });

    thread::spawn(move || loop {
        // "runner"
        let p = SPPath::from_str(&["/hopp", "std_msgs/msg/String", "data"]);
        let dummy_state = StateExternal {
            s: hashmap![p => "hello2".to_spvalue()],
        };
        tx_out.send(dummy_state);
        thread::sleep_ms(1000);
    });

    loop {
        // blocking ros spinning
        node.spin_once(std::time::Duration::from_millis(100));
    }
}

use tokio::prelude::*;
use tokio::*;

use futures::try_ready;
use std::collections::*;

use futures::Future;
use futures::future::{lazy, poll_fn};

fn launch_tokio() {

    tokio::run(future::lazy(move || {
    let (buf, to_buf, from_buf) =
        MessageBuffer::new(2, |_: &mut StateExternal, _: &StateExternal| {
            false // no merge
        });


    poll_fn(move || {
        loop {
            let res = blocking(|| {
                let msg = rx.recv().unwrap();
                println!("message = {}", msg);
                
                let send = to_buf
                    .clone()
                    .send(msg)
                    .map(move |_| ())
                    .map_err(|e| eprintln!("error = {:?}", e));
                tokio::spawn(send)

            }).map_err(|_| panic!("the threadpool shut down"));
        }
    });

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
        .for_each(|result| {
            println!("Yes: {:?}", result);
            Ok(())
        })
        .map(move |_| ())
        .map_err(|e| eprintln!("error = {:?}", e));
    tokio::spawn(getting);

    tokio::spawn(buf)
}));
}
