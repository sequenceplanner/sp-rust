use failure::Error;
use r2r;
use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap;
use std::env;
use crossbeam::channel;
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
            topic: "/r1/ref".into(),
            qos: "".into(),
            definition: RosMsgDefinition::Message(
                "std_msgs/msg/Int32".into(),
                hashmap![
                        "data".into() => RosMsgDefinition::Field(command_var_data.clone())],
            ),
        }],
        subscribers: vec![RosSubscriberDefinition {
            topic: "/r1/act".into(),
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
    tx_in: channel::Sender<StateExternal>,
) -> Result<channel::Sender<StateExternal>, Error> {
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
                println!("publishing {:#?} to {}", to_send, &p.topic);
                rp.publish(to_send).unwrap();
            }
        })
        .collect();

    let (tx_out, rx_out) = channel::unbounded();
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
    let (tx_in, rx_in) = channel::unbounded();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out = roscomm_setup(&mut node, &rc, tx_in)?;

    // "runner"
    thread::spawn(move || {
        launch_tokio(rx_in, tx_out);
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

use futures::future::{lazy, poll_fn};
use futures::Future;
use tokio_threadpool;

fn launch_tokio(rx: channel::Receiver<StateExternal>, tx: channel::Sender<StateExternal>) {
    let (runner, comm) = test_model();
    let (buf, mut to_buf, from_buf) = MessageBuffer::new(
        2,
        |_: &mut StateExternal, _: &StateExternal| {
            false // no merge
        },
    );

    let (runner, mut comm) = test_model();

    let pool = tokio_threadpool::ThreadPool::new();

    pool.spawn(future::lazy(move || {
        loop {
            let res = tokio_threadpool::blocking(|| {
                let msg = rx.recv().unwrap();
                to_buf.try_send(msg);
            })
            .map_err(|_| panic!("the threadpool shut down"));
        }
        Ok(())
    }));

    tokio::run(future::lazy(move || {
        tokio::spawn(runner);

        let mut runner_in = comm.state_input.clone();
        let getting = from_buf
            .for_each(move |result| {
                println!("into runner: {:?}", result);
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
                println!("From runner: {:?}", result);
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

fn test_model() -> (Runner, RunnerComm) {
    fn make_robot(name: &str, upper: i32) -> (SPState, RunnerTransitions) {
        let r = SPPath::from_str(&[name, "ref", "data"]);
        let a = SPPath::from_str(&[name, "act", "data"]);
        let activate = SPPath::from_str(&[name, "activ", "data"]);
        let activated = SPPath::from_str(&[name, "activated", "data"]);

        let to_upper = Transition::new(
            SPPath::from_str(&[name, "trans", "to_upper"]),
            p!(a == 0), // p!(r != upper), // added req on a == 0 just for testing
            vec![a!(r = upper)],
            vec![a!(a = upper)],
        );
        let to_lower = Transition::new(
            SPPath::from_str(&[name, "trans", "to_lower"]),
            p!(a == upper), // p!(r != 0), // added req on a == upper just for testing
            vec![a!(r = 0)],
            vec![a!(a = 0)],
        );
        let t_activate = Transition::new(
            SPPath::from_str(&[name, "trans", "activate"]),
            p!(!activated),
            vec![a!(activate)],
            vec![a!(activated)],
        );
        let t_deactivate = Transition::new(
            SPPath::from_str(&[name, "trans", "deactivate"]),
            p!(activated),
            vec![a!(!activate)],
            vec![a!(!activated)],
        );

        let h = hashmap!(
            r => 0.to_state(),
            a => 0.to_state(),
            activate => false.to_state(),
            activated => false.to_state()
        );

        let rt = RunnerTransitions {
            ctrl: vec![t_activate, t_deactivate],
            un_ctrl: vec![to_lower, to_upper],
        };

        (SPState { s: h }, rt)
    }

    let r1 = make_robot("r1", 10);
    let r2 = make_robot("r2", 10);

    let mut s = r1.0;
    s.extend(r2.0);
    let mut tr = r1.1;
    tr.extend(r2.1);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions::default(),
        ab_transitions: tr,
        plans: RunnerPlans::default(),
        state_functions: vec![],
        op_functions: vec![],
    };

    Runner::new(rm, s)
}
