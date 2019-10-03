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
    let (runner_model, initial_state) = two_robots();

    // start ros node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "spnode", "")?;

    // data from resources to runner
    let (tx_in, rx_in) = channel::unbounded();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out = roscomm_setup(&mut node, &runner_model.model, tx_in.clone())?;

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

fn spval_from_json(json: &serde_json::Value, spv_t: SPValueType) -> SPValue {
    // as we have more options than json we switch on the spval type
    let tm = |msg: &str| {
        format!("type mismatch! got {}, expected {}! re-generate ros sources!", &json.to_string(), msg)
    };
    match spv_t {
        SPValueType::Bool => json.as_bool().expect(&tm("bool")).to_spvalue(),
        SPValueType::Int32 => (json.as_i64().expect(&tm("int")) as i32).to_spvalue(),
        SPValueType::Float32 => (json.as_f64().expect(&tm("float")) as f32).to_spvalue(),
        SPValueType::String => json.as_str().expect(&tm("string")).to_spvalue(),
        // todo: check is_array
        _ => unimplemented!("TODO"),
    }
}


pub fn json_to_state(
    json: &serde_json::Value,
    md: &MessageField,
) -> StateExternal {
    fn json_to_state_<'a>(
        json: &serde_json::Value,
        md: &'a MessageField,
        p: &mut Vec<&'a str>,
        a: &mut Vec<(Vec<&'a str>, SPValue)>,
    ) {
        match md {
            MessageField::Msg(msg) => {
                let path_name = msg.node().name();
                p.push(&path_name); // keep message type in path?
                for field in msg.fields() {
                    let field_name = field.node().name();
                    if let Some(json_child) = json.get(field_name) {
                        p.push(field_name);
                        json_to_state_(json_child, &field, p, a);
                        p.pop();
                    }
                }
                p.pop();
            }
            MessageField::Var(var) => {
                let sp_val = spval_from_json(json, var.value_type());
                a.push((p.clone(), sp_val));
            }
        }
    }

    let mut p = Vec::new();
    let mut a = Vec::new();
    json_to_state_(json, md, &mut p, &mut a);
    StateExternal {
        s: a.iter()
            .map(|(path, spval)| (SPPath::from_array(path), spval.clone()))
            .collect(),
    }
}

#[cfg(test)]
mod ros_tests {
    use super::*;

    #[test]
    fn test_std_msg_string() {
        let payload = "hej".to_string();
        let msg = r2r::std_msgs::msg::String { data: payload.clone() };
        let json = serde_json::to_value(msg).unwrap();

        let data = json.get("data");
        assert!(data.is_some());

        let v = Variable::new(
            "data",
            VariableType::Measured,
            SPValueType::String,
            "".to_spvalue(),
            Vec::new(),
        );

        let msg = Message::new_with_type(
            "str".into(),
            "std_msgs/msg/String".into(),
            vec![MessageField::Var(v)],
        );

        let msg = MessageField::Msg(msg);

        let s = json_to_state(&json, &msg);
        assert_eq!(s.s.get(&SPPath::from_array(&["str", "data"])), Some(&SPValue::String(payload.clone())));
    }
}


fn roscomm_setup(
    node: &mut r2r::Node,
    model: &Model,
    tx_in: channel::Sender<StateExternal>,
) -> Result<channel::Sender<StateExternal>, Error> {
    // let mut all_ros_pubs = Vec::new();

    let rcs: Vec<_> = model
        .items()
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    for r in rcs {
        for t in r.messages() {
            if let MessageField::Msg(m) = t.msg() {
                if t.is_subscriber() {
                    let topic = t.node().global_path().as_ref().unwrap();
                    let topic_str = topic.path().join("/");

                    let tx = tx_in.clone();
                    let topic_cb = topic.clone();
                    let msgtype = t.msg().clone();
                    let cb = move |msg: r2r::Result<serde_json::Value>| {
                        let json = msg.unwrap();
                        let state = json_to_state(&json, &msgtype);
                        let state = state.prefix_paths(&topic_cb);
                        tx.send(state).unwrap();
                    };
                    println!("setting up topic: {}", topic);
                    let _subref = node.subscribe_untyped(&topic_str, m.msg_type(), Box::new(cb))?;


                }

            } else { panic!("must have a message under a topic"); }


        }
        // if let Some(rc) = r.comm.as_ros_comm() {
        //     // setup ros subscribers
        //     for s in &rc.subscribers {
        //         // todo: fix lifetime issue when R is not static...
        //         let msg_type = s.definition.toplevel_msg_type().unwrap();
        //         let tx = tx_in.clone();
        //         let def = s.definition.clone();
        //         let topic = s.topic.to_owned();
        //         let cb = move |msg: r2r::Result<serde_json::Value>| {
        //             let json = msg.unwrap();
        //             let state = json_to_state(&json, &def, &topic);
        //             tx.send(state).unwrap();
        //         };
        //         let _subref = node.subscribe_untyped(&s.topic, &msg_type, Box::new(cb))?;
        //     }

        //     // setup ros publishers
        //     let ros_pubs: Vec<_> = rc
        //         .publishers
        //         .iter()
        //         .map(|p| {
        //             let msg_type = p.definition.toplevel_msg_type().unwrap();
        //             let rp = node.create_publisher_untyped(&p.topic, &msg_type).unwrap();
        //             let def = p.definition.clone();
        //             let topic = p.topic.to_owned();
        //             move |state: &StateExternal| {
        //                 let to_send = state_to_json(state, &def, &topic);
        //                 rp.publish(to_send).unwrap();
        //             }
        //         })
        //         .collect();

        //     all_ros_pubs.extend(ros_pubs);
        // }
    }

    let (tx_out, rx_out) = channel::unbounded();
    thread::spawn(move || loop {
        let state = rx_out.recv().unwrap();
        // for rp in &all_ros_pubs {
        //     (rp)(&state);
        // }
    });

    Ok(tx_out)
}
