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


fn json_to_state(
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
                        json_to_state_(json_child, field, p, a);
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

fn spval_to_json(spval: &SPValue) -> serde_json::Value {
    match spval {
        SPValue::Bool(x) => serde_json::json!(*x),
        SPValue::Int32(x) => serde_json::json!(*x),
        SPValue::Float32(x) => serde_json::json!(*x),
        SPValue::String(x) => serde_json::json!(x),
        SPValue::Array(_, x) => {
            let v: Vec<serde_json::Value> = x.iter().map(|spval| spval_to_json(spval)).collect();
            serde_json::json!(v)
        }
        _ => unimplemented!("TODO"),
    }
}

fn state_to_json(
    state: &StateExternal,
    md: &MessageField,
) -> serde_json::Value {
    fn state_to_json_<'a>(
        state: &StateExternal,
        md: &'a MessageField,
        p: &mut Vec<&'a str>,
    ) -> serde_json::Value {
        match md {
            MessageField::Msg(msg) => {
                let mut map = serde_json::Map::new();
                let path_name = msg.node().name();
                p.push(&path_name); // keep message type in path?
                for field in msg.fields() {
                    let field_name = field.node().name();
                    p.push(field_name);
                    map.insert(field_name.to_string(), state_to_json_(state, field, p));
                    p.pop();
                }
                p.pop();
                serde_json::Value::Object(map)
            }
            MessageField::Var(var) => {
                if let Some(spval) = state.s.get(&SPPath::from_array(p)) {
                    // TODO use sp type
                    let json = spval_to_json(spval); // , var.variable_data().type_);
                    json
                } else {
                    // TODO maybe panic here
                    serde_json::Value::Null
                }
            }
        }
    }

    let mut p = Vec::new();
    state_to_json_(state, md, &mut p)
}


#[cfg(test)]
mod ros_tests {
    use super::*;

    #[test]
    fn test_json_to_state() {
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

    #[test]
    fn test_state_to_json() {
        let payload = "hej".to_string();

        let mut state = SPState::default();
        state.insert(&SPPath::GlobalPath(GlobalPath::from_str(&["x", "y", "topic", "str", "data"])),
                                         AssignStateValue::SPValue(SPValue::String(payload)));

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

        let external = state.external();
        let local_state = external.unprefix_paths(&GlobalPath::from_str(&["x", "y", "topic"]));
        let json = state_to_json(&local_state, &msg);

        let data = json.get("data");
        assert!(data.is_some());
        assert_eq!(data, Some(&serde_json::Value::String("hej".into())));
    }
}


fn roscomm_setup(
    node: &mut r2r::Node,
    model: &Model,
    tx_in: channel::Sender<StateExternal>,
) -> Result<channel::Sender<StateExternal>, Error> {
    let mut ros_pubs = Vec::new();

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
                    println!("setting up subscription to topic: {}", topic);
                    let _subref = node.subscribe_untyped(&topic_str, m.msg_type(), Box::new(cb))?;
                }

                else if t.is_publisher() {
                    let topic = t.node().global_path().as_ref().unwrap();
                    let topic_str = topic.path().join("/");
                    println!("setting up publishing to topic: {}", topic);
                    let rp = node.create_publisher_untyped(&topic_str, m.msg_type())?;
                    let topic_cb = topic.clone();
                    let msgtype = t.msg().clone();
                    let cb = move |state: &StateExternal| {
                        let local_state = state.unprefix_paths(&topic_cb);
                        let to_send = state_to_json(&local_state, &msgtype);
                        rp.publish(to_send).unwrap();
                    };
                    ros_pubs.push(cb);
                }
                else {
                    panic!("topic is neither publisher nor subscriber. check variable types");
                }

            } else { panic!("must have a message under a topic"); }
        }
    }

    let (tx_out, rx_out) = channel::unbounded();
    thread::spawn(move || loop {
        let state = rx_out.recv().unwrap();
        for rp in &ros_pubs {
            (rp)(&state);
        }
    });

    Ok(tx_out)
}
