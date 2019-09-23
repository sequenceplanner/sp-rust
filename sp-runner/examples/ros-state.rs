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
    let (runner_model, initial_state, resources) = test_model();

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
                        println!("publishing {:#?} to {}", to_send, &topic);
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

#[derive(Debug)]
pub struct TempModel {
    pub initial_state: SPState,
    pub variables: HashMap<SPPath, Variable>,
    pub runner_transitions: RunnerTransitions,
    pub resource: Resource,
}

fn test_model() -> (RunnerModel, SPState, Vec<Resource>) {
    fn make_robot(name: &str, upper: i32) -> TempModel {
        let r = SPPath::from_str(&[name, "ref", "data"]);
        let a = SPPath::from_str(&[name, "act", "data"]);
        let activate = SPPath::from_str(&[name, "activate", "data"]);
        let activated = SPPath::from_str(&[name, "activated", "data"]);

        let r_c = Variable::Command(VariableData {
            type_: SPValueType::Int32,
            initial_value: None,
            domain: (0..upper + 1).map(|v| v.to_spvalue()).collect(),
        });

        let a_m = Variable::Measured(VariableData {
            type_: SPValueType::Int32,
            initial_value: None,
            domain: (0..upper + 1).map(|v| v.to_spvalue()).collect(),
        });

        let act_c = Variable::Command(VariableData {
            type_: SPValueType::Bool,
            initial_value: None,
            domain: Vec::new(),
        });

        let act_m = Variable::Measured(VariableData {
            type_: SPValueType::Bool,
            initial_value: None,
            domain: Vec::new(),
        });

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

        let s = state!(
            r => 0,
            a => 0,
            activate => false,
            activated => false
        );

        let vars = hashmap![
            r => r_c.clone(),
            a => a_m.clone(),
            activate => act_c.clone(),
            activated => act_m.clone()
        ];

        // ros comm stuff
        let comm = ResourceComm::RosComm(RosComm {
            publishers: vec![RosPublisherDefinition {
                topic: format!("/{}/ref", name),
                qos: "".into(),
                definition: RosMsgDefinition::Message(
                    "std_msgs/msg/Int32".into(),
                    hashmap![
                        "data".into() => RosMsgDefinition::Field(r_c.clone())],
                ),
            },
            RosPublisherDefinition {
                topic: format!("/{}/activate", name),
                qos: "".into(),
                definition: RosMsgDefinition::Message(
                    "std_msgs/msg/Bool".into(),
                    hashmap![
                        "data".into() => RosMsgDefinition::Field(act_c.clone())],
                ),
            }
            ],
            subscribers: vec![
                RosSubscriberDefinition {
                    topic: format!("/{}/act", name),
                    definition: RosMsgDefinition::Message(
                        "std_msgs/msg/Int32".into(),
                        hashmap![
                            "data".into() => RosMsgDefinition::Field(a_m.clone())
                        ],
                    ),
                },
                RosSubscriberDefinition {
                    topic: format!("/{}/activated", name),
                    definition: RosMsgDefinition::Message(
                        "std_msgs/msg/Bool".into(),
                        hashmap![
                            "data".into() => RosMsgDefinition::Field(act_m.clone())
                        ],
                    ),
                },
            ],
        });

        let r = Resource {
            abilities: Vec::new(),
            parameters: Vec::new(),
            comm: comm,
        };

        TempModel {
            initial_state: s,
            variables: vars,
            runner_transitions: RunnerTransitions {
                ctrl: vec![t_activate, t_deactivate],
                un_ctrl: vec![to_lower, to_upper],
            },
            resource: r,
        }
    }

    let r1 = make_robot("r1", 10);
    let r2 = make_robot("r2", 10);

    let mut s = r1.initial_state;
    s.extend(r2.initial_state);
    let mut tr = r1.runner_transitions;
    tr.extend(r2.runner_transitions);
    let mut vars = r1.variables;
    vars.extend(r2.variables.into_iter());

    let r = vec![r1.resource, r2.resource];

    println!("Model initial state:\n{}", s.external());

    let rm = RunnerModel {
        op_transitions: RunnerTransitions::default(),
        ab_transitions: tr,
        plans: RunnerPlans::default(),
        state_functions: vec![],
        op_functions: vec![],
        vars: vars,
    };

    (rm, s, r)
}
