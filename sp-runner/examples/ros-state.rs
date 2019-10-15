use crossbeam::channel;
use failure::Error;
use futures::Future;
use sp_domain::*;
use sp_runner::*;
use sp_runner_api::*;
use sp_ros;
use std::thread;
use tokio::prelude::*;
use tokio_threadpool;

fn main() -> Result<(), Error> {
    let (runner_model, initial_state) = two_dummy_robots();

    // start ros node
    let mut node = sp_ros::start_node()?;

    // data from resources to runner
    let (tx_in, rx_in) = channel::unbounded();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out = sp_ros::roscomm_setup(&mut node, &runner_model.model, tx_in)?;

    // misc runner data to/from the network.
    let (tx_in_misc, rx_in_misc) = channel::unbounded();

    // setup ros pub/subs. tx_out to send out to network
    let tx_out_misc = sp_ros::roscomm_setup_misc(&mut node, tx_in_misc)?;

    // "runner"
    thread::spawn(move || {
        let (runner, comm) = Runner::new(runner_model, initial_state);
        launch_tokio(runner, comm, rx_in, tx_out, rx_in_misc, tx_out_misc);
    });

    loop {
        // blocking ros spinning
        sp_ros::spin(&mut node);
    }
}

fn launch_tokio(
    runner: Runner,
    comm: RunnerComm,
    rx: channel::Receiver<StateExternal>,
    tx: channel::Sender<StateExternal>,
    rx_command: channel::Receiver<RunnerCommand>,
    tx_info: channel::Sender<RunnerInfo>,
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

    let mut ci = comm.command_input.clone();
    #[allow(unreachable_code)]
    pool.spawn(future::lazy(move || {
        loop {
            let _res = tokio_threadpool::blocking(|| {
                let command = rx_command.recv().unwrap();
                let _res = ci.try_send(command);
            })
            .map_err(|_| panic!("the threadpool shut down"));
        }
        Ok(())
    }));

    let mut runner_in = comm.state_input.clone();
    #[allow(unreachable_code)]
    tokio::run(future::lazy(move || {
        tokio::spawn(runner);

        let getting = from_buf
            .for_each(move |result| {
                // println!("INTO RUNNER");
                // println!("{}", result);
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
                println!("RUNNER CURRENT STATE");
                println!("{}", result.external());
                tx.try_send(result.external()).unwrap(); // if we fail the ROS side to do not keep up
                Ok(())
            })
            .map(move |_| ())
            .map_err(|e| eprintln!("error = {:?}", e));
        tokio::spawn(from_runner);

        let ro = comm.runner_output;
        let from_runner_info = ro
            .for_each(move |info| {
                tx_info.try_send(info).unwrap(); // if we fail the ROS side to do not keep up
                Ok(())
            })
            .map(move |_| ())
            .map_err(|e| eprintln!("error = {:?}", e));
        tokio::spawn(from_runner_info);

        tokio::spawn(buf);

        Ok(())
    }));
}
