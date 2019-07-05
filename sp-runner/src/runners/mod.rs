//! The runners in sp-runner 


use sp_domain::*;

use tokio::prelude::*;
use tokio::*;
use sync::mpsc;
use sync::watch;

struct Runner {
    resources: Vec<Resource>,
    operations: Vec<Operation>,
    comm: RunnerComm,
}

struct RunnerComm {
    stateInput: mpsc::Receiver<State>,
    stateOutput: mpsc::Sender<State>,
    //commandInput: mpsc::Receiver<RunnerCommand>,
    //runnerInfo: watch::Sender<RunnerInfo>,
}
