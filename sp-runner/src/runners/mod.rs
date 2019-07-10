//! The runners in sp-runner 


use sp_domain::*;
use uuid::Uuid;

use tokio::prelude::*;
use tokio::*;
use sync::mpsc;
use sync::watch;

use std::collections::HashMap;



pub struct Runner {
    variables: RunnerVariables,
    op_transitions: Vec<RunnerTransitions>,
    ab_transitions: Vec<RunnerTransitions>,
    state: State,
    ctrl: RunnerCtrl,
    state_functions: Vec<Variable>,
    op_function: Vec<OperationFunction>,
    comm: RunnerComm,
}

struct RunnerVariables {
    variables: Vec<Variable>,
    paths: HashMap<Uuid, SPPath>,
}

struct RunnerTransitions {
    ctrl: Vec<Transition>,
    un_ctrl: Vec<Transition>
}

struct RunnerCtrl {
    pause: bool
}

struct RunnerComm {
    stateInput: mpsc::Receiver<State>,
    stateOutput: mpsc::Sender<State>,
    //commandInput: mpsc::Receiver<RunnerCommand>,
    //runnerInfo: watch::Sender<RunnerInfo>,
}
