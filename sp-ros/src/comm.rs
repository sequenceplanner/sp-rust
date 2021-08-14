use std::sync::{Arc, Mutex};
use sp_domain::*;
use futures::*;

use crate::state_service::*;
use crate::model_service::*;


pub(crate) struct SPComm {
    state_service: SPStateService,
    model_service: SPModelService,
    control_service: SPControlService,
}


struct SPControlService {
    arc_node: Arc<Mutex<r2r::Node>>,
    state_from_runner: tokio::sync::watch::Receiver<SPState>,
    state_to_runner: tokio::sync::mpsc::Sender<SPState>,
    handle: Option<tokio::task::JoinHandle<Result<(), SPError>>>,
}


