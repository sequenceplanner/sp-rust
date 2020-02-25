
use sp_domain::*;
use std::time::{Instant, Duration};
use std::collections::HashMap;

#[derive(Debug)]
pub struct NodeHandler {
    nodes: HashMap<SPPath, RosNode>,
    ability_plan: Option<PlanProgress>,
    effectProgress: Vec<EffectProgress>,
}

impl NodeHandler {
    pub fn new(nodes: &[SPPath]) -> Self {
        let xs: HashMap<SPPath, RosNode> = nodes.iter().map(|n| {
            let rn = RosNode {
                path: n.clone(),
                cmd: NodeCmd::Init,
                last_tick: None,
                last_echo: None,
            };
            (n.clone(), rn)
        }).collect();
        NodeHandler {
            nodes: xs, 
            ability_plan: None, 
            effectProgress: vec!(), 
        }
    }

    
}


#[derive(Debug, PartialEq, Clone)]
struct RosNode {
    path: SPPath, // the path of the resource
    cmd: NodeCmd,
    last_tick: Option<std::time::Instant>,  // Last massage instant
    last_echo: Option<SPState>,
}
#[derive(Debug, PartialEq, Clone)]
enum NodeCmd {
    Init,
    Run,
}

#[derive(Debug, PartialEq, Clone)]
struct PlanProgress {
    plan_no: usize,
    time_stamp: Instant,
    state: SPState,
}

#[derive(Debug, PartialEq, Clone)]
struct EffectProgress {
    waiting_for_effect: Action,  // Will all be true at the same time?
    max_time: Option<Duration>,
    time_stamp: Instant,
}