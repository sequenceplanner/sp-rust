
use sp_domain::*;
use std::time::{Instant, Duration};
use std::collections::HashMap;
use sp_ros;

#[derive(Debug)]
pub struct NodeHandler {
    nodes: HashMap<SPPath, RosNode>,
    ability_plan: Option<PlanProgress>,
    effect_progress: Vec<EffectProgress>,
}

pub struct NodeHandlerResult {
    dead_nodes: Vec<SPPath>,
    dead_effects: Vec<SPPath>,
    dead_plans: Vec<SPPath>

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
            effect_progress: vec!(), 
        }
    }

    // pub fn newMessage(&mut self, mess: sp_ros::RosMessage, state: SPState) -> NodeHandlerResult {

    // }

    
}


#[derive(Debug, PartialEq, Clone)]
struct RosNode {
    path: SPPath, // the path of the resource
    cmd: NodeCmd,
    last_tick: Option<std::time::Instant>,  // Last massage instant
    last_echo: Option<SPState>,
}
#[derive(Debug, PartialEq, Clone)]
enum NodeCommands {
    Init,
    Run,
}

#[derive(Debug, PartialEq, Clone)]
struct PlanProgress {
    plan: SPPath,
    plan_no: usize,
    time_stamp: Instant,
    state: SPState,
}

#[derive(Debug, PartialEq, Clone)]
struct EffectProgress {
    transition: SPPath,
    waiting_for_effect: Action,  // Will all be true at the same time?
    max_time: Option<Duration>,
    time_stamp: Instant,
}