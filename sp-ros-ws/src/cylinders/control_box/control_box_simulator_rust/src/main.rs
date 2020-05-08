use r2r;
use failure::Error;
use std::cell::RefCell;
use std::rc::Rc;

use r2r::control_box_msgs::msg::{Goal, State};
use r2r::sp_messages::msg::{NodeCmd, NodeMode};

fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let state_publisher = node.create_publisher::<State>("/control_box/state")?;
    let mode_publisher = node.create_publisher::<NodeMode>("/control_box/mode")?;
    let last_seen_goal = Rc::new(RefCell::new(Goal::default()));
    let node_mode = Rc::new(RefCell::new(NodeMode { mode: "init".into(), echo: "".into() }));

    // goal callback
    let last_seen_goal_cb = last_seen_goal.clone();
    let state_publisher_cb = state_publisher.clone();
    let sp_goal_cb = move |msg: Goal| {
        if *last_seen_goal_cb.borrow() == msg {
            return ;
        }
        last_seen_goal_cb.replace(msg);

        // todo: implement the rcl logger api
        // self.get_logger().info("blue light is " + ('on' if self.blue_light else 'off'))
        let blue_light_on = last_seen_goal_cb.borrow().blue_light;
        println!("blue light is {}", if blue_light_on { "on" } else { "off" });

        let msg = State { blue_light_on };
        state_publisher_cb.publish(&msg).unwrap();
    };

    let _goal_sub = node.subscribe("/control_box/goal", Box::new(sp_goal_cb))?;

    // sp node mode callback
    let last_seen_goal_cb = last_seen_goal.clone();
    let mode_publisher_cb = mode_publisher.clone();
    let node_mode_cb = node_mode.clone();
    let sp_mode_cb = move |msg: NodeCmd| {
        let new_mode = if msg.mode == "run" {
            String::from("running")
        } else {
            String::from("init")
        };

        let echo = serde_json::to_string(&*last_seen_goal_cb.borrow()).unwrap();
        node_mode_cb.replace(NodeMode { echo, mode: new_mode});
        mode_publisher_cb.publish(&node_mode_cb.borrow()).unwrap();
    };

    let _node_sub = node.subscribe("/control_box/node_cmd", Box::new(sp_mode_cb))?;

    loop {
        node.spin_once(std::time::Duration::from_millis(1000));

        // publish the state once in a while
        let msg = State { blue_light_on: last_seen_goal.borrow().blue_light };
        state_publisher.publish(&msg).unwrap();
    }
}
