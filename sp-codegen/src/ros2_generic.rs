//! ROS2 generic stuff generation for SP

extern crate dirs;

use std::fs;

pub struct Directories {}

impl Directories {
    // be sure to generate directories before generating files
    pub fn new(pn: &str) -> std::io::Result<()> {
        fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src/{}/resource", pn))?;
        fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src/{}/src", pn))?;
        fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src/{}/test", pn))?;
        fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src/{}_msgs/msg", pn))?;
        Ok(())
    }
}