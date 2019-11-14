//! ROS2 python package model based component generation for SP

extern crate askama;
extern crate dirs;

use std::fs::File;
use std::io::Write;
use std::fs;

use askama::Template;

#[derive(Template)] 
#[template(path = "ros2_python_common/configuration_file_template.cfg", print = "all", escape = "none")]
pub struct BasicEmulatorNode<'a> {
    package_name: &'a str,
    message_type_interfacer_to_emulator: &'a str,
    message_type_emulator_to_interfacer: &'a str,
    capitalized_resource_name: &'a str,
    resource_name: &'a str,
    measured_variables: Vec<&'a str>,
    command_variables: Vec<&'a str>,
    predicates: Vec<&'a str>,
    actions: Vec<&'a str>,
    effects: Vec<&'a str>
}