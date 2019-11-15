//! ROS2 python package model based component generation for SP

extern crate askama;
extern crate dirs;

use std::fs::File;
use std::io::Write;
use std::fs;

use super::*;
use sp_domain::*;
use sp_runner_api::*;
use sp_runner::*;

use askama::Template;

#[derive(Template)] 
#[template(path = "ros2_python_common/configuration_file_template.cfg", print = "all", escape = "none")]
pub struct BasicEmulatorNode<'a> {
    package_name: &'a str,
    resource_name: String,
    capitalized_resource_name: String,
    message_type_interfacer_to_emulator: String,
    message_type_emulator_to_interfacer: String,
    measured_variables: Vec<String>,
    command_variables: Vec<String>,
    predicate_variables: Vec<String>,
    predicates: Vec<String>,
    actions: Vec<String>,
    effects: Vec<String>
}

impl <'a> BasicEmulatorNode<'a> {
    pub fn new(package_name: &'a str, resource: Resource ) -> () {

        let mut resource_name_val = resource.node().to_string();
        resource_name_val.drain(resource_name_val.find('<').unwrap_or(resource_name_val.len())..);

        fn capitalize(s: &str) -> String {
            let mut c = s.chars();
            match c.next() {
                None => String::new(),
                Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                }
            }

        let mut capitalized_resource_name_val = capitalize(&resource_name_val[..]);
        capitalized_resource_name_val.retain(|c| c != '_');

        let message_type_interfacer_to_emulator_val = format!("{}InterfacerToEmulator", capitalized_resource_name_val);
        let message_type_emulator_to_interfacer_val = format!("{}EmulatorToInterfacer", capitalized_resource_name_val); 
        let measured_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).0;
        let command_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).1;
        let predicate_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).2;

        let predicates: Vec<String> = Vec::new();

        for ab in resource.abilities() {
            ab.node();
            ab.predicates();
        };

        // let file = BasicEmulatorNode {
        //     package_name: package_name,
        //     resource_name: resource_name_val,
        //     capitalized_resource_name: capitalized_resource_name_val,
        //     message_type_interfacer_to_emulator: message_type_interfacer_to_emulator_val,
        //     message_type_emulator_to_interfacer: message_type_emulator_to_interfacer_val,
        //     measured_variables: measured_variables_val,
        //     command_variables: command_variables_val,
        //     predicate_variables: predicate_variables_val,

        } // } .render().unwrap();

        // let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/test/test_flake8_template.py", package_name)).unwrap();
        // write!(f, "{}", file).unwrap();
}