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
#[template(path = "ros2_python_model_based/basic_interfacer_node_template.py", print = "all", escape = "none")]
pub struct BasicInterfacerNode<'a> {
    package_name: &'a str,
    resource_name: String,
    capitalized_resource_name: String,
    message_type_interfacer_to_driver: String,
    message_type_driver_to_interfacer: String,
    message_type_interfacer_to_sp: String,
    message_type_sp_to_interfacer: String,
    measured_variables: Vec<String>,
    command_variables: Vec<String>
}

#[derive(Template)] 
#[template(path = "ros2_python_model_based/basic_emulator_node_template.py", print = "all", escape = "none")]
pub struct BasicEmulatorNode<'a> {
    package_name: &'a str,
    resource_name: String,
    capitalized_resource_name: String,
    message_type_interfacer_to_driver: String,
    message_type_driver_to_interfacer: String,
    measured_variables: Vec<String>,
    command_variables: Vec<String>,
    predicates: Vec<String>,
    transitions: Vec<String>
}

impl <'a> BasicInterfacerNode<'a> {
    pub fn new(package_name: &'a str, resource: Resource ) -> () {

        let mut resource_name_val = resource.node().to_string();
        resource_name_val.drain(resource_name_val.find('<').unwrap_or(resource_name_val.len())..);

        let res_name = resource_name_val.clone();

        fn capitalize(s: &str) -> String {
            let mut c = s.chars();
            match c.next() {
                None => String::new(),
                Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                }
            }

        let mut capitalized_resource_name_val = capitalize(&resource_name_val[..]);
        capitalized_resource_name_val.retain(|c| c != '_');

        let message_type_interfacer_to_driver_val = format!("{}InterfacerToDriver", capitalized_resource_name_val);
        let message_type_driver_to_interfacer_val = format!("{}DriverToInterfacer", capitalized_resource_name_val); 
        let message_type_interfacer_to_sp_val = format!("{}InterfacerToSP", capitalized_resource_name_val);
        let message_type_sp_to_interfacer_val = format!("{}SPToInterfacer", capitalized_resource_name_val); 
        let measured_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).0;
        let command_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).1;

        let file = BasicInterfacerNode {
            package_name: package_name,
            resource_name: resource_name_val,
            capitalized_resource_name: capitalized_resource_name_val,
            message_type_interfacer_to_driver: message_type_interfacer_to_driver_val,
            message_type_driver_to_interfacer: message_type_driver_to_interfacer_val,
            message_type_interfacer_to_sp: message_type_interfacer_to_sp_val,
            message_type_sp_to_interfacer: message_type_sp_to_interfacer_val,
            measured_variables: measured_variables_val,
            command_variables: command_variables_val,
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/src/{}_basic_interfacer.py", package_name, res_name)).unwrap();
        write!(f, "{}", file).unwrap();

    }
}

impl <'a> BasicEmulatorNode<'a> {
    pub fn new(package_name: &'a str, resource: Resource, predicates_val: Vec<String>, transitions_val: Vec<String>) -> () {

        let mut resource_name_val = resource.node().to_string();
        resource_name_val.drain(resource_name_val.find('<').unwrap_or(resource_name_val.len())..);

        let res_name = resource_name_val.clone();

        fn capitalize(s: &str) -> String {
            let mut c = s.chars();
            match c.next() {
                None => String::new(),
                Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                }
            }

        let mut capitalized_resource_name_val = capitalize(&resource_name_val[..]);
        capitalized_resource_name_val.retain(|c| c != '_');

        let message_type_interfacer_to_driver_val = format!("{}InterfacerToDriver", capitalized_resource_name_val);
        let message_type_driver_to_interfacer_val = format!("{}DriverToInterfacer", capitalized_resource_name_val); 
        let measured_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).0;
        let command_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).1;
        let predicate_variables_val = GetSPModelVariables::new(resource.clone(), &resource_name_val[..]).2;

        // Hmmm, need to parse transitions into a python executable format
        let file = BasicEmulatorNode {
            package_name: package_name,
            resource_name: resource_name_val,
            capitalized_resource_name: capitalized_resource_name_val,
            message_type_interfacer_to_driver: message_type_interfacer_to_driver_val,
            message_type_driver_to_interfacer: message_type_driver_to_interfacer_val,
            measured_variables: measured_variables_val,
            command_variables: command_variables_val,
            predicates: predicates_val,
            transitions: transitions_val
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/src/{}_basic_emulator.py", package_name, res_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

#[macro_export]
macro_rules! generate_python_model_based {
    ($pn:expr, $rn:expr, $predicates:expr, $transitions:expr) => {
        BasicInterfacerNode::new($pn, $rn);
        BasicEmulatorNode::new($pn, $rn, $predicates, $transitions);
    }
}