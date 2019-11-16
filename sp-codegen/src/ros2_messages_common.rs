//! ROS2 message package common component generation for SP

extern crate askama;
extern crate dirs;

use std::fs::File;
use std::io::Write;
use std::fs;

use askama::Template;

#[derive(Template)] 
#[template(path = "ros2_messages_common/cmakelists_template.txt", print = "all", escape = "none")]
pub struct CMakeListsFile<'a> {
    package_name: &'a str,
    messages: Vec<String>,
    mapping_rules: Vec<String>
}

#[derive(Template)] 
#[template(path = "ros2_messages_common/description_file_template.xml", print = "all", escape = "none")]
pub struct DescriptionFile<'a> {
    package_name: &'a str,
    description: &'a str,
    email_address: &'a str,
    author_name: &'a str,
    generated_mapping_rules: Vec<String>
}

impl <'a> CMakeListsFile<'a> {
    pub fn new(package_name: &'a str, messages: Vec<String>, mapping_rules: Vec<String>) -> () {
        let file = CMakeListsFile {
            package_name: package_name,
            messages: messages,
            mapping_rules: mapping_rules
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}_msgs/CMakeLists.txt", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> DescriptionFile<'a> {
    pub fn new(package_name: &'a str,
               description: &'a str,
               email_address: &'a str,
               author_name: &'a str,
               generated_mapping_rules: Vec<String>) -> () {
        let file = DescriptionFile {
            package_name: package_name,
            description: description,
            email_address: email_address,
            author_name: author_name,
            generated_mapping_rules: generated_mapping_rules
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}_msgs/package.xml", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}
