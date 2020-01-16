//! ROS2 message package common component generation for SP

extern crate askama;
extern crate dirs;

use std::fs::File;
use std::io::Write;
use std::fs;

use askama::Template;

#[derive(Template)] 
#[template(path = "ros2_messages_common/cmakelists_template.txt", print = "all", escape = "none")]
pub struct MsgCMakeListsFile<'a> {
    package_name: &'a str,
    messages: Vec<String>,
    mapping_rules: Vec<String>
}

#[derive(Template)] 
#[template(path = "ros2_messages_common/description_file_template.xml", print = "all", escape = "none")]
pub struct MsgDescriptionFile<'a> {
    package_name: &'a str,
    description: &'a str,
    email_address: &'a str,
    author_name: &'a str,
    generated_mapping_rules: Vec<String>
}

#[derive(Template)] 
#[template(path = "ros2_messages_common/readme_template.md", print = "all", escape = "none")]
pub struct MsgReadmeFile<'a> {
    package_name: &'a str
}

impl <'a> MsgCMakeListsFile<'a> {
    pub fn new(package_name: &'a str, messages: Vec<String>, mapping_rules: Vec<String>) -> () {
        let file = MsgCMakeListsFile {
            package_name: package_name,
            messages: messages,
            mapping_rules: mapping_rules
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}_msgs/CMakeLists.txt", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> MsgDescriptionFile<'a> {
    pub fn new(package_name: &'a str,
               description: &'a str,
               email_address: &'a str,
               author_name: &'a str,
               generated_mapping_rules: Vec<String>) -> () {
        let file = MsgDescriptionFile {
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

impl <'a> MsgReadmeFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = MsgReadmeFile {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}_msgs/README.md", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}
