//! ROS2 python package generation for SP

extern crate askama;
extern crate dirs;

use std::fs::File;
use std::io::Write;
use std::fs;

use askama::Template;

#[derive(Template)] 
#[template(path = "ros2_python_package/configuration_file_template.cfg", print = "all", escape = "none")]
pub struct ConfigurationFile<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_package/description_file_template.xml", print = "all", escape = "none")]
pub struct DescriptionFile<'a> {
    package_name: &'a str,
    description: &'a str,
    email_address: &'a str,
    author_name: &'a str,
}

#[derive(Template)] 
#[template(path = "ros2_python_package/setup_file_template.py", print = "all", escape = "none")]
pub struct SetupFile<'a> {
    package_name: &'a str,
    modules: Vec<&'a str>,
    email_address: &'a str,
    author_name: &'a str,
    description: &'a str,
    scripts: Vec<&'a str>
}

impl <'a> ConfigurationFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = ConfigurationFile {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/setup.cfg", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> DescriptionFile<'a> {
    pub fn new(package_name: &'a str,
               description: &'a str,
               email_address: &'a str,
               author_name: &'a str) -> () {
        let file = DescriptionFile {
            package_name: package_name,
            description: description,
            email_address: email_address,
            author_name: author_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/package.xml", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> SetupFile<'a> {
    pub fn new(package_name: &'a str,
               modules: Vec<&'a str>,
               email_address: &'a str,
               author_name: &'a str,
               description: &'a str,
               scripts: Vec<&'a str>) -> () {
        let file = SetupFile {
            package_name: package_name,
            modules: modules,
            email_address: email_address,
            author_name: author_name,
            description: description,
            scripts: scripts
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/setup.py", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}