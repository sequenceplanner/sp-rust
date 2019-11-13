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

impl <'a> ConfigurationFile<'a> {
    pub fn new(pn: &'a str) -> () {
        let conf_file = ConfigurationFile {
            package_name: pn
        }.render().unwrap();

        // format!("generated/ros2_sp_generated_ws/src/{}/src", pn
        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/setup.cfg", pn)).unwrap();
        write!(f, "{}", conf_file).unwrap();
    }
}

// #[test]
// fn conf_file_gen() {
//     ConfigurationFile::new("random_package_name_17");
// }