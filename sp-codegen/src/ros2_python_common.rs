//! ROS2 python package common component generation for SP

extern crate askama;
extern crate dirs;

use std::fs::File;
use std::io::Write;
use std::fs;

use askama::Template;

#[derive(Template)] 
#[template(path = "ros2_python_common/configuration_file_template.cfg", print = "all", escape = "none")]
pub struct ConfigurationFile<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/readme_template.md", print = "all", escape = "none")]
pub struct ReadmeFile<'a> {
    package_name: &'a str
}

pub struct ResourceFile<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/test_copyright_template.py", print = "all", escape = "none")]
pub struct TestCopyrightFile<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/test_flake8_template.py", print = "all", escape = "none")]
pub struct TestFlake8File<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/test_pep257_template.py", print = "all", escape = "none")]
pub struct TestPep257File<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/description_file_template.xml", print = "all", escape = "none")]
pub struct DescriptionFile<'a> {
    package_name: &'a str,
    description: &'a str,
    email_address: &'a str,
    author_name: &'a str,
}

#[derive(Template)] 
#[template(path = "ros2_python_common/setup_file_template.py", print = "all", escape = "none")]
pub struct SetupFile<'a> {
    package_name: &'a str,
    modules: Vec<String>,
    email_address: &'a str,
    author_name: &'a str,
    description: &'a str,
    scripts: Vec<String>
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

impl <'a> ReadmeFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = ReadmeFile {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/README.md", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> ResourceFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/resource/{}", package_name, package_name)).unwrap();
        write!(f, "{}", "").unwrap();
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
               resources: Vec<&'a str>,
               email_address: &'a str,
               author_name: &'a str,
               description: &'a str) -> () {

        let mut modules_val: Vec<String> = Vec::new();
        let mut scripts_val: Vec<String> = Vec::new();

        for resource in resources {
            modules_val.push(format!("src.{}_basic_emulator", resource.to_string()));
            modules_val.push(format!("src.{}_basic_interfacer", resource.to_string()));

            scripts_val.push(format!("{}_basic_emulator = src.{}_basic_emulator:main", resource.to_string(), resource.to_string()));
            scripts_val.push(format!("{}_basic_interfacer = src.{}_basic_interfacer:main", resource.to_string(), resource.to_string()));
        }

        let file = SetupFile {
            package_name: package_name,
            modules: modules_val,
            email_address: email_address,
            author_name: author_name,
            description: description,
            scripts: scripts_val
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/setup.py", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> TestCopyrightFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = TestCopyrightFile {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/test/test_copyright_template.py", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> TestPep257File<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = TestPep257File {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/test/test_pep257_template.py", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> TestFlake8File<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = TestFlake8File {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/test/test_flake8_template.py", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

#[macro_export]
macro_rules! generate_python_common {
    ($pn:expr, $res:expr) => {
        Directories::new($pn);
        ConfigurationFile::new($pn);
        ReadmeFile::new($pn);
        ResourceFile::new($pn);
        TestCopyrightFile::new($pn);
        TestPep257File::new($pn);
        TestFlake8File::new($pn);
        DescriptionFile::new($pn, "somedescription", "e@e.com", "endre");
        SetupFile::new($pn, 
            vec!["door1"], 
            "e@e.com", 
            "endre",
            "somedescription")
    }
}