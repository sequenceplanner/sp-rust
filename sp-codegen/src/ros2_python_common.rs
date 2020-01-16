//! ROS2 python package common component generation for SP

extern crate askama;
extern crate dirs;

use std::fs::File;
use std::io::Write;
use std::fs;

use askama::Template;

#[derive(Template)] 
#[template(path = "ros2_python_common/configuration_file_template.cfg", print = "all", escape = "none")]
pub struct PyConfigurationFile<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/readme_template.md", print = "all", escape = "none")]
pub struct PyReadmeFile<'a> {
    package_name: &'a str
}

pub struct PyResourceFile<'a> {
    package_name: &'a str
}

pub struct PyInitFile<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/test_copyright_template.py", print = "all", escape = "none")]
pub struct PyTestCopyrightFile<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/test_flake8_template.py", print = "all", escape = "none")]
pub struct PyTestFlake8File<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/test_pep257_template.py", print = "all", escape = "none")]
pub struct PyTestPep257File<'a> {
    package_name: &'a str
}

#[derive(Template)] 
#[template(path = "ros2_python_common/description_file_template.xml", print = "all", escape = "none")]
pub struct PyDescriptionFile<'a> {
    package_name: &'a str,
    description: &'a str,
    email_address: &'a str,
    author_name: &'a str,
}

#[derive(Template)] 
#[template(path = "ros2_python_common/setup_file_template.py", print = "all", escape = "none")]
pub struct PySetupFile<'a> {
    package_name: &'a str,
    modules: Vec<String>,
    email_address: &'a str,
    author_name: &'a str,
    description: &'a str,
    scripts: Vec<String>
}

impl <'a> PyConfigurationFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = PyConfigurationFile {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/setup.cfg", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> PyReadmeFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = PyReadmeFile {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/README.md", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> PyResourceFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/resource/{}", package_name, package_name)).unwrap();
        write!(f, "{}", "").unwrap();
    }
}

impl <'a> PyInitFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/src/__init__.py", package_name)).unwrap();
        write!(f, "{}", "").unwrap();
    }
}

impl <'a> PyDescriptionFile<'a> {
    pub fn new(package_name: &'a str,
               description: &'a str,
               email_address: &'a str,
               author_name: &'a str) -> () {
        let file = PyDescriptionFile {
            package_name: package_name,
            description: description,
            email_address: email_address,
            author_name: author_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/package.xml", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> PySetupFile<'a> {
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

        let file = PySetupFile {
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

impl <'a> PyTestCopyrightFile<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = PyTestCopyrightFile {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/test/test_copyright_template.py", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> PyTestPep257File<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = PyTestPep257File {
            package_name: package_name
        }.render().unwrap();

        let mut f = File::create(format!("generated/ros2_sp_generated_ws/src/{}/test/test_pep257_template.py", package_name)).unwrap();
        write!(f, "{}", file).unwrap();
    }
}

impl <'a> PyTestFlake8File<'a> {
    pub fn new(package_name: &'a str) -> () {
        let file = PyTestFlake8File {
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
        PyConfigurationFile::new($pn);
        PyInitFile::new($pn);
        PyReadmeFile::new($pn);
        PyResourceFile::new($pn);
        PyTestCopyrightFile::new($pn);
        PyTestPep257File::new($pn);
        PyTestFlake8File::new($pn);
        PyDescriptionFile::new($pn, "somedescription", "e@e.com", "endre");
        PySetupFile::new($pn, 
            vec!["door1"], 
            "e@e.com", 
            "endre",
            "somedescription")
    }
}