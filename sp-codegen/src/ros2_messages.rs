//! ROS2 message package generation for SP

// extern crate askama;
// extern crate dirs;

// use chrono::offset::Local;
// use chrono::DateTime;
// use std::time::SystemTime;
// use std::fs::File;
// use std::io::Write;
// use std::fs;

// use askama::Template;

// pub struct Directories {}

// #[derive(Template)] 
// #[template(path = "ros2_python_package/configuration_file_template.cfg", print = "all", escape = "none")]
// pub struct ConfigurationFile<'a> {
//     package_name: &'a str
// }

// impl Directories {
//     pub fn new(pn: &str) -> std::io::Result<()> {
//         // let datetime: DateTime<Local> = SystemTime::now().into();
//         // fs::create_dir_all(format!("generated/ros2_sp_ws_{}", datetime))?;
//         // fs::create_dir_all(format!("generated/ros2_sp_ws_{}/src", datetime))?;
//         // fs::create_dir_all(format!("generated/ros2_sp_ws_{}/src/{}/resource", datetime, pn))?;
//         // fs::create_dir_all(format!("generated/ros2_sp_ws_{}/src/{}/src", datetime, pn))?;
//         // fs::create_dir_all(format!("generated/ros2_sp_ws_{}/src/{}/test", datetime, pn))?;
//         fs::create_dir_all(format!("generated/ros2_sp_generated_ws"))?;
//         fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src"))?;
//         fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src/{}/resource", pn))?;
//         fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src/{}/src", pn))?;
//         fs::create_dir_all(format!("generated/ros2_sp_generated_ws/src/{}/test", pn))?;
//         Ok(())
//     }
// }

// impl <'a> ConfigurationFile<'a> {
//     pub fn new(pn: &'a str) -> () {
//         let conf_file = ConfigurationFile {
//             package_name: pn
//         }.render().unwrap();
    
//         let home = dirs::home_dir(); //.unwrap().to_str();
//         // // let where = "/sp-rust/sp-codegen/generated/setup.cfg".to_string();
//         // format!("{}{}{}", starbase, "_to_", colony).to_owned()

//         let mut f = File::create("generated/asdf/setup.cfg").unwrap();
//         write!(f, "{}", conf_file).unwrap();
//     }
// }

// #[test]
// fn directories_gen() {
//     Directories::new("random_package_name_17");
// }

// #[test]
// fn conf_file_gen() {
//     ConfigurationFile::new("random_package_name_17");
// }