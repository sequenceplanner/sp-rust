//! ROS2 generic stuff generation for SP

extern crate dirs;

use std::fs;

use sp_domain::*;
use sp_runner_api::*;
use sp_runner::*;

pub struct GetSPModelVariables {}

impl GetSPModelVariables {
    pub fn new(resource: Resource, resource_name: &str) -> (Vec<String>, Vec<String>, Vec<String>) {
        
        let m = Model::new_root(&format!("{}_model", resource_name).to_owned()[..], vec![SPItem::Resource(resource)]);
        let rm = make_runner_model(&m);
        let s = make_initial_state(&m);

        let mut measured = Vec::new();
        let mut command = Vec::new();
        let mut predicates = Vec::new();

        for (key, value) in &s.s {
            let mut st = key.to_string();

            st.drain(..st.rfind('/').unwrap_or(st.len()));
            st.retain(|c| c != '/');

            let v: Vec<&str> = st.rsplit('_').collect();

            if v[0] == "m" {
                measured.push(st);
            } else if v[0] == "c" {
                command.push(st);
            } else {
                predicates.push(st);
            }
        };
        return (measured, command, predicates);
    }
}