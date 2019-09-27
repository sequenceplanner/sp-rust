//! The SP domain

#![allow(unused_mut)] // for some reason I get not a correct warning for mut in macros

pub mod values;
pub use values::*;

pub mod predicates;
pub use predicates::*;

pub mod states;
pub use states::*;

pub mod paths;
pub use paths::*;

pub mod node;
pub use node::*;

pub mod items;
pub use items::*;

// TODO: Probably remove this later
pub mod resources;
pub use crate::resources::*;

mod utils;
use utils::*;


use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::error;
use std::fmt;


type Result<T> = std::result::Result<T, SPError>;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SPError {
    OverwriteDelay(states::Delay, AssignStateValue),
    OverwriteNext(states::Next, AssignStateValue),
    No(String),
    Undefined,
}

impl fmt::Display for SPError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            SPError::OverwriteDelay(next, prev) => write!(
                f,
                "You are trying to overwrite a Delay in the State. current: {:?}, new: {:?} ",
                prev, next
            ),
            SPError::OverwriteNext(next, prev) => write!(
                f,
                "You are trying to overwrite a Next in the State. current: {:?}, new: {:?} ",
                prev, next
            ),
            SPError::Undefined => write!(f, "An undefined SP error!"),
            SPError::No(s) => write!(f, "Oh No: {}", s),
        }
    }
}

impl error::Error for SPError {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        None
    }
}




#[cfg(test)]
mod tests_domain {
    use super::*;
    #[test]
    fn making() {
        let mut m = Model::new_root("model", vec!());
        let mut r1 = Resource::new("r1");
        
        let a = Topic::new("act", MessageField::Var(Variable::new(
            "data", 
            VariableType::Measured, 
            SPValueType::Int32,
            0.to_spvalue(), 
            vec!(0.to_spvalue(), 10.to_spvalue())
        )));
        let r = Topic::new("ref", MessageField::Var(Variable::new(
            "data", 
            VariableType::Command, 
            SPValueType::Int32,
            0.to_spvalue(), 
            vec!(0.to_spvalue(), 10.to_spvalue())
        )));
        let active = Topic::new("active", MessageField::Var(Variable::new_boolean(
            "data", 
            VariableType::Measured
        )));
        let activate = Topic::new("activate", MessageField::Var(Variable::new_boolean(
            "data", 
            VariableType::Command, 
        )));

        let mut a = r1.add_message(a).local_path().clone().unwrap();
        a.add("data".to_string());
        let mut r = r1.add_message(r).local_path().clone().unwrap();
        r.add("data".to_string());
        let mut active = r1.add_message(active).local_path().clone().unwrap();
        active.add("data".to_string());
        let mut activate = r1.add_message(activate).local_path().clone().unwrap();
        activate.add("data".to_string());

        

        let name = "r1";
        let upper = 10;
        let to_upper = Transition::new(
            &format!("{}_to_upper", name),
            p!(a == 0), // p!(r != upper), // added req on a == 0 just for testing
            vec!(a!(r = upper)),
            vec!(a!(a = upper)),
        );
        let to_lower = Transition::new(
            &format!("{}_to_lower", name),
            p!(a == upper), // p!(r != 0), // added req on a == upper just for testing
            vec!(a!(r = 0)),
            vec!(a!(a = 0)),
        );
        let t_activate = Transition::new(
            &format!("{}_activate", name),
            p!(!active),
            vec!(a!(activate)),
            vec!(a!(active)),
        );
        let t_deactivate = Transition::new(
            &format!("{}_deactivate", name),
            p!(active),
            vec!(a!(!activate)),
            vec!(a!(!active)),
        );

        let ability = Ability::new(
            "all", 
            vec!(t_activate, t_deactivate), 
            vec!(to_upper, to_lower), 
            vec!()
        );
        
        let ability = r1.add_ability(ability);

        let r1 = m.add_item(SPItem::Resource(r1)).global_path().clone().unwrap();
    

        let resource = if let Some(SPItemRef::Resource(r)) = m.find(&r1.to_sp()) {Some(r)} else {None};
        println!("");
        println!("resource: {:?}", resource);
        println!("");

        if let Some(SPItemRef::Resource(r)) = m.find(&r1.to_sp()) {
            let a_again = r.find(&a.to_sp());
            println!("the resource {:?}", r);
            println!("the a {:?}", a_again);
        }

        



        

        
        // let n = SPItem::Model(Model{
        //     node: SPNode::new("n"),
        //     items: vec!()
        // });

        // if let SPItem::Model(ref mut my) = m {
        //     my.add_item(n);
        // }
        
        println!("{:?}", m);
    }

    #[test]
    fn find() {
        let mut m = Model::new("m", vec!(
            SPItem::Model(Model::new("a", vec!(
                SPItem::Model(Model::new("b", vec!())),
                SPItem::Model(Model::new("c", vec!(
                    SPItem::Model(Model::new("d", vec!()))
                )))
            ))),
            SPItem::Model(Model::new("k", vec!(
                SPItem::Model(Model::new("l", vec!()))
            )))
        ));

        m.update_path(&SPPaths::new(None, Some(GlobalPath::new())));

        let g_ab = SPPath::from_array_to_global(&["m", "a", "b"]);
        let g_acd = SPPath::from_array_to_global(&["m", "a", "c", "d"]);
        let g_k = SPPath::from_array_to_global(&["m", "k"]);
        let ab = m.find(&g_ab);
        let acd = m.find(&g_acd);
        let k = m.find(&g_k);

        println!("{:?}", &ab);
        println!("{:?}", &acd);
        println!("{:?}", &k);

        assert!(ab.unwrap().name() == "b");
        assert!(acd.unwrap().name() == "d");
        assert!(k.unwrap().name() == "k");
    }
}




