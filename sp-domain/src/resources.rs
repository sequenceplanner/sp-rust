//! # Resource
//!
//! A resources is a core item when defining a system.
//!
//! It includes a list of variables that defines it state and
//! a list of abilties that defines what it can do.
//!
//! The variables are connected to ROS-messages that is defined by the
//! messages SPStruct.
//!
//! The resource can also depend on external variables, which is defined by
//! the parameters. Each parameter must be instantiated before the resource
//! can be used. The paths of all variables must also be updated based on
//! the overall structure of the system
//!
//!
//! Paths includes the structure of the messages, the local variables and the parameters
//! The local variables are named under the local/... path branch and the paramteters
//! under the param/... branch. Before running the resource, all abilities must be instanciated
//! where the local and parameters get a global path.
//!

use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum RosMsgDefinition {
    Message(String, HashMap<String, RosMsgDefinition>),
    Field(Variable),
}

impl RosMsgDefinition {
    fn flatten_to_paths_<'a>(
        &'a self,
        p: &mut Vec<&'a str>,
        a: &mut Vec<(Vec<&'a str>, &'a Variable)>,
    ) {
        match self {
            RosMsgDefinition::Message(msg_type, members) => {
                // p.push(&msg_type); // keep message type in path?
                for (k, v) in members.iter() {
                    p.push(k);
                    v.flatten_to_paths_(p, a);
                    p.pop();
                }
                // p.pop();
            }
            RosMsgDefinition::Field(var) => {
                a.push((p.clone(), &var));
            }
        }
    }

    pub fn flatten_to_paths(&self) -> Vec<(SPPath, &Variable)> {
        let mut p = Vec::new();
        let mut a = Vec::new();
        self.flatten_to_paths_(&mut p, &mut a);
        a.iter()
            .map(|(path, var)| (SPPath::from_str(path), *var))
            .collect()
    }

    pub fn toplevel_msg_type(&self) -> Option<String> {
        match self {
            RosMsgDefinition::Message(msg_type, _) => Some(msg_type.to_string()),
            _ => None,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct RosSubscriberDefinition {
    pub topic: String,
    pub definition: RosMsgDefinition,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct RosPublisherDefinition {
    pub topic: String,
    pub qos: String, // todo: should be the r2r QoS type. (which doesn't exist yet)
    pub definition: RosMsgDefinition,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct RosComm {
    pub node_name: String,
    pub node_namespace: String,
    pub subscribers: Vec<RosSubscriberDefinition>,
    pub publishers: Vec<RosPublisherDefinition>,
}

// Communication with the outside world.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum ResourceComm {
    RosComm(RosComm),
    // OPCUAComm(OPCUAComm),
    // etc...
}

fn spval_from_json(json: &serde_json::Value, spv_t: SPValueType) -> SPValue {
    // as we have more options than json we switch on the spval type
    let tm = |msg: &str| {
        format!("type mismatch! got {}, expected {}! re-generate ros sources!", &json.to_string(), msg)
    };
    match spv_t {
        SPValueType::Bool => json.as_bool().expect(&tm("bool")).to_spvalue(),
        SPValueType::Int32 => (json.as_i64().expect(&tm("int")) as i32).to_spvalue(),
        SPValueType::Float32 => (json.as_f64().expect(&tm("float")) as f32).to_spvalue(),
        SPValueType::String => json.as_str().expect(&tm("string")).to_spvalue(),
        // todo: check is_array
        _ => unimplemented!("TODO"),
    }
}

fn spval_to_json(spval: &SPValue) -> serde_json::Value {
    match spval {
        SPValue::Bool(x) => serde_json::json!(*x),
        SPValue::Int32(x) => serde_json::json!(*x),
        SPValue::Float32(x) => serde_json::json!(*x),
        SPValue::String(x) => serde_json::json!(x),
        SPValue::Array(_, x) => {
            let v: Vec<serde_json::Value> = x.iter().map(|spval| spval_to_json(spval)).collect();
            serde_json::json!(v)
        }
        _ => unimplemented!("TODO"),
    }
}

pub fn json_to_state(
    json: &serde_json::Value,
    md: &RosMsgDefinition,
    topic: &str,
) -> StateExternal {
    fn json_to_state_<'a>(
        json: &serde_json::Value,
        md: &'a RosMsgDefinition,
        p: &mut Vec<&'a str>,
        a: &mut Vec<(Vec<&'a str>, SPValue)>,
    ) {
        match md {
            RosMsgDefinition::Message(msg_type, members) => {
                p.push(&msg_type); // keep message type in path?
                for (field_name, md) in members.iter() {
                    if let Some(json_child) = json.get(field_name) {
                        p.push(field_name);
                        json_to_state_(json_child, md, p, a);
                        p.pop();
                    }
                }
                p.pop();
            }
            RosMsgDefinition::Field(var) => {
                let sp_val = spval_from_json(json, var.variable_data().type_);
                a.push((p.clone(), sp_val));
            }
        }
    }

    let mut p = vec!(topic);
    let mut a = Vec::new();
    json_to_state_(json, md, &mut p, &mut a);
    StateExternal {
        s: a.iter()
            .map(|(path, spval)| (SPPath::from_str(path), spval.clone()))
            .collect(),
    }
}

pub fn state_to_json(
    state: &StateExternal,
    md: &RosMsgDefinition,
    topic: &str,
) -> serde_json::Value {
    fn state_to_json_<'a>(
        state: &StateExternal,
        md: &'a RosMsgDefinition,
        p: &mut Vec<&'a str>,
    ) -> serde_json::Value {
        match md {
            RosMsgDefinition::Message(msg_type, members) => {
                let mut map = serde_json::Map::new();
                p.push(&msg_type); // keep message type in path?
                for (field_name, md) in members.iter() {
                    p.push(field_name);
                    map.insert(field_name.to_string(), state_to_json_(state, md, p));
                    p.pop();
                }
                p.pop();
                serde_json::Value::Object(map)
            }
            RosMsgDefinition::Field(var) => {
                if let Some(spval) = state.s.get(&SPPath::from_str(p)) {
                    // TODO
                    let json = spval_to_json(spval); // , var.variable_data().type_);
                    json
                } else { serde_json::Value::Null }
            }
        }
    }

    let mut p = vec!(topic);
    state_to_json_(state, md, &mut p)
}

impl ResourceComm {
    pub fn test_subs<'a>(&'a self) -> Vec<Box<dyn Fn(&serde_json::Value) -> StateExternal + 'a>> {
        match self {
            ResourceComm::RosComm(rc) => rc
                .subscribers
                .iter()
                .map(|s| {
                    let cb =
                        move |msg: &serde_json::Value| json_to_state(msg, &s.definition, &s.topic);
                    Box::new(cb) as Box<dyn Fn(&serde_json::Value) -> StateExternal + 'a>
                })
                .collect(),

            _ => unimplemented!(),
        }
    }

    pub fn test_pubs<'a>(&'a self) -> Vec<Box<dyn Fn(&StateExternal) -> serde_json::Value + 'a>> {
        match self {
            ResourceComm::RosComm(rc) => rc
                .publishers
                .iter()
                .map(|p| {
                    let cb = move |state: &StateExternal| state_to_json(state, &p.definition, &p.topic);
                    Box::new(cb) as Box<dyn Fn(&StateExternal) -> serde_json::Value + 'a>
                })
                .collect(),

            _ => unimplemented!(),
        }
    }

    pub fn variables(&self) -> Vec<(SPPath, &Variable)> {
        match self {
            ResourceComm::RosComm(rc) => {
                let mut vars: Vec<(SPPath, &Variable)> = Vec::new();
                for sub in &rc.subscribers {
                    let with_topic: Vec<(SPPath, &Variable)> = sub
                        .definition
                        .flatten_to_paths()
                        .iter()
                        .map(|(p, v)| {
                            let mut np = p.clone();
                            np.path.insert(0, sub.topic.clone());
                            (np, v.clone())
                        })
                        .collect();
                    vars.extend(with_topic);
                }
                for pub_ in &rc.subscribers {
                    let with_topic: Vec<(SPPath, &Variable)> = pub_
                        .definition
                        .flatten_to_paths()
                        .iter()
                        .map(|(p, v)| {
                            let mut np = p.clone();
                            np.path.insert(0, pub_.topic.clone());
                            (np, v.clone())
                        })
                        .collect();
                    vars.extend(with_topic);
                }
                vars
            }
            _ => unimplemented!(),
        }
    }

    pub fn as_ros_comm(&self) -> Option<&RosComm> {
        match self {
            ResourceComm::RosComm(rc) => Some(&rc),
            _ => None,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Resource {
    pub abilities: Vec<Ability>,
    pub parameters: Vec<Variable>,
    //    pub messages: SPStruct,  // defines the variable structure
    pub comm: ResourceComm,
}

impl Resource {
    // The new path should incl the name of the resource
    pub fn change_path(&mut self, new_path: SPPath) {}
}
