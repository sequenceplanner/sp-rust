use std::collections::HashMap;
use sp_domain::*;
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct RunnerModel {
    pub vars: HashMap<SPPath, Variable>,
    pub ctrl: Vec<Transition>,
    pub un_ctrl: Vec<Transition>,
}

fn make_robot(name: &str, upper: i32) -> (SPState, RunnerModel) {
    let r = SPPath::from_str(&[name, "ref"]);
    let a = SPPath::from_str(&[name, "act"]);
    let activate = SPPath::from_str(&[name, "activate"]);
    let activated = SPPath::from_str(&[name, "activated"]);

    let r_c = Variable::Command(VariableData {
        type_: SPValueType::Int32,
        initial_value: None,
        domain: (0..upper).map(|v|v.to_spvalue()).collect(),
    });

    let a_m = Variable::Measured(VariableData {
        type_: SPValueType::Int32,
        initial_value: None,
        domain: (0..upper).map(|v|v.to_spvalue()).collect(),
    });

    let act_c = Variable::Command(VariableData {
        type_: SPValueType::Bool,
        initial_value: None,
        domain: Vec::new(),
    });

    let act_m = Variable::Measured(VariableData {
        type_: SPValueType::Bool,
        initial_value: None,
        domain: Vec::new(),
    });

    let to_upper = Transition::new(
        format!("{}_to_upper", name),
        p!(a == 0), // p!(r != upper), // added req on a == 0 just for testing
        vec![a!(r = upper)],
        vec![a!(a = upper)],
    );
    let to_lower = Transition::new(
        format!("{}_to_lower", name),
        p!(a == upper), // p!(r != 0), // added req on a == upper just for testing
        vec![a!(r = 0)],
        vec![a!(a = 0)],
    );
    let t_activate = Transition::new(
        format!("{}_activate", name),
        p!(!activated),
        vec![a!(activate)],
        vec![a!(activated)],
    );
    let t_deactivate = Transition::new(
        format!("{}_activate", name),
        p!(activated),
        vec![a!(!activate)],
        vec![a!(!activated)],
    );

    let s = state!(
        r => 0,
        a => 0,
        activate => false,
        activated => false
    );

    let vars = hashmap![
        r => r_c,
        a => a_m,
        activate => act_c,
        activated => act_m
    ];

    let rt = RunnerModel {
        vars,
        ctrl: vec![t_activate, t_deactivate],
        un_ctrl: vec![to_lower, to_upper],
    };

    (s, rt)
}

fn indent(n: u32) -> String {
    (0..n).map(|_|" ").collect::<Vec<&str>>().concat()
}

struct NuXMVPath(SPPath);
impl fmt::Display for NuXMVPath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f,"{}",self.0.path.join("#"))
    }
}

struct NuXMVValue(SPValue);
impl fmt::Display for NuXMVValue {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            SPValue::Bool(b) if b => write!(fmtr,"TRUE"),
            SPValue::Bool(b) => write!(fmtr,"FALSE"),
            SPValue::Float32(f) => write!(fmtr,"{}", f),
            SPValue::Int32(i) => write!(fmtr,"{}", i),
            SPValue::String(s) => write!(fmtr,"{}", s),
            SPValue::Time(t) => write!(fmtr,"{:?}", t),
            SPValue::Duration(d) => write!(fmtr,"{:?}", d),
            SPValue::Array(at, a) => write!(fmtr,"{:?}", a),
            SPValue::Unknown => write!(fmtr,"[unknown]"),
        }
    }
}


#[test]
fn planner() {
    let (state, model) = make_robot("r1", 10);
    let state = state.external();

    let mut lines = String::from("MODULE main");
    lines.push_str("\n");
    lines.push_str("VAR\n");

    for (path,variable) in &model.vars {
        let path = NuXMVPath(path.clone());
        if variable.variable_data().type_ == SPValueType::Bool {
            lines.push_str(&format!("{i}{v} : boolean;\n", i=indent(2),v=path));
        } else {
            let domain: Vec<_> = variable.variable_data().domain.
                iter().map(|v|NuXMVValue(v.clone()).to_string()).collect();
            let domain = domain.join(",");
            lines.push_str(&format!("{i}{v} : {{{d}}};\n",
                                    i=indent(2),
                                    v=path,
                                    d=domain));
        }
    }

    lines.push_str("\n\n");
    lines.push_str("-- CURRENT STATE --\n");
    lines.push_str("ASSIGN\n");
    for (path,value) in &state.s {
        let path = NuXMVPath(path.clone());
        let value = NuXMVValue(value.clone());
        lines.push_str(&format!("{i}init({v}) := {spv}\n",
                                i=indent(2),v=path,spv=value));
    }



    println!("{}", lines);

    println!("{}", state);
}
