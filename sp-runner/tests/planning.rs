use serde::{Deserialize, Serialize};
use sp_domain::*;
use std::collections::HashMap;
use std::fmt;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use std::error::Error;
use std::io::prelude::*;
use std::process::{Command, Stdio};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct PlanningFrame {
    pub state: HashMap<SPPath, SPValue>,
    // The controllable transition taken, if one was taken.
    pub ctrl: Option<SPPath>,
}


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
        domain: (0..upper + 1).map(|v| v.to_spvalue()).collect(),
    });

    let a_m = Variable::Measured(VariableData {
        type_: SPValueType::Int32,
        initial_value: None,
        domain: (0..upper + 1).map(|v| v.to_spvalue()).collect(),
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
        SPPath::from_str(&[name, "trans", "to_upper"]),
        p!(a == 0), // p!(r != upper), // added req on a == 0 just for testing
        vec![a!(r = upper)],
        vec![a!(a = upper)],
    );
    let to_lower = Transition::new(
        SPPath::from_str(&[name, "trans", "to_lower"]),
        p!(a == upper), // p!(r != 0), // added req on a == upper just for testing
        vec![a!(r = 0)],
        vec![a!(a = 0)],
    );
    let t_activate = Transition::new(
        SPPath::from_str(&[name, "trans", "activate"]),
        p!(!activated),
        vec![a!(activate)],
        vec![a!(activated)],
    );
    let t_deactivate = Transition::new(
        SPPath::from_str(&[name, "trans", "deactivate"]),
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
    (0..n).map(|_| " ").collect::<Vec<&str>>().concat()
}

struct NuXMVPath<'a>(&'a SPPath);
impl fmt::Display for NuXMVPath<'_> {
    fn fmt<'a>(&'a self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0.path.join("#"))
    }
}

struct NuXMVValue<'a>(&'a SPValue);
impl fmt::Display for NuXMVValue<'_> {
    fn fmt<'a>(&'a self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            SPValue::Bool(b) if *b => write!(fmtr, "TRUE"),
            SPValue::Bool(b) => write!(fmtr, "FALSE"),
            SPValue::Float32(f) => write!(fmtr, "{}", f),
            SPValue::Int32(i) => write!(fmtr, "{}", i),
            SPValue::String(s) => write!(fmtr, "{}", s),
            SPValue::Time(t) => write!(fmtr, "{:?}", t),
            SPValue::Duration(d) => write!(fmtr, "{:?}", d),
            SPValue::Array(at, a) => write!(fmtr, "{:?}", a),
            SPValue::Unknown => write!(fmtr, "[unknown]"),
        }
    }
}

struct NuXMVPredicate<'a>(&'a Predicate);

impl fmt::Display for NuXMVPredicate<'_> {
    fn fmt<'a>(&'a self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s: String = match &self.0 {
            Predicate::AND(x) => {
                let children: Vec<_> = x
                    .iter()
                    .map(|p| format!("{}", NuXMVPredicate(&p)))
                    .collect();
                children.join("&")
            }
            Predicate::OR(x) => {
                let children: Vec<_> = x
                    .iter()
                    .map(|p| format!("{}", NuXMVPredicate(&p)))
                    .collect();
                children.join("|")
            }
            Predicate::XOR(_) => "TODO".into(), // remove from pred?
            Predicate::NOT(p) => format!("(!{})", NuXMVPredicate(&p)),
            Predicate::TRUE => "TRUE".into(),
            Predicate::FALSE => "FALSE".into(),
            Predicate::EQ(x, y) => {
                let xx = match x {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(&p)),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(&p)),
                };

                format!("{} = {}", xx, yy)
            }
            Predicate::NEQ(x, y) => {
                let xx = match x {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(&p)),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(&p)),
                };

                format!("{} != {}", xx, yy)
            }

            _ => "TODO".into(),
        };

        write!(fmtr, "{}", &s)
    }
}

fn find_actions_modifying_path(
    transitions: &[Transition],
    path: &SPPath,
) -> Vec<(Transition, Action)> {
    let mut r = Vec::new();

    for t in transitions {
        let a = t.actions.iter().find(|a| &a.var == path);
        let e = t.effects.iter().find(|a| &a.var == path);

        // can not modify the same path twice... I think.
        assert!(!(a.is_some() && e.is_some()));

        if let Some(a) = a {
            println!("match!: {}", NuXMVPredicate(&t.guard));
            r.push((t.clone(), a.clone()));
        }
        if let Some(e) = e {
            r.push((t.clone(), e.clone()))
        }
    }

    r
}

fn action_to_string(a: &Action) -> Option<String> {
    // ouch. so ugly.
    match &a.value {
        Compute::PredicateValue(pv) => match pv {
            PredicateValue::SPValue(spval) => Some(format!("{}", NuXMVValue(&spval))),
            PredicateValue::SPPath(path) => Some(format!("{}", NuXMVPath(&path))),
        },
        Compute::Predicate(p) =>
            Some(format!("{}", NuXMVPredicate(&p))),
        _ => None,
    }
}

// TODO: goal should be predicate
fn create_nuxmv_problem(goal: &str, state: &StateExternal, model: &RunnerModel) -> String {
    let mut lines = String::from("MODULE main");
    lines.push_str("\n");
    lines.push_str("VAR\n");

    for (path, variable) in &model.vars {
        let path = NuXMVPath(&path);
        if variable.variable_data().type_ == SPValueType::Bool {
            lines.push_str(&format!("{i}{v} : boolean;\n", i = indent(2), v = path));
        } else {
            let domain: Vec<_> = variable
                .variable_data()
                .domain
                .iter()
                .map(|v| NuXMVValue(v).to_string())
                .collect();
            let domain = domain.join(",");
            lines.push_str(&format!(
                "{i}{v} : {{{d}}};\n",
                i = indent(2),
                v = path,
                d = domain
            ));
        }
    }

    // add a control variable for each controllable transition
    for ct in &model.ctrl {
        let path = NuXMVPath(&ct.path);
        lines.push_str(&format!("{i}{v} : boolean;\n",
                                i = indent(2), v = path));
    }

    lines.push_str("\n\n");
    lines.push_str("ASSIGN\n");
    lines.push_str("\n\n");
    lines.push_str("-- CURRENT STATE --\n");
    for (path, value) in &state.s {
        let path = NuXMVPath(path);
        let value = NuXMVValue(value);
        lines.push_str(&format!(
            "{i}init({v}) := {spv};\n",
            i = indent(2),
            v = path,
            spv = value
        ));
    }

    lines.push_str("\n\n");
    lines.push_str("-- CONTROL VARIABLE STATE --\n");
    // add a control variable for each controllable transition
    for ct in &model.ctrl {
        let path = NuXMVPath(&ct.path);
        let false_ = false.to_spvalue();
        let value = NuXMVValue(&false_); // they're all false
        lines.push_str(&format!(
            "{i}init({v}) := {spv};\n",
            i = indent(2),
            v = path,
            spv = value
        ));
    }

    lines.push_str("\n\n");
    lines.push_str("-- TRANSITIONS --\n");
    lines.push_str("");

    for path in model.vars.keys() {
        let path = NuXMVPath(path);

        lines.push_str(&format!("{i}next({v}) := case\n", i = indent(2), v = path));

        // here we need to find all relevant transitions, which are:
        // either A) actions that change the current path or B)
        // effects that change the current path
        let relevant = find_actions_modifying_path(&model.un_ctrl, &path.0);
        for (t, a) in &relevant {
            let p = NuXMVPredicate(&t.guard);

            // ouch. so ugly.
            let v = action_to_string(&a).expect("model too complicated");

            lines.push_str(&format!(
                "{i}{p} : {a};  -- {c} (un-controllable)\n",
                i = indent(4),
                p = p,
                a = v,
                c = t.path
            ));
        }

        // controllable events (for now go by name)
        // copy pasted from above....... .....
        let relevant = find_actions_modifying_path(&model.ctrl, &path.0);
        for (t, a) in &relevant {
            let p = NuXMVPredicate(&t.guard);

            let v = action_to_string(&a).expect("model too complicated!!");

            lines.push_str(&format!(
                "{i}{t} & {p} : {a};  -- {c} (controllable)\n",
                i = indent(4),
                t = NuXMVPath(&t.path),
                p = p,
                a = v,
                c = t.path
            ));
        }

        // for now just keep the current value.
        lines.push_str(&format!("{i}TRUE : {v};\n", i = indent(4), v = path));
        lines.push_str(&format!("{i}esac;\n", i = indent(2)));
        lines.push_str(&format!("\n"));
    }

    // add invariant stating only one controllable event can be active at a time
    lines.push_str("\n\n");
    lines.push_str("INVAR\n");
    let ctrl_names: Vec<_> = model.ctrl.iter().map(|c| NuXMVPath(&c.path).to_string()).collect();
    let ctrl_names_sep = ctrl_names.join(",");
    lines.push_str(&format!(
        "{i}count({n}) <= 1;\n",
        i = indent(2),
        n = ctrl_names_sep
    ));

    // finally, print out the ltl spec
    lines.push_str("\n\n");

    lines.push_str(&format!("LTLSPEC ! F ( {} );", goal));

    return lines;
}



#[test]
fn planner() {
    let (state, model) = make_robot("r1", 10);
    let state = state.external();

    let lines = create_nuxmv_problem("r1#activated", &state, &model);

    let out_fn = PathBuf::from("/home/martin/tests/bmc/slash.bmc");
    let mut f = File::create(out_fn).unwrap();
    write!(f, "{}", lines).unwrap();

    println!("{}", state);

    let process = match Command::new("nuxmv")
        .arg("-int")
        .arg("/home/martin/tests/bmc/slash.bmc")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn() {
            Err(why) => panic!("couldn't spawn nuxmv command: {}", why.description()),
            Ok(process) => process,
        };

    let command = "go_bmc\ncheck_ltlspec_bmc_inc -k 20\nshow_traces -v\nquit\n";
    match process.stdin.unwrap().write_all(command.as_bytes()) {
        Err(why) => panic!("couldn't write to stdin: {}",
                           why.description()),
        Ok(_) => println!("sent command to nuxmv"),
    }

    let mut s = String::new();
    match process.stdout.unwrap().read_to_string(&mut s) {
        Err(why) => panic!("couldn't read stdout: {}",
                           why.description()),
        _ => println!("read output from nuxmv")
    }

    let lines = s.lines();
    let s = lines.rev().take_while(|l|!l.contains("Trace Type: Counterexample"));
    let mut s: Vec<String> = s.map(|s|s.to_owned()).collect();
    s.pop(); // skip first state label
    s.reverse();

    let mut trace = Vec::new();
    let mut last = PlanningFrame::default();

    for l in &s {
        if l.contains("  -> State: ") || l.contains("nuXmv >") {
            trace.push(last);
            last = PlanningFrame::default();
        } else {
            let path_val: Vec<_> = l.split("=").map(|s|s.trim()).collect();
            let path = SPPath::from_str(path_val[0].split("#").collect::<Vec<&str>>().as_ref());

            let val = path_val.get(1).expect("no value!");

            // check for controllable actions
            if model.ctrl.iter().find(|t|t.path==path).is_some() {
                if val == &"TRUE" {
                    assert!(last.ctrl.is_none());
                    last.ctrl = Some(path);
                }
            } else {
                // get SP type from path
                let spt = model.vars.get(&path).expect(&format!("path mismatch! {}", path))
                    .variable_data().type_;


                let spval = if spt == SPValueType::Bool {
                    if val == &"TRUE" {
                        true.to_spvalue()
                    } else {
                        false.to_spvalue()
                    }
                } else {
                    0.to_spvalue()
                };

                last.state.insert(path, spval);
            }
        }
    }

    println!("result\n{}", s.join("\n"));
    println!("result\n{:#?}", trace);

}
