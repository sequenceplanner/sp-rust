use chrono::offset::Local;
use chrono::DateTime;
use serde::{Deserialize, Serialize};
use sp_domain::*;
use crate::planning::*;
use crate::runners::*;
use std::collections::HashMap;
use std::error::Error;
use std::fmt;
use std::fs::File;
use std::io::prelude::*;
use std::io::Write;
use std::process::{Command, Stdio};
use std::time::SystemTime;
use std::time::{Duration, Instant};


// for now atleast...
fn g_path_or_panic_node(n: &SPNode) -> &GlobalPath {
    n.global_path().as_ref().expect(&format!("node must have global path: {}", n.name()))
}

fn g_path_or_panic_path(p: &SPPath) -> &GlobalPath {
    p.as_global().as_ref().expect(&format!("planner requires global paths: {}", p))
}

fn indent(n: u32) -> String {
    (0..n).map(|_| " ").collect::<Vec<&str>>().concat()
}

struct NuXMVPath<'a>(&'a GlobalPath);
impl fmt::Display for NuXMVPath<'_> {
    fn fmt<'a>(&'a self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0.path().join("#"))
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
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(g_path_or_panic_path(p))),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(g_path_or_panic_path(p))),
                };

                format!("{} = {}", xx, yy)
            }
            Predicate::NEQ(x, y) => {
                let xx = match x {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(g_path_or_panic_path(p))),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p) => format!("{}", NuXMVPath(g_path_or_panic_path(p))),
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
    path: &GlobalPath,
) -> Vec<(Transition, Action)> {
    let mut r = Vec::new();

    for t in transitions {
        let a = t.actions().iter().find(|a| g_path_or_panic_path(&a.var) == path);
        let e = t.effects().iter().find(|a| g_path_or_panic_path(&a.var) == path);

        // can not modify the same path twice... I think.
        assert!(!(a.is_some() && e.is_some()));

        if let Some(a) = a {
            // println!("match!: {}", NuXMVPredicate(&t.guard));
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
            PredicateValue::SPPath(path) => Some(format!("{}", NuXMVPath(g_path_or_panic_path(path)))),
        },
        Compute::Predicate(p) => Some(format!("{}", NuXMVPredicate(&p))),
        _ => None,
    }
}

fn create_nuxmv_problem(goal: &Predicate, state: &StateExternal, model: &RunnerModel, vars: &HashMap<SPPath, Variable>) -> String {
    let mut lines = String::from("MODULE main");
    lines.push_str("\n");
    lines.push_str("VAR\n");

    for (path, variable) in vars {
        let path = NuXMVPath(g_path_or_panic_path(path));
        if variable.value_type() == SPValueType::Bool {
            lines.push_str(&format!("{i}{v} : boolean;\n", i = indent(2), v = path));
        } else {
            let domain: Vec<_> = variable
                .domain()
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
    for ct in &model.ab_transitions.ctrl {
        let path = NuXMVPath(g_path_or_panic_node(ct.node()));
        lines.push_str(&format!("{i}{v} : boolean;\n", i = indent(2), v = path));
    }

    lines.push_str("\n\n");
    lines.push_str("ASSIGN\n");
    lines.push_str("\n\n");
    lines.push_str("-- CURRENT STATE --\n");
    for (path, value) in &state.s {
        let path = NuXMVPath(g_path_or_panic_path(path));
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
    for ct in &model.ab_transitions.ctrl {
        let path = NuXMVPath(g_path_or_panic_node(ct.node()));
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

    for path in vars.keys() {
        let path = NuXMVPath(g_path_or_panic_path(path));

        lines.push_str(&format!("{i}next({v}) := case\n", i = indent(2), v = path));

        // here we need to find all relevant transitions, which are:
        // either A) actions that change the current path or B)
        // effects that change the current path
        let relevant = find_actions_modifying_path(&model.ab_transitions.un_ctrl, &path.0);
        for (t, a) in &relevant {
            let p = NuXMVPredicate(&t.guard());

            // ouch. so ugly.
            let v = action_to_string(&a).expect("model too complicated");

            lines.push_str(&format!(
                "{i}{p} : {a};  -- {c} (un-controllable)\n",
                i = indent(4),
                p = p,
                a = v,
                c = g_path_or_panic_node(t.node())
            ));
        }

        // controllable events (for now go by name)
        // copy pasted from above....... .....
        let relevant = find_actions_modifying_path(&model.ab_transitions.ctrl, &path.0);
        for (t, a) in &relevant {
            let p = NuXMVPredicate(t.guard());

            let v = action_to_string(&a).expect("model too complicated!!");

            lines.push_str(&format!(
                "{i}{t} & {p} : {a};  -- {c} (controllable)\n",
                i = indent(4),
                t = NuXMVPath(g_path_or_panic_node(t.node())),
                p = p,
                a = v,
                c = g_path_or_panic_node(t.node())
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
    let ctrl_names: Vec<_> = model.ab_transitions
        .ctrl
        .iter()
        .map(|c| NuXMVPath(g_path_or_panic_node(c.node())).to_string())
        .collect();
    let ctrl_names_sep = ctrl_names.join(",");
    lines.push_str(&format!(
        "{i}count({n}) <= 1;\n",
        i = indent(2),
        n = ctrl_names_sep
    ));

    // finally, print out the ltl spec
    lines.push_str("\n\n");

    let goal = NuXMVPredicate(goal);
    lines.push_str(&format!("LTLSPEC ! F ( {} );", goal));

    return lines;
}

fn spval_from_nuxvm(nuxmv_val: &str, spv_t: SPValueType) -> SPValue {
    // as we have more options than json we switch on the spval type
    let tm = |msg: &str| format!("type mismatch! got {}, expected {}!", nuxmv_val, msg);
    match spv_t {
        SPValueType::Bool => {
            if nuxmv_val == "TRUE" {
                true.to_spvalue()
            } else {
                false.to_spvalue()
            }
        }
        SPValueType::Int32 => {
            let intval: i32 = nuxmv_val.parse().expect(&tm("int32"));
            intval.to_spvalue()
        }
        SPValueType::Float32 => {
            let fval: f32 = nuxmv_val.parse().expect(&tm("float32"));
            fval.to_spvalue()
        }
        SPValueType::String => nuxmv_val.to_spvalue(),
        // todo: check is_array
        _ => unimplemented!("TODO"),
    }
}

fn call_nuxmv(max_steps: u32, filename: &str) -> std::io::Result<(String,String)> {
    let process = Command::new("nuxmv")
        .arg("-int")
        .arg(filename)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;

    let command = format!(
        "go_bmc\ncheck_ltlspec_bmc_inc -k {}\nshow_traces -v\nquit\n",
        max_steps
    );

    process.stdin.unwrap().write_all(command.as_bytes())?;

    let mut raw = String::new();
    process.stdout.unwrap().read_to_string(&mut raw)?;

    let mut raw_error = String::new();
    process.stderr.unwrap().read_to_string(&mut raw_error)?;


    Ok((raw,raw_error))
}

fn postprocess_nuxmv_problem(raw: &String, model: &RunnerModel, vars: &HashMap<SPPath, Variable>) -> Option<Vec<PlanningFrame>> {
    if !raw.contains("Trace Type: Counterexample") {
        return None;
    }

    let lines = raw.lines();
    let s = lines
        .rev()
        .take_while(|l| !l.contains("Trace Type: Counterexample"));
    let mut s: Vec<String> = s.map(|s| s.to_owned()).collect();
    s.pop(); // skip first state label
    s.reverse();

    let mut trace = Vec::new();
    let mut last = PlanningFrame::default();

    for l in &s {
        if l.contains("  -> State: ") || l.contains("nuXmv >") {
            trace.push(last);
            last = PlanningFrame::default();
        } else {
            let path_val: Vec<_> = l.split("=").map(|s| s.trim()).collect();
            let path = GlobalPath::from(path_val[0].split("#").map(|s|s.to_owned()).collect());
            let sppath = SPPath::GlobalPath(path.clone());

            let val = path_val.get(1).expect("no value!");

            // check for controllable actions
            if model.ab_transitions.ctrl.iter().find(|t| g_path_or_panic_node(t.node()) == &path).is_some() {
                if val == &"TRUE" {
                    assert!(last.ctrl.is_none());
                    last.ctrl = Some(sppath);
                }
            } else {
                // get SP type from path
                let spt = vars
                    .get(&sppath)
                    .expect(&format!("path mismatch! {}", path))
                    .value_type();

                let spval = spval_from_nuxvm(val, spt);
                last.state.s.insert(sppath, spval);
            }
        }
    }

    Some(trace)
}

pub fn compute_plan(
    goal: &Predicate,
    state: &StateExternal,
    model: &RunnerModel,
    max_steps: u32,
) -> PlanningResult {
    // create variable definitions based on the state
    let vars: HashMap<SPPath, Variable> = state.s.iter().flat_map(|(k,v)| match k {
        SPPath::GlobalPath(gp) => {
            let v = model.model.get(&gp.to_sp()).expect("variable must exist!");
            Some((gp.to_sp(),v.unwrap_variable()))
        }
        _ => None
    }).collect();

    let lines = create_nuxmv_problem(&goal, &state, &model, &vars);

    let datetime: DateTime<Local> = SystemTime::now().into();
    // todo: use non platform way of getting temporary folder
    // or maybe just output to a subfolder 'plans'
    let filename = &format!("/tmp/planner {}.bmc", datetime);
    let mut f = File::create(filename).unwrap();
    write!(f, "{}", lines).unwrap();

    let start = Instant::now();

    match call_nuxmv(max_steps, filename) {
        Ok((raw, raw_error)) => {
            let duration = start.elapsed();
            let trace = postprocess_nuxmv_problem(&raw, model, &vars);

            PlanningResult {
                plan_found: trace.is_some(),
                trace: trace.unwrap_or_default(),
                time_to_solve: duration,
                raw_output: raw.to_owned(),
                raw_error_output: raw_error.to_owned(),
            }
        }
        Err(e) => {
            let duration = start.elapsed();
            PlanningResult {
                plan_found: false,
                trace: Vec::new(),
                time_to_solve: duration,
                raw_output: "".into(),
                raw_error_output: e.to_string(),
            }
        }
    }
}