use chrono::offset::Local;
use chrono::DateTime;
use sp_domain::*;
use sp_runner_api::*;
use crate::planning::*;
use std::collections::HashMap;
use std::collections::HashSet;
use std::fmt;
use std::fs::File;
use std::io::prelude::*;
use std::io::Write;
use std::process::{Command, Stdio};
use std::time::{SystemTime, Instant};


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
            SPValue::Bool(_b) => write!(fmtr, "FALSE"),
            SPValue::Float32(f) => write!(fmtr, "{}", f),
            SPValue::Int32(i) => write!(fmtr, "{}", i),
            SPValue::String(s) => write!(fmtr, "{}", s),
            SPValue::Time(t) => write!(fmtr, "{:?}", t),
            SPValue::Duration(d) => write!(fmtr, "{:?}", d),
            SPValue::Array(_at, a) => write!(fmtr, "{:?}", a),
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
                format!("( {} )", children.join("&"))
            }
            Predicate::OR(x) => {
                let children: Vec<_> = x
                    .iter()
                    .map(|p| format!("{}", NuXMVPredicate(&p)))
                    .collect();
                format!("( {} )", children.join("|"))
            }
            Predicate::XOR(_) => "TODO".into(), // remove from pred?
            Predicate::NOT(p) => format!("!({})", NuXMVPredicate(&p)),
            Predicate::TRUE => "TRUE".into(),
            Predicate::FALSE => "FALSE".into(),
            Predicate::EQ(x, y) => {
                let xx = match x {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p, _) => format!("{}", NuXMVPath(p)),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p, _) => format!("{}", NuXMVPath(p)),
                };

                format!("( {} = {} )", xx, yy)
            }
            Predicate::NEQ(x, y) => {
                let xx = match x {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p, _) => format!("{}", NuXMVPath(p)),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", NuXMVValue(&v)),
                    PredicateValue::SPPath(p, _) => format!("{}", NuXMVPath(p)),
                };

                format!("( {} != {} )", xx, yy)
            }
        };

        write!(fmtr, "{}", &s)
    }
}

fn action_to_string(a: &Action) -> Option<String> {
    // ouch. so ugly.
    match &a.value {
        Compute::PredicateValue(pv) => match pv {
            PredicateValue::SPValue(spval) => Some(format!("{}", NuXMVValue(&spval))),
            PredicateValue::SPPath(path, _) => Some(format!("{}", NuXMVPath(path))),
        },
        Compute::Predicate(p) => Some(format!("{}", NuXMVPredicate(&p))),
        _ => None,
    }
}

fn create_nuxmv_problem(goals: &Vec<Predicate>, state: &SPState, model: &RunnerModel) -> String {
    let items = model.model.items();
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();
    let vars: HashMap<SPPath, Variable> = resources.iter().flat_map(|r| r.get_variables()).map(|v| (v.path().clone(), v.clone())).collect();

    // let specs: Vec<Spec> = model.model.items().iter().flat_map(|i| match i {
    //     SPItem::Spec(s) => Some(s),
    //     _ => None,
    // }).cloned().collect();

    let mut lines = String::from("MODULE main");
    lines.push_str("\n");
    lines.push_str("VAR\n");

    lines.push_str("-- MODEL VARIABLES\n");
    for (path, variable) in &vars {
        let path = NuXMVPath(path);
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

    lines.push_str("-- CONTROL VARIABLES\n");
    lines.push_str("IVAR \n\n");

    // add a control variable for each transition
    let all_trans = model.ab_transitions.ctrl.iter().chain(model.ab_transitions.un_ctrl.iter());
    for t in all_trans {
        let path = NuXMVPath(t.node().path());
        lines.push_str(&format!("{i}{v} : boolean;\n", i = indent(2), v = path));
    }
    lines.push_str("\n\n");

    // add DEFINES for specs and state predicates
    lines.push_str("DEFINE\n\n");

    lines.push_str("-- STATE PREDICATES\n");

    for sp in &model.state_predicates {
        let path = NuXMVPath(sp.node().path());
        match sp.variable_type() {
            VariableType::Predicate(p) => {
                let p = NuXMVPredicate(&p);
                lines.push_str(&format!(
                    "{i}{v} := {p};\n",
                    i = indent(2),
                    v = path,
                    p = p,
                ));
            },
            _ => {
                panic!("model error")
            }
        }
    }

    lines.push_str("\n\n");

    lines.push_str("ASSIGN\n");
    lines.push_str("\n\n");
    lines.push_str("-- CURRENT STATE --\n");

    for (path, _variable) in &vars {
        let value = state.sp_value_from_path(path).expect("all variables need a valuation!");
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
    lines.push_str("-- TRANSITIONS --\n");
    lines.push_str("");
    lines.push_str("TRANS\n\n");


    let mut all_vars = HashSet::new();
    all_vars.extend(vars.keys().cloned());

    let mut trans = Vec::new();

    let all_trans = model.ab_transitions.ctrl.iter().chain(model.ab_transitions.un_ctrl.iter());
    for t in all_trans {
        let modified = modified_by(t);
        let untouched = all_vars.difference(&modified);

        let keep: Vec<_> = untouched.map(|path| {
            let path = NuXMVPath(path);
            format!("( next({v}) = {v} )", v = path)
        }).collect();

        let assign = |a: &Action| {
                let path = NuXMVPath(&a.var);
                let value = action_to_string(&a).expect("model too complicated");
                format!("next({}) = {}", path, value)
            };
        let mut updates: Vec<_> = t.actions().iter().map(assign).collect();
        updates.extend(t.effects().iter().map(assign));
        updates.extend(keep);

        let g = NuXMVPredicate(&t.guard());
        let updates_s = updates.join(" & ");

        // tracking variable
        let ivar = NuXMVPath((t.node().path()));

        trans.push(format!("{i} & {g} & {u}", i=ivar, g=g, u=updates_s));
    }

    let trans_s = trans.join(" |\n");
    lines.push_str(&trans_s);

    lines.push_str("\n\n");

    // finally, print out the ltl spec on the form
    // LTLSPEC ! ( G s1 & G s2 & F g1 & F g2);
    // let global_str: Vec<String> = global.iter().map(|p| format!("G ( {} )", p)).collect();
    // let g = if global_str.is_empty() {
    //     "TRUE".to_string()
    // } else {
    //     global_str.join("&")
    // };

    let goal_str: Vec<String> = goals.iter().map(|p| format!("F ( {} )", &NuXMVPredicate(p))).collect();
    let goals = if goal_str.is_empty() {
        "TRUE".to_string()
    } else {
        goal_str.join("&")
    };

    // //lines.push_str(&format!("LTLSPEC ! ( {} & {} );", g, goals));
    // without safety specs
    lines.push_str(&format!("LTLSPEC ! ( {} );", goals));

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
    let mut process = Command::new("nuXmv")
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

    let mut stdin = process.stdin.take().unwrap();
    stdin.write_all(command.as_bytes())?;

    let result = process.wait_with_output()?;

    assert!(result.status.success());

    let raw = String::from_utf8(result.stdout).expect("Check character encoding");
    let raw_error = String::from_utf8(result.stderr).expect("Check character encoding");

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
        if l.contains("  -- Loop starts here") {
            // when searching for infinite paths...
        }
        else if l.contains("  -> State: ") {
            // ignore..
        }
        else if l.contains("  -> Input: ") || l.contains("nuXmv >") {
            trace.push(last);
            last = PlanningFrame::default();
        } else {
            let path_val: Vec<_> = l.split("=").map(|s| s.trim()).collect();
            let path = SPPath::from(path_val[0].split("#").map(|s|s.to_owned()).collect());
            let sppath = path.clone();

            let val = path_val.get(1).expect("no value!");

            // check for controllable actions
            let mut all_trans = model.ab_transitions.ctrl.iter().chain(
                model.ab_transitions.un_ctrl.iter());
            if all_trans.find(|t| (t.node().path()) == &path).is_some() {
            // if model.ab_transitions.ctrl.iter().find(|t| (t.node()) == &path).is_some() {
                if val == &"TRUE" {
                    assert!(last.transition == SPPath::default());
                    // last.ctrl = Some(sppath);
                    // last.ctrl.push(sppath.clone());
                    last.transition = sppath.clone();
                }
            } else {
                // get SP type from path
                let spt = if model.state_predicates.iter().
                    find(|p| (p.node().path()) == &path).is_some() {
                        SPValueType::Bool
                    } else {
                        // TODO: hacks abound
                        if !vars.contains_key(&sppath) {
                            SPValueType::Bool   // this is a spec
                        } else {
                            vars
                                .get(&sppath)
                                .expect(&format!("path mismatch! {}", path))
                                .value_type()
                        }
                    };

                let spval = spval_from_nuxvm(val, spt);

                // temp test
                if vars.contains_key(&sppath) {
                    let v = vars.get(&sppath).unwrap();
                    if v.variable_type() == VariableType::Measured {
                        last.state.force_from_path(&sppath, spval);
                    }
                }
            }
        }
    }

    Some(trace)
}

pub fn compute_plan(
    goals: &Vec<Predicate>,
    state: &SPState,
    model: &RunnerModel,
    max_steps: u32,
) -> PlanningResult {
    // create variable definitions based on the state
    // note, we need to exclude predicates...
    let items = model.model.items();
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();
    let vars: HashMap<SPPath, Variable> = resources.iter().flat_map(|r| r.get_variables()).map(|v| (v.path().clone(), v.clone())).collect();

    let lines = create_nuxmv_problem(&goals, &state, &model);

    let datetime: DateTime<Local> = SystemTime::now().into();
    // todo: use non platform way of getting temporary folder
    // or maybe just output to a subfolder 'plans'
    let filename = &format!("/tmp/planner {}.bmc", datetime);
    let mut f = File::create(filename).unwrap();
    write!(f, "{}", lines).unwrap();
    let mut f = File::create("/tmp/last_planning_request.bmc").unwrap();
    write!(f, "{}", lines).unwrap();

    let start = Instant::now();
    let result = call_nuxmv(max_steps, filename);
    let duration = start.elapsed();

    match result {
        Ok((raw, raw_error)) => {
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



fn create_offline_nuxmv_problem(model: &RunnerModel, initial: &Predicate) -> String {
    let items = model.model.items();
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();
    let vars: HashMap<SPPath, Variable> = resources.iter().flat_map(|r| r.get_variables()).map(|v| (v.path().clone(), v.clone())).collect();

    let specs: Vec<Spec> = model.model.items().iter().flat_map(|i| match i {
        SPItem::Spec(s) => Some(s),
        _ => None,
    }).cloned().collect();

    let mut lines = String::from("MODULE main");
    lines.push_str("\n");
    lines.push_str("VAR\n");

    lines.push_str("-- MODEL VARIABLES\n");
    for (path, variable) in &vars {
        let path = NuXMVPath(path);
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

    lines.push_str("-- CONTROL VARIABLES\n");
    lines.push_str("IVAR \n\n");

    // add a control variable for each transition
    let all_trans = model.ab_transitions.ctrl.iter().chain(model.ab_transitions.un_ctrl.iter());
    for t in all_trans {
        let path = NuXMVPath((t.node().path()));
        lines.push_str(&format!("{i}{v} : boolean;\n", i = indent(2), v = path));
    }
    lines.push_str("\n\n");

    // add DEFINES for specs and state predicates
    lines.push_str("DEFINE\n\n");

    lines.push_str("-- GLOBAL SPECIFICATIONS\n");
    let mut global = Vec::new();
    for s in &specs {
        let path = (s.node().path());

        for (i,p) in s.always().iter().enumerate() {
            let mut pp = path.clone();
            let pp = pp.add_child(&i.to_string());
            let path = NuXMVPath(&pp);
            let p = NuXMVPredicate(&p);
            lines.push_str(&format!(
                    "{i}{v} := {p};\n",
                    i = indent(2),
                    v = path,
                    p = p,
            ));
            global.push(p);
        }
    }

    lines.push_str("\n\n");
    lines.push_str("-- STATE PREDICATES\n");

    for sp in &model.state_predicates {
        let path = NuXMVPath(sp.node().path());
        match sp.variable_type() {
            VariableType::Predicate(p) => {
                let p = NuXMVPredicate(&p);
                lines.push_str(&format!(
                    "{i}{v} := {p};\n",
                    i = indent(2),
                    v = path,
                    p = p,
                ));
            },
            _ => {
                panic!("model error")
            }
        }
    }

    lines.push_str("\n\n");


    lines.push_str("INIT\n");
    lines.push_str("\n\n");
    // big ass initial state expression...
    // wow it works :)
    let ip = NuXMVPredicate(&initial);
    lines.push_str(&format!(
        "{i}{e};\n",
        i = indent(2),
        e = ip
    ));


    // lines.push_str("ASSIGN\n");
    // lines.push_str("\n\n");
    // lines.push_str("-- CURRENT STATE --\n");

    // for (path, _variable) in vars {
    //     let value = state.s.get(path).expect("all variables need a valuation!");
    //     let path = NuXMVPath(path);
    //     let value = NuXMVValue(value);
    //     lines.push_str(&format!(
    //         "{i}init({v}) := {spv};\n",
    //         i = indent(2),
    //         v = path,
    //         spv = value
    //     ));
    // }

    // lines.push_str("\n\n");
    // lines.push_str("-- CONTROL VARIABLE STATE --\n");
    // // add a control variable for each controllable transition
    // for ct in &model.ab_transitions.ctrl {
    //     let path = NuXMVPath((ct.node()));
    //     let false_ = false.to_spvalue();
    //     let value = NuXMVValue(&false_); // they're all false
    //     lines.push_str(&format!(
    //         "{i}init({v}) := {spv};\n",
    //         i = indent(2),
    //         v = path,
    //         spv = value
    //     ));
    // }

    lines.push_str("\n\n");
    lines.push_str("-- TRANSITIONS --\n");
    lines.push_str("");
    lines.push_str("TRANS\n\n");


    let mut all_vars = HashSet::new();
    all_vars.extend(vars.keys().cloned());

    let mut trans = Vec::new();

    let all_trans = model.ab_transitions.ctrl.iter().chain(model.ab_transitions.un_ctrl.iter());
    for t in all_trans {
        let modified = modified_by(t);
        let untouched = all_vars.difference(&modified);

        let keep: Vec<_> = untouched.map(|path| {
            let path = NuXMVPath(path);
            format!("( next({v}) = {v} )", v = path)
        }).collect();

        let assign = |a: &Action| {
                let path = NuXMVPath(&a.var);
                let value = action_to_string(&a).expect("model too complicated");
                format!("next({}) = {}", path, value)
            };
        let mut updates: Vec<_> = t.actions().iter().map(assign).collect();
        updates.extend(t.effects().iter().map(assign));
        updates.extend(keep);

        let g = NuXMVPredicate(&t.guard());
        let updates_s = updates.join(" & ");

        // tracking variable
        let ivar = NuXMVPath((t.node().path()));

        trans.push(format!("{i} & {g} & {u}", i=ivar, g=g, u=updates_s));
    }

    let trans_s = trans.join(" |\n");
    lines.push_str(&trans_s);

    lines.push_str("\n\n");

    // finally, print out the ltl spec on the form
    // LTLSPEC ! ( G s1 & G s2 & F g1 & F g2);
    // let global_str: Vec<String> = global.iter().map(|p| format!("G ( {} )", p)).collect();
    // let g = if global_str.is_empty() {
    //     "TRUE".to_string()
    // } else {
    //     global_str.join("&")
    // };

    // let goal_str: Vec<String> = goals.iter().map(|p| format!("F ( {} )", &NuXMVPredicate(p))).collect();
    // let goals = if goal_str.is_empty() {
    //     "TRUE".to_string()
    // } else {
    //     goal_str.join("&")
    // };

    // //lines.push_str(&format!("LTLSPEC ! ( {} & {} );", g, goals));
    // // without safety specs
    // lines.push_str(&format!("LTLSPEC ! ( {} );", goals));

    return lines;
}


fn modified_by(t: &Transition) -> HashSet<SPPath> {
    let mut r = HashSet::new();

    r.extend(t.actions().iter().map(|a| a.var.clone()));
    r.extend(t.effects().iter().map(|a| a.var.clone()));

    r
}


// make sure specs hold.
pub fn generate_offline_nuxvm(model: &RunnerModel, initial: &Predicate) {
    let lines = create_offline_nuxmv_problem(model, initial);

    let datetime: DateTime<Local> = SystemTime::now().into();
    // todo: use non platform way of getting temporary folder
    // or maybe just output to a subfolder 'plans'
    let filename = &format!("/tmp/model_out {}.bmc", datetime);
    let mut f = File::create(filename).unwrap();
    write!(f, "{}", lines).unwrap();
}
