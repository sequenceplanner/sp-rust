use crate::planning::*;
// use chrono::offset::Local;
// use chrono::DateTime;
use std::collections::HashSet;
use std::fmt;
use std::fs::File;
use std::io::{Read, Write};
use std::process::{Command, Stdio};
use std::time::Instant;
// use std::time::{SystemTime};
use std::time::Duration;

mod nuxmv_async;
use nuxmv_async::*;

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
            SPValue::Time(t) => write!(fmtr, "time: {:?}", t),
            SPValue::Path(d) => write!(fmtr, "{:?}", d),
            SPValue::Array(_at, a) => write!(fmtr, "{:?}", a),
            SPValue::Unknown => write!(fmtr, "SPUNKNOWN"),
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
                // assert!(!children.is_empty());
                format!("( {} )", children.join("&"))
            }
            Predicate::OR(x) => {
                let children: Vec<_> = x
                    .iter()
                    .map(|p| format!("{}", NuXMVPredicate(&p)))
                    .collect();
                // assert!(!children.is_empty());
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
            x => {
                panic!(format!("We can not convert this predicate to nuXMV: {}", x));
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

fn call_nuxmv(max_steps: u32, filename: &str) -> std::io::Result<(String, String)> {
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

    Ok((raw, raw_error))
}

fn postprocess_nuxmv_problem(
    model: &TransitionSystemModel, raw: &String,
) -> Option<Vec<PlanningFrame>> {
    if !raw.contains("Trace Type: Counterexample") {
        // we didn't find a counter-example, which means we already fulfil the goal.
        return None;
    }

    let lines = raw.lines();
    let s = lines
        .rev()
        .take_while(|l| !l.contains("Trace Type: Counterexample"));
    let mut s: Vec<String> = s.map(|s| s.to_owned()).collect();
    s.reverse();

    let mut trace = Vec::new();
    let mut last = PlanningFrame::default();

    for l in &s {
        if l.contains("  -- Loop starts here") {
            // when searching for infinite paths...
            // TODO: we should search back to the first ok state when getting this
            println!("NOTE: INFINITE PATH");
        } else if l.contains("  -> State: ") {
            // ignore the difference between state and input.
        } else if l.contains("  -> Input: ") || l.contains("nuXmv >") {
            trace.push(last);
            last = PlanningFrame::default();
        } else {
            let path_val: Vec<_> = l.split("=").map(|s| s.trim()).collect();
            let path = SPPath::from(path_val[0].split("#").map(|s| s.to_owned()).collect());
            let sppath = path.clone();
            let val = path_val.get(1).expect("no value!");

            if model
                .transitions
                .iter()
                .find(|t| (t.node().path()) == &path)
                .is_some()
            {
                if val == &"TRUE" {
                    assert!(last.transition == SPPath::default());
                    last.transition = sppath.clone();
                }
            } else {
                // get SP type from path
                let spt = if model
                    .state_predicates
                    .iter()
                    .find(|p| (p.node().path()) == &path)
                    .is_some()
                {
                    SPValueType::Bool
                } else {
                    if let Some(v) = model.vars.iter().find(|v| v.path() == &sppath) {
                        v.value_type()
                    } else {
                        SPValueType::Bool // this is a spec
                    }
                };

                let spval = spval_from_nuxvm(val, spt);
                last.state.add_variable(sppath, spval);
            }
        }
    }

    Some(trace)
}

pub struct NuXmvPlanner {}

pub fn plan_async(
    model: &TransitionSystemModel, goals: &[(Predicate, Option<Predicate>)], state: &SPState,
    max_steps: u32, cutoff: u32, lookout: f32, max_time: Duration,
) -> PlanningResult {
    let lines = create_nuxmv_problem(&model, &goals, &state);

    //let datetime: DateTime<Local> = SystemTime::now().into();
    // todo: use non platform way of getting temporary folder
    // or maybe just output to a subfolder 'plans'
    // let filename = &format!("/tmp/async_planner {}.bmc", datetime);
    // let mut f = File::create(filename).unwrap();
    // write!(f, "{}", lines).unwrap();
    let filename_last_plan = "./last_async_planning_request.bmc";
    let mut f = File::create(filename_last_plan).unwrap();
    write!(f, "{}", lines).unwrap();

    let start = Instant::now();
    let result =
        block_on_search_heuristic(filename_last_plan, cutoff, max_steps, lookout, max_time);
    let duration = start.elapsed();

    let res = match result {
        Ok((_, raw, raw_error)) => {
            if raw_error.len() > 0
                && !raw_error.contains("There are no traces currently available.")
            {
                // just to more easily find syntax errors
                panic!("{}", raw_error);
            }
            let plan = postprocess_nuxmv_problem(&model, &raw);
            let plan_found = plan.is_some();
            let trace = plan.unwrap_or_else(|| {
                vec![PlanningFrame {
                    transition: SPPath::new(),
                    state: state.clone(),
                }]
            });

            PlanningResult {
                plan_found,
                plan_length: trace.len() as u32 - 1, // hack :)
                trace,
                time_to_solve: duration,
                raw_output: raw.to_owned(),
                raw_error_output: raw_error.to_owned(),
            }
        }
        Err(e) => PlanningResult {
            plan_found: false,
            plan_length: 0,
            trace: Vec::new(),
            time_to_solve: duration,
            raw_output: "".into(),
            raw_error_output: e.to_string(),
        },
    };

    // if res.plan_found {
    //     // usually dont care to debug these
    //     let _ = std::fs::remove_file(filename);
    // }

    res
}

// this version is a bit more interesting... we can only store optimal
// plans, as otherwise we might end up in a situation where we have
// stored two plans of different length for doing the same thing. the redundant moves
// can then make us livelock (by moving away from the goal)
use std::sync::{Arc, Mutex};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AsyncPlanningStore {
    // key must be string to serde::to_string
    cache: HashMap<String, Option<PlanningResult>>, // none if being computed already
    hits: i64,
    lookups: i64,
}

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

fn calculate_hash<T: Hash>(t: &T) -> u64 {
    let mut s = DefaultHasher::new();
    t.hash(&mut s);
    s.finish()
}

fn filename_from_model(
    model: &TransitionSystemModel,
) -> Result<String, Box<dyn std::error::Error>> {
    let mod_ser = serde_json::to_string(model)?;
    let mod_ser_hash = calculate_hash(&mod_ser);
    Ok(format!("store-{}.sz", mod_ser_hash))
}

fn load_store(
    model: &TransitionSystemModel,
) -> Result<AsyncPlanningStore, Box<dyn std::error::Error>> {
    let path = filename_from_model(model)?;
    let file = File::open(path)?;
    let mut buffer = String::new();
    let mut reader = snap::read::FrameDecoder::new(file);
    reader.read_to_string(&mut buffer)?;
    let s = serde_json::from_str(&buffer)?;

    Ok(s)
}

fn save_store(
    model: &TransitionSystemModel, store: &AsyncPlanningStore,
) -> Result<(), Box<dyn std::error::Error>> {
    let path = filename_from_model(model)?;
    let file = File::create(path)?;
    let mut writer = snap::write::FrameEncoder::new(file);
    let s = serde_json::to_string(store)?;
    writer.write_all(s.as_bytes())?;

    Ok(())
}

impl AsyncPlanningStore {
    pub fn load(model: &TransitionSystemModel) -> Self {
        match load_store(model) {
            Ok(store) => {
                println!("Loaded cache store with {} plans.", store.cache.len());
                store
            }
            Err(err) => {
                println!("Could not load planning store: {}", err);
                AsyncPlanningStore::default()
            }
        }
    }

    pub fn save(&self, model: &TransitionSystemModel) {
        save_store(model, &self).expect("failed to save store")
    }
}

pub fn plan_async_with_cache(
    model: &TransitionSystemModel, goals: &[(Predicate, Option<Predicate>)], state: &SPState,
    disabled: &[SPPath], max_steps: u32, cutoff: u32, lookout: f32, max_time: Duration,
    store: Arc<Mutex<AsyncPlanningStore>>,
) -> PlanningResult {
    let now = std::time::Instant::now();
    // filter the state based on the ts model and serialize it to make it hashable
    let state_str = state
        .projection()
        .sorted()
        .state
        .iter()
        .filter(|(k, _)| model.vars.iter().any(|v| &v.path() == k))
        .map(|(k, v)| {
            let s = format!("{}{}", k, serde_json::to_string(v.value()).unwrap());
            s
        })
        .fold("".to_string(), |acum, s| format!("{}{}", acum, s));

    // serialize goals
    let goal_str = goals
        .iter()
        .map(|(g, i)| {
            let i = if let Some(i) = i {
                i.to_string()
            } else {
                "".to_string()
            };
            format!("{}+{}", g, i)
        })
        .collect::<Vec<_>>()
        .join("");
    // let key = PlannerRequestKey { goal: goal_str, state: state_str };
    let disabled_str = disabled
        .iter()
        .map(|p| p.to_string())
        .collect::<Vec<_>>()
        .join(",");
    let key = format!("{}=={}=={}", goal_str, state_str, disabled_str);

    {
        let mut store = store.lock().unwrap();
        store.lookups += 1;
        if let Some(Some(plan)) = store.cache.get(&key) {
            let plan = plan.clone();
            store.hits += 1;
            println!(
                "Used cached async plan! Current plan count {}, hit% {}, lookup time {} ms",
                store.cache.len(),
                ((100 * store.hits) / store.lookups),
                now.elapsed().as_millis()
            );
            return plan;
        }
    }

    // start computing the optimal plan in a thread.
    let t_key = key.clone();
    let t_model = model.clone();
    let t_goals = goals.to_vec();
    let t_state = state.clone();
    let t_store = store.clone();
    std::thread::spawn(move || {
        {
            let mut store = t_store.lock().unwrap();
            if let Some(None) = store.cache.get(&t_key) {
                // already being computed by someone else, bail
                return;
            } else {
                // we are computing this one!
                store.cache.insert(t_key.clone(), None);
            }
        }
        let result = plan(&t_model, &t_goals, &t_state, max_steps);
        {
            let mut store = t_store.lock().unwrap();
            store.cache.insert(t_key, Some(result.clone()));
            println!("Added new state/goal pair to async plan store. Current async plan count {}. time to solve: {}ms", store.cache.len(), result.time_to_solve.as_millis());
            store.save(&t_model);
        }
    });

    // return best guess for now!
    plan_async(model, goals, state, max_steps, cutoff, lookout, max_time)
}

impl Planner for NuXmvPlanner {
    fn plan(
        model: &TransitionSystemModel, goals: &[(Predicate, Option<Predicate>)], state: &SPState,
        max_steps: u32,
    ) -> PlanningResult {
        let lines = create_nuxmv_problem(&model, &goals, &state);

        //let datetime: DateTime<Local> = SystemTime::now().into();
        // todo: use non platform way of getting temporary folder
        // or maybe just output to a subfolder 'plans'
        //let filename = &format!("/tmp/planner {}.bmc", datetime);
        //let mut f = File::create(filename).unwrap();
        //write!(f, "{}", lines).unwrap();
        let filename_last_plan = "./last_planning_request.bmc";
        let mut f = File::create(filename_last_plan).unwrap();
        write!(f, "{}", lines).unwrap();

        let start = Instant::now();
        let result = call_nuxmv(max_steps, filename_last_plan);
        let duration = start.elapsed();

        let res = match result {
            Ok((raw, raw_error)) => {
                if raw_error.len() > 0
                    && !raw_error.contains("There are no traces currently available.")
                {
                    // just to more easily find syntax errors
                    panic!("{}", raw_error);
                }
                let plan = postprocess_nuxmv_problem(&model, &raw);
                let plan_found = plan.is_some();
                let trace = plan.unwrap_or_else(|| {
                    vec![PlanningFrame {
                        transition: SPPath::new(),
                        state: state.clone(),
                    }]
                });

                PlanningResult {
                    plan_found,
                    plan_length: trace.len() as u32 - 1, // hack :)
                    trace,
                    time_to_solve: duration,
                    raw_output: raw.to_owned(),
                    raw_error_output: raw_error.to_owned(),
                }
            }
            Err(e) => PlanningResult {
                plan_found: false,
                plan_length: 0,
                trace: Vec::new(),
                time_to_solve: duration,
                raw_output: "".into(),
                raw_error_output: e.to_string(),
            },
        };
        // if res.plan_found {
        //     // usually dont care to debug these
        //     let _ = std::fs::remove_file(filename);
        // }
        res
    }
}

fn create_offline_nuxmv_problem(model: &TransitionSystemModel, initial: &Predicate) -> String {
    let mut lines = make_base_problem(model);

    add_initial_states(&mut lines, initial);

    lines
}

fn create_nuxmv_problem(
    model: &TransitionSystemModel, goal_invs: &[(Predicate, Option<Predicate>)], state: &SPState,
) -> String {
    let mut lines = make_base_problem(model);

    add_current_valuations(&mut lines, &model.vars, state);

    add_goals(&mut lines, goal_invs);

    return lines;
}

fn create_nuxmv_problem_ctl(
    model: &TransitionSystemModel, initial: &Predicate, ops: &[(String, Predicate, Predicate)],
) -> String {
    let mut lines = make_base_problem(model);

    add_initial_states(&mut lines, initial);

    add_ctl_specs(&mut lines, ops);

    lines
}

fn make_base_problem(model: &TransitionSystemModel) -> String {
    let mut lines = String::new();

    add_preamble(&mut lines, &model.name);

    add_vars(&mut lines, &model.vars);

    add_ivars(&mut lines, &model.transitions);

    add_statepreds(&mut lines, &model.state_predicates);

    let mut var_set: HashSet<SPPath> = HashSet::new();
    var_set.extend(model.vars.iter().map(|v| v.path()).cloned());

    add_transitions(&mut lines, &var_set, &model.transitions);

    // for now, don't add this. taken care of by runner + guard extraction.
    // perhaps later we should have different kinds of specifications in the
    // model instead.
    // for now we actually do this instead of GE.
    add_global_specifications(&mut lines, &model.specs);

    return lines;
}

pub fn generate_offline_nuxvm_ctl(
    model: &TransitionSystemModel, initial: &Predicate, ops: &[(String, Predicate, Predicate)],
) {
    let lines = create_nuxmv_problem_ctl(&model, initial, ops);

    //let datetime: DateTime<Local> = SystemTime::now().into();
    // todo: use non platform way of getting temporary folder
    // or maybe just output to a subfolder 'plans'
    // let filename = &format!("/tmp/model_out_{} {}.bmc", model.name, datetime);
    // let mut f = File::create(filename).unwrap();
    // write!(f, "{}", lines).unwrap();

    let filename = &format!("./last_model_out_{}.bmc", model.name);
    let mut f = File::create(filename).unwrap();
    write!(f, "{}", lines).unwrap();
}

pub fn generate_offline_nuxvm(model: &TransitionSystemModel, initial: &Predicate) {
    let lines = create_offline_nuxmv_problem(&model, initial);

    //let datetime: DateTime<Local> = SystemTime::now().into();
    // todo: use non platform way of getting temporary folder
    // or maybe just output to a subfolder 'plans'
    // let filename = &format!("/tmp/model_out_{} {}.bmc", model.name, datetime);
    // let mut f = File::create(filename).unwrap();
    // write!(f, "{}", lines).unwrap();

    let filename = &format!("./last_model_out_{}.bmc", model.name);
    let mut f = File::create(filename).unwrap();
    write!(f, "{}", lines).unwrap();
}

fn add_preamble(lines: &mut String, module_name: &str) {
    lines.push_str(&format!("-- MODULE: {}\n", module_name));
    lines.push_str("MODULE main\n\n");
}

fn add_vars(lines: &mut String, vars: &[Variable]) {
    lines.push_str("VAR\n\n");
    for v in vars {
        let path = NuXMVPath(v.path());
        if v.value_type() == SPValueType::Bool {
            lines.push_str(&format!("{i}{v} : boolean;\n", i = indent(2), v = path));
        } else {
            let domain: Vec<_> = v
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
    lines.push_str("\n\n");
}

fn add_ivars(lines: &mut String, transitions: &[Transition]) {
    lines.push_str("IVAR\n\n");

    // add a control variable for each transition
    for t in transitions {
        let path = NuXMVPath(t.node().path());
        lines.push_str(&format!("{i}{v} : boolean;\n", i = indent(2), v = path));
    }
    lines.push_str("\n\n");
}

fn add_statepreds(lines: &mut String, predicates: &[Variable]) {
    // add DEFINES for specs and state predicates
    lines.push_str("DEFINE\n\n");

    for sp in predicates {
        let path = NuXMVPath(sp.node().path());
        match sp.variable_type() {
            VariableType::Predicate(p) => {
                let p = NuXMVPredicate(&p);
                lines.push_str(&format!("{i}{v} := {p};\n", i = indent(2), v = path, p = p,));
            }
            _ => panic!("model error"),
        }
    }

    lines.push_str("\n\n");
}

fn modified_by(t: &Transition) -> HashSet<SPPath> {
    let mut r = HashSet::new();

    r.extend(t.actions().iter().map(|a| a.var.clone()));
    r
}

fn add_transitions(lines: &mut String, all_vars: &HashSet<SPPath>, transitions: &[Transition]) {
    lines.push_str("TRANS\n\n");

    let mut trans = Vec::new();

    for t in transitions {
        let modified = modified_by(t);
        let untouched = all_vars.difference(&modified);

        let keep: Vec<_> = untouched
            .map(|path| {
                let path = NuXMVPath(path);
                format!("( next({v}) = {v} )", v = path)
            })
            .collect();

        let assign = |a: &Action| {
            let path = NuXMVPath(&a.var);
            let value = action_to_string(&a).expect(&format!("model too complicated {:?}", a));
            format!("next({}) = {}", path, value)
        };
        let upd: Vec<_> = t
            .actions()
            .iter()
            .flat_map(|a| match a.value {
                Compute::Any => None,
                _ => Some(a.clone()),
            })
            .collect();
        let mut updates: Vec<_> = upd.iter().map(assign).collect();
        updates.extend(keep);

        let g = NuXMVPredicate(&t.guard());
        let updates_s = updates.join(" & ");

        // tracking variable
        let ivar = NuXMVPath(t.node().path());

        trans.push(format!("{i} & {g} & {u}", i = ivar, g = g, u = updates_s));
    }

    let trans_s = trans.join(" |\n\n");
    lines.push_str(&trans_s);

    lines.push_str("\n\n");
}

fn add_initial_states(lines: &mut String, initial: &Predicate) {
    lines.push_str("INIT\n\n");
    let ip = NuXMVPredicate(&initial);
    lines.push_str(&format!("{i}{e}\n;\n", i = indent(2), e = ip));
    lines.push_str("\n\n");
}

fn add_global_specifications(lines: &mut String, specs: &[Spec]) {
    // now put in the specs as invariants to refine the model.

    // if we do guard extraction, the specs leading to these do not
    // need to be put back as they are redundant.

    lines.push_str("INVAR\n\n");
    let mut global = Vec::new();
    for s in specs {
        global.push(format!(
            "-- spec: {}\n{}\n",
            s.path(),
            NuXMVPredicate(s.invariant())
        ));
    }
    let invars = if global.is_empty() {
        "TRUE".to_string()
    } else {
        global.join("&\n")
    };
    let invars = format!("{}\n;\n", invars);

    lines.push_str(&invars);
    lines.push_str("\n\n");
}

fn add_current_valuations(lines: &mut String, vars: &[Variable], state: &SPState) {
    lines.push_str("ASSIGN\n\n");

    for v in vars {
        let path = v.path();
        //let value = state.sp_value_from_path(path).expect("all variables need a valuation!");
        if let Some(value) = state.sp_value_from_path(path) {
            let path = NuXMVPath(path);
            let value = NuXMVValue(value);
            lines.push_str(&format!(
                "{i}init({v}) := {spv};\n",
                i = indent(2),
                v = path,
                spv = value
            ));
        }
    }

    lines.push_str("\n\n");
}

fn add_goals(lines: &mut String, goal_invs: &[(Predicate, Option<Predicate>)]) {
    let goal_str: Vec<String> = goal_invs
        .iter()
        .map(|(goal, inv)| {
            if let Some(inv) = inv {
                // invariant until goal
                format!(
                    "({inv} U {goal})",
                    goal = &NuXMVPredicate(goal),
                    inv = &NuXMVPredicate(inv)
                )
            } else {
                // no invariant, simple "exists" goal.
                format!("F ( {} )", &NuXMVPredicate(goal))
            }
        })
        .collect();
    let goals = if goal_str.is_empty() {
        // TODO: check this before doing everything else....
        "TRUE".to_string()
    } else {
        goal_str.join("&")
    };

    // TODO: clean this up!
    lines.push_str(&format!("LTLSPEC ! ( {} );", goals));
}

fn add_ctl_specs(lines: &mut String, operations: &[(String, Predicate, Predicate)]) {
    let mut checked = Vec::new();
    for (op_name, pre, goal) in operations {
        match checked.iter().find(|(_, p, g)| p == pre && g == goal) {
            Some((name, _, _)) => lines.push_str(&format!(
                "-- {} goal already checked by {}\n\n",
                op_name, name
            )),
            None => {
                lines.push_str(&format!("-- {}\n", op_name));
                lines.push_str(&format!(
                    "CTLSPEC AG ( {} -> EF ( {} ));\n\n",
                    NuXMVPredicate(pre),
                    NuXMVPredicate(goal)
                ));
                checked.push((op_name.clone(), pre.clone(), goal.clone()));
            }
        }
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use serial_test::serial;

//     #[test]
//     #[serial]
//     fn test_post_process() {
//         use indoc::indoc;
//         let result = indoc!("
//     <!-- ################### Trace number: 1 ################### -->
// Trace Description: BMC Counterexample
// Trace Type: Counterexample
//   -> State: 1.1 <-
//     one_robot_model#r1#State#0#active = FALSE
//     one_robot_model#r1#Control#0#activate = FALSE
//     one_robot_model#r1#Control#0#ref_pos = unknown
//     one_robot_model#r1#State#0#act_pos = unknown
//     one_robot_model#r1#deactivate#enabled = FALSE
//     one_robot_model#r1#deactivate#executing = FALSE
//     one_robot_model#r1#deactivate#finished = TRUE
//     one_robot_model#r1#activate#executing = FALSE
//     one_robot_model#r1#activate#finished = FALSE
//     one_robot_model#r1#activate#enabled = TRUE
//     one_robot_model#r1#to_away#enabled = FALSE
//     one_robot_model#r1#to_away#finished = FALSE
//     one_robot_model#r1#to_away#executing = FALSE
//     one_robot_model#r1#to_table#executing = FALSE
//     one_robot_model#r1#to_table#enabled = FALSE
//     one_robot_model#r1#to_table#finished = FALSE
//   -> Input: 1.2 <-
//     one_robot_model#r1#to_table#start = FALSE
//     one_robot_model#r1#to_away#start = FALSE
//     one_robot_model#r1#activate#start = TRUE
//     one_robot_model#r1#deactivate#start = FALSE
//     one_robot_model#r1#to_table#finish = FALSE
//     one_robot_model#r1#to_away#finish = FALSE
//     one_robot_model#r1#activate#finish = FALSE
//     one_robot_model#r1#deactivate#finish = FALSE
//   -> State: 1.2 <-
//     one_robot_model#r1#State#0#active = FALSE
//     one_robot_model#r1#Control#0#activate = TRUE
//     one_robot_model#r1#Control#0#ref_pos = unknown
//     one_robot_model#r1#State#0#act_pos = unknown
//     one_robot_model#r1#deactivate#enabled = FALSE
//     one_robot_model#r1#deactivate#executing = FALSE
//     one_robot_model#r1#deactivate#finished = FALSE
//     one_robot_model#r1#activate#executing = TRUE
//     one_robot_model#r1#activate#finished = FALSE
//     one_robot_model#r1#activate#enabled = FALSE
//     one_robot_model#r1#to_away#enabled = FALSE
//     one_robot_model#r1#to_away#finished = FALSE
//     one_robot_model#r1#to_away#executing = FALSE
//     one_robot_model#r1#to_table#executing = FALSE
//     one_robot_model#r1#to_table#enabled = FALSE
//     one_robot_model#r1#to_table#finished = FALSE
//   -> Input: 1.3 <-
//     one_robot_model#r1#to_table#start = FALSE
//     one_robot_model#r1#to_away#start = FALSE
//     one_robot_model#r1#activate#start = FALSE
//     one_robot_model#r1#deactivate#start = FALSE
//     one_robot_model#r1#to_table#finish = FALSE
//     one_robot_model#r1#to_away#finish = FALSE
//     one_robot_model#r1#activate#finish = TRUE
//     one_robot_model#r1#deactivate#finish = FALSE
//   -> State: 1.3 <-
//     one_robot_model#r1#State#0#active = TRUE
//     one_robot_model#r1#Control#0#activate = TRUE
//     one_robot_model#r1#Control#0#ref_pos = unknown
//     one_robot_model#r1#State#0#act_pos = unknown
//     one_robot_model#r1#deactivate#enabled = TRUE
//     one_robot_model#r1#deactivate#executing = FALSE
//     one_robot_model#r1#deactivate#finished = FALSE
//     one_robot_model#r1#activate#executing = FALSE
//     one_robot_model#r1#activate#finished = TRUE
//     one_robot_model#r1#activate#enabled = FALSE
//     one_robot_model#r1#to_away#enabled = TRUE
//     one_robot_model#r1#to_away#finished = FALSE
//     one_robot_model#r1#to_away#executing = FALSE
//     one_robot_model#r1#to_table#executing = FALSE
//     one_robot_model#r1#to_table#enabled = TRUE
//     one_robot_model#r1#to_table#finished = FALSE
// nuXmv >
// ");

//         let (model, state) = crate::testing::one_dummy_robot();
//         let ts_model = TransitionSystemModel::from(&model);
//         let trace = postprocess_nuxmv_problem(&ts_model, &result.to_string());
//         let trace = trace.unwrap();
//         assert_eq!(trace[0].transition,
//                    SPPath::new());
//         assert_eq!(trace[1].transition,
//                    SPPath::from_string("one_robot_model/r1/activate/start"));
//         assert_eq!(trace[2].transition,
//                    SPPath::from_string("one_robot_model/r1/activate/finish"));

//         let activate = SPPath::from_string("one_robot_model/r1/Control/0/activate");
//         let active = SPPath::from_string("one_robot_model/r1/State/0/active");

//         assert_eq!(trace[0].state.sp_value_from_path(&activate).unwrap(),
//                    &false.to_spvalue());

//         assert_eq!(trace[1].state.sp_value_from_path(&activate).unwrap(),
//                    &true.to_spvalue());

//         assert_eq!(trace[0].state.sp_value_from_path(&active).unwrap(),
//                    &false.to_spvalue());

//         assert_eq!(trace[1].state.sp_value_from_path(&active).unwrap(),
//                    &false.to_spvalue());

//         assert_eq!(trace[2].state.sp_value_from_path(&active).unwrap(),
//                    &true.to_spvalue());

//     }
// }
