use super::*;
use serde::{Deserialize, Serialize};
/// In this file both predicates and actions are defined
use std::collections::HashMap;

mod parser;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Predicate {
    AND(Vec<Predicate>),
    OR(Vec<Predicate>),
    XOR(Vec<Predicate>),
    NOT(Box<Predicate>),
    TRUE,
    FALSE,
    EQ(PredicateValue, PredicateValue),
    NEQ(PredicateValue, PredicateValue),
    TON(PredicateValue, PredicateValue),
    TOFF(PredicateValue, PredicateValue),
    MEMBER(PredicateValue, PredicateValue), // GT(PredicateValue, PredicateValue),
                                            // LT(PredicateValue, PredicateValue),
                                            // INDOMAIN(PredicateValue, Vec<PredicateValue>)
}

#[derive(Debug, Serialize, Deserialize, Clone, Default)]
pub struct Action {
    pub var: SPPath,
    pub value: Compute,
    state_path: Option<StatePath>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum PredicateValue {
    SPValue(SPValue),
    SPPath(SPPath, Option<StatePath>),
}

/// Used in actions to compute a new SPValue.
/// When using delay and fetching a value from another variable, the current value of that
/// variable will be taken and assigned to the action variable after the delay, and not the
/// value that the other has after the delay.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Compute {
    PredicateValue(PredicateValue),
    Predicate(Predicate), // used for boolean actions
    Function(Vec<(Predicate, PredicateValue)>),
    TimeStamp,
    Any, // Free variable, can take on any value after this action.
         // If we need more advanced functions we can add them here
         //TakeNext(SPValue, Vec<SPValue>), // to be impl when needed
         //TakeBefore(SPValue, Vec<SPValue>),
         // Add(Box<Compute>, Box<Compute>),
         // Sub(Box<Compute>, Box<Compute>),
         // Join(Box<Compute>, Box<Compute>),
}

impl<'a> PredicateValue {
    pub fn sp_value(&'a self, state: &'a SPState) -> Option<&'a SPValue> {
        match self {
            PredicateValue::SPValue(x) => Some(x),
            PredicateValue::SPPath(path, sp) => {
                if sp.is_none() {
                    state.sp_value_from_path(&path)
                } else if let Some(the_path) = sp {
                    state.sp_value(&the_path)
                } else {
                    None
                }
            }
        }
    }

    pub fn upd_state_path(&mut self, state: &SPState) {
        match self {
            PredicateValue::SPPath(path, sp) => {
                if sp.is_none() {
                    *sp = state.state_path(path)
                } else if sp
                    .clone()
                    .map(|x| x.state_id != state.id())
                    .unwrap_or(false)
                {
                    *sp = state.state_path(path);
                }
            }
            _ => {}
        }
    }

    pub fn value(v: SPValue) -> Self {
        PredicateValue::SPValue(v)
    }
    pub fn path(p: SPPath) -> Self {
        PredicateValue::SPPath(p, None)
    }

    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match self {
            PredicateValue::SPValue(_) => {}
            PredicateValue::SPPath(op, _) => {
                if let Some(np) = mapping.get(op) {
                    *op = np.clone();
                }
            }
        }
    }
}

impl PartialEq for Action {
    fn eq(&self, other: &Self) -> bool {
        self.var == other.var && self.value == other.value
    }
}

impl PartialEq for PredicateValue {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (PredicateValue::SPValue(a), PredicateValue::SPValue(b)) => a == b,
            (PredicateValue::SPPath(a, _), PredicateValue::SPPath(b, _)) => a == b,
            _ => false,
        }
    }
}

impl fmt::Display for PredicateValue {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PredicateValue::SPValue(v) => write!(fmtr, "{}", v),
            PredicateValue::SPPath(p, _) => write!(fmtr, "{}", p),
        }
    }
}

impl fmt::Display for Predicate {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s: String = match &self {
            Predicate::AND(x) => {
                let children: Vec<_> = x.iter().map(|p| format!("{}", p)).collect();
                format!("({})", children.join(" && "))
            }
            Predicate::OR(x) => {
                let children: Vec<_> = x.iter().map(|p| format!("{}", p)).collect();
                format!("({})", children.join(" || "))
            }
            Predicate::XOR(_) => "TODO".into(), // remove from pred?
            Predicate::NOT(p) => format!("!({})", p),
            Predicate::TRUE => "TRUE".into(),
            Predicate::FALSE => "FALSE".into(),
            Predicate::EQ(x, y) => format!("{} = {}", x, y),
            Predicate::NEQ(x, y) => format!("{} != {}", x, y),
            Predicate::TON(t, d) => {
                format! {"TON(t:{} d:{})", t, d}
            }
            Predicate::TOFF(t, d) => {
                format! {"TOFF(t:{} d:{})", t, d}
            }
            Predicate::MEMBER(t, d) => {
                format! {"is {} a member of {}?", t, d}
            }
        };

        write!(fmtr, "{}", &s)
    }
}

impl Default for PredicateValue {
    fn default() -> Self {
        PredicateValue::SPValue(false.to_spvalue())
    }
}

impl Predicate {
    pub fn from_string(from: &str) -> Option<Self> {
        parser::pred_parser::pred(from).ok()
    }

    pub fn upd_state_path(&mut self, state: &SPState) {
        match self {
            Predicate::AND(x) | Predicate::OR(x) | Predicate::XOR(x) => {
                x.iter_mut().for_each(|p| p.upd_state_path(state))
            }
            Predicate::NOT(x) => x.upd_state_path(state),
            Predicate::TRUE | Predicate::FALSE => {}
            Predicate::EQ(x, y)
            | Predicate::NEQ(x, y)
            | Predicate::TON(x, y)
            | Predicate::TOFF(x, y)
            | Predicate::MEMBER(x, y) => {
                x.upd_state_path(state);
                y.upd_state_path(state);
            }
        }
    }

    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match self {
            Predicate::AND(v) | Predicate::OR(v) | Predicate::XOR(v) => {
                v.iter_mut().for_each(|e| e.replace_variable_path(mapping));
            }
            Predicate::NOT(b) => {
                b.replace_variable_path(mapping);
            }
            Predicate::TRUE | Predicate::FALSE => {}
            Predicate::EQ(pv1, pv2)
            | Predicate::NEQ(pv1, pv2)
            | Predicate::TON(pv1, pv2)
            | Predicate::TOFF(pv1, pv2)
            | Predicate::MEMBER(pv1, pv2) => {
                pv1.replace_variable_path(mapping);
                pv2.replace_variable_path(mapping);
            }
        }
    }

    /// Return the supporting variables of this expression
    pub fn support(&self) -> Vec<SPPath> {
        let mut s = Vec::new();
        match &self {
            Predicate::AND(x) | Predicate::OR(x) | Predicate::XOR(x) => {
                s.extend(x.iter().flat_map(|p| p.support()))
            }
            Predicate::NOT(x) => s.extend(x.support()),
            Predicate::TRUE | Predicate::FALSE => {}
            Predicate::EQ(x, y)
            | Predicate::NEQ(x, y)
            | Predicate::TON(x, y)
            | Predicate::TOFF(x, y)
            | Predicate::MEMBER(x, y) => {
                if let PredicateValue::SPPath(p, _) = x {
                    s.push(p.clone())
                }
                if let PredicateValue::SPPath(p, _) = y {
                    s.push(p.clone())
                }
            }
        };
        s.sort();
        s.dedup();
        s
    }

    /// Recursively clean expression and keep only constants and allowed paths
    pub fn keep_only(&self, only: &[SPPath]) -> Option<Predicate> {
        match &self {
            Predicate::AND(x) => {
                let mut new: Vec<_> = x.iter().flat_map(|p| p.keep_only(only)).collect();
                new.dedup();
                if new.len() == 0 {
                    None
                } else if new.len() == 1 {
                    Some(new[0].clone())
                } else {
                    Some(Predicate::AND(new))
                }
            }
            Predicate::OR(x) => {
                let mut new: Vec<_> = x.iter().flat_map(|p| p.keep_only(only)).collect();
                new.dedup();
                if new.len() == 0 {
                    None
                } else if new.len() == 1 {
                    Some(new[0].clone())
                } else {
                    Some(Predicate::OR(new))
                }
            }
            Predicate::XOR(x) => {
                let mut new: Vec<_> = x.iter().flat_map(|p| p.keep_only(only)).collect();
                new.dedup();
                if new.len() == 0 {
                    None
                } else if new.len() == 1 {
                    Some(new[0].clone())
                } else {
                    Some(Predicate::XOR(new))
                }
            }
            Predicate::NOT(x) => match x.keep_only(only) {
                Some(x) => Some(Predicate::NOT(Box::new(x))),
                None => None,
            },
            Predicate::TRUE => Some(Predicate::TRUE),
            Predicate::FALSE => Some(Predicate::FALSE),
            Predicate::EQ(x, y)
            | Predicate::NEQ(x, y)
            | Predicate::TON(x, y)
            | Predicate::TOFF(x, y)
            | Predicate::MEMBER(x, y) => {
                let remove_x = match x {
                    PredicateValue::SPValue(_) => false,
                    PredicateValue::SPPath(p, _) => !only.contains(p),
                };
                let remove_y = match y {
                    PredicateValue::SPValue(_) => false,
                    PredicateValue::SPPath(p, _) => !only.contains(p),
                };

                if remove_x || remove_y {
                    None
                } else {
                    Some(self.clone())
                }
            }
        }
    }
}

impl Action {
    pub fn new(var: SPPath, value: Compute) -> Self {
        Action {
            var,
            value,
            state_path: None,
        }
    }

    pub fn upd_state_path(&mut self, state: &SPState) {
        match &self.state_path {
            Some(sp) if sp.state_id != state.id() => self.state_path = state.state_path(&self.var),
            None => self.state_path = state.state_path(&self.var),
            _ => {}
        }
    }

    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        if let Some(np) = mapping.get(&self.var) {
            self.var = np.clone();
        }
        match &mut self.value {
            Compute::PredicateValue(pv) => {
                pv.replace_variable_path(mapping);
            }
            Compute::Predicate(p) => {
                p.replace_variable_path(mapping);
            }
            Compute::Function(xs) => {
                xs.iter_mut().for_each(|(p, v)| {
                    p.replace_variable_path(mapping);
                    v.replace_variable_path(mapping);
                });
            }
            Compute::TimeStamp | Compute::Any => {}
        }
    }

    pub fn revert_action(&self, state: &mut SPState) -> SPResult<()> {
        match &self.state_path {
            Some(sp) => state.revert_next(&sp),
            None => state.revert_next_from_path(&self.var),
        }
    }

    pub fn to_predicate(&self) -> Option<Predicate> {
        match &self.value {
            Compute::PredicateValue(p) => Some(Predicate::EQ(
                PredicateValue::SPPath(self.var.clone(), None),
                p.clone(),
            )),
            _ => None,
        }
    }

    pub fn to_concrete_predicate(&self, state: &SPState) -> Option<Predicate> {
        match &self.value {
            Compute::PredicateValue(PredicateValue::SPPath(p, _)) => {
                let pv = state
                    .state_value_from_path(p)
                    .expect("no such value in the state")
                    .current_value();
                Some(Predicate::EQ(
                    PredicateValue::SPPath(self.var.clone(), None),
                    PredicateValue::SPValue(pv.clone()),
                ))
            }
            Compute::PredicateValue(p) => Some(Predicate::EQ(
                PredicateValue::SPPath(self.var.clone(), None),
                p.clone(),
            )),
            _ => None,
        }
    }

    pub fn val_to_string(&self) -> String {
        match &self.value {
            Compute::PredicateValue(PredicateValue::SPValue(v)) => v.to_string(),
            Compute::PredicateValue(PredicateValue::SPPath(p, _)) => p.to_string(),
            Compute::Predicate(p) => p.to_string(),
            Compute::Function(xs) => xs.iter().fold(String::default(), |mut acc, (p, v)| {
                format!("{}[if {} then {}]", acc, p, v)
            }),
            Compute::Any => "?".to_string(),
            Compute::TimeStamp => "T".to_string(),
        }
    }

    pub fn to_string_short(&self) -> String {
        format!("{} := {}", self.var.leaf(), self.val_to_string())
    }
}

impl fmt::Display for Action {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = format!("{} := {}", self.var, self.val_to_string());
        write!(fmtr, "{}", &s)
    }
}

impl Default for Predicate {
    fn default() -> Self {
        Predicate::TRUE
    }
}

impl Default for Compute {
    fn default() -> Self {
        Compute::PredicateValue(PredicateValue::default())
    }
}

/// Eval is used to evaluate a predicate (or an operation ).
pub trait EvaluatePredicate {
    fn eval(&self, state: &SPState) -> bool;
}

pub trait NextAction {
    fn next(&self, state: &mut SPState) -> SPResult<()>;
}

impl EvaluatePredicate for Predicate {
    fn eval(&self, state: &SPState) -> bool {
        match self {
            Predicate::AND(ps) => ps.iter().all(|p| p.eval(state)),
            Predicate::OR(ps) => ps.iter().any(|p| p.eval(state)),
            Predicate::XOR(ps) => {
                let mut c = 0;
                for p in ps.iter() {
                    if p.eval(state) {
                        c += 1;
                    }
                }
                c == 1
                // ps.iter_mut()
                //     .filter(|p| p.eval(state))  // for some reason does not filter with &mut
                //     .count()
                //     == 1
            }
            Predicate::NOT(p) => !p.eval(state),
            Predicate::TRUE => true,
            Predicate::FALSE => false,
            Predicate::EQ(lp, rp) => {
                let a = lp.sp_value(state);
                let b = rp.sp_value(state);
                if let (Some(a), Some(b)) = (a, b) {
                    a == b
                } else {
                    eprintln!(
                        "ERROR: eval in predicate EQ: path {} or {} not found in\n{}",
                        lp, rp, state
                    );
                    false
                }
            }
            Predicate::NEQ(lp, rp) => {
                let a = lp.sp_value(state);
                let b = rp.sp_value(state);
                if let (Some(a), Some(b)) = (a, b) {
                    a != b
                } else {
                    eprintln!(
                        "ERROR: eval in predicate NEQ: path {} or {} not found in\n{}",
                        lp, rp, state
                    );
                    false
                }
            }
            Predicate::TON(lp, rp) => {
                if let (Some(t), Some(d)) = (lp.sp_value(state), rp.sp_value(state)) {
                    if let SPValue::Time(time) = t {
                        let current_duration = time.elapsed().unwrap_or_default();
                        let delay = match d {
                            SPValue::Float32(x) => *x as i32,
                            SPValue::Int32(x) => *x,
                            _ => 0,
                        };
                        current_duration.as_millis() > delay.abs() as u128
                    } else {
                        eprintln!("TON must point to a timestamp, and not: {:?} i", t);
                        false
                    }
                } else {
                    eprintln!(
                        "ERROR: eval in predicate TON: path {} or {} not found in\n{}",
                        lp, rp, state
                    );
                    false
                }
            }
            Predicate::TOFF(lp, rp) => {
                if let (Some(t), Some(d)) = (lp.sp_value(state), rp.sp_value(state)) {
                    if let SPValue::Time(time) = t {
                        let current_duration = time.elapsed().unwrap_or_default();
                        let delay = match d {
                            SPValue::Float32(x) => *x as i32,
                            SPValue::Int32(x) => *x,
                            _ => 0,
                        };
                        current_duration.as_millis() < delay.abs() as u128
                    } else {
                        eprintln!("TON must point to a timestamp, and not: {:?} i", t);
                        false
                    }
                } else {
                    eprintln!(
                        "ERROR: eval in predicate TON: path {} or {} not found in\n{}",
                        lp, rp, state
                    );
                    false
                }
            }
            Predicate::MEMBER(lp, rp) => {
                if let (Some(v), Some(xs)) = (lp.sp_value(state), rp.sp_value(state)) {
                    if let SPValue::Array(_, xs) = xs {
                        xs.contains(v)
                    } else {
                        eprintln!("Member must point to an array, and not: {:?} i", xs);
                        false
                    }
                } else {
                    eprintln!(
                        "ERROR: eval in predicate MEMBER: path {} or {} not found in\n{}",
                        lp, rp, state
                    );
                    false
                }
            } // Predicate::GT(lp, rp) => {}
              // Predicate::LT(lp, rp) => {}
              // Predicate::INDOMAIN(value, domain) => {}
        }
    }
}

impl NextAction for Action {
    fn next(&self, state: &mut SPState) -> SPResult<()> {
        let c = match &self.value {
            Compute::PredicateValue(pv) => match pv.sp_value(state).cloned() {
                Some(x) => Some(x),
                None => {
                    eprintln!(
                        "The action PredicateValue, next did not find a value for variable: {:?}",
                        pv
                    );
                    return Err(SPError::No(format!(
                        "The action PredicateValue, next did not find a value for variable: {:?}",
                        pv
                    )));
                }
            },
            Compute::Predicate(p) => {
                let res = p.eval(state);
                Some(res.to_spvalue())
            }
            Compute::Function(xs) => {
                let res = xs
                    .iter()
                    .find(|(p, _)| p.eval(state))
                    .map(|(_, v)| v.sp_value(state))
                    .flatten();
                match res {
                    Some(x) => Some(x.clone()),
                    None => {
                        eprintln!("No predicates in the action Function was true: {:?}", self);
                        return Err(SPError::No(format!(
                            "No predicates in the action Function was true: {:?}",
                            self
                        )));
                    }
                }
            }
            Compute::TimeStamp => Some(SPValue::Time(std::time::SystemTime::now())),
            Compute::Any => None,
        };

        if let Some(c) = c {
            match &self.state_path {
                Some(sp) => state.next(&sp, c),
                None => state.next_from_path(&self.var, c),
            }
        } else {
            Ok(())
        }
    }
}

impl EvaluatePredicate for Action {
    fn eval(&self, state: &SPState) -> bool {
        let sp = match &self.state_path {
            Some(x) => state.state_value(x),
            None => state.state_value_from_path(&self.var),
        };
        match sp {
            Some(x) => !x.has_next(), // MD: I assume we meant to fail if we *already* had a next value for this action
            None => false, // We do not allow actions to add new state variables. But maybe this should change?
        }
    }
}

#[macro_export]
macro_rules! p {
    // parens
    (($($inner:tt)+) ) => {{
        // println!("matched parens: {}", stringify!($($inner)+));
        p! ( $($inner)+ )
    }};
    ([$($inner:tt)+] ) => {{
        // println!("matched square parens: {}", stringify!($($inner)+));
        p! ( $($inner)+ )
    }};

    // AND: the brackets are needed because "tt" includes && which
    // leads to ambiguity without an additional delimeter
    ([$($first:tt)+] $(&& [$($rest:tt)+])+) => {{
        // println!("matched &&: {}", stringify!($($first)+));
        let first = p! ( $($first)+ );
        let mut v = vec![first];
        $(
            // println!(" && ...: {}", stringify!($($rest)+));
            let r = p!($($rest)+);
            v.push(r);
        )*
        Predicate::AND(v)
    }};

    // OR: same as and.
    ([$($first:tt)+] $(|| [$($rest:tt)+])+) => {{
        // println!("matched ||: {}", stringify!($($first)+));
        let first = p! ( $($first)+ );
        let mut v = vec![first];
        $(
            let r = p!($($rest)+);
            v.push(r);
        )*
        Predicate::OR(v)
    }};

    // implication
    ([$($x:tt)+] => [$($y:tt)+]) => {{
        // println!("matched implication: {} => {}", stringify!($($x)+), stringify!($($y)+));
        let x = p! ( $($x)+ );
        let y = p! ( $($y)+ );
        Predicate::OR(vec![Predicate::NOT(Box::new(x)), y])
    }};

    // equals is very limited. can only match on the form path == spvalue
    (p:$path:ident == $value:tt) => {{
        // println!("matched p:{} == {}", stringify!($path), stringify!($value));
        Predicate::EQ(
            PredicateValue::SPPath($path.clone(), None),
            PredicateValue::SPValue($value.to_spvalue()),
        )
    }};

    (p:$path:ident != $value:tt) => {{
        // println!("matched !=");
        Predicate::NEQ(
            PredicateValue::SPPath($path.clone(), None),
            PredicateValue::SPValue($value.to_spvalue()),
        )
    }};

    ($path:tt == $value:tt) => {{
        // println!("matched {} == {}", stringify!($path), stringify!($value));
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Predicate::EQ(
            PredicateValue::SPPath(p, None),
            PredicateValue::SPValue($value.to_spvalue()),
        )
    }};

    // introduce <-> for equality between variables....
    ($path:tt <-> $other:tt) => {{
        // println!("matched {} == {}", stringify!($path), stringify!($value));
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        let other = SPPath::from_string(&stringify!($other).replace("\"", ""));
        Predicate::EQ(
            PredicateValue::SPPath(p, None),
            PredicateValue::SPPath(other, None),
        )
    }};
    (p:$path:tt <-> p:$other:tt) => {{
        Predicate::EQ(
            PredicateValue::SPPath($path.clone(), None),
            PredicateValue::SPPath($other.clone(), None),
        )
    }};

    // introduce <!> for inequality between variables....
    ($path:tt <!> $other:tt) => {{
        // println!("matched {} == {}", stringify!($path), stringify!($value));
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        let other = SPPath::from_string(&stringify!($other).replace("\"", ""));
        Predicate::NEQ(
            PredicateValue::SPPath(p, None),
            PredicateValue::SPPath(other, None),
        )
    }};

    // introduce <!> for inequality between variables....
    (p:$path:ident <!> p:$other:ident) => {{
        Predicate::NEQ(
            PredicateValue::SPPath($path.clone(), None),
            PredicateValue::SPPath($other.clone(), None),
        )
    }};


    ($path:tt != $value:tt) => {{
        // println!("matched !=");
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Predicate::NEQ(
            PredicateValue::SPPath(p, None),
            PredicateValue::SPValue($value.to_spvalue()),
        )
    }};

    // negation
    (! $($inner:tt)+ ) => {{
        // println!("matched negation: {}", stringify!($($inner)+));
        let inner = p! ( $($inner)+ );
        Predicate::NOT(Box::new( inner ))
    }};

    // if we already have a predicate we need to prefix it with pp:
    (pp:$i:expr) => {{
        // println!("matched base: {}", stringify!($p));
        $i.clone()
    }};

    // if we already have a path reference we need to prefix it with p:
    (p:$i:ident) => {{
        // println!("matched base: {}", stringify!($p));
        Predicate::EQ(
            PredicateValue::SPPath($i.clone(), None),
            PredicateValue::SPValue(true.to_spvalue()),
        )
    }};

    // reduced to a single token, assume its a path to a boolean variable
    ($p:tt) => {{
        // println!("matched base: {}", stringify!($p));
        let p = SPPath::from_string(&stringify!($p).replace("\"", ""));
        Predicate::EQ(
            PredicateValue::SPPath(p, None),
            PredicateValue::SPValue(true.to_spvalue()),
        )
    }};

    // TON predicate where t is the path to the timestamp and d the delay in ms
    (TON(t:$timer:ident d:$delay:ident)) => {{
        Predicate::TON(
            PredicateValue::SPPath($timer.clone(), None),
            PredicateValue::SPPath($delay.clone(), None),
        )
    }};
    // TOFF predicate where t is the path to the timestamp and d the delay in ms
    (TOFF(t:$timer:ident d:$delay:ident)) => {{
        Predicate::TOFF(
            PredicateValue::SPPath($timer.clone(), None),
            PredicateValue::SPPath($delay.clone(), None),
        )
    }};

}

#[macro_export]
macro_rules! a {
    (p:$path:ident = $val:expr) => {{
        Action::new(
            $path.clone(),
            Compute::PredicateValue(PredicateValue::SPValue($val.to_spvalue())),
        )
    }};
    ($path:tt = $val:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(PredicateValue::SPValue($val.to_spvalue())),
        )
    }};
    (p:$path:ident <- $other:tt) => {{
        let other = SPPath::from_string(&stringify!($other).replace("\"", ""));
        Action::new(
            $path.clone(),
            Compute::PredicateValue(PredicateValue::SPPath(other, None)),
        )
    }};
    ($path:tt <- $other:tt) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        let other = SPPath::from_string(&stringify!($other).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(PredicateValue::SPPath(other, None)),
        )
    }};
    (p:$path:ident <- p:$other:expr) => {{
        Action::new(
            $path.clone(),
            Compute::PredicateValue(PredicateValue::SPPath($other.clone(), None)),
        )
    }};
    ($path:tt <- p:$other:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(PredicateValue::SPPath($other.clone(), None)),
        )
    }};
    ($path:tt ? $val:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(p, Compute::Predicate($val.clone()))
    }};
    (p:$path:ident ? $val:expr) => {{
        Action::new($path.clone(), Compute::Predicate($val.clone()))
    }};
    ($path:tt ? $val:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(p, Compute::Predicate($val.clone()))
    }};
    (!p:$path:ident) => {{
        Action::new(
            $path.clone(),
            Compute::PredicateValue(PredicateValue::SPValue(false.to_spvalue())),
        )
    }};
    (!$path:tt) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(PredicateValue::SPValue(false.to_spvalue())),
        )
    }};

    // syntax for free variables
    (p:$path:ident ?) => {{
        Action::new($path.clone(), Compute::Any)
    }};
    ($path:tt ?) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(p, Compute::Any)
    }};

    (p:$path:ident) => {
        Action::new(
            $path.clone(),
            Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())),
        )
    };
    ($path:tt) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(PredicateValue::SPValue(true.to_spvalue())),
        )
    }};
}

/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    #![warn(unused_variables)]

    use super::*;

    // we want to make sure that the macro and string parser
    // have the same semantics. They probably differ a bit a
    // the moment, especially with deciding what is a path and
    // what is a value.
    #[test]
    fn macro_vs_parser() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);

        let p1 = format!("p:{} && p:{} && p:{}", ab, ac, kl);
        assert_eq!(
            Predicate::from_string(&p1).unwrap(),
            p!([p: ab] && [p: ac] && [p: kl])
        );

        let p1 = format!("(!p:{}) && p:{} && p:{}", ab, ac, kl);
        assert_eq!(
            Predicate::from_string(&p1).unwrap(),
            p!([!p: ab] && [p: ac] && [p: kl])
        );

        let p1 = format!("(!p:{}) && p:{} || p:{}", ab, ac, kl);
        assert_eq!(
            Predicate::from_string(&p1).unwrap(),
            p!([[!p: ab] && [p: ac]] || [p: kl])
        );

        let p1 = format!("(!p:{}) && p:{} -> p:{}", ab, ac, kl);
        assert_eq!(
            Predicate::from_string(&p1).unwrap(),
            p!( [[ !p:ab] && [p:ac]] => [p:kl ])
        );

        // samve expr as above but with whitespaces interspersed
        let p1 = format!(" ( ( ! p: {} ) && p: {} ) -> ( p:{} ) ", ab, ac, kl);
        assert_eq!(
            Predicate::from_string(&p1).unwrap(),
            p!( [[ !p:ab] && [p:ac]] => [p:kl ])
        );
    }

    #[test]
    fn macro_pred() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);

        p!([p: ab] && [p: ac] && [p: kl]);
        p!([!p: ab] && [p: ac] && [p: kl]);
        p!([[!p: ab] && [p: ac]] || [p: kl]);
        p!( [[ !p:ab] && [p:ac]] => [p:kl ]);
        p!( [[ !p:ab] && [p:ac]] => [p:kl ]);

        let p = p!(TON(t:ab d:ac));
        println!("{}", p);
    }

    #[test]
    fn eval_pred() {
        let s = state!(["a", "b"] => 2, ["a", "c"] => true, ["k", "l"] => true);
        let v = SPPath::from_slice(&["a", "b"]);
        let mut eq = Predicate::EQ(
            PredicateValue::SPValue(2.to_spvalue()),
            PredicateValue::SPPath(v.clone(), None),
        );
        let mut eq2 = Predicate::EQ(
            PredicateValue::SPValue(3.to_spvalue()),
            PredicateValue::SPPath(v.clone(), None),
        );
        assert!(eq.eval(&s));
        assert!(!eq2.eval(&s));
    }

    #[test]
    fn support_pred() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);

        let mut eq = Predicate::EQ(
            PredicateValue::SPValue(2.to_spvalue()),
            PredicateValue::SPPath(ac.clone(), None),
        );
        let mut eq2 = Predicate::NEQ(
            PredicateValue::SPValue(3.to_spvalue()),
            PredicateValue::SPPath(kl.clone(), None),
        );
        let mut eq3 = Predicate::EQ(
            PredicateValue::SPValue(3.to_spvalue()),
            PredicateValue::SPPath(ab.clone(), None),
        );
        let x = Predicate::AND(vec![eq, eq2]);
        let x = Predicate::OR(vec![x, eq3]);

        println!("{}", x);

        assert_eq!(x.support(), vec![ab.clone(), ac.clone(), kl.clone()]);
    }

    #[test]
    fn next_pred() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let xy = SPPath::from_slice(&["x", "y"]);
        let mut s = state!(ab => 2, ac => true, kl => true, xy => false);
        let p = p!([p: ac] && [p: kl]);

        let mut a = Action::new(
            ac.clone(),
            Compute::PredicateValue(PredicateValue::default()),
        );
        let mut a2 = Action::new(
            ab.clone(),
            Compute::PredicateValue(PredicateValue::SPPath(kl, None)),
        );
        let mut a3 = Action::new(xy.clone(), Compute::Predicate(p));

        a3.next(&mut s).unwrap();
        // let next = StateValue::Next(states::Next {
        //     current_value: false.to_spvalue(),
        //     next_value: true.to_spvalue(),
        // });
        println! {"next pred: {:?}", a3};
        //assert_eq!(s.state_value_from_path(&xy), Some(&next));

        a.next(&mut s).unwrap();
        // let next = StateValue::Next(states::Next {
        //     current_value: true.to_spvalue(),
        //     next_value: false.to_spvalue(),
        // });
        //assert_eq!(s.state_value_from_path(&ac), Some(&next));

        a2.next(&mut s).unwrap();
        // let next = StateValue::Next(states::Next {
        //     current_value: 2.to_spvalue(),
        //     next_value: true.to_spvalue(),
        // });
        // assert_eq!(s.state_value_from_path(&ab), Some(&next));

        s.take_transition();

        a3.next(&mut s).unwrap();
        // let next = StateValue::Next(states::Next {
        //     current_value: true.to_spvalue(),
        //     next_value: false.to_spvalue(),
        // });
        // assert_eq!(s.state_value_from_path(&xy), Some(&next));

        println!("res from action: {:?} is: {:?}", &a2, &s);
    }

    #[test]
    fn action_eval() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let mut s = state!(ab => 2, ac => true, kl => true);

        let mut a = Action {
            var: ac.clone(),
            value: Compute::PredicateValue(PredicateValue::default()),
            state_path: None,
        };
        let mut a2 = Action {
            var: ab.clone(),
            value: Compute::PredicateValue(PredicateValue::SPPath(kl, None)),
            state_path: None,
        };

        a.next(&mut s).unwrap();

        assert!(a2.eval(&s));
        assert!(!a.eval(&s));
    }

    #[test]
    fn ton_toff_predicate_testing() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let now = std::time::SystemTime::now();

        let mut s = state!(ab => 2, ac => true, kl => now);

        let p1 = Predicate::TON(
            PredicateValue::path(kl.clone()),
            PredicateValue::SPValue(10.to_spvalue()),
        );
        let p2 = Predicate::TOFF(
            PredicateValue::path(kl),
            PredicateValue::SPValue(10.to_spvalue()),
        );

        println!("TON before: {}", p1.eval(&s));
        assert!(!p1.eval(&s));
        println!("TOFF before: {}", p2.eval(&s));
        assert!(p2.eval(&s));

        std::thread::sleep(std::time::Duration::from_millis(20));

        println!("TON after: {}", p1.eval(&s));
        assert!(p1.eval(&s));
        println!("TOFF after: {}", p2.eval(&s));
        assert!(!p2.eval(&s));
    }

    #[test]
    fn member_predicate_testing() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let m = SPPath::from_slice(&["m"]);
        let xs = SPValue::Array(
            SPValueType::Int32,
            vec![
                1.to_spvalue(),
                2.to_spvalue(),
                3.to_spvalue(),
                4.to_spvalue(),
            ],
        );
        let xs2 = SPValue::Array(
            SPValueType::Int32,
            vec![
                1.to_spvalue(),
                2.to_spvalue(),
                3.to_spvalue(),
                4.to_spvalue(),
            ],
        );
        let xs3 = SPValue::Array(SPValueType::Int32, vec![ab.to_spvalue(), ac.to_spvalue()]);

        let mut s = state!(ab => 2, ac => 20);
        s.add_variable(kl.clone(), xs);
        s.add_variable(m.clone(), xs3.clone());

        let p1 = Predicate::MEMBER(
            PredicateValue::path(ab.clone()),
            PredicateValue::path(kl.clone()),
        );
        let p2 = Predicate::MEMBER(
            PredicateValue::path(ac.clone()),
            PredicateValue::path(kl.clone()),
        );
        let p3 = Predicate::MEMBER(
            PredicateValue::value(3.to_spvalue()),
            PredicateValue::path(kl.clone()),
        );
        let p4 = Predicate::MEMBER(
            PredicateValue::value(3.to_spvalue()),
            PredicateValue::value(xs2.clone()),
        );
        let p5 = Predicate::MEMBER(
            PredicateValue::value(30.to_spvalue()),
            PredicateValue::value(xs2),
        );
        let p6 = Predicate::MEMBER(
            PredicateValue::value(ab.to_spvalue()),
            PredicateValue::path(m.clone()),
        );
        let p7 = Predicate::MEMBER(
            PredicateValue::value(kl.to_spvalue()),
            PredicateValue::value(xs3),
        );

        println!("MEMBER predicate: {}", p1);
        assert!(p1.eval(&s));
        println!("MEMBER predicate: {}", p2);
        assert!(!p2.eval(&s));
        println!("MEMBER predicate: {}", p3);
        assert!(p3.eval(&s));
        println!("MEMBER predicate: {}", p4);
        assert!(p4.eval(&s));
        println!("MEMBER predicate: {}", p5);
        assert!(!p5.eval(&s));
        println!("MEMBER predicate: {}", p6);
        assert!(p6.eval(&s));
        println!("MEMBER predicate: {}", p7);
        assert!(!p7.eval(&s));
    }

    #[test]
    fn timestamp_testing() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let now = std::time::SystemTime::now();

        let mut s = state!(ab => 2, ac => true, kl => now);

        let a = Action::new(kl.clone(), Compute::TimeStamp);

        let p1 = Predicate::TON(
            PredicateValue::path(kl.clone()),
            PredicateValue::SPValue(10.to_spvalue()),
        );

        println!("TON before: {}", p1.eval(&s));
        assert!(!p1.eval(&s));

        std::thread::sleep(std::time::Duration::from_millis(20));

        println!("TON after: {}", p1.eval(&s));
        assert!(p1.eval(&s));

        a.next(&mut s).unwrap();
        println!("TON after action: {}", p1.eval(&s));
        assert!(!p1.eval(&s));
    }

    #[test]
    fn function_action() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);

        let mut s = state!(ab => true, ac => false, kl => false);
        let p1 = p!(p: ab);
        let p2 = p!(p: ac);

        let a = Action::new(
            kl.clone(),
            Compute::Function(vec![
                (p1.clone(), PredicateValue::value("hej".to_spvalue())),
                (p2.clone(), PredicateValue::value("då".to_spvalue())),
            ]),
        );
        let a2 = Action::new(
            ab.clone(),
            Compute::PredicateValue(PredicateValue::value(false.to_spvalue())),
        );
        let a3 = Action::new(
            ac.clone(),
            Compute::PredicateValue(PredicateValue::value(true.to_spvalue())),
        );

        println! {"{}", &s};
        println!();
        a.next(&mut s).unwrap();
        s.take_transition();
        println! {"{}", s};
        println!();
        assert!(s.sp_value_from_path(&kl.clone()).unwrap() == &"hej".to_spvalue());

        a2.next(&mut s).unwrap();
        s.take_transition();
        println! {"{}", s};
        println!();

        let e = a.next(&mut s);
        assert!(e.is_err());

        a3.next(&mut s).unwrap();
        a.next(&mut s).unwrap();
        s.take_transition();
        println! {"{}", s};
        println!();
        assert!(s.sp_value_from_path(&kl.clone()).unwrap() == &"då".to_spvalue());
    }
}
