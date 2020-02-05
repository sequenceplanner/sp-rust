use super::*;
/// In this file both predicates and actions are defined
use std::collections::HashMap;
use serde::{Deserialize, Serialize};

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
    // GT(PredicateValue, PredicateValue),
    // LT(PredicateValue, PredicateValue),
    // INDOMAIN(PredicateValue, Vec<PredicateValue>),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Action {
    pub var: SPPath,
    pub value: Compute,
    state_path: Option<StatePath>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
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
                    if let Some(the_path) = state.state_path(&path) {
                        return state.sp_value(&the_path);
                    }
                } else {
                    if let Some(the_path) = sp {
                        return state.sp_value(&the_path);
                    }
                }
                return None;
            }
        }
    }

    pub fn upd_state_path(&mut self, state: &SPState) {
        match self {
            PredicateValue::SPPath(path, sp) => {
                if sp.is_none() {
                    *sp = state.state_path(path)
                } else if sp.clone().map(|x| x.state_id != state.id()).unwrap_or(false) {
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
            PredicateValue::SPValue(_) => {} ,
            PredicateValue::SPPath(op, _) => {
                if let Some(np) = mapping.get(op) {
                    *op = np.clone();
                }
            }
        }
    }
}

impl fmt::Display for Predicate {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s: String = match &self {
            Predicate::AND(x) => {
                let children: Vec<_> = x
                    .iter()
                    .map(|p| format!("{}", p))
                    .collect();
                format!("( {} )", children.join("&&"))
            }
            Predicate::OR(x) => {
                let children: Vec<_> = x
                    .iter()
                    .map(|p| format!("{}", p))
                    .collect();
                format!("( {} )", children.join("||"))
            }
            Predicate::XOR(_) => "TODO".into(), // remove from pred?
            Predicate::NOT(p) => format!("!({})", p),
            Predicate::TRUE => "TRUE".into(),
            Predicate::FALSE => "FALSE".into(),
            Predicate::EQ(x, y) => {
                let xx = match x {
                    PredicateValue::SPValue(v) => format!("{}", v),
                    PredicateValue::SPPath(p, _) => format!("{}", p),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", v),
                    PredicateValue::SPPath(p, _) => format!("{}", p),
                };

                format!("{} = {}", xx, yy)
            }
            Predicate::NEQ(x, y) => {
                let xx = match x {
                    PredicateValue::SPValue(v) => format!("{}", v),
                    PredicateValue::SPPath(p, _) => format!("{}", p),
                };
                let yy = match y {
                    PredicateValue::SPValue(v) => format!("{}", v),
                    PredicateValue::SPPath(p, _) => format!("{}", p),
                };

                format!("{} != {}", xx, yy)
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
    pub fn upd_state_path(&mut self, state: &SPState) {
        match self {
                Predicate::AND(x) => { x.iter_mut().for_each(|p| p.upd_state_path(state)) },
                Predicate::OR(x) => { x.iter_mut().for_each(|p| p.upd_state_path(state)) },
                Predicate::XOR(x) => { x.iter_mut().for_each(|p| p.upd_state_path(state)) },
                Predicate::NOT(x) => { x.upd_state_path(state) },
                Predicate::TRUE => {},
                Predicate::FALSE => {},
                Predicate::EQ(x, y) => {
                    x.upd_state_path(state);
                    y.upd_state_path(state);
                },
                Predicate::NEQ(x, y) => {
                    x.upd_state_path(state);
                    y.upd_state_path(state);
                },
            }
    }


    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        match self {
            Predicate::AND(v) => { v.iter_mut().for_each(|e| e.replace_variable_path(mapping)); },
            Predicate::OR(v) => { v.iter_mut().for_each(|e| e.replace_variable_path(mapping)); },
            Predicate::XOR(v) => { v.iter_mut().for_each(|e| e.replace_variable_path(mapping)); },
            Predicate::NOT(b) => { b.replace_variable_path(mapping);},
            Predicate::TRUE => {},
            Predicate::FALSE => {},
            Predicate::EQ(pv1, pv2) => {
                pv1.replace_variable_path(mapping);
                pv2.replace_variable_path(mapping);
            },
            Predicate::NEQ(pv1, pv2) => {
                pv1.replace_variable_path(mapping);
                pv2.replace_variable_path(mapping);
            },
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
            Some(sp) if sp.state_id != state.id() =>
                self.state_path = state.state_path(&self.var),
            None =>
                self.state_path = state.state_path(&self.var),
            _ => {}
        }
    }


    pub fn replace_variable_path(&mut self, mapping: &HashMap<SPPath, SPPath>) {
        if let Some(np) = mapping.get(&self.var) {
            self.var = np.clone();
        }
        match &mut self.value {
            Compute::PredicateValue(pv) => { pv.replace_variable_path(mapping); }
            Compute::Predicate(p) => { p.replace_variable_path(mapping); }
        }
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
            Predicate::EQ(lp, rp) => lp.sp_value(state) == rp.sp_value(state),
            Predicate::NEQ(lp, rp) => lp.sp_value(state) != rp.sp_value(state),
            // Predicate::GT(lp, rp) => {}
            // Predicate::LT(lp, rp) => {}
            // Predicate::INDOMAIN(value, domain) => {}
        }
    }
}

impl NextAction for Action {
    fn next(&self, state: &mut SPState) -> SPResult<()> {
        let c = match &self.value {
            Compute::PredicateValue(pv) => {
                match pv
                    .sp_value(state)
                    .map(|x| x.clone())
                {
                    Some(x) => x,
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
                }
            },
            Compute::Predicate(p) => {
                let res = p.eval(state);
                res.to_spvalue()
            }
        };

        match &self.state_path {
            Some(sp) => {
                state.next(&sp, c)
            },
            None => {
                state.next_from_path(&self.var, c)
            }
        }
    }

}

impl EvaluatePredicate for Action {
    fn eval(&self, state: &SPState) -> bool {
        let sp = match &self.state_path {
            Some(x) => {
                state.state_value(x)
            },
            None => state.state_value_from_path(&self.var)
        };
        match sp
        {
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

    // AND: the brackets are needed because "tt" includes && which
    // leads to ambiguity without an additional delimeter
    ([$($first:tt)+] && $([$($rest:tt)+] $(&&)?)+) => {{
        // println!("matched &&: {}", stringify!($($first)+));
        let first = p! ( $($first)+ );
        let mut v = vec![first];
        $(
            let r = p!($($rest)+);
            v.push(r);
        )*
        Predicate::AND(v)
    }};

    // OR: same as and.
    ([$($first:tt)+] || $([$($rest:tt)+] $(||)?)+) => {{
        // println!("matched &&: {}", stringify!($($first)+));
        let first = p! ( $($first)+ );
        let mut v = vec![first];
        $(
            let r = p!($($rest)+);
            v.push(r);
        )*
        Predicate::OR(v)
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

}

#[macro_export]
macro_rules! a {
    (p:$path:ident = $val:expr) => {{
        Action::new(
            $path.clone(),
            Compute::PredicateValue(
                PredicateValue::SPValue($val.to_spvalue()),
            ),
        )
    }};
    ($path:tt = $val:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(
                PredicateValue::SPValue($val.to_spvalue()),
            ),
        )
    }};
    (p:$path:ident <- $other:tt) => {{
        let other = SPPath::from_string(&stringify!($other).replace("\"", ""));
        Action::new(
            $path.clone(),
            Compute::PredicateValue(
                PredicateValue::SPPath(other, None)),
        )
    }};
    ($path:tt <- $other:tt) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        let other = SPPath::from_string(&stringify!($other).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(
                PredicateValue::SPPath(other, None)),
        )
    }};
    (p:$path:ident <- p:$other:expr) => {{
        Action::new(
            $path.clone(),
            Compute::PredicateValue(
                PredicateValue::SPPath($other.clone(), None)),
        )
    }};
    ($path:tt <- p:$other:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(
                PredicateValue::SPPath($other.clone(), None)),
        )
    }};
    ($path:tt ? $val:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::Predicate($val.clone()),
        )
    }};
    (p:$path:ident ? $val:expr) => {{
        Action::new(
            $path.clone(),
            Compute::Predicate($val.clone()),
        )
    }};
    ($path:tt ? $val:expr) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::Predicate($val.clone()),
        )
    }};
    (!p:$path:ident) => {{
        Action::new(
            $path.clone(),
            Compute::PredicateValue(
                PredicateValue::SPValue(false.to_spvalue()),
            ),
        )
    }};
    (!$path:tt) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(
                PredicateValue::SPValue(false.to_spvalue()),
            ),
        )
    }};
    (p:$path:ident) => {
        Action::new(
            $path.clone(),
            Compute::PredicateValue(
                PredicateValue::SPValue(true.to_spvalue()),
            ),
        )
    };
    ($path:tt) => {{
        let p = SPPath::from_string(&stringify!($path).replace("\"", ""));
        Action::new(
            p,
            Compute::PredicateValue(
                PredicateValue::SPValue(true.to_spvalue()),
            ),
        )
    }};
}

/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    #![warn(unused_must_use)]
    #![warn(unused_variables)]

    use super::*;

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
    fn next_pred() {
        let ab = SPPath::from_slice(&["a", "b"]);
        let ac = SPPath::from_slice(&["a", "c"]);
        let kl = SPPath::from_slice(&["k", "l"]);
        let xy = SPPath::from_slice(&["x", "y"]);
        let mut s = state!(ab => 2, ac => true, kl => true, xy => false);
        let p = p!([p:ac] && [p:kl]);

        let mut a = Action::new(
            ac.clone(),
            Compute::PredicateValue(PredicateValue::default()),
        );
        let mut a2 = Action::new(
            ab.clone(),
            Compute::PredicateValue(PredicateValue::SPPath(kl, None)),
        );
        let mut a3 = Action::new(xy.clone(), Compute::Predicate(p));

        a3.next(&mut s);
        // let next = StateValue::Next(states::Next {
        //     current_value: false.to_spvalue(),
        //     next_value: true.to_spvalue(),
        // });
        println!{"next pred: {:?}", a3};
        //assert_eq!(s.state_value_from_path(&xy), Some(&next));

        a.next(&mut s);
        // let next = StateValue::Next(states::Next {
        //     current_value: true.to_spvalue(),
        //     next_value: false.to_spvalue(),
        // });
        //assert_eq!(s.state_value_from_path(&ac), Some(&next));

        a2.next(&mut s);
        // let next = StateValue::Next(states::Next {
        //     current_value: 2.to_spvalue(),
        //     next_value: true.to_spvalue(),
        // });
        // assert_eq!(s.state_value_from_path(&ab), Some(&next));

        s.take_transition();

        a3.next(&mut s);
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

        a.next(&mut s);

        assert!(a2.eval(&s));
        assert!(!a.eval(&s));
    }


}
