//! The transitions used in SP

use super::*;



#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct Transition {
    pub spid: SPID,
    pub guard: Predicate,
    pub action: Vec<Action>,
    pub effects: Vec<Action>,
    // E, TODO: maybe, for alternative effects add probabilty
}

impl Transition {
    pub fn replace_variable_path(&mut self, map: &HashMap<SPPath, SPPath>) {
        self.guard.replace_variable_path(map);
        self.action.iter_mut().for_each(|a|{
            a.replace_variable_path(map)
        });
        self.effects.iter_mut().for_each(|a|{
            a.replace_variable_path(map)
        });
    }
}

impl EvaluatePredicate for Transition {
    fn eval(&self, state: &State) -> bool {
        self.guard.eval(state) && self.action.iter().all(|a| a.eval(state))
    }
}

impl NextAction for Transition {
    fn next(&self, state: &State) -> Result<HashMap<SPPath, AssignStateValue>> {
        let mut res: HashMap<SPPath, AssignStateValue> = HashMap::new();
        for a in self.action.iter() {
            let next = a.next(state)?;
            res.extend(next);
        }
        Ok(res)
    }
}

/// A helper macro when creating transitions
/// # Example
/// ```
/// use sp_domain::*;
/// 
/// let ab = SPPath::from_str(&["a", "b"]);
/// let kl = SPPath::from_str(&["k", "l"]);
/// let t = transition!("hej1", Predicate::TRUE);
/// let t = transition!("hej2", p!(ab), a!(ab), a!(!kl));
/// let t = transition!("hej3", p!(ab), a!(ab) ; a!(!kl));
/// let t = transition!("hej4", p!(ab), a!(ab), a!(ab); a!(!kl), a!(!kl));
/// ```
/// 
#[macro_export]
macro_rules! transition { 
    ($name:expr, $guard: expr) => {
        Transition{
            spid: SPID::new($name),
            guard: $guard.clone(),
            action: vec!(),
            effects: vec!(),
        }
    };
    ($name:expr, $guard: expr, $($action: expr),*) => {
        Transition{
            spid: SPID::new($name),
            guard: $guard.clone(),
            action: vec!($($action.clone()),*),
            effects: vec!(),
        }
    };
    ($name:expr, $guard: expr, $($action: expr),* ; $($effect: expr),* )=> {
        Transition{
            spid: SPID::new($name),
            guard: $guard.clone(),
            action: vec!($($action.clone()),*),
            effects: vec!($($effect.clone()),*),
        }
    };
}


/// ********** TESTS ***************

#[cfg(test)]
mod runner_tests {
    use super::*;
    #[test]
    fn testing_transitions() {
        let ab = SPPath::from_str(&["a", "b"]);
        let ac = SPPath::from_str(&["a", "c"]);
        let kl = SPPath::from_str(&["k", "l"]);
        let xy = SPPath::from_str(&["x", "y"]);

        let mut s = state!(ab => 2, ac => true, kl => true, xy => false);
        let p = pr!{{p!(ac)} && {p!(kl)}};

        let a = a!(ac = false);
        let b = a!(ab <- kl);
        let c = a!(xy ? p);

        let t1 = Transition {
            spid: SPID::new("t1"),
            guard: Predicate::TRUE,
            action: vec!(a, b, c),
            effects: vec!(),
        };

        let res = t1.eval(&s);
        println!("{:?}", res);

        let res = t1.next(&s).unwrap();
        println!("{:?}", res);

        s.insert_map(res).unwrap();
        println!("{:?}", s);

        let res = t1.eval(&s);
        println!("{:?}", res);

        let res = t1.next(&s).unwrap();
        println!("{:?}", res);

        let no = s.insert_map(res);
        println!("{:?}", no);

    }

    #[test]
    fn transition_macros() {
        let ab = SPPath::from_str(&["a", "b"]);
        let ac = SPPath::from_str(&["a", "c"]);
        let kl = SPPath::from_str(&["k", "l"]);
        let xy = SPPath::from_str(&["x", "y"]);

        let t = transition!("t", Predicate::TRUE);
        let res = Transition {
            spid: t.spid.clone(),
            guard: Predicate::TRUE,
            action: vec!(),
            effects: vec!(),
        };
        assert_eq!(t, res);

        let t = transition!("t", p!(ab), a!(ab), a!(!kl));
        let res = Transition {
            spid: t.spid.clone(),
            guard: p!(ab),
            action: vec!(a!(ab), a!(!kl)),
            effects: vec!(),
        };
        assert_eq!(t, res);

        let t = transition!("t", p!(ab), a!(ab) ; a!(!kl));
        let res = Transition {
            spid: t.spid.clone(),
            guard: p!(ab),
            action: vec!(a!(ab)),
            effects: vec!(a!(!kl)),
        };
        assert_eq!(t, res);

        let t = transition!("t", p!(ab), a!(ab), a!(ab); a!(!kl), a!(!kl));
        let res = Transition {
            spid: t.spid.clone(),
            guard: p!(ab),
            action: vec!(a!(ab), a!(ab)),
            effects: vec!(a!(!kl), a!(!kl)),
        };
        assert_eq!(t, res);
    }
}