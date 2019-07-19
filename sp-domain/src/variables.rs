//! Variables in SP


use serde::{Deserialize, Serialize};
use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum Variable {
    Measured(VariableData),
    Estimated(VariableData), 
    Command(VariableData), 
    StatePredicate(VariableData, Action),  // Maybe have these here?
}


/// Var is the attributes in all types of variables, but should not be used by itself. Use Variable
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct VariableData {
    pub spid: SPID,
    pub initial_value: SPValue, 
    pub domain: Vec<SPValue>,
}


/// The possible variable types used by operations to define parameters
/// Must be the same as Variable
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Copy)]
pub enum VariableType {
    Measured,
    Estimated,
    Command,
    StatePredicate,
}

impl Variable {
    pub fn is_type(&self, t: VariableType) -> bool {
        match self {
            Variable::Measured(_) => { VariableType::Measured == t },
            Variable::Estimated(_) => { VariableType::Estimated == t },
            Variable::Command(_) => { VariableType::Command == t },
            Variable::StatePredicate(_, _) => { VariableType::StatePredicate == t },
        }
    }
    pub fn has_type(&self) -> VariableType {
        match self {
            Variable::Measured(_) => { VariableType::Measured },
            Variable::Estimated(_) => { VariableType::Estimated },
            Variable::Command(_) => { VariableType::Command },
            Variable::StatePredicate(_, _) => { VariableType::StatePredicate },
        }
    }
    pub fn spid(&self) -> SPID {
        self.variable_data().spid.clone()
    }
    pub fn initial_value(&self) -> SPValue {
        self.variable_data().initial_value.clone()
    }
    pub fn domain(&self) -> Vec<SPValue> {
        self.variable_data().domain.clone()
    }
    pub fn state_predicate(&self) -> Option<Action> {
        if let Variable::StatePredicate(d, p) = self {
            Some(p.clone())
        } else {
            None
        }
    }

    fn variable_data(&self) -> &VariableData {
        match self {
            Variable::Measured(d) => d,
            Variable::Command(d) => d,
            Variable::Estimated(d) => d,
            Variable::StatePredicate(d, _) => d,
        }
    }
}

impl VariableType {
    pub fn is_type(&self, v: &Variable) -> bool {
        v.is_type(*self)
    }
}

impl Default for VariableType {
    fn default() -> Self {
        VariableType::Estimated
    }
}

/// A helper macros when creating variables
/// # Example
/// ```
/// use sp_domain::*;
/// 
/// let v_c = variable!(C "v", 0, 0, 1, 2, 3, 4);
/// let v_e = variable!(E "v");
/// 
/// ```
/// 
#[macro_export]
macro_rules! variable {
    ($name:expr, $init:expr, $($domain: expr),*) => {
        VariableData {
            spid: SPID::new($name),
            initial_value: $init.to_spvalue(),
            domain: vec!($($domain.to_spvalue()),*)
        }
    };
    (M $name:expr, $init:expr, $($domain: expr),*) => {
        {let data = variable!($name, $init, $($domain),*);
        Variable::Measured(data)}
    };
    (C $name:expr, $init:expr, $($domain: expr),*) => {
        {let data = variable!($name, $init, $($domain),*);
        Variable::Command(data)}
    };
    (E $name:expr, $init:expr, $($domain: expr),*) => {
        {let data = variable!($name, $init, $($domain),*);
        Variable::Estimated(data)}
    };
    (SP $name:expr, $action:expr, $init:expr, $($domain: expr),*) => {
        {let data = variable!($name, $init, $($domain),*);
        Variable::StatePredicate(data, $action.clone())}
    };
    (M $name:expr) => {
        {let data = variable!($name, false, false, true);
        Variable::Measured(data)}
    };
    (C $name:expr) => {
        {let data = variable!($name, false, false, true);
        Variable::Command(data)}
    };
    (E $name:expr) => {
        {let data = variable!($name, false, false, true);
        Variable::Estimated(data)}
    };
    (SP $name:expr, $action:expr) => {
        {let data = variable!($name, false, false, true);
        Variable::StatePredicate(data, $action.clone())}
    };
}


#[cfg(test)]
mod runner_tests {
    use super::*;
    #[test]
    fn variabel_macros() {
        let ab = SPPath::from_str(&["a", "b"]);
        let v_m = variable!(M "v");
        let v_c = variable!(C "v");
        let v_e = variable!(E "v");
        let v_sp = variable!(SP "v", a!(ab));
        let vd = VariableData {
            spid: v_m.spid(),
            initial_value: false.to_spvalue(),
            domain: vec!(false.to_spvalue(), true.to_spvalue()),
        };
        assert_eq!(v_m, Variable::Measured(vd));

        let v_m = variable!(M "v", 0, 0, 1, 2, 3, 4);
        let v_c = variable!(C "v", 0, 0, 1, 2, 3, 4);
        let v_e = variable!(E "v", 0, 0, 1, 2, 3, 4);
        let v_sp = variable!(SP "v", a!(ab), 0, 0, 1, 2, 3, 4);
        let vd = VariableData {
            spid: v_sp.spid(),
            initial_value: 0.to_spvalue(),
            domain: vec!(0.to_spvalue(), 1.to_spvalue(), 2.to_spvalue(), 3.to_spvalue(), 4.to_spvalue()),
        };
        assert_eq!(v_sp, Variable::StatePredicate(vd, a!(ab)));

        //let xs = vec!(v_m, v_c, v_sp).iter().map(|x| serde_json::to_string(x));

        println!("Test json: {}", serde_json::to_string(&vec!(v_m, v_c, v_sp)).unwrap());

    }

}