//! Variables in SP are represented by SPValue, which is a direct mapping
//! to ROS-types.
//!
use serde::{Deserialize, Serialize};
use std::fmt;

/// SPValue represent a variable value of a specific type. The types used are
/// matched with ROS types for easy mapping between messages and SP
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum SPValue {
    Bool(bool),
    Float32(f32),
    Int32(i32),
    String(String),
    Time(std::time::SystemTime),
    Path(super::SPPath),
    Array(SPValueType, Vec<SPValue>),
    Unknown,
    //byte(u8), //deprecated
    //char(char), //deprecated
    //Float64(f64), // Let us add these back if we need them. Just to simplify matching
    //Int8(i8),
    //Uint8(u8),
    //Int16(i16),
    //Uint16(u16),
    //Uint32(u32),
    //Int64(i64),
    //Uint64(u64),
    //ID(Uuid), // trying to use variables that has static names
    //Map(HashMap<String, SPValue>), // Maybe we should not allow this and flatten in the DSL.
}

/// Used by Variables for defining type. Must be the same as SPValue
#[derive(Debug, PartialEq, Copy, Clone, Serialize, Deserialize)]
pub enum SPValueType {
    Bool,
    Float32,
    Int32,
    String,
    Time,
    Path,
    Array,
    Unknown,
}

/// A trait for converting a value to SPValue
pub trait ToSPValue {
    fn to_spvalue(&self) -> SPValue;
}

impl SPValue {
    pub fn is_type(&self, t: SPValueType) -> bool {
        match self {
            SPValue::Bool(_) => SPValueType::Bool == t,
            SPValue::Float32(_) => SPValueType::Float32 == t,
            SPValue::Int32(_) => SPValueType::Int32 == t,
            SPValue::String(_) => SPValueType::String == t,
            SPValue::Time(_) => SPValueType::Time == t,
            SPValue::Path(_) => SPValueType::Path == t,
            SPValue::Array(at, _) => at == &t,
            SPValue::Unknown => SPValueType::Unknown == t,
        }
    }

    pub fn is_array(&self) -> bool {
        match self {
            SPValue::Array(_, _) => true,
            _ => false,
        }
    }

    pub fn has_type(&self) -> SPValueType {
        match self {
            SPValue::Bool(_) => SPValueType::Bool,
            SPValue::Float32(_) => SPValueType::Float32,
            SPValue::Int32(_) => SPValueType::Int32,
            SPValue::String(_) => SPValueType::String,
            SPValue::Time(_) => SPValueType::Time,
            SPValue::Path(_) => SPValueType::Path,
            SPValue::Array(t, _) => *t,
            SPValue::Unknown => SPValueType::Unknown,
        }
    }

    pub fn from_json_type_hint(json: &serde_json::Value, spv_t: SPValueType) -> SPValue {
        // as we have more options than json we switch on the spval type
        let tm = |msg: &str| {
            format!(
                "type mismatch! got {}, expected {}! re-generate ros sources!",
                &json.to_string(),
                msg
            )
        };
        match spv_t {
            SPValueType::Bool => json
                .as_bool()
                .unwrap_or_else(|| panic!("{}", tm("bool")))
                .to_spvalue(),
            SPValueType::Int32 => {
                (json.as_i64().unwrap_or_else(|| panic!("{}", tm("int"))) as i32).to_spvalue()
            }
            SPValueType::Float32 => {
                (json.as_f64().unwrap_or_else(|| panic!("{}", tm("float"))) as f32).to_spvalue()
            }
            SPValueType::String => json
                .as_str()
                .unwrap_or_else(|| panic!("{}", tm("string")))
                .to_spvalue(),
            SPValueType::Time => {
                let t: std::time::SystemTime =
                    serde_json::from_value(json.clone()).expect(&tm("time"));
                SPValue::Time(t)
            }
            SPValueType::Path => {
                let p: super::SPPath = serde_json::from_value(json.clone()).expect(&tm("path"));
                SPValue::Path(p)
            }
            // todo: check is_array
            _ => unimplemented!("TODO {:?}", spv_t),
        }
    }

    pub fn to_json(&self) -> serde_json::Value {
        match self {
            SPValue::Bool(x) => serde_json::json!(*x),
            SPValue::Int32(x) => serde_json::json!(*x),
            SPValue::Float32(x) => serde_json::json!(*x),
            SPValue::String(x) => serde_json::json!(x),
            SPValue::Time(x) => serde_json::json!(x),
            SPValue::Path(x) => serde_json::json!(x),
            SPValue::Array(_, x) => {
                let v: Vec<serde_json::Value> = x.iter().map(|spval| spval.to_json()).collect();
                serde_json::json!(v)
            }
            SPValue::Unknown => serde_json::json!("[Unknown]"),
        }
    }

    pub fn from_json(value: &serde_json::Value) -> SPValue {
        match value {
            serde_json::Value::Array(x) => {
                let value_type =
                    SPValue::from_json(x.first().unwrap_or(&serde_json::Value::Null)).has_type();
                let array = x.iter().map(|v| SPValue::from_json(v)).collect();
                SPValue::Array(value_type, array)
            }
            serde_json::Value::Bool(x) => SPValue::Bool(x.clone()),
            serde_json::Value::Number(x) if x.is_f64() => {
                SPValue::Float32(x.as_f64().unwrap() as f32)
            }
            serde_json::Value::Number(x) if x.is_i64() => {
                SPValue::Int32(x.as_i64().unwrap() as i32)
            }
            serde_json::Value::String(x) => SPValue::String(x.clone()),
            serde_json::Value::Object(_) => {
                if let Some(p) = serde_json::from_value(value.clone()).ok() {
                    return SPValue::Path(p);
                }
                if let Some(p) = serde_json::from_value(value.clone()).ok() {
                    return SPValue::Time(p);
                }
                SPValue::Unknown
            }
            _ => SPValue::Unknown,
        }
    }

    pub fn now() -> Self {
        SPValue::Time(std::time::SystemTime::now())
    }
}

impl SPValueType {
    pub fn is_type(self, v: &SPValue) -> bool {
        v.is_type(self)
    }
}

impl Default for SPValue {
    fn default() -> Self {
        SPValue::Bool(false)
    }
}

impl Default for SPValueType {
    fn default() -> Self {
        SPValueType::Bool
    }
}

impl fmt::Display for SPValue {
    fn fmt(&self, fmtr: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SPValue::Bool(b) if *b => write!(fmtr, "true"),
            SPValue::Bool(_) => write!(fmtr, "false"),
            SPValue::Float32(f) => write!(fmtr, "{}", f),
            SPValue::Int32(i) => write!(fmtr, "{}", i),
            SPValue::String(s) => write!(fmtr, "{}", s),
            SPValue::Time(t) => write!(fmtr, "Time({:?} ago)", t.elapsed().unwrap_or_default()),
            SPValue::Path(d) => write!(fmtr, "{}", d),
            SPValue::Array(_, a) => write!(fmtr, "{:?}", a),
            SPValue::Unknown => write!(fmtr, "[unknown]"),
        }
    }
}

impl ToSPValue for bool {
    fn to_spvalue(&self) -> SPValue {
        SPValue::Bool(*self)
    }
}

impl ToSPValue for f32 {
    fn to_spvalue(&self) -> SPValue {
        SPValue::Float32(*self)
    }
}
impl ToSPValue for i32 {
    fn to_spvalue(&self) -> SPValue {
        SPValue::Int32(*self)
    }
}
impl ToSPValue for usize {
    fn to_spvalue(&self) -> SPValue {
        SPValue::Int32(*self as i32)
    }
}

impl ToSPValue for String {
    fn to_spvalue(&self) -> SPValue {
        SPValue::String(self.clone())
    }
}
impl ToSPValue for &str {
    fn to_spvalue(&self) -> SPValue {
        SPValue::String((*self).to_string())
    }
}
impl ToSPValue for super::SPPath {
    fn to_spvalue(&self) -> SPValue {
        SPValue::Path(self.clone())
    }
}
impl ToSPValue for std::time::SystemTime {
    fn to_spvalue(&self) -> SPValue {
        SPValue::Time(*self)
    }
}

impl<T> ToSPValue for Vec<T>
where
    T: ToSPValue,
{
    fn to_spvalue(&self) -> SPValue {
        let res = self
            .iter()
            .map(|x| x.to_spvalue())
            .collect::<Vec<SPValue>>();
        res.to_spvalue()
    }
}
impl ToSPValue for Vec<SPValue> {
    fn to_spvalue(&self) -> SPValue {
        if self.is_empty() {
            SPValue::Array(SPValueType::Unknown, self.clone())
        } else {
            let spvaltype = self[0].has_type();
            assert!(self.iter().all(|e| e.has_type() == spvaltype));
            SPValue::Array(spvaltype, self.clone())
        }
    }
}

// impl<T> ToSPValue for HashMap<String, T> where T: ToSPValue {
//     fn to_spvalue(&self) -> SPValue {
//         let res: HashMap<String, SPValue> = self.iter().map(|(key, value)| {(key.clone(), value.to_spvalue())}).collect();
//         SPValue::Map(res)
//     }
// }
// impl ToSPValue for HashMap<String, SPValue> {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Map(self.clone())
//     }
// }

/// helping making spvalue maps macros
// #[macro_export]
// macro_rules! sp_value_map {
//     ($( $key: expr => $val: expr ),*) => {{
//         let mut map = ::std::collections::HashMap::new();
//         $( map.insert($key.to_string(), $val.to_spvalue()); )*
//         SPValue::Map(map)
//     }}
// }

/// ********** TESTS ***************

#[cfg(test)]
mod sp_value_test {
    use super::*;
    #[test]
    fn create() {
        assert_eq!(true.to_spvalue(), SPValue::Bool(true));
        assert_eq!(false.to_spvalue(), SPValue::Bool(false));
        assert_eq!(1.to_spvalue(), SPValue::Int32(1));
        // assert_eq!((1 as u64).to_spvalue(), SPValue::Uint64(1));
        assert_eq!((1.2 as f32).to_spvalue(), SPValue::Float32(1.2));
        assert_eq!("hej".to_spvalue(), SPValue::String("hej".to_string()));
        assert_eq!(
            String::from("hej").to_spvalue(),
            SPValue::String("hej".to_string())
        );
        //let id = Uuid::new_v4();
        //assert_eq!(id.to_spvalue(), SPValue::ID(id));
        assert_eq!(
            vec!("hej", "1", "id").to_spvalue(),
            SPValue::Array(
                SPValueType::String,
                vec!(
                    SPValue::String("hej".to_string()),
                    SPValue::String("1".to_string()),
                    SPValue::String("id".to_string())
                )
            )
        );

        // let map = sp_value_map!("hej" => 1, "nu" => "ja");
        // if let SPValue::Map(m) = map {
        //     println!("{:?}", m);
        //     assert_eq!(m.get("hej"), Some(&(1.to_spvalue())));
        // } else {
        //     assert!(false);
        // };

        let a = 1.to_spvalue();
        let b = 1.to_spvalue();
        assert!(a == b);

        // let a = sp_value_map!("hej" => 1, "nu" => "ja");
        // let b = sp_value_map!("hej" => 1, "nu" => "ja");
        // assert!(a == b);
    }

    #[test]
    fn type_handling() {
        let x = true.to_spvalue();
        let y = 1.to_spvalue();
        let z = vec!["hej", "då"].to_spvalue();

        assert_eq!(x.has_type(), SPValueType::Bool);
        assert_eq!(y.has_type(), SPValueType::Int32);
        assert_eq!(z.has_type(), SPValueType::String);
        assert!(z.is_array());

        assert!(SPValueType::Bool.is_type(&x));
        assert!(y.is_type(SPValueType::Int32));
        assert!(z.is_type(SPValueType::String));
    }
}
