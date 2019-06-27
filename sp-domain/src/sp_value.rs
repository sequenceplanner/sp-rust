
//! SPValue represent a variable value of a specific type. The types used are
//! matched with ROS types for easy mapping between messages and SP
//! 

use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SPValue {
    Bool(bool),
    Float32(f32),
    Int32(i32),
    String(String),
    Time(u32),
    Duration(u32),
    Array(Vec<SPValue>),
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

impl SPValue {
    // pub fn get(&self, key: &str) -> Option<&SPValue> {
    //     match self {
    //         SPValue::Map(m) => { m.get(key) },
    //         _ => None
    //     }
    // }

}

impl Default for SPValue {
    fn default() -> Self {
        SPValue::Bool(false)
    }
}

/// A trait for converting a value to SPValue
pub trait ToSPValue {
    fn to_spvalue(&self) -> SPValue;
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
// impl ToSPValue for f64 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Float64(*self)
//     }
// }
// impl ToSPValue for i8 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Int8(*self)
//     }
// }
// impl ToSPValue for i16 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Int16(*self)
//     }
// }
impl ToSPValue for i32 {
    fn to_spvalue(&self) -> SPValue {
        SPValue::Int32(*self)
    }
}
// impl ToSPValue for i64 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Int64(*self)
//     }
// }
// impl ToSPValue for u8 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Uint8(*self)
//     }
// }
// impl ToSPValue for u16 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Uint16(*self)
//     }
// }
// impl ToSPValue for u32 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Uint32(*self)
//     }
// }
// impl ToSPValue for u64 {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::Uint64(*self)
//     }
// }
impl ToSPValue for String {
    fn to_spvalue(&self) -> SPValue {
        SPValue::String(self.clone())
    }
}
impl ToSPValue for &str {
    fn to_spvalue(&self) -> SPValue {
        SPValue::String(self.to_string())
    }
}
// impl ToSPValue for Uuid {
//     fn to_spvalue(&self) -> SPValue {
//         SPValue::ID(*self)
//     }
// }
impl<T> ToSPValue for Vec<T> where T: ToSPValue {
    fn to_spvalue(&self) -> SPValue {
        let res = self.iter().map(|x| x.to_spvalue()).collect::<Vec<SPValue>>();
        SPValue::Array(res)
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
        assert_eq!(String::from("hej").to_spvalue(), SPValue::String("hej".to_string()));
        //let id = Uuid::new_v4();
        //assert_eq!(id.to_spvalue(), SPValue::ID(id));
        assert_eq!(vec!("hej", "1", "id").to_spvalue(), SPValue::Array(vec!(SPValue::String("hej".to_string()), SPValue::String("1".to_string()), SPValue::String("id".to_string()))));

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

}