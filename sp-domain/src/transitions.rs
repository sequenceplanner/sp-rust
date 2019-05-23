use uuid::Uuid;
use serde::{Serialize, Deserialize, };
use std::collections::HashMap;


struct Transition {
    
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
enum Predicate {
    AND(Vec<Predicate>),
    OR(Vec<Predicate>),
    NOT(Box<Predicate>),
    TRUE,
    FALSE,
    EQ(SPValue, SPValue),  // use SPValue::ID to fetch the value from the state
    NEQ(SPValue, SPValue),
    INDOMAIN(SPValue, Vec<SPValue>)
}


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
enum SPValue {
    Bool(bool),
    //byte(u8), deprecated
    //char(char), deprecated
    Float32(f32), 
    Float64(f64),
    Int8(i8), 
    Uint8(u8),
    Int16(i16), 
    Uint16(u16),
    Int32(i32), 
    Uint32(u32),
    Int64(i64), 
    Uint64(u64),
    String(String),
    Time(u32),
    Duration(u32),
    ID(Uuid),     // use to also find the value in a state of variable with id
    Array(Vec<SPValue>),
    Map(HashMap<String, SPValue>)
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
struct Action {
    var: Uuid,
    value: Compute
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
/// Used in actions to compute a new SPValue. 
enum Compute {
    Get(SPValue),
    //TakeNext(SPValue, Vec<SPValue>), // to be impl when needed
    //TakeBefore(SPValue, Vec<SPValue>),
    // Add(Box<Compute>, Box<Compute>),
    // Sub(Box<Compute>, Box<Compute>),
    // Join(Box<Compute>, Box<Compute>),
}