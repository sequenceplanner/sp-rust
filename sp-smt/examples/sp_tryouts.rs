use sp_smt::*;

use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
use sp_runner::*;

fn main() {

    pub fn r(name: &str) -> Resource {
        // let mut domain = vec!["unknown"];
        // domain.extend(poses.iter());
        resource!{
            name: name,
            estimated!{
                x: bool,
            },
    
            ability!{
                name: ability_x,
    
                change_x: p!(!x) => [ a!(x) ] / []
            }
        }
    }

    let mut m = Model::new_root("model_x", Vec::new());
    m.add_item(SPItem::Resource(r("resource_x")));

    let x = m.find_item("x", &[]).expect("check spelling1").path();

    let x = SPPath::from_string("model_x/resource_x/x");
    let state = SPState::new_from_values(&[
        (x.clone(), false.to_spvalue()),
    ]);

    let v: Vec<_> = state.clone().extract().iter().map(|(path,value)|path.clone()).collect();
    let vs: Vec<_> = v.iter().map(|x|x.to_string()).collect();
    let init_string =  vs.join(",");
    println!("{}", init_string);
   
    // println!("{:#?}", state.state_path(&x));
    println!("x is {:?}", state.sp_value_from_path(&x));
    // let sp_value = state.sp_value_from_path(&x);

    let init_type: SPValueType = match state.sp_value_from_path(&x) {
        Some(x) => x.has_type(),
        None    => SPValueType::Unknown,
    };

    let init_value: String = match state.sp_value_from_path(&x) {
        Some(x) => x.to_string(),
        None    => SPValue::Unknown.to_string(),
    };

    println!("{}", init_value);

    if init_type == SPValueType::Bool {
        println!("yaay");
    }
    println!("{:?}", init_type);

    // let type =
    // match sp_value {
    //     // The division was valid
    //     Some(x) => println!("{:?}", x.has_type()),
    //     // The division was invalid
    //     None    => println!("Cannot divide by 0"),
    // }

    // match sp_value {
    //     // The division was valid
    //     Some(x) => println!("{:?}", x.has_type()),
    //     // The division was invalid
    //     None    => println!("Cannot divide by 0"),
    // }

    // println!("x is {:?}", sp_value.has_type());
    // println!("x is {:?}", state.sp_va);

    // let ts_model = TransitionSystemModel::from(&m);
    // println!("{:#?}", ts_model);

    // let result = plan(&ts_model, &[(p!(p:x),None)], &state);
    // println!("{:#?}", result);

    

    //let result = compute_plan(&ts_model, &[(goal,Some(invar))], &state, 20);


    // sp stuff:  
    // let x = SPPath::from_string("x");
    // let y = SPPath::from_string("y");
    // let vars = vec![x, y];
    
    // let t = Transition::new("tx", p!(!x), vec![a!(x)], vec![], true);

    // let init = p!([!x] && [!y]);

    // println!("vars: {:?}", vars);
    // println!("t: {:?}",t );
    // println!("init: {:?}",init );


    // problem descriprion:
    
}