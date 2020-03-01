use sp_smt::*;

use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
use sp_runner::*;

fn main() {

    pub fn r(name: &str) -> Resource {
    
        let mut domain = vec!["a", "b", "c"];

        resource!{
            name: name,
            estimated!{
                x: bool,
                y: domain,
                z: domain,
            },
    
            ability!{
                name: ability_x,
    
                change_x: p!([!x] && [y == "a"]) => [ a!(x), a!(z <- y) ] / [a!(y = "c")]
            }
        }
    }

    let mut m = Model::new_root("model_x", Vec::new());
    m.add_item(SPItem::Resource(r("resource_x")));

    let x = m.find_item("x", &[]).expect("check spelling1").path();
    let y = m.find_item("y", &[]).expect("check spelling2").path();
    let z = m.find_item("z", &[]).expect("check spelling3").path();

    let state = SPState::new_from_values(&[
        (x.clone(), false.to_spvalue()),
        (y.clone(), "a".to_spvalue()),
        (z.clone(), "b".to_spvalue()),
    ]);

    let ts_model = TransitionSystemModel::from(&m);

    println!("{:#?}", ts_model);

    let cfg = ConfigZ3::new();
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let state_vec: Vec<_> = state.clone().extract().iter().map(|(path,value)|path.clone()).collect();
 
    // name, spvaluetype, value, domain_name, domain
    let mut initial_state: Vec<(String, SPValueType, String, Vec<String>)> = vec!();

    for elem in &state_vec {
        let mut initial_substate: (String, SPValueType, String, Vec<String>) = ("".to_string(), SPValueType::Unknown, "".to_string(), vec!());

        let init_name = elem.to_string();
        println!("{}", init_name);
        initial_substate.0 = init_name.clone();

        let init_type: SPValueType = match state.sp_value_from_path(elem) {
            Some(x) => x.has_type(),
            None    => SPValueType::Unknown,
        };
        println!("{:?}", init_type);
        initial_substate.1 = init_type.clone();
        
        let init_value: String = match state.sp_value_from_path(elem) {
            Some(x) => x.to_string(),
            None    => SPValue::Unknown.to_string(),
        };
        println!("{}", init_value);
        initial_substate.2 = init_value.clone();

        let vars = &ts_model.vars;
        let mut init_domain_strings = vec!();
        for var in vars {
            if init_name == var.path().to_string() {
                for v in var.domain() {
                    init_domain_strings.push(v.to_string());
                }
            }
        }
        println!("{:?}", init_domain_strings);

        for i in init_domain_strings {
            initial_substate.3.push(i)
        }
        initial_state.push(initial_substate);
    }

    let mut init_assert_vec = vec!();

    for i in &initial_state {

        if i.1 == SPValueType::Bool {
            let bool_sort = BoolSortZ3::new(&ctx);
            let init = BoolVarZ3::new(&ctx, &bool_sort, &format!("{}_s0", i.0.as_str()));
            if i.2 == "false" {
                init_assert_vec.push(EQZ3::new(&ctx, init, BoolZ3::new(&ctx, false)));
            } else {
                init_assert_vec.push(EQZ3::new(&ctx, init, BoolZ3::new(&ctx, true)));
            }
        
        } else {
            let domain_name = &format!("{}_sort", i.3.join(".").to_string());
            let dom_str_vec: Vec<&str> = i.3.iter().map(|s| &**s).collect();
            let enum_sort = EnumSortZ3::new(&ctx, domain_name, dom_str_vec);
            let enum_domain = &enum_sort.enum_asts;
            let index = enum_domain.iter().position(|ast| i.2 == ast_to_string_z3!(&ctx, *ast)).unwrap();
            let init = EnumVarZ3::new(&ctx, enum_sort.r, &format!("{}_s0", i.0.as_str()));
            init_assert_vec.push(EQZ3::new(&ctx, init, enum_domain[index]));
        }
    }

    let init_state = ANDZ3::new(&ctx, init_assert_vec);
    slv_assert_z3!(&ctx, &slv, init_state);
    println!("Added initial state to context");
    println!("{}", slv_to_string_z3!(&ctx, &slv));

    // trans = Vec(trans_name, Vec(var, var_type, var_value))
    let mut trans: Vec<(String, Vec<(String, String, String)>)> = vec!();
    for t in &ts_model.transitions {

        // fn guard_into_z3(p: Predicate) -> String {
        //     let s: String = match t.guard() {
        //         // Predicate::AND(p) => println!("{:?}", format!("{:?}", p)),
        //         Predicate::OR(p) => format!("{:?}", p),
        //         Predicate::TRUE => "TRUE",
        //         Predicate::FALSE => "FALSE",
        //         Predicate::NOT(p) => format!("NOOOOOOOOOT {:?}", p),
        //         Predicate::XOR(_) => println!("TODO"),
        //         Predicate::EQ(x, y) => println!("{:?}", format!("{:?} {:?}", x, y)),
        //         Predicate::NEQ(x, y) => println!("{:?}", format!("{:?}  {:?}", x, y)),
    
        //         Predicate::AND(x) => {
        //             let children: Vec<_> = x
        //                 .iter()
        //                 .map(|p| format!("{}", &p))
        //                 .collect();
        //             println!("{:?}", format!("( {} )", children.join("&")));
        //             for c in &children {
        //                 guard_into_z3(c);
        //             }
        //         }
        //     };
        // }

        // println!("GUARD : {}", t.guard());
        // match t.guard() {
        //     // Predicate::AND(p) => println!("{:?}", format!("{:?}", p)),
        //     Predicate::OR(p) => println!("{:?}", format!("{:?}", p)),
        //     Predicate::TRUE => println!("TRUE"),
        //     Predicate::FALSE => println!("FALSE"),
        //     Predicate::NOT(p) => println!("NOOOOOOOOOT {:?}", p),
        //     Predicate::XOR(_) => println!("TODO"),
        //     Predicate::EQ(x, y) => println!("{:?}", format!("{:?} {:?}", x, y)),
        //     Predicate::NEQ(x, y) => println!("{:?}", format!("{:?}  {:?}", x, y)),

        //     Predicate::AND(x) => {
        //             let children: Vec<_> = x
        //                 .iter()
        //                 .map(|p| format!("{}", &p))
        //                 .collect();
        //                 println!("{:?}", format!("( {} )", children.join("&")));
        //     }
            
            // Predicate::NOT(p) => println!("{}", format!("{}", p)),
            // Predicate::NOT(p) => println!("{}", format!("{}", p)),
            // Predicate::NOT(p) => println!("{}", format!("{}", p)),
            // Predicate::NOT(p) => println!("{}", format!("{}", p)),
            // // Predicate::AND(x) => {
            //     let children: Vec<_> = x
            //         .iter()
            //         .map(|p| format!("{}", NuXMVPredicate(&p)))
            //         .collect();
            //     format!("( {} )", children.join("&"))

        let trans_node = t.node().to_string();
        let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
        let trans_name: String = trans_vec[0].to_string();

        let mut updates: Vec<(String, String, String)> = vec!();
        let mut act_and_eff = vec!();
        act_and_eff.extend(t.actions());
        act_and_eff.extend(t.effects());

        for a in act_and_eff {
            let v: Vec<_> = a.var.path.clone().iter().map(|path|path.clone()).collect();
            let var_name = v.join("/").to_string();

            let init_var_type = ts_model.vars.iter()
                .find_map(|x| if x.path() == &a.var { Some(x.value_type()) } else {None} )
                .expect("could not find variable");

            let vars = &ts_model.vars;
            let mut init_domain_strings = vec!();
            for var in vars {
                if var_name == var.path().to_string() {
                    for v in var.domain() {
                        init_domain_strings.push(v.to_string());
                    }
                }
            }

            let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());            
            let var_type: String = match init_var_type {
                SPValueType::Array => "array".to_string(),
                SPValueType::Bool => "bool".to_string(),
                SPValueType::Int32 => "int32".to_string(),
                SPValueType::Float32 => "float32".to_string(),
                SPValueType::String => domain_name,
                _ => "TODO: Other SP value types to Z3".to_string(),
            };

            let var_value: String = match &a.value {
                Compute::PredicateValue(pv) => match pv {
                    PredicateValue::SPValue(spval) => format!("{}", spval),
                    PredicateValue::SPPath(path, _) => format!("{}", path),
                },
                Compute::Predicate(p) => format!("{}", p),
                _ => format!("{}", "blah"),
            };
            updates.push((var_name, var_type, var_value));
        }
        println!("updates: {:?}", updates);
    }
}