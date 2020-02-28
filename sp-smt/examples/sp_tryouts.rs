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
            },
    
            ability!{
                name: ability_x,
    
                change_x: p!([!x] && [y == "a"]) => [ a!(x) ] / [a!(y = "c")]
            }
        }
    }

    let mut m = Model::new_root("model_x", Vec::new());
    m.add_item(SPItem::Resource(r("resource_x")));

    let x = m.find_item("x", &[]).expect("check spelling1").path();
    let y = m.find_item("y", &[]).expect("check spelling2").path();

    let state = SPState::new_from_values(&[
        (x.clone(), false.to_spvalue()),
        (y.clone(), "a".to_spvalue()),
    ]);

    let ts_model = TransitionSystemModel::from(&m);

    println!("{:#?}", ts_model);

    let cfg = ConfigZ3::new();
    let ctx = ContextZ3::new(&cfg);
    let slv = SolverZ3::new(&ctx);

    let v: Vec<_> = state.clone().extract().iter().map(|(path,value)|path.clone()).collect();
 
    let mut init_assert_vec = vec!();
    for v1 in &v {
        
        let init_name = v1.to_string();
        println!("{}", init_name);

        let init_type: SPValueType = match state.sp_value_from_path(v1) {
            Some(x) => x.has_type(),
            None    => SPValueType::Unknown,
        };
        println!("{:?}", init_type);
        
        let init_value: String = match state.sp_value_from_path(v1) {
            Some(x) => x.to_string(),
            None    => SPValue::Unknown.to_string(),
        };
        println!("{}", init_value);

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

        let mut init_domain = vec!();
        for i in &init_domain_strings {
            init_domain.push(i.as_str())
        }

        if init_type == SPValueType::Bool {
            let bool_sort = BoolSortZ3::new(&ctx);
            let init = BoolVarZ3::new(&ctx, &bool_sort, &format!("{}_s0", init_name.as_str()));
            if init_value == "false" {
                init_assert_vec.push(EQZ3::new(&ctx, init, BoolZ3::new(&ctx, false)));
            } else {
                init_assert_vec.push(EQZ3::new(&ctx, init, BoolZ3::new(&ctx, true)));
            }
        
        } else {
            let slice =  &format!("{}_sort", init_name);
            let enum_sort = EnumSortZ3::new(&ctx, slice, init_domain);
            let enum_domain = &enum_sort.enum_asts;
            let index = enum_domain.iter().position(|ast| init_value == ast_to_string_z3!(&ctx, *ast)).unwrap();
            let init = EnumVarZ3::new(&ctx, enum_sort.r, &format!("{}_s0", init_name.as_str()));
            init_assert_vec.push(EQZ3::new(&ctx, init, enum_domain[index]));
        }
    }

    let init_state = ANDZ3::new(&ctx, init_assert_vec);
    slv_assert_z3!(&ctx, &slv, init_state);
    println!("Added initial state to context");
    println!("{}", slv_to_string_z3!(&ctx, &slv));

    // trans = Vec(trans_name, Vec(var, var_type, var_value))
    let mut trans: Vec<(String, Vec<(String, SPValueType, String)>)> = vec!();
    for t in &ts_model.transitions {

        // println!("GUARD : {}", t.guard());
        // match t.guard() {
        //     Predicate::NOT(p) => format!("!({})", NuXMVPredicate(&p)),
        //     Predicate::AND(x) => {
        //         let children: Vec<_> = x
        //             .iter()
        //             .map(|p| format!("{}", NuXMVPredicate(&p)))
        //             .collect();
        //         format!("( {} )", children.join("&"))
        //     }
        // }

        let trans_node = t.node().to_string();
        let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
        let trans_name: String = trans_vec[0].to_string();

        let mut updates: Vec<(String, SPValueType, String)> = vec!();
        let mut act_and_eff = vec!();
        act_and_eff.extend(t.actions());
        act_and_eff.extend(t.effects());

        for a in act_and_eff {
            let v: Vec<_> = a.var.path.clone().iter().map(|path|path.clone()).collect();
            let var_name = v.join("/").to_string();

            let var_type = ts_model.vars.iter()
                .find_map(|x| if x.path() == &a.var { Some(x.value_type()) } else {None} )
                .expect("could not find variable");
            // println!("{}", var_type);

            let var_value: String = match &a.value {
                Compute::PredicateValue(pv) => match pv {
                    PredicateValue::SPValue(spval) => format!("{}", spval),
                    _ => panic!("TODO: handle assign"),
                    // PredicateValue::SPPath(path, _) => Some(println!("{}", format!("{:?}", path))),
                },
                _ => panic!("TODO: handle predicate for action"),
                // Compute::Predicate(p) => None,
                // _ => None,
            };
            updates.push((var_name, var_type, var_value));
        }
        println!("updates: {:?}", updates);
    }
}