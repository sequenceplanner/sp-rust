use sp_smt::*;

use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
// use sp_runner::*;

fn main() {

    // pub fn r(name: &str) -> Resource {
    
    //     let mut domain = vec!["a", "b", "c"];

    //     resource!{
    //         name: name,
    //         estimated!{
    //             x: bool,
    //             y: domain,
    //             z: domain,
    //         },
    
    //         ability!{
    //             name: ability_x,
    
    //             change_x: p!([[!x] && [y != "a"]] || [y != "b"] || [x == false]) => [ a!(x), a!(z <- y) ] / [a!(y = "c")]
    //         }
    //     }
    // }

    // let mut m = Model::new_root("model_x", Vec::new());
    // m.add_item(SPItem::Resource(r("resource_x")));

    // let x = m.find_item("x", &[]).expect("check spelling1").path();
    // let y = m.find_item("y", &[]).expect("check spelling2").path();
    // let z = m.find_item("z", &[]).expect("check spelling3").path();

    // let state = SPState::new_from_values(&[
    //     (x.clone(), false.to_spvalue()),
    //     (y.clone(), "a".to_spvalue()),
    //     (z.clone(), "b".to_spvalue()),
    // ]);

    // let ts_model = TransitionSystemModel::from(&m);

    // println!("{:#?}", ts_model);

    // let cfg = ConfigZ3::new();
    // let ctx = ContextZ3::new(&cfg);
    // let slv = SolverZ3::new(&ctx);

    // let state_vec: Vec<_> = state.clone().extract().iter().map(|(path,value)|path.clone()).collect();
 
    // // name, spvaluetype, value, domain_name, domain
    // let mut initial_state: Vec<(String, SPValueType, String, Vec<String>)> = vec!();

    // for elem in &state_vec {
    //     let mut initial_substate: (String, SPValueType, String, Vec<String>) = ("".to_string(), SPValueType::Unknown, "".to_string(), vec!());

    //     let init_name = elem.to_string();
    //     println!("{}", init_name);
    //     initial_substate.0 = init_name.clone();

    //     let init_type: SPValueType = match state.sp_value_from_path(elem) {
    //         Some(x) => x.has_type(),
    //         None    => SPValueType::Unknown,
    //     };
    //     println!("{:?}", init_type);
    //     initial_substate.1 = init_type.clone();
        
    //     let init_value: String = match state.sp_value_from_path(elem) {
    //         Some(x) => x.to_string(),
    //         None    => SPValue::Unknown.to_string(),
    //     };
    //     println!("{}", init_value);
    //     initial_substate.2 = init_value.clone();

    //     let vars = &ts_model.vars;
    //     let mut init_domain_strings = vec!();
    //     for var in vars {
    //         if init_name == var.path().to_string() {
    //             for v in var.domain() {
    //                 init_domain_strings.push(v.to_string());
    //             }
    //         }
    //     }
    //     println!("{:?}", init_domain_strings);

    //     for i in init_domain_strings {
    //         initial_substate.3.push(i)
    //     }
    //     initial_state.push(initial_substate);
    // }

    // let mut init_assert_vec = vec!();

    // for i in &initial_state {
    //     if i.1 == SPValueType::Bool {
    //         let bool_sort = BoolSortZ3::new(&ctx);
    //         let init = BoolVarZ3::new(&ctx, &bool_sort, &format!("{}_s0", i.0.as_str()));
    //         if i.2 == "false" {
    //             init_assert_vec.push(EQZ3::new(&ctx, init, BoolZ3::new(&ctx, false)));
    //         } else {
    //             init_assert_vec.push(EQZ3::new(&ctx, init, BoolZ3::new(&ctx, true)));
    //         }
        
    //     } else {
    //         let domain_name = &format!("{}_sort", i.3.join(".").to_string());
    //         let dom_str_vec: Vec<&str> = i.3.iter().map(|s| &**s).collect();
    //         let enum_sort = EnumSortZ3::new(&ctx, domain_name, dom_str_vec);
    //         let enum_domain = &enum_sort.enum_asts;
    //         let index = enum_domain.iter().position(|ast| i.2 == ast_to_string_z3!(&ctx, *ast)).unwrap();
    //         let init = EnumVarZ3::new(&ctx, enum_sort.r, &format!("{}_s0", i.0.as_str()));
    //         init_assert_vec.push(EQZ3::new(&ctx, init, enum_domain[index]));
    //     }
    // }

    // let init_state = ANDZ3::new(&ctx, init_assert_vec);
    // slv_assert_z3!(&ctx, &slv, init_state);
    // println!("Added initial state to context");
    // println!("{}", slv_to_string_z3!(&ctx, &slv));

    // pub fn guard_to_z3(ctx: &ContextZ3, ts_model: &TransitionSystemModel, step: u32, p: &Predicate) -> Z3_ast {
    //     let res_main: Z3_ast = match p {
    //         Predicate::TRUE => BoolZ3::new(&ctx, true),
    //         Predicate::FALSE => BoolZ3::new(&ctx, false),
    //         Predicate::NOT(p) => {
    //             let v = guard_to_z3(&ctx, &ts_model, step, p);
    //             NOTZ3::new(&ctx, v)
    //         },
    //         Predicate::AND(p) => {
    //             let v: Vec<_> = p.iter().map(|x| guard_to_z3(&ctx, &ts_model, step, x)).collect();
    //             ANDZ3::new(&ctx, v)
    //             },
    //         Predicate::OR(p) => {
    //             let v: Vec<_> = p.iter().map(|x| guard_to_z3(&ctx, &ts_model, step, x)).collect();
    //             ORZ3::new(&ctx, v)
    //             },
    //         Predicate::XOR(p) => panic!("TODO: Implement XOR"),
    //         Predicate::EQ(PredicateValue::SPPath(var, _),
    //                       PredicateValue::SPValue(value)) => {
    //             let init_var_type = &ts_model.vars.iter()
    //                 .find_map(|x| if x.path() == var { Some(x.value_type()) } else {None} )
    //                 .expect("could not find variable");
    //             let res: Z3_ast = match init_var_type {
    //                 // let var_name = format!("{}_s{}", var.to_string(), step);
    //                 SPValueType::Bool => EQZ3::new(&ctx, 
    //                     BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
    //                     BoolZ3::new(&ctx, false)),
    //                 _ => panic!("implement stuff")
    //             };
    //             res
    //         },

    //         Predicate::NEQ(PredicateValue::SPPath(var, _),
    //                       PredicateValue::SPValue(value)) => {
    //             let init_var_type = &ts_model.vars.iter()
    //                 .find_map(|x| if x.path() == var { Some(x.value_type()) } else {None} )
    //                 .expect("could not find variable");

    //             let res: Z3_ast = match init_var_type {
    //                 // let var_name = format!("{}_s{}", var.to_string(), step);
    //                 SPValueType::Bool => NEQZ3::new(&ctx, 
    //                     BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
    //                     BoolZ3::new(&ctx, false)),
    //                 SPValueType::String => {

    //                     let var_vec: Vec<_> = var.path.clone().iter().map(|path|path.clone()).collect();
    //                     let var_name = var_vec.join("/").to_string();

    //                     let mut init_domain_strings = vec!();
    //                     for ts_var in &ts_model.vars {
    //                         if var_name == ts_var.path().to_string() {
    //                             for v in ts_var.domain() {
    //                                 init_domain_strings.push(v.to_string());
    //                             }
    //                         }
    //                     }                   
                       
    //                     let val_index = init_domain_strings.iter().position(|r| *r == value.to_string()).unwrap();
    //                     let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());
    //                     let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
    //                     let elements = &enum_sort.enum_asts;
    //                     NEQZ3::new(&ctx, EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), elements[val_index])
    //                 },
    //                 _ => panic!("implement stuff")
    //             };
    //             res
    //         },
    //         x => panic!("NO X {:?}", x)
    //     };
    //     res_main
    // }
    
    // let mut step: u32 = 0;
    // let max_steps: u32 = 10;
    // // while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
    // while step < max_steps {
    //     step = step + 1;
    //     // SlvPopZ3::new(&ctx, &slv, 1);
    // // trans = Vec(trans_name, Vec(var, var_type, var_value))
    //     let mut trans: Vec<(String, Z3_ast, Vec<(String, String, String)>)> = vec!();
    //     for t in &ts_model.transitions {
    //         let mut tr: (Z3_ast, Z3_ast, Vec<(Z3_ast)>);
    //         println!("{:?}", ast_to_string_z3!(&ctx, guard_to_z3(&ctx, &ts_model, step, t.guard())));

    //         // get action and effects:
    //         let trans_node = t.node().to_string();
    //         let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
    //         let trans_name: String = format!("{}_t{}", trans_vec[0].to_string(), step);
    //         let trans_name_assert: Z3_ast = EQZ3::new(&ctx, 
    //             BoolZ3::new(&ctx, true), 
    //             BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), trans_name.as_str()));

    //         // tr.0 = trans_name_assert;
            
    //         // get guard:
    //         let guard = guard_to_z3(&ctx, &ts_model, step, t.guard());
    //         // tr.1 = guard;

    //         let mut updates = vec!();
    //         let mut act_and_eff = vec!();
    //         act_and_eff.extend(t.actions());
    //         act_and_eff.extend(t.effects());

    //         for a in act_and_eff {
    //             let v: Vec<_> = a.var.path.clone().iter().map(|path|path.clone()).collect();
    //             let var_name = v.join("/").to_string();

    //             let init_var_type = ts_model.vars.iter()
    //                 .find_map(|x| if x.path() == &a.var { Some(x.value_type()) } else {None} )
    //                 .expect("could not find variable");

    //             let vars = &ts_model.vars;
    //             let mut vars_string = vec!();
    //             let mut init_domain_strings = vec!();
    //             for var in vars {
    //                 vars_string.push(var.path().to_string());
    //                 if var_name == var.path().to_string() {
    //                     for v in var.domain() {
    //                         init_domain_strings.push(v.to_string());
    //                     }
    //                 }
    //             }

    //             let var_value: String = match &a.value {
    //                 Compute::PredicateValue(pv) => match pv {
    //                     PredicateValue::SPValue(spval) => format!("{}", spval),
    //                     PredicateValue::SPPath(path, _) => format!("{}", path),
    //                 },
    //                 Compute::Predicate(p) => panic!("Predicate for action?"), // format!("{}", p), // still not sure how this works
    //                 _ => panic!("What else?")
    //             };

    //             let mut astr: Z3_ast = EQZ3::new(&ctx, BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), "FAULT"), BoolZ3::new(&ctx, true));
    //             let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());      
    //             if var_value == "false" {
    //                 astr = EQZ3::new(&ctx, 
    //                     BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var_name.to_string(), step).as_str()),
    //                     BoolZ3::new(&ctx, false));
    //             } else if var_value == "true" {
    //                 astr = EQZ3::new(&ctx, 
    //                     BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var_name.to_string(), step).as_str()),
    //                     BoolZ3::new(&ctx, true));
    //             } else {
    //                 if init_domain_strings.contains(&var_value) {
    //                     let val_index = init_domain_strings.iter().position(|r| *r == var_value.to_string()).unwrap();
    //                     let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
    //                     let elements = &enum_sort.enum_asts;
    //                     astr = EQZ3::new(&ctx, EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), elements[val_index]);
    //                 } else if vars_string.contains(&var_value) {
    //                     let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
    //                     let elements = &enum_sort.enum_asts;
    //                     astr = EQZ3::new(&ctx, 
    //                         EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), 
    //                         EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_value.to_string(), step).as_str()));
    //                 }
    //             }
    //             updates.push(ast_to_string_z3!(&ctx, astr));
    //         }
    //         println!("updates: {:?}", updates);
    //     }

    //     // let goal = guard_to_z3(&ctx, &ts_model, step: u32, p: &Predicate); 
    // }
}