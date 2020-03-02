//! Z3 plans

use super::*;
use std::ffi::{CStr, CString};
use std::collections::HashMap;
use z3_sys::*;
use sp_domain::*;
use sp_runner::*;

pub struct ComputePlanSPModelZ3 {
    pub model: TransitionSystemModel,
    pub state: SPState,
    pub goals: Vec<(Predicate, Option<Predicate>)>,
    pub max_steps: u32,
    pub r: Z3_model
}

pub struct GetSPPredicateZ3<'ctx>{
    pub ctx: &'ctx ContextZ3,
    pub model: TransitionSystemModel,
    pub step: u32,
    pub pred: Predicate,
    pub r: Z3_ast
}

pub struct GetSPUpdatesZ3<'ctx>{
    pub ctx: &'ctx ContextZ3,
    pub model: TransitionSystemModel,
    pub trans: Transition,
    pub step: u32,
    pub r: Z3_ast
}

pub struct GetInitialStateZ3<'ctx>{
    pub ctx: &'ctx ContextZ3,
    pub model: TransitionSystemModel,
    pub state: SPState,
    pub init: Z3_ast
}

pub struct GetPlanningFramesZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub nr_steps: u32,
    pub frames: Vec<(i32, Vec<String>, String)> 
}

impl <'ctx> GetInitialStateZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, ts_model: &TransitionSystemModel, state: &SPState) -> Z3_ast {
        let state_vec: Vec<_> = state.clone().extract().iter().map(|(path,value)|path.clone()).collect();
 
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
    ANDZ3::new(&ctx, init_assert_vec)
    }
}

impl <'ctx> GetSPPredicateZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, step: u32, p: &Predicate) -> Z3_ast {
        let res_main: Z3_ast = match p {
            Predicate::TRUE => BoolZ3::new(&ctx, true),
            Predicate::FALSE => BoolZ3::new(&ctx, false),
            Predicate::NOT(p) => {
                let v = GetSPPredicateZ3::new(&ctx, &ts_model, step, p);
                NOTZ3::new(&ctx, v)
            },
            Predicate::AND(p) => {
                let v: Vec<_> = p.iter().map(|x| GetSPPredicateZ3::new(&ctx, &ts_model, step, x)).collect();
                ANDZ3::new(&ctx, v)
                },
            Predicate::OR(p) => {
                let v: Vec<_> = p.iter().map(|x| GetSPPredicateZ3::new(&ctx, &ts_model, step, x)).collect();
                ORZ3::new(&ctx, v)
                },
            Predicate::XOR(p) => panic!("TODO: Implement XOR"),
            Predicate::EQ(PredicateValue::SPPath(var, _),
                          PredicateValue::SPValue(value)) => {
                let init_var_type = &ts_model.vars.iter()
                    .find_map(|x| if x.path() == var { Some(x.value_type()) } else {None} )
                    .expect(&format!("3: could not find variable: {:?}", ts_model.vars));
                let res: Z3_ast = match init_var_type {
                    // let var_name = format!("{}_s{}", var.to_string(), step);
                    SPValueType::Bool => EQZ3::new(&ctx, 
                        BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                        BoolZ3::new(&ctx, false)),
                    SPValueType::String => {

                        let var_vec: Vec<_> = var.path.clone().iter().map(|path|path.clone()).collect();
                        let var_name = var_vec.join("/").to_string();
    
                        let mut init_domain_strings = vec!();
                        for ts_var in &ts_model.vars {
                            if var_name == ts_var.path().to_string() {
                                for v in ts_var.domain() {
                                    init_domain_strings.push(v.to_string());
                                }
                            }
                        }                   
                           
                        let val_index = init_domain_strings.iter().position(|r| *r == value.to_string()).unwrap();
                        let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());
                        let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                        let elements = &enum_sort.enum_asts;
                        EQZ3::new(&ctx, EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), elements[val_index])
                    },
                    _ => panic!("implement stuff")
                };
                res
            },

            Predicate::NEQ(PredicateValue::SPPath(var, _),
                          PredicateValue::SPValue(value)) => {
                let init_var_type = &ts_model.vars.iter()
                    .find_map(|x| if x.path() == var { Some(x.value_type()) } else {None} )
                    .expect(&format!("2: could not find variable: {:?}", ts_model.vars));

                let res: Z3_ast = match init_var_type {
                    // let var_name = format!("{}_s{}", var.to_string(), step);
                    SPValueType::Bool => NEQZ3::new(&ctx, 
                        BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                        BoolZ3::new(&ctx, false)),
                    SPValueType::String => {

                        let var_vec: Vec<_> = var.path.clone().iter().map(|path|path.clone()).collect();
                        let var_name = var_vec.join("/").to_string();

                        let mut init_domain_strings = vec!();
                        for ts_var in &ts_model.vars {
                            if var_name == ts_var.path().to_string() {
                                for v in ts_var.domain() {
                                    init_domain_strings.push(v.to_string());
                                }
                            }
                        }                   
                       
                        let val_index = init_domain_strings.iter().position(|r| *r == value.to_string()).unwrap();
                        let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());
                        let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                        let elements = &enum_sort.enum_asts;
                        NEQZ3::new(&ctx, EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), elements[val_index])
                    },
                    _ => panic!("implement stuff")
                };
                res
            },
            x => panic!("NO X {:?}", x)
        };
        res_main
    }
}

impl <'ctx> GetSPUpdatesZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, t: &Transition, step: u32) -> Z3_ast {
        
        let trans_node = t.node().to_string();
        let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
        let trans_name: String = format!("{}_t{}", trans_vec[0].to_string(), step);
        let trans_name_assert: Z3_ast = EQZ3::new(&ctx, 
            BoolZ3::new(&ctx, true), 
            BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), trans_name.as_str()));

        let mut updates = vec!();
        let mut act_and_eff = vec!();
        act_and_eff.extend(t.actions());
        act_and_eff.extend(t.effects());

        for a in act_and_eff {
            let v: Vec<_> = a.var.path.clone().iter().map(|path|path.clone()).collect();
            let var_name = v.join("/").to_string();

            let init_var_type = ts_model.vars.iter()
                .find_map(|x| if x.path() == &a.var { Some(x.value_type()) } else {None} )
                .expect(&format!("1: could not find variable: {:?}", ts_model.vars));

            let vars = &ts_model.vars;
            let mut vars_string = vec!();
            let mut init_domain_strings = vec!();
            for var in vars {
                vars_string.push(var.path().to_string());
                if var_name == var.path().to_string() {
                    for v in var.domain() {
                        init_domain_strings.push(v.to_string());
                    }
                }
            }

            let var_value: String = match &a.value {
                Compute::PredicateValue(pv) => match pv {
                    PredicateValue::SPValue(spval) => format!("{}", spval),
                    PredicateValue::SPPath(path, _) => format!("{}", path),
                },
                Compute::Predicate(p) => panic!("Predicate for action?"), // format!("{}", p), // still not sure how this works
                _ => panic!("What else?")
            };

            let mut astr: Z3_ast = EQZ3::new(&ctx, BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), "FAULT"), BoolZ3::new(&ctx, true));
            let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());      
            if var_value == "false" {
                astr = EQZ3::new(&ctx, 
                    BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var_name.to_string(), step).as_str()),
                    BoolZ3::new(&ctx, false));
            } else if var_value == "true" {
                astr = EQZ3::new(&ctx, 
                    BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var_name.to_string(), step).as_str()),
                    BoolZ3::new(&ctx, true));
            } else {
                if init_domain_strings.contains(&var_value) {
                    let val_index = init_domain_strings.iter().position(|r| *r == var_value.to_string()).unwrap();
                    let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                    let elements = &enum_sort.enum_asts;
                    astr = EQZ3::new(&ctx, EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), elements[val_index]);
                } else if vars_string.contains(&var_value) {
                    let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                    let elements = &enum_sort.enum_asts;
                    astr = EQZ3::new(&ctx, 
                        EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), 
                        EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_value.to_string(), step).as_str()));
                }
            }
            updates.push(astr);
        }
        ANDZ3::new(&ctx, updates)
    }
}

// impl Planner for ComputePlanSPModelZ3 {
impl ComputePlanSPModelZ3 {
    pub fn plan(model: &TransitionSystemModel,
            goals: &[(Predicate, Option<Predicate>)],
            state: &SPState,
            max_steps: u32) -> Vec<(u32, Vec<String>, String)> {

        let mut step: u32 = 0;

        let cfg = ConfigZ3::new();
        let ctx = ContextZ3::new(&cfg);
        let slv = SolverZ3::new(&ctx);

        slv_assert_z3!(&ctx, &slv, GetInitialStateZ3::new(&ctx, &model, &state));

        SlvPushZ3::new(&ctx, &slv);
        // for g in &goals.iter() {
        slv_assert_z3!(&ctx, &slv, GetSPPredicateZ3::new(&ctx, &model, 0, &goals[0].0));
        // }
        
        while SlvCheckZ3::new(&ctx, &slv) != 1 && step < max_steps {
            step = step + 1;
            SlvPopZ3::new(&ctx, &slv, 1);

            let mut all_trans = vec!();
            for t in &model.transitions {
                let trans_node = t.node().to_string();
                let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
                let trans_name: String = format!("{}_t{}", trans_vec[0].to_string(), step);
                let guard = GetSPPredicateZ3::new(&ctx, &model, step - 1, t.guard());
                let updates = GetSPUpdatesZ3::new(&ctx, &model, &t, step);
                all_trans.push(ANDZ3::new(&ctx, vec!(
                    EQZ3::new(&ctx, BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), trans_name.as_str()), BoolZ3::new(&ctx, true)),
                    guard,
                    updates
                )));
            }

            slv_assert_z3!(&ctx, &slv, ORZ3::new(&ctx, all_trans));

            SlvPushZ3::new(&ctx, &slv);
            // for g in &goals {
            slv_assert_z3!(&ctx, &slv, GetSPPredicateZ3::new(&ctx, &model, step, &goals[0].0));
            // }
        }
        let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
        let frames = GetPlanningFramesZ3::new(&ctx, model, step);
        frames
    }
}

impl <'ctx> GetPlanningFramesZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, model: Z3_model, nr_steps: u32) -> Vec<(u32, Vec<String>, String)> {
        let model_str = ModelToStringZ3::new(&ctx, model);
        let mut model_vec = vec!();

        let num = ModelGetNumConstsZ3::new(&ctx, model);
        let mut lines = model_str.lines();
        let mut i: u32 = 0;
        let mut frames: Vec<(u32, Vec<String>, String)> = vec!();

        while i <= num {
            model_vec.push(lines.next().unwrap_or(""));
            i = i + 1;
        }

        for i in 0..nr_steps + 2 {
            let mut frame: (u32, Vec<String>, String) = (0, vec!(), "".to_string());
            for j in &model_vec {
                let sep: Vec<&str> = j.split(" -> ").collect();
                frame.0 = i;
                if sep[0].ends_with(&format!("_s{}", i)){
                    frame.1.push(sep[1].to_string());
                } else if sep[0].ends_with(&format!("_t{}", i)) && sep[1] == "true" {
                    let trimmed = sep[0].trim_end_matches(&format!("_t{}", i));
                    frame.2 = trimmed.to_string();
                }
            }
            if frame.1.len() != 0 {
                frames.push(frame);
            } 
        }
        frames
    }
}