//! Z3 planning

use super::*;
use z3_sys::*;
use sp_domain::*;
use std::time::{Duration, Instant};
use std::collections::HashSet;
// use Iterator::*;
use std::thread;
use crossbeam::channel;


#[derive(Default, Debug)]
pub struct PlanningFrameZ3 {
    pub state: SPState,
    pub transition: SPPath,
}

#[derive(Debug)]
pub struct PlanningResultZ3 {
    pub plan_found: bool,
    pub plan_length: u32,
    pub trace: Vec<PlanningFrameZ3>,
    pub time_to_solve: std::time::Duration,
    pub raw_output: String,
    pub raw_error_output: String,
}

pub struct SeqComputePlanSPModelZ3 {
    pub model: TransitionSystemModel,
    pub state: SPState,
    pub goals: Vec<(Predicate, Option<Predicate>)>,
    pub max_steps: u32,
    pub z3_model: Z3_model,
    pub result: PlanningResultZ3
}

pub struct SubParComputePlanSPModelZ3 {
    pub model: TransitionSystemModel,
    pub state: SPState,
    pub goals: Vec<(Predicate, Option<Predicate>)>,
    pub max_steps: u32,
    pub z3_model: Z3_model,
    pub result: PlanningResultZ3
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

pub struct GetSPPlanningResultZ3<'ctx> {
    pub ctx: &'ctx ContextZ3,
    pub model: Z3_model,
    pub nr_steps: u32,
    pub frames: PlanningResultZ3
}

pub struct GetSPVarDomain {}

pub struct GetModifiedVars {}

pub struct GetSPKeepValueUpdatesZ3 {}

impl GetModifiedVars {
    pub fn new(t: &Transition) -> HashSet<SPPath> {
        let mut r = HashSet::new();

        r.extend(t.actions().iter().map(|a| a.var.clone()));
        r.extend(t.effects().iter().map(|a| a.var.clone()));

        r
    }
}

impl GetSPVarDomain {
    pub fn new(ts_model: &TransitionSystemModel, var_name: &String) -> Vec<String> {
        let vars = &ts_model.vars;
        let mut domain = vec!();
        for var in vars {
            if var_name == &var.path().to_string() {
                for v in var.domain() {
                    domain.push(v.to_string());
                }
            }
        }
        domain
    }
}

impl GetSPKeepValueUpdatesZ3 {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, t: &Transition, step: u32) -> Z3_ast {
        let modified = GetModifiedVars::new(t);
        let mut all_vars: HashSet<SPPath> = HashSet::new();
        all_vars.extend(ts_model.vars.iter().map(|v|v.path()).cloned());
        let untouched = all_vars.difference(&modified);
        let mut updates = vec!();
        for u in untouched {
            match ts_model.vars.iter().find(|x| u == x.path()) {
                Some(x) => {
                    match x.value_type() {
                        SPValueType::String => {
                            let name = u.to_string();
                            let domain = GetSPVarDomain::new(&ts_model, &name);
                            let domain_name = format!("{}_sort", domain.join(".").to_string());
                            let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), domain.iter().map(|x| x.as_str()).collect());
                            updates.push(EQZ3::new(&ctx, 
                                EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", u.to_string(), step).as_str()), 
                                EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", u.to_string(), step - 1).as_str())));
                        },
                        SPValueType::Bool => {
                            updates.push(EQZ3::new(&ctx, 
                                BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", u.to_string(), step).as_str()), 
                                BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", u.to_string(), step - 1).as_str())));
                        },
                        SPValueType::Int32 => {
                            updates.push(EQZ3::new(&ctx, 
                                IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), format!("{}_s{}", u.to_string(), step).as_str()), 
                                IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), format!("{}_s{}", u.to_string(), step - 1).as_str())));
                        },
                        _=> panic!("Implement"),
                    }
                },
                None => panic!("there has to be a variable"),
            }
        }
        ANDZ3::new(&ctx, updates)
    }
}


impl <'ctx> GetInitialStateZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, ts_model: &TransitionSystemModel, state: &SPState) -> Z3_ast {
        let state_vec: Vec<_> = state.clone().extract().iter().map(|(path,value)|path.clone()).collect();
        
        let mut init_assert_vec = vec!();
        for elem in &state_vec {
            let init_name = elem.to_string();
            match state.sp_value_from_path(elem) {
                Some(x) => match x.has_type() {
                    SPValueType::Bool => match x {
                        SPValue::Bool(false) => init_assert_vec.push(EQZ3::new(&ctx, 
                            BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), &format!("{}_s0", init_name.as_str())), 
                            BoolZ3::new(&ctx, false))),
                        SPValue::Bool(true) => init_assert_vec.push(EQZ3::new(&ctx, 
                            BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), &format!("{}_s0", init_name.as_str())), 
                            BoolZ3::new(&ctx, true))),
                        _ => panic!("Impossible"),
                    },
                    SPValueType::String => {
                        let domain = GetSPVarDomain::new(&ts_model, &init_name);
                        let domain_name = &format!("{}_sort", domain.join(".").to_string());
                        let dom_str_vec: Vec<&str> = domain.iter().map(|s| &**s).collect();
                        let enum_sort = EnumSortZ3::new(&ctx, domain_name, dom_str_vec);
                        let enum_domain = &enum_sort.enum_asts;
                        let index = enum_domain.iter().position(|ast| x.to_string() == ast_to_string_z3!(&ctx, *ast)).unwrap();
                        let init = EnumVarZ3::new(&ctx, enum_sort.r, &format!("{}_s0", init_name.as_str()));
                        init_assert_vec.push(EQZ3::new(&ctx, init, enum_domain[index]));
                    },
                    SPValueType::Int32 => {
                        let domain = GetSPVarDomain::new(&ts_model, &init_name);
                        let domain_ints: Vec<u32> = domain.iter().map(|x| x.parse().unwrap()).collect();
                        let domain_asts: Vec<Z3_ast> = domain_ints.iter().map(|x| IntZ3::new(&ctx, &IntSortZ3::new(&ctx), x.to_string().parse().unwrap())).collect();
                        init_assert_vec.push(EQZ3::new(&ctx, 
                            IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), &format!("{}_s0", init_name.as_str())),
                            IntZ3::new(&ctx, &IntSortZ3::new(&ctx), x.to_string().parse().unwrap())));
                        let mut int_domain_disj = vec!();
                        
                        for i in domain_asts {
                            // println!("{:?}", ast_to_string_z3!(&ctx, i));
                            int_domain_disj.push(EQZ3::new(&ctx, 
                                IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), &format!("{}_s0", init_name.as_str())), i));
                        }
                        
                        init_assert_vec.push(ORZ3::new(&ctx, int_domain_disj)); 
                        for j in &init_assert_vec {
                            println!("================================");
                            println!("{:?}", ast_to_string_z3!(&ctx, *j));
                        }
                        // ));
                        // init_assert_vec.push(EQZ3::new(&ctx, 
                        //     IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), &format!("{}_s0", init_name.as_str())),
                        //     ORZ3::new(&ctx, domain_asts)));
                    },
                    _ => panic!("Implement"),
                },
                None => (),
            }
        }
        ANDZ3::new(&ctx, init_assert_vec)
    }
}

impl <'ctx> GetSPPredicateZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, step: u32, p: &Predicate) -> Z3_ast {
        let res: Z3_ast = match p {
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
            Predicate::EQ(PredicateValue::SPPath(var, _),
                          PredicateValue::SPValue(value)) => {
                let sub_res: Z3_ast = match value.has_type() {
                    SPValueType::Bool => {
                        let sub_sub_res: Z3_ast = match ts_model.state_predicates.iter().find(|x| var == x.path()) {
                            Some(x) => {
                                if let VariableType::Predicate(pred) = x.variable_type() {
                                    GetSPPredicateZ3::new(&ctx, &ts_model, step, &pred)
                                } else { panic!("can not happen"); }
                            },
                            None => {
                                let sub_sub_sub_res: Z3_ast = match value {
                                    SPValue::Bool(false) => EQZ3::new(&ctx, 
                                        BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                                        BoolZ3::new(&ctx, false)),
                                    SPValue::Bool(true) => EQZ3::new(&ctx, 
                                        BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                                        BoolZ3::new(&ctx, true)),
                                    _ => panic!("Impossible"),
                                };
                                sub_sub_sub_res
                            },
                        };
                        sub_sub_res
                    },
                    SPValueType::String => {
                        let var_vec: Vec<_> = var.path.clone().iter().map(|path|path.clone()).collect();
                        let var_name = var_vec.join("/").to_string();
                        let init_domain_strings = GetSPVarDomain::new(&ts_model, &var_name);     
                        let val_index = init_domain_strings.iter().position(|r| *r == value.to_string()).unwrap();
                        let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());
                        let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                        let elements = &enum_sort.enum_asts;
                        EQZ3::new(&ctx, EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), elements[val_index])
                    },
                    SPValueType::Int32 => {
                        EQZ3::new(&ctx, 
                            IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                            IntZ3::new(&ctx, &IntSortZ3::new(&ctx), value.to_string().parse().unwrap()))
                    },
                    _ => panic!("implement stuff")
                };
                sub_res
            },
            Predicate::EQ(PredicateValue::SPPath(var, _),
                           PredicateValue::SPPath(other, _)) => {

                let init_var_type = &ts_model.vars.iter()
                    .find_map(|x| if x.path() == var { Some(x.value_type()) } else {None} )
                    .expect(&format!("could not find variable (2)"));
                
                let init_other_type = &ts_model.vars.iter()
                    .find_map(|x| if x.path() == other { Some(x.value_type()) } else {None} )
                    .expect(&format!("could not find variable (3)"));
                
                let var_name = var.to_string();
                let other_name = other.to_string();
                let vds = GetSPVarDomain::new(&ts_model, &var_name);
                let ods = GetSPVarDomain::new(&ts_model, &other_name);

                let res: Z3_ast = match init_var_type == init_other_type {
                    true => {
                        let sub_res: Z3_ast = match vds.iter().zip(&ods).filter(|&(a, b)| a == b).count() == vds.len() {
                            true => {
                                let domain_name = format!("{}_sort", vds.join(".").to_string());
                                let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), vds.iter().map(|x| x.as_str()).collect());
                                EQZ3::new(&ctx, 
                                    EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), 
                                    EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", other_name.to_string(), step).as_str()))
                            },
                            false => panic!("Not same domain size"),
                        };
                        sub_res
                    },
                    false => panic!("Trying to assign with different types"),
                };
                res
            },
            Predicate::NEQ(PredicateValue::SPPath(var, _),
                          PredicateValue::SPValue(value)) => {
                let sub_res: Z3_ast = match value.has_type() {
                    SPValueType::Bool => {
                        let sub_sub_res: Z3_ast = match ts_model.state_predicates.iter().find(|x| var == x.path()) {
                            Some(x) => {
                                if let VariableType::Predicate(pred) = x.variable_type() {
                                    GetSPPredicateZ3::new(&ctx, &ts_model, step, &pred)
                                } else { panic!("can not happen"); }
                            },
                            None => {
                                let sub_sub_sub_res: Z3_ast = match value {
                                    SPValue::Bool(false) => NEQZ3::new(&ctx, 
                                        BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                                        BoolZ3::new(&ctx, false)),
                                    SPValue::Bool(true) => NEQZ3::new(&ctx, 
                                        BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                                        BoolZ3::new(&ctx, true)),
                                    _ => panic!("Impossible"),
                                };
                                sub_sub_sub_res
                            },
                        };
                        sub_sub_res
                    },
                   SPValueType::String => {
                        let var_vec: Vec<_> = var.path.clone().iter().map(|path|path.clone()).collect();
                        let var_name = var_vec.join("/").to_string();
                        let init_domain_strings = GetSPVarDomain::new(&ts_model, &var_name);     
                        let val_index = init_domain_strings.iter().position(|r| *r == value.to_string()).unwrap();
                        let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());
                        let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                        let elements = &enum_sort.enum_asts;
                        NEQZ3::new(&ctx, EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), elements[val_index])
                    },
                    SPValueType::Int32 => {
                        NEQZ3::new(&ctx, 
                            IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), format!("{}_s{}", var.to_string(), step).as_str()),
                            IntZ3::new(&ctx, &IntSortZ3::new(&ctx), value.to_string().parse().unwrap()))
                    },
                _ => panic!("implement")
                };
                sub_res
            },
            Predicate::NEQ(PredicateValue::SPPath(var, _),
                           PredicateValue::SPPath(other, _)) => {

                let init_var_type = &ts_model.vars.iter()
                    .find_map(|x| if x.path() == var { Some(x.value_type()) } else {None} )
                    .expect(&format!("could not find variable (2)"));
                
                let init_other_type = &ts_model.vars.iter()
                    .find_map(|x| if x.path() == other { Some(x.value_type()) } else {None} )
                    .expect(&format!("could not find variable (3)"));
                
                let var_name = var.to_string();
                let other_name = other.to_string();
                let vds = GetSPVarDomain::new(&ts_model, &var_name);
                let ods = GetSPVarDomain::new(&ts_model, &other_name);

                let sub_res: Z3_ast = match init_var_type == init_other_type {
                    true => {
                        let sub_sub_res: Z3_ast = match vds.iter().zip(&ods).filter(|&(a, b)| a == b).count() == vds.len() {
                            true => {
                                let domain_name = format!("{}_sort", vds.join(".").to_string());
                                let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), vds.iter().map(|x| x.as_str()).collect());
                                NEQZ3::new(&ctx, 
                                    EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), 
                                    EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", other_name.to_string(), step).as_str()))
                            },
                            false => panic!("Not same domain size"),
                        };
                        sub_sub_res
                    },
                    false => panic!("Trying to assign with different types"),
                };
                sub_res
            },
            _ => panic!("Implement"),
        };
        res
    }

}

impl <'ctx> GetSPUpdatesZ3<'ctx> {
    pub fn new(ctx: &ContextZ3, ts_model: &TransitionSystemModel, t: &Transition, step: u32) -> Z3_ast {
        
        let mut updates = vec!();
        let mut act_and_eff = vec!();
        act_and_eff.extend(t.actions());
        act_and_eff.extend(t.effects());

        for a in act_and_eff {
            let var_name = a.var.to_string();
        
            let init_domain_strings = GetSPVarDomain::new(&ts_model, &var_name);
            let mut vars_string = vec!();
            for var in &ts_model.vars {
                vars_string.push(var.path().to_string());
            }
            
            match &a.value {
                Compute::PredicateValue(pv) => match pv {
                    PredicateValue::SPValue(spval) => match spval.has_type() {
                        SPValueType::Bool => match spval {
                            SPValue::Bool(false) => updates.push(EQZ3::new(&ctx,
                                BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var_name.to_string(), step).as_str()),
                                BoolZ3::new(&ctx, false))),
                            SPValue::Bool(true) => updates.push(EQZ3::new(&ctx,
                                BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), format!("{}_s{}", var_name.to_string(), step).as_str()),
                                BoolZ3::new(&ctx, true))),
                            _ => panic!("Impossible"),
                        },
                        SPValueType::String => {    
                            let val_index = init_domain_strings.iter().position(|r| *r == spval.to_string()).unwrap();
                            let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());
                            let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                            let elements = &enum_sort.enum_asts;
                            updates.push(EQZ3::new(&ctx,
                                EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), 
                                elements[val_index]));
                        },
                        SPValueType::Int32 => { // not sure if this will work...
                            let val_index = init_domain_strings.iter().position(|r| *r == spval.to_string()).unwrap();
                            let domain = GetSPVarDomain::new(&ts_model, &var_name);
                            // let domain_ints: Vec<u32> = domain.iter().map(|x| x.parse().unwrap()).collect();
                            // let domain_asts: Vec<Z3_ast> = domain_ints.iter().map(|x| IntZ3::new(&ctx, &IntSortZ3::new(&ctx), x.to_string().parse().unwrap())).collect();
                            updates.push(EQZ3::new(&ctx, 
                                IntVarZ3::new(&ctx, &IntSortZ3::new(&ctx), format!("{}_s{}", var_name, step).as_str()),
                                IntZ3::new(&ctx, &IntSortZ3::new(&ctx), domain[val_index].to_string().parse().unwrap())));
                        },
                        _ => panic!("Implement"),
                    },
                    PredicateValue::SPPath(path, _) => {
                        let domain_name = format!("{}_sort", init_domain_strings.join(".").to_string());
                        let enum_sort = EnumSortZ3::new(&ctx, domain_name.as_str(), init_domain_strings.iter().map(|x| x.as_str()).collect());
                        updates.push(EQZ3::new(&ctx, 
                            EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", var_name.to_string(), step).as_str()), 
                            EnumVarZ3::new(&ctx, enum_sort.r, format!("{}_s{}", path.to_string(), step).as_str())));
                    },
                },
                Compute::Predicate(p) => {
                    updates.push(GetSPPredicateZ3::new(&ctx, &ts_model, step, &p));
                },
                _ => panic!("Implement")
            };
        }
        ANDZ3::new(&ctx, updates)
    }
}

impl SeqComputePlanSPModelZ3 {
    pub fn plan(model: &TransitionSystemModel,
            goals: &[(Predicate, Option<Predicate>)],
            state: &SPState,
            max_steps: u32) -> PlanningResultZ3 {
    
        let mut step: u32 = 0;
    
        let cfg = ConfigZ3::new();
        let ctx = ContextZ3::new(&cfg);
        let slv = SolverZ3::new(&ctx);
    
        let invariants: Vec<_> = model.specs.iter()
            .map(|s| GetSPPredicateZ3::new(&ctx, &model, 0, s.invariant())).collect();
        for i in invariants {
            slv_assert_z3!(&ctx, &slv, i);
        }

        slv_assert_z3!(&ctx, &slv, GetInitialStateZ3::new(&ctx, &model, &state));

        SlvPushZ3::new(&ctx, &slv);

        for g in goals {
            match &g.1 {
                Some(x) => {
                    slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(x, &g.0), 0, 0));
                    slv_assert_z3!(&ctx, &slv, UntilZ3::new(&ctx, &model, x, &vec!(x, &g.0), 0, 0)); // unnecessary?
                }
                None => {
                    slv_assert_z3!(&ctx, &slv, GetSPPredicateZ3::new(&ctx, &model, 0, &g.0));
                },
            }
        }

        let now = Instant::now();
        let mut plan_found: bool = false;
    
        while step < max_steps + 1 {
            step = step + 1;
            let now2 = Instant::now();
            if SlvCheckZ3::new(&ctx, &slv) != 1 {
                SlvPopZ3::new(&ctx, &slv, 1);
                let step_slv_time = now2.elapsed();
                println!("STEP SOLVING TIME: {:?}", step_slv_time);
                
                let now3 = Instant::now();
                
                let mut all_trans = vec!();
                for t in &model.transitions {
                    let trans_node = t.node().to_string();
                    let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
                    let trans_name: String = format!("{}_t{}", trans_vec[0].to_string(), step);
                    let guard = GetSPPredicateZ3::new(&ctx, &model, step - 1, t.guard());
                    let updates = GetSPUpdatesZ3::new(&ctx, &model, &t, step);
                    let keep_updates = GetSPKeepValueUpdatesZ3::new(&ctx, &model, &t, step);

                    all_trans.push(ANDZ3::new(&ctx, 
                        vec!(EQZ3::new(&ctx, 
                            BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), trans_name.as_str()), 
                            BoolZ3::new(&ctx, true)),
                        guard, keep_updates, updates)));
                    }

                slv_assert_z3!(&ctx, &slv, ORZ3::new(&ctx, all_trans));

                // offline specs for all steps:
                let invariants: Vec<_> = model.specs.iter()
                    .map(|s| GetSPPredicateZ3::new(&ctx, &model, step, &s.invariant())).collect();
                for i in invariants {
                    slv_assert_z3!(&ctx, &slv, i);
                }

                SlvPushZ3::new(&ctx, &slv);
                
                for g in goals {
                    match &g.1 {
                        Some(x) => {
                            slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(&g.0), 0, step));
                            // slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(&g.0), 0, step));
                            slv_assert_z3!(&ctx, &slv, UntilZ3::new(&ctx, &model, x, &vec!(x, &g.0), 0, step)); // unnecessary?
                            },
                        None => {
                            slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(&g.0), 0, step));
                        },
                    }
                }     

                let internal_stuff_time = now3.elapsed();
                println!("INTERNAL TIME: {:?}", internal_stuff_time);

            } else {
                let step_slv_time = now2.elapsed();
                println!("STEP SOLVING TIME: {:?}", step_slv_time);
                plan_found = true;
                break;
            }
        }

        let planning_time = now.elapsed();
        
        if plan_found == true {
            let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
            let result = GetSPPlanningResultZ3::new(&ctx, model, step, planning_time, plan_found);
            result
        } else {
            let model = FreshModelZ3::new(&ctx);
            let result = GetSPPlanningResultZ3::new(&ctx, model, step, planning_time, plan_found);
            result
        }              
    }   
}

impl SubParComputePlanSPModelZ3 {
    pub fn plan(model: &TransitionSystemModel,
            goals: &[(Predicate, Option<Predicate>)],
            state: &SPState,
            for_step: u32) -> PlanningResultZ3 {
    
        let mut step: u32 = 0;
    
        let cfg = ConfigZ3::new();
        let ctx = ContextZ3::new(&cfg);
        let slv = SolverZ3::new(&ctx);
    
        let invariants: Vec<_> = model.specs.iter()
            .map(|s| GetSPPredicateZ3::new(&ctx, &model, 0, s.invariant())).collect();
        for i in invariants {
            slv_assert_z3!(&ctx, &slv, i);
        }

        slv_assert_z3!(&ctx, &slv, GetInitialStateZ3::new(&ctx, &model, &state));

        for g in goals {
            match &g.1 {
                Some(x) => {
                    slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(x, &g.0), 0, for_step));
                    slv_assert_z3!(&ctx, &slv, UntilZ3::new(&ctx, &model, x, &vec!(&g.0), 0, for_step)); // unnecessary?
                    },
                None => {
                    slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(&g.0), 0, for_step));
                },
            }
        }     

        let now = Instant::now();
        let mut plan_found: bool = false;
    
        while step < for_step + 1 {
            step = step + 1;
            println!("{:?}", step);
            let now2 = Instant::now();
            // let step_slv_time = now2.elapsed();
            // println!("STEP SOLVING TIME: {:?}", step_slv_time);
                
            // let now3 = Instant::now();
                
            let mut all_trans = vec!();
            for t in &model.transitions {
                let trans_node = t.node().to_string();
                let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
                let trans_name: String = format!("{}_t{}", trans_vec[0].to_string(), step);
                let guard = GetSPPredicateZ3::new(&ctx, &model, step - 1, t.guard());
                let updates = GetSPUpdatesZ3::new(&ctx, &model, &t, step);
                let keep_updates = GetSPKeepValueUpdatesZ3::new(&ctx, &model, &t, step);

                all_trans.push(ANDZ3::new(&ctx, 
                    vec!(EQZ3::new(&ctx, 
                        BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), trans_name.as_str()), 
                        BoolZ3::new(&ctx, true)),
                    guard, keep_updates, updates)));
                }

            slv_assert_z3!(&ctx, &slv, ORZ3::new(&ctx, all_trans));

            // offline specs for all steps:
            let invariants: Vec<_> = model.specs.iter()
                .map(|s| GetSPPredicateZ3::new(&ctx, &model, step, &s.invariant())).collect();
            for i in invariants {
                slv_assert_z3!(&ctx, &slv, i);
                }
            } 

        if SlvCheckZ3::new(&ctx, &slv) != 1 {
            println!("PLANFOUND");
            let planning_time = now.elapsed();
            plan_found = true;
            let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
            let result = GetSPPlanningResultZ3::new(&ctx, model, step, planning_time, plan_found);
            result
        } else {
            let planning_time = now.elapsed();
            let model = FreshModelZ3::new(&ctx);
            let result = GetSPPlanningResultZ3::new(&ctx, model, step, planning_time, plan_found);
            result
        }          
    }   
}

// impl ParComputePlanSPModelZ3 {
//     pub fn plan(model: &TransitionSystemModel,
//             goals: &[(Predicate, Option<Predicate>)],
//             state: &SPState,
//             max_steps: u32) -> PlanningResultZ3 {
    
//         let (tx, rx) = channel::unbounded();

//         thread::spawn(move || {
//             let mut step: u32 = 0;

//             let cfg = ConfigZ3::new();
//             let ctx = ContextZ3::new(&cfg);
//             let slv = SolverZ3::new(&ctx);

//             slv_assert_z3!(&ctx, &slv, GetInitialStateZ3::new(&ctx, &model, &state));

//             let invariants: Vec<_> = model.specs.iter()
//                 .map(|s| GetSPPredicateZ3::new(&ctx, &model, step, s.invariant())).collect();
//             for i in invariants {
//                 slv_assert_z3!(&ctx, &slv, i);
//                 }

//             SlvPushZ3::new(&ctx, &slv);

//             for g in goals {
//                 match &g.1 {
//                     Some(x) => {
//                         slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(x, &g.0), 0, 0));
//                         slv_assert_z3!(&ctx, &slv, UntilZ3::new(&ctx, &model, x, &g.0, 0, 0)); // unnecessary?
//                     }
//                     None => {
//                         slv_assert_z3!(&ctx, &slv, GetSPPredicateZ3::new(&ctx, &model, 0, &g.0));
//                     },
//                 }
//             }

//             let now = Instant::now();
//             let mut plan_found: bool = false;
            
//             for step in (0..max_steps).step_by(2) {
//                 let now2 = Instant::now();
//                 if SlvCheckZ3::new(&ctx, &slv) != 1 {
//                     SlvPopZ3::new(&ctx, &slv, 1);
//                     let step_slv_time = now2.elapsed();
//                     println!("STEP SOLVING TIME: {:?}", step_slv_time);

//                     let now3 = Instant::now();

//                     let mut all_trans = vec!();
//                     for t in &model.transitions {
//                         let trans_node = t.node().to_string();
//                         let trans_vec: Vec<&str> = trans_node.rsplit(':').collect();
//                         let trans_name: String = format!("{}_t{}", trans_vec[0].to_string(), step + 1);
//                         let guard = GetSPPredicateZ3::new(&ctx, &model, step, t.guard());
//                         let updates = GetSPUpdatesZ3::new(&ctx, &model, &t, step + 1);
//                         let keep_updates = GetSPKeepValueUpdatesZ3::new(&ctx, &model, &t, step + 1);

//                         all_trans.push(ANDZ3::new(&ctx, 
//                             vec!(EQZ3::new(&ctx, 
//                                 BoolVarZ3::new(&ctx, &BoolSortZ3::new(&ctx), trans_name.as_str()), 
//                                 BoolZ3::new(&ctx, true)),
//                             guard, keep_updates, updates)));
//                         }

//                     slv_assert_z3!(&ctx, &slv, ORZ3::new(&ctx, all_trans));

//                     // offline specs for all steps:
//                     let invariants: Vec<_> = model.specs.iter()
//                         .map(|s| GetSPPredicateZ3::new(&ctx, &model, step + 1, &s.invariant())).collect();
//                     for i in invariants {
//                         slv_assert_z3!(&ctx, &slv, i);
//                     }

//                     SlvPushZ3::new(&ctx, &slv);

//                     for g in goals {
//                         match &g.1 {
//                             Some(x) => {
//                                 slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(x, &g.0), 0, step + 1));
//                                 slv_assert_z3!(&ctx, &slv, UntilZ3::new(&ctx, &model, x, &g.0, 0, step + 1)); // unnecessary?
//                                 },
//                             None => {
//                                 slv_assert_z3!(&ctx, &slv, AtLeastOnceZ3::new(&ctx, &model, vec!(&g.0), 0, step + 1));
//                             },
//                         }
//                     }     

//                     let internal_stuff_time = now3.elapsed();
//                     println!("INTERNAL TIME: {:?}", internal_stuff_time);

//                 } else {
//                     let step_slv_time = now2.elapsed();
//                     println!("STEP SOLVING TIME: {:?}", step_slv_time);
//                     plan_found = true;
//                     break;
//                 }
//             }

//             let planning_time = now.elapsed();
        
//             if plan_found == true {
//                 let model = SlvGetModelAndForbidZ3::new(&ctx, &slv);
//                 let result = GetSPPlanningResultZ3::new(&ctx, model, step, planning_time, plan_found);
//                 tx.send(result).unwrap();
//             } else {
//                 let model = FreshModelZ3::new(&ctx);
//                 let result = GetSPPlanningResultZ3::new(&ctx, model, step, planning_time, plan_found);
//                 tx.send(result).unwrap();
//             }
//         });
//         rx.recv().unwrap()
//     }   
// }

impl <'ctx> GetSPPlanningResultZ3<'ctx> {
    pub fn new(ctx: &'ctx ContextZ3, model: Z3_model, nr_steps: u32, 
    planning_time: std::time::Duration, plan_found: bool) -> PlanningResultZ3 {
        let model_str = ModelToStringZ3::new(&ctx, model);
        let mut model_vec = vec!();

        let num = ModelGetNumConstsZ3::new(&ctx, model);
        let mut lines = model_str.lines();
        let mut i: u32 = 0;

        while i < num {
            model_vec.push(lines.next().unwrap_or(""));
            i = i + 1;
        }

        let mut trace: Vec<PlanningFrameZ3> = vec!();
        
        for i in 0..nr_steps {
            let mut frame: PlanningFrameZ3 = PlanningFrameZ3::default();
            for j in &model_vec {
                let sep: Vec<&str> = j.split(" -> ").collect();
                if sep[0].ends_with(&format!("_s{}", i)){
                    let trimmed_state = sep[0].trim_end_matches(&format!("_s{}", i));
                    match sep[1] {
                        "false" => frame.state
                            .add_variable(SPPath::from_string(trimmed_state), false.to_spvalue()),
                        "true" => frame.state
                            .add_variable(SPPath::from_string(trimmed_state), true.to_spvalue()),
                        _ => frame.state
                            .add_variable(SPPath::from_string(trimmed_state), sep[1].to_spvalue()),
                    }
                } else if sep[0].ends_with(&format!("_t{}", i)) && sep[1] == "true" {
                    let trimmed_trans = sep[0].trim_end_matches(&format!("_t{}", i));
                    frame.transition = SPPath::from_string(trimmed_trans);
                }
            }
            trace.push(frame);
        }

        PlanningResultZ3 {
            plan_found: plan_found,
            // plan_length: nr_steps,
            plan_length: nr_steps - 1,
            trace: trace,
            time_to_solve: planning_time,
            raw_output: "".to_string(),
            raw_error_output: "".to_string(),
        }
    }
}