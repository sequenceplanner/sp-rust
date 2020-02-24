use sp_domain::*;
use crate::planning::*;
use crate::formal_model::*;
use guard_extraction::*;
use cryptominisat::*;

pub struct SatPlanner {}

// make a conjunction out of a state
fn state_to_predicate(vars: &[Variable], state: &SPState) -> Predicate {
    let s = state.projection();
    let eqs = s.state
        .into_iter()
        .filter(|(p,_v)| vars.iter().any(|x| &x.path() == p))
        .map(|(p,v)|
             Predicate::EQ(PredicateValue::SPPath(p.clone(), None),
                           PredicateValue::SPValue(v.value().clone())))
        .collect();

    Predicate::AND(eqs)
}

impl Planner for SatPlanner {
    fn plan(model: &TransitionSystemModel,
            goals: &[(Predicate, Option<Predicate>)],
            state: &SPState,
            max_steps: u32) -> PlanningResult {
        let c = FormalContext::from(&model);

        let initial = state_to_predicate(&model.vars, state);
        let initial = c.sp_pred_to_ex(&initial);
        let goals: Vec<(Ex,Ex)> = goals
            .iter()
            .map(|(i,g)|
                 (c.sp_pred_to_ex(&i),
                  c.sp_pred_to_ex(&g.as_ref().unwrap_or(&Predicate::TRUE)))
            ).collect();

        let sat_model = c.context.model_as_sat_model(&initial, &goals);

        // println!("{:?}", sat_model);
        println!("start planning");

        let now = std::time::Instant::now();

        let mut s = Solver::new();

        type GLit = guard_extraction::Lit;
        type CLit = cryptominisat::Lit;

        // s.set_num_threads(1); // play with this some day

        let vars_per_step = sat_model.num_aux_vars
            + sat_model.norm_vars.len()
            + sat_model.trans_map.len()
            + 1; // goal activation literal

        let mut vars: Vec<CLit> = (0 .. vars_per_step).map(|_v| s.new_var()).collect();

        let pairing: Vec<_> = sat_model.norm_vars
            .iter()
            .zip(sat_model.next_vars.iter())
            .map(|(x, y)| (*x, *y))
            .collect();

        let map_lit = |l: &GLit| {
            if sat_model.norm_vars.contains(&l.var) {
                sat_model.norm_vars.iter().position(|&r| r == l.var).unwrap()
            } else if sat_model.next_vars.contains(&l.var) {
                panic!("dont use");
            } else if sat_model.trans_map.values().any(|&val| val == l.var as i32) {
                l.var - sat_model.next_vars.len()
            } else {
                l.var - sat_model.next_vars.len()
            }
        };

        let map_at_step = |vars: &Vec<CLit>, l: &GLit, step: u32| {
            let lit = vars[map_lit(l) + (step as usize)*vars_per_step];
            if l.neg { !lit } else { lit }
        };

        let is_next = |l: &GLit| {
            sat_model.next_vars.contains(&l.var)
        };

        // add clauses for the initial state
        for c in &sat_model.init_clauses {
            let clause: Vec<CLit> = c.0.iter().map(|l| {
                map_at_step(&vars, l, 0)
            }
            ).collect();
            s.add_clause(&clause);
        }

        // unroll transitions, goals, and later invariants up to n steps
        let mut steps = -1i32;
        for step in 0..max_steps {
            let now = std::time::Instant::now();

            let goal_active = vars[vars.len()-1];

            // in all steps, add all global invariants.
            for c in &sat_model.global_invariants {
                let clause: Vec<CLit> = c.0.iter().map(|l| {
                    map_at_step(&vars, l, step)
                }).collect();
                s.add_clause(&clause);
            }

            if step > 0 {
                for c in &sat_model.trans_clauses {
                    // transition relation clauses has cur and next. let cur refer to previous step.
                    let clause: Vec<CLit> = c.0.iter().map(|l| {
                        if is_next(&l) {
                            let mut l = l.clone();
                            let i = pairing.iter().find(|(_idx,jdx)| &l.var == jdx).unwrap().0;
                            l.var = i;
                            map_at_step(&vars, &l, step)
                        } else {
                            map_at_step(&vars, l, step - 1)
                        }
                    }).collect();
                    s.add_clause(&clause);
                }

                // this actually resulted in worse performance when solving.
                // make a clause for the disjunction of the transitions.
                // let clause: Vec<CLit> = sat_model.trans_map.values().map(|l| {
                //     if is_norm(&l) {
                //         panic!("bad");
                //         if l.neg { !vars[ci(l.var)+(step-1)*nv] } else { vars[ci(l.var)+(step-1)*nv] }
                //     } else if is_next(&l) {
                //         panic!("bad21");
                //         let i = pairing.iter().find(|(_idx,jdx)| &l.var == jdx).unwrap().0;
                //         if l.neg { !vars[ci(i)+step*nv] } else { vars[ci(i)+step*nv] }
                //     } else {
                //         if l.neg { !vars[ti(l.var)+(step-1)*nv] } else { vars[ti(l.var)+(step-1)*nv] }
                //     }
                // }).collect();
                // s.add_clause(&clause);
            }

            // add new goals
            for c in &sat_model.goal_clauses {
                let clause: Vec<CLit> = c.0.iter().map(|l| {
                    map_at_step(&vars, l,step)
                }).collect();

                s.add_clause(&clause);
            }

            // add all invar clauses
            for c in &sat_model.invar_clauses {
                let clause: Vec<CLit> = c.0.iter().map(|l| {
                    map_at_step(&vars, l,step)
                }).collect();

                s.add_clause(&clause);
            }

            // add a disjunction that allows the goal to be active in any time step
            // (so we can finish one goal early but then keep going)
            // for top in &sat_model.goal_tops {
            for (invar, top) in sat_model.invar_tops.iter().zip(sat_model.goal_tops.iter()) {
                let mut clause: Vec<CLit> = Vec::new();
                for i in 0..(step+1) {
                    let l = map_at_step(&vars, top, i);
                    clause.push(l);
                }

                // here we force only TOP to be true. which should propagate down...
                let ga = goal_active;
                // println!("goal clause: {:?}, activation: {:?}", clause, ga);
                clause.push(ga);
                s.add_clause(&clause);

                // we also need to make sure our invariants hold for all timesteps
                // until the goal holds.
                // ie. (i0 | g0) & (i1 | g0 | g1) & (i2 | g0 | g1 | g2) ...

                // this is the same semantics as the LTL UNTIL operator.


                for i in 0..step {
                    let mut clause: Vec<CLit> = Vec::new();

                    let l = map_at_step(&vars, invar, i);
                    clause.push(l);

                    for j in 0..(i+1) {
                        let l = map_at_step(&vars, top, j);
                        clause.push(l);
                    }

                    s.add_clause(&clause);
                }
            }

            let ga = !goal_active;
            let res = s.solve_with_assumptions(&[ga]);

            // println!("res is {:?}", res);


            let reached_goal = res == Lbool::True;

            println!("Step {} computed in: {}ms\n", step, now.elapsed().as_millis());
            if reached_goal {
                println!("Found plan after {} steps", step+1);
                steps = step as i32 + 1;
                // println!("{:#?}", s.get_model());


                // print all transition names.
                for i in 0..step {
                    for (n, l) in &sat_model.trans_map {
                        let v = map_at_step(&vars, &GLit { var: *l as usize, neg: false }, i);
                        if s.is_true(v) {
                            println!("{}: {}", i, n);
                        }
                    }
                    println!("------------------");

                    let mut clause = Vec::new();
                    for v in &sat_model.norm_vars {
                        let mv = map_at_step(&vars, &GLit { var: *v as usize, neg: false }, i);
                        if s.is_true(mv) {
                            clause.push(GLit { var: *v as usize, neg: false });
                        } else {
                            clause.push(GLit { var: *v as usize, neg: true });
                        }
                    }
                    let vals = c.context.sat_result_to_values(&Clause(clause));
                    let state = c.sat_clause_to_spstate(&vals);
                    println!("{}", state);
                }

                break;
            } else {
                let new_vars: Vec<CLit> = (0 .. vars_per_step).map(|_v| s.new_var()).collect();

                vars.extend(new_vars.iter());

            }
        }

        println!("Plan computed in: {}ms\n", now.elapsed().as_millis());

        //return steps; // for now :)
        return PlanningResult::default()

    }
}

pub fn plan(model: &TransitionSystemModel, initial: &Predicate, goals: &[(Predicate,Predicate)]) -> i32 {
    let c = FormalContext::from(&model);

    let initial = c.sp_pred_to_ex(&initial);
    let goals: Vec<(Ex,Ex)> = goals
        .iter()
        .map(|(i,g)|
             (c.sp_pred_to_ex(&i),
              c.sp_pred_to_ex(&g))
             ).collect();

    let sat_model = c.context.model_as_sat_model(&initial, &goals);

    // println!("{:?}", sat_model);
    println!("start planning");

    let now = std::time::Instant::now();

    let mut s = Solver::new();

    type GLit = guard_extraction::Lit;
    type CLit = cryptominisat::Lit;

    // s.set_num_threads(1); // play with this some day

    let vars_per_step = sat_model.num_aux_vars
        + sat_model.norm_vars.len()
        + sat_model.trans_map.len()
        + 1; // goal activation literal

    let mut vars: Vec<CLit> = (0 .. vars_per_step).map(|_v| s.new_var()).collect();

    let pairing: Vec<_> = sat_model.norm_vars
        .iter()
        .zip(sat_model.next_vars.iter())
        .map(|(x, y)| (*x, *y))
        .collect();

    let map_lit = |l: &GLit| {
        if sat_model.norm_vars.contains(&l.var) {
            sat_model.norm_vars.iter().position(|&r| r == l.var).unwrap()
        } else if sat_model.next_vars.contains(&l.var) {
            panic!("dont use");
        } else if sat_model.trans_map.values().any(|&val| val == l.var as i32) {
            l.var - sat_model.next_vars.len()
        } else {
            l.var - sat_model.next_vars.len()
        }
    };

    let map_at_step = |vars: &Vec<CLit>, l: &GLit, step: usize| {
        let lit = vars[map_lit(l) + step*vars_per_step];
        if l.neg { !lit } else { lit }
    };

    let is_next = |l: &GLit| {
        sat_model.next_vars.contains(&l.var)
    };

    // add clauses for the initial state
    for c in &sat_model.init_clauses {
        let clause: Vec<CLit> = c.0.iter().map(|l| {
            map_at_step(&vars, l, 0)
        }
        ).collect();
        s.add_clause(&clause);
    }

    // unroll transitions, goals, and later invariants up to n steps
    let mut steps = -1i32;
    for step in 0..20 {
        let now = std::time::Instant::now();

        let goal_active = vars[vars.len()-1];

        // in all steps, add all global invariants.
        for c in &sat_model.global_invariants {
            let clause: Vec<CLit> = c.0.iter().map(|l| {
                map_at_step(&vars, l, step)
            }).collect();
            s.add_clause(&clause);
        }

        if step > 0 {
            for c in &sat_model.trans_clauses {
                // transition relation clauses has cur and next. let cur refer to previous step.
                let clause: Vec<CLit> = c.0.iter().map(|l| {
                    if is_next(&l) {
                        let mut l = l.clone();
                        let i = pairing.iter().find(|(_idx,jdx)| &l.var == jdx).unwrap().0;
                        l.var = i;
                        map_at_step(&vars, &l, step)
                    } else {
                        map_at_step(&vars, l, step - 1)
                    }
                }).collect();
                s.add_clause(&clause);
            }

            // this actually resulted in worse performance when solving.
            // make a clause for the disjunction of the transitions.
            // let clause: Vec<CLit> = sat_model.trans_map.values().map(|l| {
            //     if is_norm(&l) {
            //         panic!("bad");
            //         if l.neg { !vars[ci(l.var)+(step-1)*nv] } else { vars[ci(l.var)+(step-1)*nv] }
            //     } else if is_next(&l) {
            //         panic!("bad21");
            //         let i = pairing.iter().find(|(_idx,jdx)| &l.var == jdx).unwrap().0;
            //         if l.neg { !vars[ci(i)+step*nv] } else { vars[ci(i)+step*nv] }
            //     } else {
            //         if l.neg { !vars[ti(l.var)+(step-1)*nv] } else { vars[ti(l.var)+(step-1)*nv] }
            //     }
            // }).collect();
            // s.add_clause(&clause);
        }

        // add new goals
        for c in &sat_model.goal_clauses {
            let clause: Vec<CLit> = c.0.iter().map(|l| {
                map_at_step(&vars, l,step)
            }).collect();

            s.add_clause(&clause);
        }

        // add all invar clauses
        for c in &sat_model.invar_clauses {
            let clause: Vec<CLit> = c.0.iter().map(|l| {
                map_at_step(&vars, l,step)
            }).collect();

            s.add_clause(&clause);
        }

        // add a disjunction that allows the goal to be active in any time step
        // (so we can finish one goal early but then keep going)
        // for top in &sat_model.goal_tops {
        for (invar, top) in sat_model.invar_tops.iter().zip(sat_model.goal_tops.iter()) {
            let mut clause: Vec<CLit> = Vec::new();
            for i in 0..(step+1) {
                let l = map_at_step(&vars, top, i);
                clause.push(l);
            }

            // here we force only TOP to be true. which should propagate down...
            let ga = goal_active;
            // println!("goal clause: {:?}, activation: {:?}", clause, ga);
            clause.push(ga);
            s.add_clause(&clause);

            // we also need to make sure our invariants hold for all timesteps
            // until the goal holds.
            // ie. (i0 | g0) & (i1 | g0 | g1) & (i2 | g0 | g1 | g2) ...

            // this is the same semantics as the LTL UNTIL operator.


            for i in 0..step {
                let i = i as usize;
                let mut clause: Vec<CLit> = Vec::new();

                let l = map_at_step(&vars, invar, i);
                clause.push(l);

                for j in 0..(i+1) {
                    let l = map_at_step(&vars, top, j);
                    clause.push(l);
                }

                s.add_clause(&clause);
            }
        }

        let ga = !goal_active;
        let res = s.solve_with_assumptions(&[ga]);

        // println!("res is {:?}", res);


        let reached_goal = res == Lbool::True;

        println!("Step {} computed in: {}ms\n", step, now.elapsed().as_millis());
        if reached_goal {
            println!("Found plan after {} steps", step+1);
            steps = step as i32 + 1;
            // println!("{:#?}", s.get_model());


            // print all transition names.
            for i in 0..step {
                for (n, l) in &sat_model.trans_map {
                    let v = map_at_step(&vars, &GLit { var: *l as usize, neg: false }, i);
                    if s.is_true(v) {
                        println!("{}: {}", i, n);
                    }
                }
                println!("------------------");

                let mut clause = Vec::new();
                for v in &sat_model.norm_vars {
                    let mv = map_at_step(&vars, &GLit { var: *v as usize, neg: false }, i);
                    if s.is_true(mv) {
                        clause.push(GLit { var: *v as usize, neg: false });
                    } else {
                        clause.push(GLit { var: *v as usize, neg: true });
                    }
                }
                let vals = c.context.sat_result_to_values(&Clause(clause));
                let state = c.sat_clause_to_spstate(&vals);
                println!("{}", state);
            }

            break;
        } else {
            let new_vars: Vec<CLit> = (0 .. vars_per_step).map(|_v| s.new_var()).collect();

            vars.extend(new_vars.iter());

        }
    }

    println!("Plan computed in: {}ms\n", now.elapsed().as_millis());

    return steps; // for now :)
}
