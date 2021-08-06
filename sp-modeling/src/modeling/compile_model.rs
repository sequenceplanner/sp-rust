use sp_domain::*;
use super::*;
use rayon::prelude::*;



pub fn compile_model(model: &Model, initial_state: SPState, generate_mc_problems: bool) {
    let mut ts_model = TransitionSystemModel::from(&model);

    // refine invariants
    println!("refining model invariants");
    let tsm = ts_model.clone();
    ts_model.specs.par_iter_mut().for_each(|s| {
        s.invariant = refine_invariant(tsm.clone(), s.invariant.clone())
            .expect("crash in refine sp-fm");
        println!("spec done...");
    });
    println!("refining invariants done");

    // add runner transitions
    let runner_transitions = model.all_runner_transitions();

    // add global op transitions
    let global_ops: Vec<&Operation> = model.all_operations();
    let global_ops_trans: Vec<Transition> = global_ops
        .iter()
        .map(|o| o.make_runner_transitions())
        .flatten()
        .collect();

    let global_ops_ctrl: Vec<_> = global_ops_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Controlled)
        .cloned()
        .collect();
    let global_ops_un_ctrl: Vec<_> = global_ops_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Auto)
        .cloned()
        .collect();

    let operations: Vec<Operation> = model.all_operations().into_iter().cloned().collect();

    // unchanged. todo
    let global_intentions: Vec<&Intention> = model.all_intentions();
    let global_int_trans: Vec<_> = global_intentions
        .iter()
        .map(|i| i.make_runner_transitions())
        .flatten()
        .collect();
    let global_int_ctrl: Vec<_> = global_int_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Controlled)
        .cloned()
        .collect();
    let global_int_un_ctrl: Vec<_> = global_int_trans
        .iter()
        .filter(|t| t.type_ == TransitionType::Auto)
        .cloned()
        .collect();
    let global_hl_goals: Vec<IfThen> = global_intentions.iter().map(|o| o.make_goal()).collect();

    // debug high level model

    let ts_model_op = TransitionSystemModel::from_op(&model);

    if generate_mc_problems {
        crate::planning::generate_offline_nuxvm(&ts_model_op, &Predicate::TRUE);
        crate::planning::generate_offline_nuxvm(&ts_model, &Predicate::TRUE);

        // debug low level model
        let all_op_trans = global_ops
            .iter()
            .map(|o| (o.clone(), o.make_lowlevel_transitions()))
            .collect::<Vec<_>>();
        println!("refining operation forbidden specs");
        global_ops.iter().for_each(|o| {
            // check if a "real" operation or really just an autotransition
            if o.make_runner_transitions().is_empty() {
                return;
            }
            // here we should find all unctronllable actions that can
            // modify the variables of GUARD and disallow them.
            println!("CHECKING OP: {}", o.name());
            let mut temp_ts_model = ts_model.clone();

            let mut new_invariants = vec![];
            temp_ts_model.transitions.retain(|t| {
                let belongs_to_other_op = all_op_trans.iter().find(|(op, ts)| {
                    o.path() != op.path() && ts.iter().any(|x| x.path() == t.path())
                });

                if let Some((op, _)) = belongs_to_other_op {
                    if op.make_runner_transitions().is_empty() {
                        // "auto transition operation", keep this
                        return true;
                    }
                    // println!("FOR OP: {}, filtering transition: {}", op.path(), t.path());
                    if t.type_ == TransitionType::Auto {
                        // this also means we need to forbid this state!
                        let opg = op.make_verification_goal();
                        // println!("FOR OP: {}, forbidding: {}", op.path(), opg);
                        new_invariants.push((op.path(), opg));
                    }
                    false
                } else {
                    true
                }
            });

            let new_specs = new_invariants
                .par_iter()
                .map(|(op, p)| {
                    let i = Predicate::NOT(Box::new(p.clone()));
                    // println!("REFINING: {}", i);
                    let ri = refine_invariant(temp_ts_model.clone(), i)
                        .expect("crash in refine sp-fm");
                    println!("spec done...");
                    let mut s = Spec::new("extended", ri);
                    s.node_mut().update_path(op);
                    s
                })
                .collect::<Vec<_>>();
            temp_ts_model.specs.extend(new_specs);

            // here we check if the operation has an assertion
            // which is assumed to be fulfilled for nominal behavior
            // (eg "resource not in failure mode")
            let guard = if let Some(c) = o.mc_constraint.as_ref() {
                Predicate::AND(vec![o.guard.clone(), c.clone()])
            } else {
                o.guard.clone()
            };
            let op = vec![(o.path().to_string(), guard, o.make_verification_goal())];
            temp_ts_model.name += &format!("_{}", o.name());
            crate::planning::generate_offline_nuxvm_ctl(&temp_ts_model, &Predicate::TRUE, &op);
        });
        println!("refining operation forbidden specs done");
    }

    // old runner model as code here.
    let rm_ab_transitions_ctrl: Vec<Transition> = ts_model
        .transitions
        .iter()
        .filter(|t| t.type_ == TransitionType::Controlled)
        .cloned()
        .collect();

    let rm_ab_transitions_un_ctrl: Vec<Transition> = ts_model
        .transitions
        .iter()
        .filter(|t| t.type_ == TransitionType::Auto || t.type_ == TransitionType::Runner)
        .cloned()
        .collect();

    let mut trans = vec![];
    trans.extend(runner_transitions);
    let mut restrict_controllable = vec![];
    let mut restrict_op_controllable = vec![];
    let false_trans = Transition::new(
        "empty",
        Predicate::FALSE,
        vec![],
        TransitionType::Controlled,
    );
    global_ops_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_op_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    global_ops_un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_op_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });

    // intentions are never restricted
    global_int_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });
    global_int_un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    rm_ab_transitions_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
        restrict_controllable.push(TransitionSpec::new(
            &format!("s_{}_false", t.path()),
            false_trans.clone(),
            vec![t.path().clone()],
        ))
    });
    rm_ab_transitions_un_ctrl.iter().for_each(|t| {
        trans.push(t.clone());
    });

    let intentions: Vec<SPPath> = global_intentions.iter().map(|v| v.path().clone()).collect();

    let mut all_vars = ts_model.vars.clone();
    all_vars.extend(ts_model.state_predicates.iter().cloned());

    // add runner variables.
    // TODO: also look in resources/sub-models
    let runner_vars: Vec<Variable> = model
        .items()
        .iter()
        .flat_map(|i| match i {
            SPItem::Variable(s) if s.type_ == VariableType::Runner => Some(s.clone()),
            _ => None,
        })
        .collect();
    all_vars.extend(runner_vars.iter().cloned());

    println!("refining replan specs");
    let replan_specs: Vec<Spec> = operations
        .par_iter()
        .map(|o| {
            let mut s = o.make_replan_specs();
            for mut s in &mut s {
                // Hmm this probably does not belong here...
                s.invariant = refine_invariant(ts_model.clone(), s.invariant.clone())
                    .expect("crash in refine sp-fm");
                println!("spec done...");
            }
            s
        })
        .flatten()
        .collect();
    println!("refining replan specs done");

    /* 
    runner.update_state_variables(initial_state);

    // planning active or not
    let planner0 = SPPath::from_slice(&["runner", "planner", "0"]);
    let planner1 = SPPath::from_slice(&["runner", "planner", "1"]);
    let planners_initially_on =
        SPState::new_from_values(&[(planner0, true.to_spvalue()), (planner1, true.to_spvalue())]);
    runner.update_state_variables(planners_initially_on);

    // monotonic planning mode
    let mono = SPPath::from_slice(&["runner", "planner", "monotonic"]);
    let monotonic_initially_on = SPState::new_from_values(&[(mono, true.to_spvalue())]);
    runner.update_state_variables(monotonic_initially_on);

    // experiment with timeout on effects...
    ts_model.transitions.iter().for_each(|t| {
        if t.type_ == TransitionType::Effect {
            let path = t.path().add_parent("effects");
            let effect_enabled = SPState::new_from_values(&[(path, true.to_spvalue())]);
            runner.update_state_variables(effect_enabled);
        }
    });

    */



}