use crate::formal_model::*;
use sp_domain::*;
use sp_runner_api::*;

// remember to also change in planner
// TODO: make this constant crate-wide and use in more places
const USE_GUARD_EXTRACTION: bool = false;

pub fn make_runner_model(model: &Model) -> RunnerModel {
    // each resource contains a supervisor defining its good states
    let inits: Vec<Predicate> = model
        .all_resources()
        .iter()
        .flat_map(|r| r.sub_items())
        .flat_map(|si| match si {
            SPItem::Spec(s) if s.name() == "supervisor" => Some(s.invariant().clone()),
            _ => None,
        })
        .collect();

    // we need to assume that we are in a state that adheres to the resources
    let initial = Predicate::AND(inits);

    let mut ts_model = TransitionSystemModel::from(&model);

    // TODO:
    // for now we dont do guard extraction because it appears we plan alot faster
    // using just the invariants instead. but we need to make proper measurements
    if USE_GUARD_EXTRACTION {
        let (new_guards, supervisor) = extract_guards(&ts_model, &initial);

        // The specs are converted into guards + a global supervisor
        ts_model.specs.clear();
        ts_model.specs.push(Spec::new("global_supervisor", supervisor));

        // TODO: right now its very cumbersome to update the original Model.
        // but it would be nice if we could.
        update_guards(&mut ts_model, &new_guards);
    } else {
        // we can refine all invariants instead of performing GE
        let mut new_specs = Vec::new();
        for s in &ts_model.specs {
            println!("refining invariant {}", s.path());
            let ri = refine_invariant(&ts_model, s.invariant());
            let mut ns = s.clone();
            ns.invariant = ri;
            new_specs.push(ns);
        }
        ts_model.specs = new_specs;
    }

    // runner model has everything from planning model + operations and their global state

    // add global op transitions
    let global_ops: Vec<&Operation> = model.all_operations();

    let global_ops_ctrl: Vec<_> = global_ops.iter().map(|o| o.runner_start.clone()).collect();
    let global_ops_un_ctrl: Vec<_> = global_ops.iter().map(|o| o.runner_finish.clone()).collect();
    let global_op_goals: Vec<IfThen> = global_ops.iter().map(|o| o.goal.clone()).collect();
    let op_states: Vec<Variable> = global_ops.iter().map(|o| o.state_variable()).cloned().collect();

    // unchanged. todo
    let global_intentions: Vec<&Intention> = model.all_intentions();
    let global_int_trans: Vec<_> = global_intentions
        .iter()
        .flat_map(|i| i.transitions())
        .cloned()
        .collect();
    let global_hl_ops_ctrl: Vec<_> = global_int_trans
        .iter()
        .filter(|t| t.controlled)
        .cloned()
        .collect();
    let global_hl_ops_un_ctrl: Vec<_> = global_int_trans
        .iter()
        .filter(|t| !t.controlled)
        .cloned()
        .collect();
    let global_hl_goals: Vec<IfThen> = global_intentions
        .iter()
        .flat_map(|o| o.goal().as_ref())
        .cloned()
        .collect();

    let hl_op_states: Vec<Variable> = global_intentions
        .iter()
        .map(|o| o.state_variable())
        .cloned()
        .collect();


    // debug high level model
    let ts_model_op = TransitionSystemModel::from_op(&model);
    crate::planning::generate_offline_nuxvm(&ts_model_op, &Predicate::TRUE);


    // debug low level model
    let all_op_names = global_ops.iter().map(|o|o.name().to_string()).collect::<Vec<_>>();
    global_ops.iter().for_each(|o| {
        // here we should find all unctronllable actions that can
        // modify the variables of GUARD and disallow them.
        println!("CHECKING OP: {}", o.name());
        let mut temp_ts_model = ts_model.clone();

        // remove "replan" spec
        temp_ts_model.specs.retain(|s| s.path().parent().leaf() != "replan_specs");

        let mut new_invariants = vec![];
        temp_ts_model.transitions.retain(|t| {
            //if t.actions.len() > 0 && t.actions.iter().any(|a| no_change.contains(&a.var)) {
            if t.name() != o.name() && all_op_names.contains(&t.name().to_string()) {
                if let Some(op) = global_ops.iter().find(|oo|t.name()==oo.name()) {
                    println!("FOR OP: {}, filtering transition: {}", o.name(), t.path());
                    if !t.controlled() && op.fvg != Predicate::TRUE {
                        // this also means we need to forbid this state!
                        new_invariants.push(op.fvg.clone());
                    }
                }
                false
            } else { true }
        });

        let new_specs = new_invariants.iter().map(|p| {
            let i = Predicate::NOT(Box::new(p.clone()));
            println!("REFINING: {}", i);
            let ri = refine_invariant(&temp_ts_model, &i);
            Spec::new("extended", ri)
        }).collect::<Vec<_>>();
        temp_ts_model.specs.extend(new_specs);

        // here we check if the operation has an assertion
        // which is assumed to be fulfilled for nominal behavior
        // (eg "resource not in failure mode")
        let guard = if let Some(c) = o.fvc.as_ref() {
            Predicate::AND(vec![o.guard.clone(), c.clone()])
        } else {
            o.guard.clone()
        };
        let op = vec![(o.path().to_string(), guard, o.fvg.clone())];
        temp_ts_model.name += &format!("_{}", o.name());
        crate::planning::generate_offline_nuxvm_ctl(&temp_ts_model, &initial, &op);
    });

    let mut un_ctrl: Vec<Transition> = ts_model
        .transitions
        .iter()
        .filter(|t| !t.controlled())
        .cloned()
        .collect();

    let runner_transitions: Vec<Transition> = model.find_item("runner_transitions",&[])
        .and_then(|m|
                  m.as_model()
                  .map(|m| m
                       .items().iter().flat_map(|i| match i {
                           SPItem::Transition(t) => Some(t.clone()),
                           _ => None,
                       })
                       .collect())).unwrap_or(vec![]);
    un_ctrl.extend(runner_transitions);

    let rm = RunnerModel {
        hl_op_transitions: RunnerTransitions {
            ctrl: global_hl_ops_ctrl,
            un_ctrl: global_hl_ops_un_ctrl,
        },
        op_transitions: RunnerTransitions {
            ctrl: global_ops_ctrl,
            un_ctrl: global_ops_un_ctrl,
        },
        ab_transitions: RunnerTransitions {
            ctrl: ts_model
                .transitions
                .iter()
                .filter(|t| t.controlled())
                .cloned()
                .collect(),
            un_ctrl,
        },
        plans: RunnerPlans::default(),
        state_predicates: ts_model.state_predicates.clone(),
        goals: global_op_goals,
        hl_goals: global_hl_goals,
        model: ts_model.clone(),
        op_model: ts_model_op.clone(),
        op_states,
        hl_op_states,
    };

    return rm;
}
