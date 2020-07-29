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
            new_specs.push(Spec::new(s.name(), ri));
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


    // spit out a nuxmv files for debugging.
    let ops: Vec<_> = global_ops.iter().map(|o| (o.path().to_string(), o.guard.clone(), o.goal.goal.clone())).collect();
    crate::planning::generate_offline_nuxvm(&ts_model,&initial);
    crate::planning::generate_offline_nuxvm_ctl(&ts_model, &initial, &ops);

    let ts_model_op = TransitionSystemModel::from_op(&model);
    crate::planning::generate_offline_nuxvm(&ts_model_op, &Predicate::TRUE);

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
