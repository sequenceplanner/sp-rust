use sp_domain::*;
use sp_runner_api::*;
use crate::formal_model::*;

pub fn make_runner_model(model: &Model) -> RunnerModel {
    // each resource contains a supervisor defining its good states
    let inits: Vec<Predicate> = model.resources().iter().flat_map(|r| r.sub_items())
        .flat_map(|si| match si {
            SPItem::Spec(s) if s.name() == "supervisor" => Some(s.invariant().clone()),
            _ => None
        }).collect();

    // we need to assume that we are in a state that adheres to the resources
    let initial = Predicate::AND(inits);

    let mut ts_model = TransitionSystemModel::from(&model);

    let (new_guards, supervisor) = extract_guards(&ts_model, &initial);

    // The specs are converted into guards + a global supervisor
    ts_model.specs.clear();
    ts_model.specs.push(Spec::new("global_supervisor", supervisor));

    // TODO: right now its very cumbersome to update the original Model.
    // but it would be nice if we could.
    update_guards(&mut ts_model, &new_guards);

    // runner model has everything from planning model + operations and their global state
    let items = model.items();

    // add global op transitions (all automatic for now).
    let global_ops: Vec<&Operation> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Operation(o) => Some(o),
            _ => None,
        })
        .collect();

    let global_ops_trans:Vec<_> = global_ops.iter().flat_map(|o|o.transitinos()).cloned().collect();
    let global_ops_ctrl: Vec<_> = global_ops_trans.iter().filter(|o|o.controlled).cloned().collect();
    let global_ops_un_ctrl: Vec<_> = global_ops_trans.iter().filter(|o|!o.controlled).cloned().collect();
    let global_goals: Vec<IfThen> = global_ops.iter().flat_map(|o|o.goal().as_ref()).cloned().collect();

    let rm = RunnerModel {
        op_transitions: RunnerTransitions {
            ctrl: global_ops_ctrl,
            un_ctrl: global_ops_un_ctrl,
        },
        ab_transitions: RunnerTransitions {
            ctrl: ts_model.transitions.iter().filter(|t|t.controlled()).cloned().collect(),
            un_ctrl: ts_model.transitions.iter().filter(|t|!t.controlled()).cloned().collect(),
        },
        plans: RunnerPlans::default(),
        state_predicates: ts_model.state_predicates.clone(),
        goals: global_goals,
        model: ts_model.clone(), // TODO: borrow?
    };

    return rm;
}
