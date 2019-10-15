use sp_domain::*;
use sp_runner_api::*;

pub fn make_runner_model(model: &Model) -> RunnerModel {
    let items = model.items();

    // find "ab" transitions from resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let trans: Vec<_> = resources.iter().flat_map(|r| r.make_global_transitions()).collect();
    let ctrl = trans.iter().filter(|t|t.controlled()).cloned().collect();
    let unctrl = trans.iter().filter(|t|!t.controlled()).cloned().collect();

    // TODO: add global transitions.

    // add global op transitions (all automatic for now).
    let global_ops: Vec<&Operation> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Operation(o) => Some(o),
            _ => None,
        })
        .collect();

    let global_goals: Vec<IfThen> = global_ops.iter().flat_map(|o|o.goal.as_ref()).cloned().collect();

    // println!("{:?}", global_ops);

    // println!("{:?}", resources);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions::default(),
        ab_transitions: RunnerTransitions {
            ctrl: ctrl,
            un_ctrl: unctrl,
        },
        plans: RunnerPlans::default(),
        state_predicates: Vec::new(),
        goals: global_goals,
        invariants: Vec::new(),
        model: model.clone(), // TODO: borrow?
    };

    return rm;
}

pub fn make_initial_state(model: &Model) -> SPState {
    let items = model.items();

    // find all variables in resources
    let resources: Vec<&Resource> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Resource(r) => Some(r),
            _ => None,
        })
        .collect();

    let mut s = SPState::default();

    for r in &resources {
        s.extend(r.make_initial_state());
    }

    return s;
}
