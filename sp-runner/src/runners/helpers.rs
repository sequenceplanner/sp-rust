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
    let ab_ctrl = trans.iter().filter(|t|t.controlled()).cloned().collect();
    let ab_un_ctrl = trans.iter().filter(|t|!t.controlled()).cloned().collect();

    let preds: Vec<_> = resources.iter().flat_map(|r| r.make_global_state_predicates()).collect();

    // TODO: handle resource "sub items"

    // TODO: add global transitions.


    // TODO: add global state.?

    // add global op transitions (all automatic for now).
    let global_ops: Vec<&Operation> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Operation(o) => Some(o),
            _ => None,
        })
        .collect();

    let global_ops_ctrl: Vec<_> = global_ops.iter().flat_map(|o|o.start()).cloned().collect();
    let global_ops_un_ctrl: Vec<_> = global_ops.iter().flat_map(|o|o.finish()).cloned().collect();

    let global_goals: Vec<IfThen> = global_ops.iter().flat_map(|o|o.goal().as_ref()).cloned().collect();

    // println!("{:?}", global_ops);

    // println!("{:?}", resources);

    let rm = RunnerModel {
        op_transitions: RunnerTransitions {
            ctrl: global_ops_ctrl,
            un_ctrl: global_ops_un_ctrl,
        },
        ab_transitions: RunnerTransitions {
            ctrl: ab_ctrl,
            un_ctrl: ab_un_ctrl,
        },
        plans: RunnerPlans::default(),
        state_predicates: preds,
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

    // add global state
    let vars: Vec<&Variable> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Variable(v) => Some(v),
            _ => None,
        })
        .collect();

    for v in &vars {
        let _r = s.insert(&v.node().global_path().as_ref().unwrap().to_sp(),
                          AssignStateValue::SPValue(v.initial_value()));
    }

    return s;
}
