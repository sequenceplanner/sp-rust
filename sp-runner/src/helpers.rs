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
            let ri = refine_invariant(&model, s.invariant());
            new_specs.push(Spec::new(s.name(), ri));
        }
        ts_model.specs = new_specs;
    }

    // spit out a nuxmv file for debugging.
    crate::planning::generate_offline_nuxvm(&ts_model,&initial);

    // runner model has everything from planning model + operations and their global state
    let items = model.items();

    // add global op transitions
    let global_ops: Vec<&Operation> = model.all_operations()
        .into_iter().filter(|o| !o.high_level).collect();

    let global_ops_trans: Vec<_> = global_ops
        .iter()
        .flat_map(|o| o.transitions())
        .cloned()
        .collect();
    let global_ops_ctrl: Vec<_> = global_ops_trans
        .iter()
        // we remove the planning representation of the operation here.
        .filter(|o| o.controlled && o.name() != "planning")
        .cloned()
        .collect();

    let global_ops_un_ctrl: Vec<_> = global_ops_trans
        .iter()
        .filter(|o| !o.controlled)
        .cloned()
        .collect();
    let global_goals: Vec<IfThen> = global_ops
        .iter()
        .flat_map(|o| o.goal().as_ref())
        .cloned()
        .collect();

    let op_states: Vec<Variable> = global_ops
        .iter()
        .map(|o| o.state_variable())
        .cloned()
        .collect();

    let global_hl_ops: Vec<&Operation> = items
        .iter()
        .flat_map(|i| match i {
            SPItem::Operation(o) if o.high_level => Some(o),
            _ => None,
        })
        .collect();

    let global_hl_ops_trans: Vec<_> = global_hl_ops
        .iter()
        .flat_map(|o| o.transitions())
        .cloned()
        .collect();
    let global_hl_ops_ctrl: Vec<_> = global_hl_ops_trans
        .iter()
        .filter(|o| o.controlled)
        .cloned()
        .collect();
    let global_hl_ops_un_ctrl: Vec<_> = global_hl_ops_trans
        .iter()
        .filter(|o| !o.controlled)
        .cloned()
        .collect();
    let global_hl_goals: Vec<IfThen> = global_hl_ops
        .iter()
        .flat_map(|o| o.goal().as_ref())
        .cloned()
        .collect();

    let ts_model_op = TransitionSystemModel::from_op(&model);
    crate::planning::generate_offline_nuxvm(&ts_model_op, &Predicate::TRUE);

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
            un_ctrl: ts_model
                .transitions
                .iter()
                .filter(|t| !t.controlled())
                .cloned()
                .collect(),
        },
        plans: RunnerPlans::default(),
        state_predicates: ts_model.state_predicates.clone(),
        goals: global_goals,
        hl_goals: global_hl_goals,
        model: ts_model.clone(),
        op_model: ts_model_op.clone(),
        op_states,
    };

    return rm;
}
