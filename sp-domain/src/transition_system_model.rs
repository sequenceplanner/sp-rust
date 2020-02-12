/// This module contain a simple model type that we can use for the
/// formal verification stuff.

use super::*;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct TransitionSystemModel {
    pub name: String,
    pub vars: Vec<Variable>,
    pub state_predicates: Vec<Variable>,
    pub transitions: Vec<Transition>,
    pub specs: Vec<Spec>,
}

impl TransitionSystemModel {
    pub fn from(model: &Model) -> Self {
        let mut vars: Vec<Variable> = model.resources()
            .iter()
            .flat_map(|r| r.get_variables())
            .collect();

        let sub_items: Vec<_> = model.resources()
            .iter()
            .flat_map(|r| r.sub_items())
            .collect();

        let sub_item_variables: Vec<Variable> = sub_items
            .iter()
            .flat_map(|i| match i {
                SPItem::Variable(v) => Some(v.clone()),
                _ => None,
            }).collect();

        let mut transitions: Vec<Transition> = model.resources()
            .iter().flat_map(|r| r.get_transitions()).collect();

        // EXPERIMENT:

        // pre-process the transitions to force all auto trans to be
        // taken before the controllable by adding guards to the
        // controllable.

        // if we leave it like this, we need to be sure that
        // there are no cycles among the auto trans
        // the benefit is that planning becomes more predicable.
        // the planner is forced to "switch off" any enabled auto trans,
        // just like the runner does.
        // otoh, planning horizons may increase slightly, as we no longer can
        // ignore "useless" (wrt the goal) auto trans.

        // e.g. we get this plan:
        // drm/r1/move_to/start_with_at
        // drm/r1/move_to/finish
        // drm/r1/move_to/sync_prev       <-- notice this part
        // drm/r1/move_to/start_with_away
        // drm/r2/move_to/start_with_at
        // drm/r2/move_to/finish

        // instead of this plan:
        // drm/r2/move_to/start_with_at
        // drm/r2/move_to/finish
        // drm/r2/move_to/start_with_away   <-- this plan does not care about keeping prev_pos in sync.
        // drm/r1/activate/start
        // drm/r1/activate/finish
        // drm/r1/move_to/start_with_at
        // drm/r1/move_to/finish

        let auto = transitions.iter().filter(|t|!t.controlled() && !t.actions().is_empty());
        let auto_guards: Vec<Predicate> = auto.map(|a|Predicate::NOT(Box::new(a.guard().clone()))).collect();
        let auto_guards = Predicate::AND(auto_guards);
        transitions.iter_mut().for_each(|t| {
            if t.controlled() {
                let orig = t.guard().clone();
                let new = Predicate::AND(vec![orig, auto_guards.clone()]);
                *t.mut_guard() = new;
            }
        });

        let state_predicates: Vec<Variable> = model.resources()
            .iter().flat_map(|r| r.get_state_predicates()).collect();

        vars.extend(sub_item_variables.iter().cloned());

        let mut specs: Vec<Spec> = model.items().iter().flat_map(|i| match i {
            SPItem::Spec(s) => Some(s),
            _ => None,
        }).cloned().collect();
        let sub_item_specs: Vec<Spec> = sub_items.iter().flat_map(|i| match i {
            SPItem::Spec(s) => Some(s.clone()),
            _ => None,
        }).collect();
        specs.extend(sub_item_specs.iter().cloned());

        TransitionSystemModel {
            name: model.name().into(),
            vars,
            state_predicates,
            transitions,
            specs
        }
    }
}
