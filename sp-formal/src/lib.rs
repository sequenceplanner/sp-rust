mod sp_fm_api;
use sp_fm_api::*;
pub mod planning;

use sp_domain::*;
use rayon::prelude::*;
use serde::{Serialize, Deserialize};

/// A "compiled" model that includes specifications generated by formal methods.
#[derive(Debug, PartialEq, Clone, Default, Serialize, Deserialize)]
pub struct CompiledModel {
    pub model: Model,
    pub computed_transition_planner_invariants: Vec<Specification>,
    pub computed_operation_planner_invariants: Vec<Specification>,
}

fn refine_transition_invariant(invariant: &Specification, ts_model: &TransitionSystemModel) -> Specification {
    let orig_name = invariant.path().leaf();
    let new_name = format!("{}_refined", orig_name);
    let new_path = invariant.path().parent().add_child(&new_name);
    let new_predicate = refine_invariant(ts_model.clone(), invariant.invariant().clone()).expect("crash in refine sp-fm");
    return Specification {
        path: new_path,
        type_: SpecificationType::TransitionInvariant(new_predicate)
    };
}

impl CompiledModel {
    pub fn from(model: Model) -> Self {

        // Compute additional transition invariants

        println!("refining model invariants");
        let ts_model = TransitionSystemModel::from(&model);

        let mut new_invariants = ts_model.invariants.clone();
        new_invariants.par_iter_mut().for_each(|i| {
            *i = refine_transition_invariant(i, &ts_model);
            println!("spec done...");
        });
        println!("refining invariants done");


        // Compute additional operation invariants.

        println!("refining replan specs");
        let mut new_replan_invariants: Vec<Specification> = model.operations.iter()
            .flat_map(|o| o.make_replan_invariants())
            .collect();

        new_replan_invariants.par_iter_mut().for_each(|i| {
            *i = refine_transition_invariant(i, &ts_model);
            println!("replan spec done...");
        });
        println!("refining replan specs done");

        CompiledModel {
            model,
            computed_transition_planner_invariants: new_invariants,
            computed_operation_planner_invariants: new_replan_invariants,
        }
    }
}


/// Produce model checking problems for debugging the operation model
pub fn generate_mc_problems(model: &Model) {
    let mut ts_model = TransitionSystemModel::from(&model);

    // refine invariants
    println!("refining model invariants");
    let temp_clone = ts_model.clone(); // borrow checker...
    ts_model.invariants.par_iter_mut().for_each(|i| {
        *i = refine_transition_invariant(i, &temp_clone);
        println!("spec done...");
    });
    println!("refining invariants done");

    // add operation specs after this. TODO: where should this logic live.
    let global_invariants: Vec<_> = model
        .global_specs
        .iter()
        .flat_map(|s|
                  if let SpecificationType::OperationInvariant(_) = &s.type_ {
                      Some(s.clone())
                  } else {
                      None
                  })
        .collect();

    ts_model.invariants.extend(global_invariants);


    let ts_model_op = TransitionSystemModel::from_op(&model);

    crate::planning::generate_offline_nuxvm(&ts_model_op, &Predicate::TRUE);
    crate::planning::generate_offline_nuxvm(&ts_model, &Predicate::TRUE);

    // debug low level model
    let all_op_trans = model.operations
        .iter()
        .map(|o| (o.clone(), o.make_lowlevel_transitions()))
        .collect::<Vec<_>>();
    println!("refining operation forbidden specs");
    model.operations.iter().for_each(|o| {
        // check if a "real" operation or really just an autotransition
        if o.make_runner_transitions().is_empty() {
            return;
        }
        // here we should find all unctronllable actions that can
        // modify the variables of GUARD and disallow them.
        println!("CHECKING OP: {}", o.path());
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
                println!("REFINING: {} -- {}", op, p);
                let i = Specification::new_transition_invariant(&op.to_string(), Predicate::NOT(Box::new(p.clone())));
                let ri = refine_transition_invariant(&i, &temp_ts_model);
                println!("spec done...");
                ri
            })
            .collect::<Vec<_>>();
        temp_ts_model.invariants.extend(new_specs);

        // here we check if the operation has an assertion
        // which is assumed to be fulfilled for nominal behavior
        // (eg "resource not in failure mode")
        let guard = if let Some(c) = o.mc_constraint.as_ref() {
            Predicate::AND(vec![o.guard.clone(), c.clone()])
        } else {
            o.guard.clone()
        };
        let op = vec![(o.path().to_string(), guard, o.make_verification_goal())];
        temp_ts_model.name += &format!("_{}", o.path().leaf());
        println!("temp_ts_model: {}", temp_ts_model.name);
        planning::generate_offline_nuxvm_ctl(&temp_ts_model, &Predicate::TRUE, &op);
    });
    println!("refining operation forbidden specs done");
}
