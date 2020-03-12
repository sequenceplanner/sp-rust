use sp_domain::*;

pub fn add_op(
    m: &mut Model,
    name: &str,
    resets: bool,
    pre: Predicate,
    post: Predicate,
    post_actions: Vec<Action>,
    invariant: Option<Predicate>,
) -> SPPath {
    let op_state = Variable::new(
        name,
        VariableType::Estimated,
        SPValueType::String,
        vec!["i", "e", "f"].iter().map(|v| v.to_spvalue()).collect(),
    );
    let op_state = m.add_item(SPItem::Variable(op_state));

    let op_start = Transition::new(
        "start",
        Predicate::AND(vec![p!(p: op_state == "i"), pre.clone()]),
        vec![a!(p: op_state = "e")],
        vec![],
        true,
    );
    let mut f_actions = if resets {
        vec![a!(p: op_state = "i")]
    } else {
        vec![a!(p: op_state = "f")]
    };
    f_actions.extend(post_actions);
    let op_finish = Transition::new(
        "finish",
        Predicate::AND(vec![p!(p: op_state == "e"), post.clone()]),
        f_actions,
        vec![],
        false,
    );
    let op_goal = IfThen::new("goal", p!(p: op_state == "e"), post.clone(), invariant);

    let op = Operation::new(name, &[op_start, op_finish], Some(op_goal));

    m.add_item(SPItem::Operation(op))
}
