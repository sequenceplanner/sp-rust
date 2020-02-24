use sp_domain::*;
use crate::helpers::*;
use crate::testing::*;
use crate::modeling::*;
use std::ops::Index;

// helpers for creating the "global" model

#[derive(Debug, PartialEq, Clone)]
struct GResource {
    name: String,
    variables: Vec<Variable>
}

impl Index<&str> for GResource {
    type Output = SPPath;
    fn index(&self, name: &str) -> &SPPath {
        let result = self.variables.iter().find(|v|v.name() == name);
        result.expect(&format!("{}/{} not found", self.name, name)).path()
    }
}

#[derive(Debug, PartialEq, Clone)]
struct GOperation {
    name: String,
    resets: bool,
    pre: Predicate,
    post: Predicate,
    post_actions: Vec<Action>,
    invariant: Option<Predicate>,
}

#[derive(Debug, PartialEq, Clone)]
struct GModel {
    model: Model,
    initial_state: SPState,
}

impl GModel {
    pub fn new(name: &str) -> Self {
        GModel {
            model: Model::new_root(name, Vec::new()),
            initial_state: SPState::new(),
        }
    }

    pub fn use_resource(&mut self, r: Resource) -> GResource {
        let rp = self.model.add_item(SPItem::Resource(r));
        let r = self.model.get(&rp).unwrap().unwrap_resource();
        let vars = r.get_variables();
        GResource { name: r.name().to_owned(), variables: vars }
    }

    pub fn add_op(&mut self, name: &str, resets: bool, pre: Predicate,
                  post: Predicate, post_actions: Vec<Action>,
                  invariant: Option<Predicate>) -> SPPath {
        let op_state = Variable::new(
            name,
            VariableType::Estimated,
            SPValueType::String,
            "i".to_spvalue(),
            vec!["i", "e", "f"].iter().map(|v| v.to_spvalue()).collect(),
        );
        let op_state = self.model.add_item(SPItem::Variable(op_state));

        let op_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p:op_state == "i"), pre.clone()]),
            vec![a!(p:op_state = "e")],
            vec![],
            true,
        );
        let mut f_actions =
            if resets {
                vec![a!(p:op_state = "i")]
            } else {
                vec![a!(p:op_state = "f")]
            };
        f_actions.extend(post_actions);
        let op_finish = Transition::new(
            "finish",
            Predicate::AND(vec![p!(p:op_state == "e"), post.clone()]),
            f_actions,
            vec![],
            false,
        );
        let op_goal = IfThen::new("goal", p!(p:op_state == "e"), post.clone(), invariant);

        let op = Operation::new(
            name,
            &[op_start, op_finish],
            Some(op_goal),
        );

        self.model.add_item(SPItem::Operation(op))
    }

    pub fn add_invar(&mut self, name: &str, invariant: &Predicate) {
        self.model.add_item(SPItem::Spec(Spec::new(name, invariant.clone())));
    }

    pub fn initial_state(&mut self, state: &[(&SPPath, SPValue)]) {
        let s: Vec<(SPPath, SPValue)> = state.iter().map(|(p,v)| {
            ((*p).clone(), v.clone())
        }).collect();
        self.initial_state.add_variables(s);
    }

    pub fn make_model(self) -> (Model, SPState) {
        // lastly, add all specifications.
        (self.model, self.initial_state)
    }
}

pub fn two_dummy_robots_new() -> (Model, SPState) {
    let mut m = GModel::new("drm");
    let r1 = m.use_resource(make_dummy_robot("r1", &["at", "away"]));
    let r2 = m.use_resource(make_dummy_robot("r2", &["at", "away"]));

    let r1act = &r1["act_pos"];
    let r2act = &r2["act_pos"];

    let r1_to_at = m.add_op("r1_to_at", false, p!(p:r1act != "at"), p!(p:r1act == "at"), vec![], None);
    let r2_to_at = m.add_op("r2_to_at", false, p!(p:r2act != "at"), p!(p:r2act == "at"), vec![], None);

    // reset previous ops so we can start over
    let _both_to_away = m.add_op("both_to_away", true,
                                 p!([p:r1_to_at == "f"] && [p:r2_to_at == "f"]),
                                 p!([p:r1act == "away"] && [p:r2act == "away"]),
                                 vec![a!(p:r1_to_at = "i"), a!(p:r2_to_at = "i")], None);

    m.add_invar("table_zone", &p!(!([p:r1act == "at"] && [p:r2act == "at"])));

    let r1prev = &r1["prev_pos"];
    let r2prev = &r2["prev_pos"];
    m.initial_state(&[(r1prev, "away".to_spvalue()),
                      (r2prev, "away".to_spvalue())]);

    m.make_model()
}

#[test]
fn test_new_two_dummy_robots() {
    let (m, s) = two_dummy_robots_new();

    println!("{:#?}", m);
    println!("=================================");
    println!("{:#?}", s);
    assert!(false);
}
