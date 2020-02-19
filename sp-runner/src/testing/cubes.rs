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

    pub fn add_estimated_domain(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
        let v = Variable::new(name, VariableType::Estimated, SPValueType::String, domain[0].clone(),
                              domain.to_vec());
        self.model.add_item(SPItem::Variable(v))
    }

    pub fn add_auto(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![], // no effects
            false, // auto!
        );
        self.model.add_item(SPItem::Transition(trans));
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




pub fn cubes() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cubes");
    let r1 = m.use_resource(make_dummy_mecademic("r1", &["at", "away"]));
    let r2 = m.use_resource(make_dummy_mecademic("r2", &["at", "away"]));
    let sm = m.use_resource(make_dummy_sm("sm"));

    let cube = m.add_estimated_domain("cube_pos", &[
        "rob1".to_spvalue(), "rob2".to_spvalue(),
        "table".to_spvalue(), "leave1".to_spvalue(),
        "leave2".to_spvalue()]);

    let r1act = &r1["act_pos"];
    let r2act = &r2["act_pos"];

    m.add_invar("table_zone", &p!(!([p:r1act == "at"] && [p:r2act == "at"])));

    let r1prev = &r1["prev_pos"];
    let r2prev = &r2["prev_pos"];
    let r1ref = &r1["ref_pos"];
    let r2ref = &r2["ref_pos"];
    let attach_box_r1 = &sm["attach_r1_box"];
    let attach_box_r2 = &sm["attach_r2_box"];
    let attached_box_r1 = &sm["attached_r1_box"];
    let attached_box_r2 = &sm["attached_r2_box"];

    m.add_auto("r1_take_cube", &p!([p:r1act == "at"] && [p:cube == "table"] && [p:attached_box_r1]),
               &[a!(p:cube = "rob1")]);

    // its never the case that rob1 holds the cube and we are not sending attach to the scene mgr
    m.add_invar("sync_scene_status", &p!(!([p:cube == "rob1"] && [!p:attach_box_r1])));

    // its not allowed to take the cube unless the robot is in position
    let p1 = p!(p:attached_box_r1);
    let p2 = p!([p:r1act == "at"] || [p:cube == "rob1"]);
    let p1_imp_p2 = Predicate::OR(vec![Predicate::NOT(Box::new(p1)), p2]);
    m.add_invar("interact with sm at the right position", &p1_imp_p2);


    // goal for testing
    let g = p!([p:r1act == "away"] && [p:cube == "rob1"]);

    m.initial_state(&[
        (r1prev, "away".to_spvalue()),
        (r2prev, "away".to_spvalue()),
        (r1act, "away".to_spvalue()),
        (r2act, "away".to_spvalue()),
        (r1ref, "away".to_spvalue()),
        (r2ref, "away".to_spvalue()),

        (attach_box_r1, false.to_spvalue()),
        (attach_box_r2, false.to_spvalue()),
        (attached_box_r1, false.to_spvalue()),
        (attached_box_r2, false.to_spvalue()),

        (&cube, "table".to_spvalue()),
    ]);

    println!("MAKING MODEL");
    let (m,s) = m.make_model();
    (m,s,g)
}

#[test]
fn test_cubes() {
    let (m, s, g) = cubes();


    let inits: Vec<Predicate> = m.resources().iter().flat_map(|r| r.sub_items())
        .flat_map(|si| match si {
            SPItem::Spec(s) if s.name() == "supervisor" => Some(s.invariant().clone()),
            _ => None
        }).collect();

    // we need to assume that we are in a state that adheres to the resources
    let initial = Predicate::AND(inits);

    let mut ts_model = TransitionSystemModel::from(&m);
    println!("G GEN");
    let (new_guards, new_initial) = extract_guards(&ts_model, &initial);
    println!("G GEN DONE");
    update_guards(&mut ts_model, &new_guards);

    ts_model.specs.clear();

    let goal = (g, None);
    let plan = crate::planning::compute_plan(&ts_model, &[goal], &s, 20);

    // crate::planning::generate_offline_nuxvm(&ts_model, &new_initial);

    println!("\n\n\n");

    if plan.plan_found {
        plan.trace.iter().enumerate().for_each(|(i,t)| {
            println!("{}: {}", i, t.transition);
        });
    } else {
        println!("no plan found");
    }

    println!("\n\n\n");

    assert!(false);
}
