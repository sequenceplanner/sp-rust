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

    pub fn add_predicate_variable(&mut self, name: &str, domain: &[SPValue]) -> SPPath {
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

    pub fn add_delib(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![],
            true,
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_effect(&mut self, name: &str, guard: &Predicate, effects: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            vec![],
            effects.to_vec(),
            false,
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn add_simulation_auto(&mut self, name: &str, guard: &Predicate, actions: &[Action]) {
        let trans = Transition::new(
            name,
            guard.clone(),
            actions.to_vec(),
            vec![], // no effects
            false, // auto!
        );
        self.model.add_item(SPItem::Transition(trans));
    }

    pub fn find(&mut self, name: &str, path_sections: &[&str]) -> SPPath {
        self.model.find_item(name, path_sections)
            .expect(&format!("cannot find {:?} / {}", path_sections, name)).path()
    }

    pub fn add_op(&mut self, name: &str, resets: bool, pre: &Predicate,
                  post: &Predicate, post_actions: &[Action],
                  invariant: Option<Predicate>) -> SPPath {

        let state = SPPath::from_slice(&[self.model.name(), name, "state"]);

        let op_start = Transition::new(
            "start",
            Predicate::AND(vec![p!(p:state == "i"), pre.clone()]),
            vec![a!(p:state = "e")],
            vec![],
            true,
        );
        let mut f_actions =
            if resets {
                vec![a!(p:state = "i")]
            } else {
                vec![a!(p:state = "f")]
            };
        f_actions.extend(post_actions.iter().cloned());
        let op_finish = Transition::new(
            "finish",
            Predicate::AND(vec![p!(p:state == "e"), post.clone()]),
            f_actions,
            vec![],
            false,
        );
        let op_goal = IfThen::new("goal", p!(p:state == "e"), post.clone(), invariant);

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

        let mut s = self.initial_state.clone();

        // add operation to the initial state here
        let global_ops: Vec<&Operation> = self.model.items()
            .iter()
            .flat_map(|i| match i {
                SPItem::Operation(o) => Some(o),
                _ => None,
            })
            .collect();
        let global_ops_vars = global_ops.iter().map(|o|o.state_variable());

        let state: Vec<_> = global_ops_vars.map(|v| (v.node().path().clone(), v.initial_value())).collect();
        s.extend(SPState::new_from_values(&state[..]));


        // lastly, add all specifications.
        (self.model, s)
    }
}




pub fn cubes() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cubes");

    let r1 = m.use_resource(make_dummy_mecademic("r1", &["home", "r1table", "r1buffer"]));
    let r2 = m.use_resource(make_dummy_mecademic("r2", &["home", "r2table", "r2buffer"]));

    let products = &[0.to_spvalue(), 1.to_spvalue(),
                     2.to_spvalue(), 3.to_spvalue()];

    let r1_holding = m.add_estimated_domain("r1_holding", products);
    let r2_holding = m.add_estimated_domain("r2_holding", products);
    let table1_holding = m.add_estimated_domain("table1_holding", products);
    let table2_holding = m.add_estimated_domain("table2_holding", products);
    let buffer1_holding = m.add_estimated_domain("buffer1_holding", products);
    let buffer2_holding = m.add_estimated_domain("buffer2_holding", products);

    let r1act = &r1["act_pos"];
    let r2act = &r2["act_pos"];

    m.add_invar("table_zone", &p!(!([p:r1act == "r1table"] && [p:r2act == "r2table"])));

    let r1prev = &r1["prev_pos"];
    let r2prev = &r2["prev_pos"];
    let r1ref = &r1["ref_pos"];
    let r2ref = &r2["ref_pos"];

    // r1 take/leave products

    m.add_delib("r1_take_table1", &p!([p:r1act == "r1table"] && [p:table1_holding != 0] && [p:r1_holding == 0]),
                &[a!(p:r1_holding <- p:table1_holding), a!(p:table1_holding = 0)]);

    m.add_delib("r1_take_table2", &p!([p:r1act == "r1table"] && [p:table2_holding != 0] && [p:r1_holding == 0]),
               &[a!(p:r1_holding <- p:table2_holding), a!(p:table2_holding = 0)]);

    m.add_delib("r1_take_buffer1", &p!([p:r1act == "r1buffer"] && [p:buffer1_holding != 0] && [p:r1_holding == 0]),
               &[a!(p:r1_holding <- p:buffer1_holding), a!(p:buffer1_holding = 0)]);

    m.add_delib("r1_leave_table1", &p!([p:r1act == "r1table"] && [p:r1_holding != 0] && [p:table1_holding == 0]),
                &[a!(p:table1_holding <- p:r1_holding), a!(p:r1_holding = 0)]);

    m.add_delib("r1_leave_table2", &p!([p:r1act == "r1table"] && [p:r1_holding != 0] && [p:table2_holding == 0]),
               &[a!(p:table2_holding <- p:r1_holding), a!(p:r1_holding = 0)]);

    m.add_delib("r1_leave_buffer1", &p!([p:r1act == "r1buffer"] && [p:r1_holding != 0] && [p:buffer1_holding == 0]),
               &[a!(p:buffer1_holding <- p:r1_holding), a!(p:r1_holding = 0)]);

    // r2 take/leave products

    m.add_delib("r2_take_table1", &p!([p:r2act == "r2table"] && [p:table1_holding != 0] && [p:r2_holding == 0]),
                &[a!(p:r2_holding <- p:table1_holding), a!(p:table1_holding = 0)]);

    m.add_delib("r2_take_table2", &p!([p:r2act == "r2table"] && [p:table2_holding != 0] && [p:r2_holding == 0]),
               &[a!(p:r2_holding <- p:table2_holding), a!(p:table2_holding = 0)]);

    m.add_delib("r2_take_buffer2", &p!([p:r2act == "r2buffer"] && [p:buffer2_holding != 0] && [p:r2_holding == 0]),
               &[a!(p:r2_holding <- p:buffer2_holding), a!(p:buffer2_holding = 0)]);

    m.add_delib("r2_leave_table1", &p!([p:r2act == "r2table"] && [p:r2_holding != 0] && [p:table1_holding == 0]),
                &[a!(p:table1_holding <- p:r2_holding), a!(p:r2_holding = 0)]);

    m.add_delib("r2_leave_table2", &p!([p:r2act == "r2table"] && [p:r2_holding != 0] && [p:table2_holding == 0]),
               &[a!(p:table2_holding <- p:r2_holding), a!(p:r2_holding = 0)]);

    m.add_delib("r2_leave_buffer", &p!([p:r2act == "r2buffer"] && [p:r2_holding != 0] && [p:buffer2_holding == 0]),
               &[a!(p:buffer2_holding <- p:r2_holding), a!(p:r2_holding = 0)]);


    // robots cannot both be at table
    m.add_invar("mutex table",
                &p!(!( [p:r1act == "r1table"] && [p:r2act == "r2table"])));

    // products can not be at the same place
    // m.add_invar("unique_products",
    //             &p!([p:r1_holding <!> p:r2_holding] && [p:r2_holding <!> p:table1_holding]));

    // goal for testing
    //let g = p!([p:buffer1_holding == 1] && [p:r1act == "r1table"]);
    let g = p!(p:buffer2_holding == 1);

    // make an operation with that goal
    // m.add_op("take_part1", true, &p!(p:buffer1_holding == 0), &p!(p:buffer1_holding == 1), &[], None);
    // m.add_op("take_part2", true, &p!(p:buffer2_holding == 0), &p!(p:buffer2_holding == 2), &[], None);

    m.add_op("swap_parts", true, &p!([p:buffer1_holding == 2] && [p:buffer2_holding == 1]),
             &p!([p:buffer1_holding == 1] && [p:buffer2_holding == 2]), &[], None);
    m.add_op("swap_parts_again", true, &p!([p:buffer1_holding == 1] && [p:buffer2_holding == 2]),
             &p!([p:buffer1_holding == 2] && [p:buffer2_holding == 1]), &[], None);

    m.initial_state(&[
        (r1prev, "r1buffer".to_spvalue()),
        (r2prev, "r2buffer".to_spvalue()),
        (r1act, "r1buffer".to_spvalue()),
        (r2act, "r2buffer".to_spvalue()),
        (r1ref, "r1buffer".to_spvalue()),
        (r2ref, "r2buffer".to_spvalue()),

        (&r1_holding, 0.to_spvalue()),
        (&r2_holding, 0.to_spvalue()),
        (&table1_holding, 3.to_spvalue()),
        (&table2_holding, 0.to_spvalue()),
        (&buffer1_holding, 2.to_spvalue()),
        (&buffer2_holding, 1.to_spvalue()),
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
    // guard gen takes too much time....
    // println!("G GEN");
    let (new_guards, _new_initial) = extract_guards(&ts_model, &initial);
    // println!("G GEN DONE");
    // update_guards(&mut ts_model, &new_guards);

    //ts_model.specs.clear();

    for s in &mut ts_model.specs {
        *s = Spec::new(s.name(), crate::helpers::refine_invariant(&m, s.invariant()));
    }

    let goal = (g, None);
    let plan = crate::planning::plan(&ts_model, &[goal], &s, 20);
    //crate::planning::compute_plan(&ts_model, &[goal], &s, 20);

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

    // println!("initial state");
    // println!("{}",s);

    assert!(false);
}
