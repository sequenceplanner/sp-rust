use sp_domain::*;
use sp_runner::*;
use crate::mecademic::*;

pub fn simple_monolithic() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cubes");

    let robot = m.use_resource(make_mecademic("r", &["home", "table", "buffer"]));

    let products = &["none".to_spvalue(), "cube".to_spvalue()];

    let robot_holding = m.add_estimated_domain("robot_holding", products);
    let table_holding = m.add_estimated_domain("table_holding", products);
    let buffer_holding = m.add_estimated_domain("buffer_holding", products);

    let r1act = &robot["act_pos"];
    let r1prev = &robot["prev_pos"];
    let r1ref = &robot["ref_pos"];

    // must go to table positions via the home pose
    // this leads to RIDICULOUSLY long plans (52 steps for the long operation below) :)
    // m.add_invar("via_home_r_table", &p!([p:r1act == "table"] => [[p:r1prev == "table"] || [p:r1prev == "home"]]));

    // same for bufferszzz
    // m.add_invar("via_home_buffer", &p!([p:r1act == "buffer"] => [[p:r1prev == "buffer"] || [p:r1prev == "home"]]));

    // r1 take/leave products

    m.add_delib("r_take_table", &p!([p:r1act == "table"] && [p:table_holding != "none"] && [p:robot_holding == "none"]),
                &[a!(p:robot_holding <- p:table_holding), a!(p:table_holding = "none")]);

    m.add_delib("r_take_buffer", &p!([p:r1act == "buffer"] && [p:buffer_holding != "none"] && [p:robot_holding == "none"]),
               &[a!(p:robot_holding <- p:buffer_holding), a!(p:buffer_holding = "none")]);

    m.add_delib("r_leave_table", &p!([p:r1act == "table"] && [p:robot_holding != "none"] && [p:table_holding == "none"]),
                &[a!(p:table_holding <- p:robot_holding), a!(p:robot_holding = "none")]);

    m.add_delib("r_leave_buffer", &p!([p:r1act == "buffer"] && [p:robot_holding != "none"] && [p:buffer_holding == "none"]),
               &[a!(p:buffer_holding <- p:robot_holding), a!(p:robot_holding = "none")]);

    let g = p!([p:table_holding == "cube"]); // && [p:buffer2_holding == 2]);

    // // same as above but broken to reduce plan lengths...
    // m.add_op("swap_parts_step_1", true,
    //          &p!([p:buffer_holding == "cube"]),
    //          &p!([p:table_holding == "cube"]), &[], None);

    // m.add_op("swap_parts_step_2", true,
    //          &p!([p:table1_holding == "b"] && [p:table2_holding == "c"]),
    //          &p!([p:buffer1_holding == "b"] && [p:buffer2_holding == "c"]), &[], None);

    // m.add_op("swap_parts_again_step_1", true,
    //          &p!([p:buffer1_holding == "b"] && [p:buffer2_holding == "c"]),
    //          &p!([p:table1_holding == "c"] && [p:table2_holding == "b"]), &[], None);

    // m.add_op("swap_parts_again_step_2", true,
    //          &p!([p:table1_holding == "c"] && [p:table2_holding == "b"]),
    //          &p!([p:buffer1_holding == "c"] && [p:buffer2_holding == "b"]), &[], None);

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (r1prev, "home".to_spvalue()),
        (r1ref, "unknown".to_spvalue()),
        (r1act, "table".to_spvalue()),
        (&robot_holding, "none".to_spvalue()),
        (&table_holding, "none".to_spvalue()),
        (&buffer_holding, "cube".to_spvalue()),
    ]);

    println!("MAKING MODEL");
    let (m,s) = m.make_model();
    (m,s,g)
}

#[test]
fn test_simple_monolithic() {
    let (m, s, g) = simple_monolithic();

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
//    let (new_guards, supervisor) = extract_guards(&ts_model, &initial);

    // for t in ts_model.transitions.iter_mut() {
    //     *t.mut_guard() = Predicate::TRUE;
    // }

    // println!("G GEN DONE");
    //update_guards(&mut ts_model, &new_guards);


    // println!("{:?}", ts_model.specs);

    // ts_model.specs.clear();

    // println!("{:?}", ts_model.specs);

    println!("START INVAR REFINEMENT");
    let now = std::time::Instant::now();
    let mut new_specs = Vec::new();
    for s in &ts_model.specs {
        new_specs.push(Spec::new(s.name(), refine_invariant(&m, s.invariant())));
    }
    ts_model.specs = new_specs;
    println!("INVAR REFINEMENT DONE IN {}ms", now.elapsed().as_millis());

//    ts_model.specs.push(Spec::new("supervisor", supervisor));

    // let mut new_specs = Vec::new();
    // for s in &ts_model.specs {
    //     new_specs.push(Spec::new(s.name(), refine_invariant(&m, s.invariant())));
    // }
    // ts_model.specs = new_specs;

    let goal = (g, None);
    let plan = plan(&ts_model, &[goal], &s, 20);

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
