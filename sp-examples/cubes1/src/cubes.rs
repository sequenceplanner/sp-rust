use crate::mecademic::*;
use sp_domain::*;
use sp_runner::*;

pub fn cubes() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cubes");

    let r1 = m.use_resource(make_mecademic(
        "r1",
        &["home", "table1", "table2", "buffer1"],
    ));
    let r2 = m.use_resource(make_mecademic(
        "r2",
        &["home", "table1", "table2", "buffer2"],
    ));

    let products = &[
        0.to_spvalue(),
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let r1_holding = m.add_estimated_domain("r1_holding", products);
    let r2_holding = m.add_estimated_domain("r2_holding", products);
    let table1_holding = m.add_estimated_domain("table1_holding", products);
    let table2_holding = m.add_estimated_domain("table2_holding", products);
    let buffer1_holding = m.add_estimated_domain("buffer1_holding", products);
    let buffer2_holding = m.add_estimated_domain("buffer2_holding", products);

    let r1act = &r1["act_pos"];
    let r2act = &r2["act_pos"];
    let r1prev = &r1["prev_pos"];
    let r2prev = &r2["prev_pos"];
    let r1ref = &r1["ref_pos"];
    let r2ref = &r2["ref_pos"];

    m.add_invar(
        "table_zone",
        &p!(!([p: r1act == "table1"] && [p: r2act == "table1"])),
    );
    m.add_invar(
        "table_zone",
        &p!(!([p: r1act == "table2"] && [p: r2act == "table2"])),
    );

    // must go to table positions via the home pose
    // this leads to RIDICULOUSLY long plans (52 steps for the long operation below) :)
    m.add_invar(
        "via_home_r1_table1",
        &p!([p:r1act == "table1"] => [[p:r1prev == "table1"] || [p:r1prev == "home"]]),
    );
    m.add_invar(
        "via_home_r1_table2",
        &p!([p:r1act == "table2"] => [[p:r1prev == "table2"] || [p:r1prev == "home"]]),
    );
    m.add_invar(
        "via_home_r2_table1",
        &p!([p:r2act == "table1"] => [[p:r2prev == "table1"] || [p:r2prev == "home"]]),
    );
    m.add_invar(
        "via_home_r2_table2",
        &p!([p:r2act == "table2"] => [[p:r2prev == "table2"] || [p:r2prev == "home"]]),
    );

    // same for buffers
    m.add_invar(
        "via_home_buffer1",
        &p!([p:r1act == "buffer1"] => [[p:r1prev == "buffer1"] || [p:r1prev == "home"]]),
    );
    m.add_invar(
        "via_home_buffer2",
        &p!([p:r2act == "buffer2"] => [[p:r2prev == "buffer2"] || [p:r2prev == "home"]]),
    );

    // r1 take/leave products

    m.add_delib(
        "r1_take_table1",
        &p!([p: r1act == "table1"] && [p: table1_holding != 0] && [p: r1_holding == 0]),
        &[
            a!(p:r1_holding <- p:table1_holding),
            a!(p: table1_holding = 0),
        ],
    );

    m.add_delib(
        "r1_take_table2",
        &p!([p: r1act == "table2"] && [p: table2_holding != 0] && [p: r1_holding == 0]),
        &[
            a!(p:r1_holding <- p:table2_holding),
            a!(p: table2_holding = 0),
        ],
    );

    m.add_delib(
        "r1_take_buffer1",
        &p!([p: r1act == "buffer1"] && [p: buffer1_holding != 0] && [p: r1_holding == 0]),
        &[
            a!(p:r1_holding <- p:buffer1_holding),
            a!(p: buffer1_holding = 0),
        ],
    );

    m.add_delib(
        "r1_leave_table1",
        &p!([p: r1act == "table1"] && [p: r1_holding != 0] && [p: table1_holding == 0]),
        &[a!(p:table1_holding <- p:r1_holding), a!(p: r1_holding = 0)],
    );

    m.add_delib(
        "r1_leave_table2",
        &p!([p: r1act == "table2"] && [p: r1_holding != 0] && [p: table2_holding == 0]),
        &[a!(p:table2_holding <- p:r1_holding), a!(p: r1_holding = 0)],
    );

    m.add_delib(
        "r1_leave_buffer1",
        &p!([p: r1act == "buffer1"] && [p: r1_holding != 0] && [p: buffer1_holding == 0]),
        &[a!(p:buffer1_holding <- p:r1_holding), a!(p: r1_holding = 0)],
    );

    // r2 take/leave products

    m.add_delib(
        "r2_take_table1",
        &p!([p: r2act == "table1"] && [p: table1_holding != 0] && [p: r2_holding == 0]),
        &[
            a!(p:r2_holding <- p:table1_holding),
            a!(p: table1_holding = 0),
        ],
    );

    m.add_delib(
        "r2_take_table2",
        &p!([p: r2act == "table2"] && [p: table2_holding != 0] && [p: r2_holding == 0]),
        &[
            a!(p:r2_holding <- p:table2_holding),
            a!(p: table2_holding = 0),
        ],
    );

    m.add_delib(
        "r2_take_buffer2",
        &p!([p: r2act == "buffer2"] && [p: buffer2_holding != 0] && [p: r2_holding == 0]),
        &[
            a!(p:r2_holding <- p:buffer2_holding),
            a!(p: buffer2_holding = 0),
        ],
    );

    m.add_delib(
        "r2_leave_table1",
        &p!([p: r2act == "table1"] && [p: r2_holding != 0] && [p: table1_holding == 0]),
        &[a!(p:table1_holding <- p:r2_holding), a!(p: r2_holding = 0)],
    );

    m.add_delib(
        "r2_leave_table2",
        &p!([p: r2act == "table2"] && [p: r2_holding != 0] && [p: table2_holding == 0]),
        &[a!(p:table2_holding <- p:r2_holding), a!(p: r2_holding = 0)],
    );

    m.add_delib(
        "r2_leave_buffer",
        &p!([p: r2act == "buffer2"] && [p: r2_holding != 0] && [p: buffer2_holding == 0]),
        &[a!(p:buffer2_holding <- p:r2_holding), a!(p: r2_holding = 0)],
    );

    // products can not be in two places at once
    // this is actually true already given a good initial state as per the transitions above
    // question is: do we also feed this info to the planner for faster solving?
    // needs more testing.

    // let r1 = p!(p:r1_holding != 0);
    // let others = p!([p:r1_holding <!> p:r2_holding] && [p:r1_holding <!> p:table1_holding] &&
    //                 [p:r1_holding <!> p:table2_holding] && [p:r1_holding <!> p:buffer1_holding] &&
    //                 [p:r1_holding <!> p:buffer2_holding]);
    // let p = Predicate::OR(vec![Predicate::NOT(Box::new(r1)), others]);
    // m.add_invar("r1 imples others", &p);

    // let r2 = p!(p:r2_holding != 0);
    // let others = p!([p:r2_holding <!> p:r1_holding] && [p:r2_holding <!> p:table1_holding] &&
    //                 [p:r2_holding <!> p:table2_holding] && [p:r2_holding <!> p:buffer1_holding] &&
    //                 [p:r2_holding <!> p:buffer2_holding]);
    // let p = Predicate::OR(vec![Predicate::NOT(Box::new(r2)), others]);
    // m.add_invar("r2 imples others", &p);

    // let t1 = p!(p:table1_holding != 0);
    // let others = p!([p:table1_holding <!> p:r1_holding] && [p:table1_holding <!> p:r2_holding] &&
    //                 [p:table1_holding <!> p:table2_holding] && [p:table1_holding <!> p:buffer1_holding] &&
    //                 [p:table1_holding <!> p:buffer2_holding]);
    // let p = Predicate::OR(vec![Predicate::NOT(Box::new(t1)), others]);
    // m.add_invar("table1 imples others", &p);

    // let t2 = p!(p:table2_holding != 0);
    // let others = p!([p:table2_holding <!> p:r1_holding] && [p:table2_holding <!> p:r2_holding] &&
    //                 [p:table2_holding <!> p:table1_holding] && [p:table2_holding <!> p:buffer1_holding] &&
    //                 [p:table2_holding <!> p:buffer2_holding]);
    // let p = Predicate::OR(vec![Predicate::NOT(Box::new(t2)), others]);
    // m.add_invar("table2 imples others", &p);

    // let b1 = p!(p:buffer1_holding != 0);
    // let others = p!([p:buffer1_holding <!> p:r1_holding] && [p:buffer1_holding <!> p:r2_holding] &&
    //                 [p:buffer1_holding <!> p:table1_holding] && [p:buffer1_holding <!> p:table2_holding] &&
    //                 [p:buffer1_holding <!> p:buffer2_holding]);
    // let p = Predicate::OR(vec![Predicate::NOT(Box::new(b1)), others]);
    // m.add_invar("buffer1 imples others", &p);

    // let b2 = p!(p:buffer2_holding != 0);
    // let others = p!([p:buffer2_holding <!> p:r1_holding] && [p:buffer2_holding <!> p:r2_holding] &&
    //                 [p:buffer2_holding <!> p:table1_holding] && [p:buffer2_holding <!> p:table2_holding] &&
    //                 [p:buffer2_holding <!> p:buffer1_holding]);
    // let p = Predicate::OR(vec![Predicate::NOT(Box::new(b2)), others]);
    // m.add_invar("buffer2 imples others", &p);

    // goal for testing
    //let g = p!([p:buffer1_holding == 1] && [p:r1act == "r1table"]);
    let g = p!([p: buffer1_holding == 1] && [p: buffer2_holding == 2]);

    // m.add_op("swap_parts", true,
    //          &p!([p:buffer1_holding == 2] && [p:buffer2_holding == 1]),
    //          &p!([p:buffer1_holding == 1] && [p:buffer2_holding == 2]), &[], None);

    // m.add_op("swap_parts_again", true,
    //          &p!([p:buffer1_holding == 1] && [p:buffer2_holding == 2]),
    //          &p!([p:buffer1_holding == 2] && [p:buffer2_holding == 1]), &[], None);

    // same as above but broken to reduce plan lengths...
    m.add_op(
        "swap_parts_step_1",
        true,
        &p!([p: buffer1_holding == 2] && [p: buffer2_holding == 1]),
        &p!([p: table1_holding == 1] && [p: table2_holding == 2]),
        &[],
        None,
    );

    m.add_op(
        "swap_parts_step_2",
        true,
        &p!([p: table1_holding == 1] && [p: table2_holding == 2]),
        &p!([p: buffer1_holding == 1] && [p: buffer2_holding == 2]),
        &[],
        None,
    );

    m.add_op(
        "swap_parts_again_step_1",
        true,
        &p!([p: buffer1_holding == 1] && [p: buffer2_holding == 2]),
        &p!([p: table1_holding == 2] && [p: table2_holding == 1]),
        &[],
        None,
    );

    m.add_op(
        "swap_parts_again_step_2",
        true,
        &p!([p: table1_holding == 2] && [p: table2_holding == 1]),
        &p!([p: buffer1_holding == 2] && [p: buffer2_holding == 1]),
        &[],
        None,
    );

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (r1prev, "home".to_spvalue()),
        (r2prev, "home".to_spvalue()),
        (&r1_holding, 3.to_spvalue()),
        (&r2_holding, 0.to_spvalue()),
        (&table1_holding, 0.to_spvalue()),
        (&table2_holding, 0.to_spvalue()),
        (&buffer1_holding, 2.to_spvalue()),
        (&buffer2_holding, 1.to_spvalue()),
    ]);

    println!("MAKING MODEL");
    let (m, s) = m.make_model();
    (m, s, g)
}

#[test]
fn test_cubes() {
    let (m, s, g) = cubes();

    let inits: Vec<Predicate> = m
        .resources()
        .iter()
        .flat_map(|r| r.sub_items())
        .flat_map(|si| match si {
            SPItem::Spec(s) if s.name() == "supervisor" => Some(s.invariant().clone()),
            _ => None,
        })
        .collect();

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

    ts_model.specs.clear();

    //    ts_model.specs.push(Spec::new("supervisor", supervisor));

    let mut new_specs = Vec::new();
    for s in &ts_model.specs {
        new_specs.push(Spec::new(s.name(), refine_invariant(&m, s.invariant())));
    }
    ts_model.specs = new_specs;

    let goal = (g, None);
    let plan = plan(&ts_model, &[goal], &s, 50);

    // crate::planning::generate_offline_nuxvm(&ts_model, &new_initial);

    println!("\n\n\n");

    if plan.plan_found {
        plan.trace.iter().enumerate().for_each(|(i, t)| {
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
