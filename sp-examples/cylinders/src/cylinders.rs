use sp_domain::*;
use sp_runner::*;
use crate::dorna::*;

pub fn cylinders() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cylinders");

    let pt = "pre_take";
    let scan = "scan";
    let t1 = "take1";  // shelf poses
    let t2 = "take2";
    let t3 = "take3";
    let leave = "leave"; // down at conveyor

    let dorna = m.use_resource(make_dorna("dorna", &[pt, scan, t1, t2, t3, leave]));

    let products = &[100.to_spvalue(), // SPValue::Unknown,   macros need better support for Unknown
                     0.to_spvalue(), 1.to_spvalue(), 2.to_spvalue(), 3.to_spvalue()];

    let shelf1 = m.add_estimated_domain("shelf1", products);
    let shelf2 = m.add_estimated_domain("shelf2", products);
    let shelf3 = m.add_estimated_domain("shelf3", products);
    let conveyor = m.add_estimated_domain("conveyor", products);
    let dorna_holding = m.add_estimated_domain("dorna_holding", products);

    let ap = &dorna["act_pos"];
    let pp = &dorna["prev_pos"];
    let rp = &dorna["ref_pos"];

    // define robot movement

    // pre_take can be reached from all positions.

    // shelves can be reached from each other and pre_take
    m.add_invar("to_take1", &p!([p:ap == t1] => [[p:pp == t2] || [p:pp == t3] || [p:pp == pt]]));
    m.add_invar("to_take2", &p!([p:ap == t2] => [[p:pp == t1] || [p:pp == t3] || [p:pp == pt]]));
    m.add_invar("to_take3", &p!([p:ap == t3] => [[p:pp == t1] || [p:pp == t2] || [p:pp == pt]]));

    // scan and leave can only be reached from pre_take
    m.add_invar("to_scan", &p!([p:ap == scan] => [p:pp == pt]));
    m.add_invar("to_leave", &p!([p:ap == leave] => [p:pp == pt]));

    // dorna take/leave products
    let pos = vec![(t1, shelf1.clone()),(t2, shelf2.clone()),
                   (t3, shelf3.clone()),(leave, conveyor.clone())];

    for (pos_name, pos) in pos.iter() {
        m.add_delib(&format!("take_{}", pos.leaf()),
                    &p!([p:ap == pos_name] && [p:pos != 0] && [p:dorna_holding == 0]),
                    &[a!(p:dorna_holding <- p:pos), a!(p:pos = 0)]);

        m.add_delib(&format!("leave_{}", pos.leaf()),
                    &p!([p:ap == pos_name] && [p:dorna_holding != 0] && [p:pos == 0]),
                    &[a!(p:pos <- p:dorna_holding), a!(p:dorna_holding = 0)]);
    }

    // scan to figure out the which product we are holding
    m.add_delib("scan_1",
                &p!([p:ap == scan] && [p:dorna_holding == 100]),
                &[a!(p:dorna_holding = 1)]);
    m.add_delib("scan_2",
                &p!([p:ap == scan] && [p:dorna_holding == 100]),
                &[a!(p:dorna_holding = 2)]);
    m.add_delib("scan_3",
                &p!([p:ap == scan] && [p:dorna_holding == 100]),
                &[a!(p:dorna_holding = 3)]);


    let g = p!([p:shelf1 == 1] && [p:shelf2 == 2]);


    // HIGH LEVEL OPS

    m.add_hl_op("identify_parts", true,
                &p!([p:shelf1 == 100] && [p:shelf2 == 100] && [p:shelf3 == 100]),
                &p!([p:shelf1 == 1] && [p:shelf2 == 2] && [p:shelf3 == 3]),
                &[a!(p:shelf1 = 100), a!(p:shelf2 = 100), a!(p:shelf3 = 100)],
                None);

    // OPERATIONS

    let prods = vec![100, 1, 2, 3];
    let pos = vec![shelf1.clone(), shelf2.clone(), shelf3.clone(), conveyor.clone()];

    for pos in &pos {
        for p in &prods {
            m.add_op(&format!("pick_{}_at_{}", p, pos.leaf()), true,
                     &p!([p:pos == p] && [p:dorna_holding == 0]),
                     &p!([p:pos == 0] && [p:dorna_holding == p]),

                     &[a!(p:pos = 0), a!(p:dorna_holding = p)],
                     None);

            m.add_op(&format!("place_{}_at_{}", p, pos.leaf()), true,
                     &p!([p:pos == 0] && [p:dorna_holding == p]),
                     &p!([p:pos == p] && [p:dorna_holding == 0]),

                     &[a!(p:pos = p), a!(p:dorna_holding = 0)],
                     None);
        }
    }

    m.add_op("scan_1", true,
             &p!([p:dorna_holding == 100]),
             &p!([p:dorna_holding == 1]),
             &[a!(p:dorna_holding = 1)],
             None);

    m.add_op("scan_2", true,
             &p!([p:dorna_holding == 100]),
             &p!([p:dorna_holding == 2]),
             &[a!(p:dorna_holding = 2)],
             None);

    m.add_op("scan_3", true,
             &p!([p:dorna_holding == 100]),
             &p!([p:dorna_holding == 3]),
             &[a!(p:dorna_holding = 3)],
             None);


    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (ap, pt.to_spvalue()),
        (rp, pt.to_spvalue()),
        (pp, pt.to_spvalue()),
        (&dorna_holding, 0.to_spvalue()),
        (&shelf1, 100.to_spvalue()), //SPValue::Unknown),
        (&shelf2, 100.to_spvalue()),
        (&shelf3, 100.to_spvalue()),
        (&conveyor, 0.to_spvalue()),
    ]);

    println!("MAKING MODEL");
    let (m,s) = m.make_model();
    (m,s,g)
}

#[test]
fn test_cylinders() {
    let (m, s, g) = cylinders();

    make_runner_model(&m);

    let inits: Vec<Predicate> = m.resources().iter().flat_map(|r| r.sub_items())
        .flat_map(|si| match si {
            SPItem::Spec(s) if s.name() == "supervisor" => Some(s.invariant().clone()),
            _ => None
        }).collect();

    // we need to assume that we are in a state that adheres to the resources
    let initial = Predicate::AND(inits);

    let mut ts_model = TransitionSystemModel::from(&m);

    // println!("START GUARD GEN");
    // let now = std::time::Instant::now();
    // let (new_guards, supervisor) = extract_guards(&ts_model, &initial);
    // update_guards(&mut ts_model, &new_guards);
    // ts_model.specs.clear();
    // ts_model.specs.push(Spec::new("supervisor", supervisor));
    // println!("GUARD GEN DONE IN {}ms", now.elapsed().as_millis());

    println!("START INVAR REFINEMENT");
    let now = std::time::Instant::now();
    let mut new_specs = Vec::new();
    for s in &ts_model.specs {
        println!("refining {}", s.name());
        new_specs.push(Spec::new(s.name(), refine_invariant(&m, s.invariant())));
    }
    ts_model.specs = new_specs;
    println!("INVAR REFINEMENT DONE IN {}ms", now.elapsed().as_millis());

    let goal = (g, None);
    let plan = plan(&ts_model, &[goal], &s, 50);

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
