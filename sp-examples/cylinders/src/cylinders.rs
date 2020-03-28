use crate::camera::*;
use crate::control_box::*;
use crate::dorna::*;
use sp_domain::*;
use sp_runner::*;

pub fn cylinders() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cylinders");

    let pt = "pre_take";
    let scan = "scan";
    let t1 = "take1"; // shelf poses
    let t2 = "take2";
    let t3 = "take3";
    let leave = "leave"; // down at conveyor

    let dorna = m.use_resource(make_dorna("dorna", &[pt, scan, t1, t2, t3, leave]));
    let cb = m.use_resource(make_control_box("control_box"));
    let camera = m.use_resource(make_camera("camera"));

    let products = &[
        100.to_spvalue(), // SPValue::Unknown,   macros need better support for Unknown
        0.to_spvalue(),
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let shelf1 = m.add_estimated_domain("shelf1", products);
    let shelf2 = m.add_estimated_domain("shelf2", products);
    let shelf3 = m.add_estimated_domain("shelf3", products);
    let conveyor = m.add_estimated_domain("conveyor", products);
    let dorna_holding = m.add_estimated_domain("dorna_holding", products);

    let ap = &dorna["act_pos"];
    let rp = &dorna["ref_pos"];
    let pp = &dorna["prev_pos"];
    let blue = &cb["blue_light_on"];
    // todo
    let cf = SPPath::from_string("cylinders/camera/scan/finished");
    let cs = SPPath::from_string("cylinders/camera/scan/started");
    let cr = &camera["result"];
    let cd = &camera["do_scan"];

    // define robot movement

    // pre_take can be reached from all positions.

    // shelves can be reached from each other and pre_take
    m.add_invar(
        "to_take1",
        &p!([p:ap == t1] => [[p:pp == t1] || [p:pp == t2] || [p:pp == t3] || [p:pp == pt]]),
    );
    m.add_invar(
        "to_take2",
        &p!([p:ap == t2] => [[p:pp == t1] || [p:pp == t2] || [p:pp == t3] || [p:pp == pt]]),
    );
    m.add_invar(
        "to_take3",
        &p!([p:ap == t3] => [[p:pp == t1] || [p:pp == t2] || [p:pp == t3] || [p:pp == pt]]),
    );

    // scan and leave can only be reached from pre_take
    m.add_invar(
        "to_scan",
        &p!([p:ap == scan] => [[p:pp == scan] || [p:pp == pt]]),
    );
    m.add_invar(
        "to_leave",
        &p!([p:ap == leave] => [[p:pp == leave] || [p:pp == pt]]),
    );

    // we must always be blue when going to scan
    m.add_invar("blue_scan", &p!([p:rp == scan] => [p:blue]));
    // but only then...
    m.add_invar(
        "blue_scan_2",
        &p!([[p:rp == t1]||[p:rp == t2]||[p:rp == t3]||[p:rp == leave]] => [!p:blue]),
    );

    // we can only scan the product in front of the camera
    m.add_invar("product_at_camera", &p!([p:cs] => [p:ap == scan]));

    // dorna take/leave products
    let pos = vec![
        (t1, shelf1.clone()),
        (t2, shelf2.clone()),
        (t3, shelf3.clone()),
        (leave, conveyor.clone()),
    ];

    for (pos_name, pos) in pos.iter() {
        m.add_delib(
            &format!("take_{}", pos.leaf()),
            &p!([p: ap == pos_name] && [p: pos != 0] && [p: dorna_holding == 0]),
            &[a!(p:dorna_holding <- p:pos), a!(p: pos = 0)],
        );

        m.add_delib(
            &format!("leave_{}", pos.leaf()),
            &p!([p: ap == pos_name] && [p: dorna_holding != 0] && [p: pos == 0]),
            &[a!(p:pos <- p:dorna_holding), a!(p: dorna_holding = 0)],
        );
    }


    // this is what we want
    // m.add_spec("take_scan_result1", camera.reset,
    //            &p!([p: dorna_holding == 100] && [p: cr == 1]),
    //            &[a!(p: dorna_holding = 1)]);

    // scan to figure out the which product we are holding
    m.add_auto("take_scan_result1",
               &p!([p: dorna_holding == 100] && [p: cf] && [p: cr == 1]),
               &[a!(!p: cd), a!(p: dorna_holding = 1)]);

    m.add_auto("take_scan_result2",
               &p!([p: dorna_holding == 100] && [p: cf] && [p: cr == 2]),
               &[a!(!p: cd), a!(p: dorna_holding = 2)]);
    m.add_auto("take_scan_result3",
               &p!([p: dorna_holding == 100] && [p: cf] && [p: cr == 3]),
               &[a!(!p: cd), a!(p: dorna_holding = 3)]);

    // product sink is at conveyor, only accepts identified products.
    m.add_auto(
        "consume_known_product",
        &p!([p: conveyor != 0] && [p: conveyor != 100]),
        &[a!(p: conveyor = 0)],
    );

    // HIGH LEVEL OPS

    let np = |p: i32| {
        p!([p: shelf1 != p]
            && [p: shelf2 != p]
            && [p: shelf3 != p]
            && [p: dorna_holding != p]
            && [p: conveyor != p])
    };

    let no_products = Predicate::AND(vec![np(1), np(2), np(3), np(100)]);

    // m.add_hl_op("identify_parts", true,
    //             &p!([p:shelf1 == 100] && [p:shelf2 == 100] && [p:shelf3 == 100]),
    //             &p!([p:shelf1 == 1] && [p:shelf2 == 2] && [p:shelf3 == 3]),
    //             &[a!(p:shelf1 = 100), a!(p:shelf2 = 100), a!(p:shelf3 = 100)],
    //             None);

    m.add_hl_op(
        "identify_and_consume_parts",
        false,
        &p!([p: shelf1 == 100] && [p: shelf2 == 100] && [p: shelf3 == 100]),
        &no_products,
        &[
            a!(p: shelf1 = 100),
            a!(p: shelf2 = 100),
            a!(p: shelf3 = 100),
        ],
        None,
    );

    // OPERATIONS

    let prods = vec![100, 1, 2, 3];
    let pos = vec![
        shelf1.clone(),
        shelf2.clone(),
        shelf3.clone(),
        conveyor.clone(),
    ];

    for pos in &pos {
        for p in &prods {
            m.add_op(
                &format!("pick_{}_at_{}", p, pos.leaf()),
                true,
                &p!([p: pos == p] && [p: dorna_holding == 0]),
                &p!([p: pos == 0] && [p: dorna_holding == p]),
                &[a!(p: pos = 0), a!(p: dorna_holding = p)],
                None,
            );

            // These levels quickly become tricky... We need to
            // account for the auto trans that consumes all known
            // products from the conveyor.

            if pos.leaf() == "conveyor" && p != &100 {
                m.add_op(
                    &format!("place_{}_at_{}", p, pos.leaf()),
                    true,
                    &p!([p: pos == 0] && [p: dorna_holding == p]),
                    &p!([p: pos == p] && [p: dorna_holding == 0]),
                    &[a!(p: pos = 0), a!(p: dorna_holding = 0)],
                    None,
                );
            } else {
                m.add_op(
                    &format!("place_{}_at_{}", p, pos.leaf()),
                    true,
                    &p!([p: pos == 0] && [p: dorna_holding == p]),
                    &p!([p: pos == p] && [p: dorna_holding == 0]),
                    &[a!(p: pos = p), a!(p: dorna_holding = 0)],
                    None,
                );
            }
        }
    }

    m.add_op(
        "scan_op_1",
        true,
        &p!([p: dorna_holding == 100]),
        &p!([p: dorna_holding == 1]),
        &[a!(p: dorna_holding = 1)],
        None,
    );

    m.add_op(
        "scan_op_2",
        true,
        &p!([p: dorna_holding == 100]),
        &p!([p: dorna_holding == 2]),
        &[a!(p: dorna_holding = 2)],
        None,
    );

    m.add_op(
        "scan_op_3",
        true,
        &p!([p: dorna_holding == 100]),
        &p!([p: dorna_holding == 3]),
        &[a!(p: dorna_holding = 3)],
        None,
    );

    m.add_op(
        "consume",
        true,
        &p!([p: conveyor != 100]),
        &p!([p: conveyor == 0]),
        &[a!(p: conveyor = 0)],
        None,
    );

    // goal for testing
    // let g = p!([p:shelf1 == 1] && [p:shelf2 == 2] && [p:shelf3 == 3]);
    //let g = p!([p:shelf1 == 1]);
    let g = p!([p: conveyor == 0]);

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (pp, pt.to_spvalue()), // TODO: move to measured in robot driver?
        (&dorna_holding, 0.to_spvalue()),
        (&shelf1, 100.to_spvalue()), //SPValue::Unknown),
        (&shelf2, 100.to_spvalue()),
        (&shelf3, 100.to_spvalue()),
        (&conveyor, 0.to_spvalue()),
    ]);

    println!("MAKING MODEL");
    let (m, s) = m.make_model();
    (m, s, g)
}

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
    fn test_cylinders() {
        let (m, s, g) = cylinders();

        make_runner_model(&m);

        let mut ts_model = TransitionSystemModel::from(&m);

        let mut new_specs = Vec::new();
        for s in &ts_model.specs {
            new_specs.push(Spec::new(s.name(), refine_invariant(&m, s.invariant())));
        }
        ts_model.specs = new_specs;

        let goal = (g, None);
        let plan = plan(&ts_model, &[goal], &s, 50);

        println!("\n\n\n");

        if plan.plan_found {
            plan.trace.iter().enumerate().skip(1).for_each(|(i, t)| {
                println!("{}: {}", i, t.transition);
            });
        } else {
            println!("no plan found");
        }

        println!("\n\n\n");
    }
}
