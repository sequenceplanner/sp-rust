use sp_domain::*;
use crate::modeling::*;
use super::*;

pub fn cylinders() -> (Model, SPState) {
    let mut m = GModel::new("cylinders");

    let pt = "pre_take";
    let scan = "scan";
    let t1 = "take1"; // shelf poses
    let t2 = "take2";
    let t3 = "take3";
    let leave = "leave"; // down at conveyor

    let dorna = m.use_named_resource("dorna", dorna::create_instance("r1", &[pt, scan, t1, t2, t3, leave]));
    let dorna_moving = dorna.find_item("moving", &[]);
    let dorna2 = m.use_named_resource("dorna", dorna::create_instance("r2", &[pt, scan, leave]));
    let dorna2_moving = dorna2.find_item("moving", &[]);

    let cb = m.use_resource(control_box::create_instance("control_box"));
    let camera = m.use_resource(camera::create_instance("camera"));
    let gripper = m.use_resource(gripper_fail::create_instance("gripper"));

    let buffer_domain = &[
        0.to_spvalue(), // buffer empty
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let product_kinds = &[
        100.to_spvalue(), // unknown type
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let product_1_kind = m.add_estimated_domain("product_1_kind", product_kinds, true);
    let product_2_kind = m.add_estimated_domain("product_2_kind", product_kinds, true);
    let product_3_kind = m.add_estimated_domain("product_3_kind", product_kinds, true);

    // poison low level only
    let poison = m.add_estimated_bool("poison", false);

    let shelf1 = m.add_estimated_domain("shelf1", buffer_domain, true);
    let shelf2 = m.add_estimated_domain("shelf2", buffer_domain, true);
    let shelf3 = m.add_estimated_domain("shelf3", buffer_domain, true);
    let conveyor = m.add_estimated_domain("conveyor", buffer_domain, true);
    let dorna_holding = m.add_estimated_domain("dorna_holding", buffer_domain, true);
    let dorna2_holding = m.add_estimated_domain("dorna2_holding", buffer_domain, true);
    let conveyor2 = m.add_estimated_domain("conveyor2", buffer_domain, true);

    let ap = &dorna["act_pos"];
    let rp = &dorna["ref_pos"];
    let pp = &dorna["prev_pos"];
    let blue = &cb["blue_light_on"];

    let cf = camera.find_item("finished", &[]);
    let camera_start = camera.find_item("start", &[]);

    let cr = &camera["result"];
    let cd = &camera["do_scan"];

    let gripper_part = &gripper["part_sensor"];
    let gripper_closed = &gripper["closed"];
    let gripper_opening = gripper.find_item("opening", &[]);
    let gripper_closing = gripper.find_item("closing", &[]);
    let gripper_fc = &gripper["fail_count"];

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


    // if we are holding a part, we cannot open the gripper freely!
    m.add_invar(
        "gripper_open",
        &p!([[p:gripper_opening] && [p: dorna_holding !=0]] =>
            [[[p:ap == t1] && [p:shelf1 == 0]] ||
             [[p:ap == t2] && [p:shelf2 == 0]] ||
             [[p:ap == t3] && [p:shelf3 == 0]] ||
             [[p:ap == leave] && [p:conveyor == 0]]]),
    );

    // we can only close the gripper
    m.add_invar(
        "gripper_close",
        &p!([p:gripper_closing] => [[p:ap == t1] || [p:ap == t2] || [p:ap == t3] || [p:ap == leave]]),
    );

    m.add_invar("close_gripper_while_dorna_moving", &p!(![[p:gripper_closing] && [p:dorna_moving]]));
    m.add_invar("open_gripper_while_dorna_moving", &p!(![[p:gripper_opening] && [p:dorna_moving]]));

    // dont open gripper again after failure unless we have moved away.
    m.add_invar("dont_open_gripper_after_failure",
                &p!([[p:gripper_opening] && [[p:ap == t1] || [p:ap == t2] || [p:ap == t3] || [p:ap == leave]]] => [p:gripper_part]));

    // if the gripper is closed, with no part in it, it is impossible for it to hold a part.
    // m.add_invar("gripper_no_sensor_implies_no_part",
    //             &p!([[p:gripper_closed] && [!p:gripper_part]] => [ p:dorna_holding == 0 ]));

    // if there is something on the shelves, we can only try to move there with the gripper open.
    m.add_invar(
        "to_take1_occupied",
        &p!([[p:rp == t1] && [p: dorna_moving]] => [[p:shelf1 == 0] || [! p:gripper_closed]]),
    );

    m.add_invar(
        "to_take2_occupied",
        &p!([[p:rp == t2] && [p: dorna_moving]] => [[p:shelf2 == 0] || [! p:gripper_closed]]),
    );

    m.add_invar(
        "to_take3_occupied",
        &p!([[p:rp == t3] && [p: dorna_moving]] => [[p:shelf3 == 0] || [! p:gripper_closed]]),
    );

    m.add_invar(
        "to_conveyor_occupied",
        &p!([[p:rp == leave] && [p: dorna_moving]] => [[p:conveyor == 0] || [! p:gripper_closed]]),
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
    m.synchronize(&camera_start, "scan_with_dorna_in_place",
                  p!([p:ap == scan] && [!p:dorna_moving]), &[]);

    // dorna take/leave products
    let pos = vec![
        (t1, shelf1.clone()),
        (t2, shelf2.clone()),
        (t3, shelf3.clone()),
        (leave, conveyor.clone()),
    ];

    let extra = p!([p: product_1_kind <-> p: product_1_kind] &&
                   [p: product_2_kind <-> p: product_2_kind] &&
                   [p: product_3_kind <-> p: product_3_kind]);

    for (pos_name, pos) in pos.iter() {
        m.add_op(&format!("r1_take_{}", pos.leaf()),
                 // operation model guard.
                 &Predicate::AND(vec![p!([p: pos != 0] && [p: dorna_holding == 0]), extra.clone()]),
                 // operation model effects.
                 &[a!(p:dorna_holding <- p:pos), a!(p: pos = 0)],
                 // low level goal
                 &p!([p: ap == pos_name] && [p: gripper_part]),
                 // low level actions (should not be needed)
                 &[],
                 // auto
                 true, Some(p!([p: gripper_fc == 0] || [p: gripper_fc == 1])));

        let goal = if pos_name == &t1 {
            p!([p: ap == pos_name] && [! p: gripper_part] && [!p: poison])
        } else {
            p!([p: ap == pos_name] && [! p: gripper_part])
        };

        m.add_op(&format!("r1_leave_{}", pos.leaf()),
                 // operation model guard.
                 &Predicate::AND(vec![p!([p: dorna_holding != 0] && [p: pos == 0]), extra.clone()]),
                 // operation model effects.
                 &[a!(p:pos <- p:dorna_holding), a!(p: dorna_holding = 0)],
                 // low level goal
                 &goal,
                 // low level actions (should not be needed)
                 &[],
                 // auto
                 true, Some(p!([p: gripper_fc == 0] || [p: gripper_fc == 1])));
    }

    // dorna2 take/leave products
    let pos = vec![
        (leave, conveyor.clone()),
        (pt, conveyor2.clone()),
    ];

    let ap3 = &dorna2["act_pos"];
    let rp3 = &dorna2["ref_pos"];

    for (pos_name, pos) in pos.iter() {
        let buffer_predicate = if pos_name == &pt {
            // when taking the product, because we have an auto transition that consumes it,
            // we need to be sure that the product will still be there
            p!([p: dorna2_holding == 0] && [
                [[p: pos == 1] && [p: product_1_kind == 100]] ||
                    [[p: pos == 2] && [p: product_2_kind == 100]] ||
                    [[p: pos == 3] && [p: product_3_kind == 100]]
            ])
        } else {
            p!([p: pos != 0] && [p: dorna2_holding == 0])
        };

        m.add_op(&format!("r3_take_{}", pos.leaf()),
                 // operation model guard.
                 &buffer_predicate,
                 // operation model effects.
                 &[a!(p:dorna2_holding <- p:pos), a!(p: pos = 0)],
                 // low level goal
                 &p!([p: ap3 == pos_name] && [!p: dorna2_moving]),
                 // low level actions (should not be needed)
                 &[],
                 // not auto
                 false, None);

        m.add_op(&format!("r3_leave_{}", pos.leaf()),
                 // operation model guard.
                 &p!([p: dorna2_holding != 0] && [p: pos == 0]),
                 // operation model effects.
                 &[a!(p:pos <- p:dorna2_holding), a!(p: dorna2_holding = 0)],
                 // low level goal
                 &p!([p: ap3 == pos_name] && [!p: dorna2_moving]),
                 // low level actions (should not be needed)
                 &[],
                 // not auto
                 false, None);
    }

    let np = |p: i32| {
        p!([p: shelf1 != p]
           && [p: shelf2 != p]
           && [p: shelf3 != p]
           && [p: dorna_holding != p]
           && [p: dorna2_holding != p]
           && [p: conveyor != p]
           && [p: conveyor2 != p]
        )
    };

    let products = &[(1, product_1_kind.clone()),
                     (2, product_2_kind.clone()),
                     (3, product_3_kind.clone())];

    for p in products {
        let n = p.0;

        // scan to figure out the which product we are holding
        let kind = p.1.clone();

        m.add_op_alt(&format!("scan_{}", n),
                     &p!([p: dorna_holding == n] && [p: kind == 100]),
                     &[
                         (&[a!(p: kind = 1)], &p!([p: cf] && [p: cr == 1] && [p: ap == scan]), &[a!(!p: cd)]),
                         (&[a!(p: kind = 2)], &p!([p: cf] && [p: cr == 2] && [p: ap == scan]), &[a!(!p: cd)]),
                         (&[a!(p: kind = 3)], &p!([p: cf] && [p: cr == 3] && [p: ap == scan]), &[a!(!p: cd)]),
                     ],
                     true, None);

        // product sink is at conveyor2, only accepts identified products.
        m.add_op(&format!("consume_known_product_{}", n),
                 // operation model guard.
                 &p!([p: conveyor2 == n] && [p: kind != 100]),
                 // operation model effects.
                 &[a!(p: conveyor2 = 0), a!(p: kind = 100)],
                 // low level goal
                 &Predicate::TRUE,
                 // low level actions (should not be needed)
                 &[],
                 // auto
                 true, None);

        // product source also at conveyor2, we can add a new, unknown,
        // unique product if there is room.
        m.add_op(&format!("add_conveyor2_{}", n),
                 // operation model guard.n
                 &Predicate::AND(vec![p!([p: conveyor2 == 0] && [p: dorna2_holding == 0]), np(p.0)]),
                 // operation model effects.
                 &[a!(p:conveyor2 = p.0), a!(p: kind = 100)],
                 // low level goal: away from buffer and not moving. OR the robot is not holding anything.
                 &p!([[[p:ap3 != pt] && [p: ap3 <-> p: rp3]] || [p: dorna2_holding == 0]]),
                 //&p!([p:ap3 != pt] && [p: ap3 <-> p: rp3]),
                 // low level actions (should not be needed)
                 &[],
                 // not auto
                 false, None);
    }


    // HIGH LEVEL OPS

    let no_products = p!([p: shelf1 == 0]
           && [p: shelf2 == 0]
           && [p: shelf3 == 0]
           && [p: dorna_holding == 0]
           && [p: dorna2_holding == 0]
           && [p: conveyor == 0]
           && [p: conveyor2 == 0]
        );

    m.add_intention(
        "identify_and_consume_parts",
        false,
        //&p!([p: shelf1 == 1] && [p: shelf2 == 2] && [p: shelf3 == 3]),
        &Predicate::FALSE,
        &no_products,
        &[
            // a!(p: shelf1 = 1),
            // a!(p: shelf2 = 2),
            // a!(p: shelf3 = 3),
            // a!(p: product_1_kind = 100),
            // a!(p: product_2_kind = 100),
            // a!(p: product_3_kind = 100),
        ],
        None,
    );

    m.add_intention(
        "get_new_products",
        false,
        &Predicate::FALSE,
        &p!([p: shelf1 == 1] && [p: shelf2 == 2] && [p: shelf3 == 3] &&
            [p: product_1_kind == 100] && [p: product_2_kind == 100] &&
            [p: product_3_kind == 100]),
        &[],
        None,
    );

    m.add_intention(
        "identify_types_r_g_b",
        false,
        &Predicate::FALSE,
        &p!([[p: shelf1 != 0] && [p: shelf2 != 0] && [p: shelf3 != 0] && [
            [[p: product_1_kind == 1] && [p: product_2_kind == 2] && [p: product_3_kind == 3]] ||
             [[p: product_1_kind == 2] && [p: product_2_kind == 3] && [p: product_3_kind == 1]] ||
             [[p: product_1_kind == 3] && [p: product_2_kind == 1] && [p: product_3_kind == 2]] ||
             [[p: product_1_kind == 1] && [p: product_2_kind == 3] && [p: product_3_kind == 2]] ||
             [[p: product_1_kind == 2] && [p: product_2_kind == 1] && [p: product_3_kind == 3]] ||
             [[p: product_1_kind == 3] && [p: product_2_kind == 2] && [p: product_3_kind == 1]]]
        ]
        ),
        &[],
        None,
    );

    m.add_intention(
        "identify_two_blue",
        false,
        &Predicate::FALSE,
        &p!([[[p: product_1_kind == 3] && [p: product_2_kind == 3]] &&
             [[[p: shelf1 == 1] && [p: shelf2 == 2]] || [[p: shelf2 == 1] && [p: shelf3 == 2]] || [[p: shelf1 == 1] && [p: shelf3 == 2]]]] ||
            [[[p: product_2_kind == 3] && [p: product_3_kind == 3]] &&
             [[[p: shelf1 == 2] && [p: shelf2 == 3]] || [[p: shelf2 == 2] && [p: shelf3 == 3]] || [[p: shelf1 == 2] && [p: shelf3 == 3]]]] ||
            [[[p: product_1_kind == 3] && [p: product_3_kind == 3]] &&
             [[[p: shelf1 == 1] && [p: shelf2 == 3]] || [[p: shelf2 == 1] && [p: shelf3 == 3]] || [[p: shelf1 == 1] && [p: shelf3 == 3]]]]),
        &[],
        None,
    );

    m.add_intention(
        "sort_shelves_r_g_b",
        false,
        &Predicate::FALSE,
        &p!([[[p: product_1_kind == 1] => [p: shelf1 == 1]] &&
             [[p: product_1_kind == 2] => [p: shelf2 == 1]] &&
             [[p: product_1_kind == 3] => [p: shelf3 == 1]] &&
             [[p: product_2_kind == 1] => [p: shelf1 == 2]] &&
             [[p: product_2_kind == 2] => [p: shelf2 == 2]] &&
             [[p: product_2_kind == 3] => [p: shelf3 == 2]] &&
             [[p: product_3_kind == 1] => [p: shelf1 == 3]] &&
             [[p: product_3_kind == 2] => [p: shelf2 == 3]] &&
             [[p: product_3_kind == 3] => [p: shelf3 == 3]] &&
             [[p: product_1_kind != 100] && [p: product_2_kind != 100] && [p: product_3_kind != 100]]
        ]),
        // &p!([p: shelf1 == 1] && [p: shelf2 == 2] && [p: shelf3 == 3] &&
        //     [p: product_1_kind == 1] && [p: product_2_kind == 2] && [p: product_3_kind == 3]
        // ),
        &[],
        None,
    );

    // ensure uniqueness of products
    let vars = vec![&shelf1, &shelf2, &shelf3, &conveyor,
                    &dorna_holding, &dorna2_holding, &conveyor2];
    for v in &vars {
        let v = v.clone();
        let ne = Predicate::AND(vars.iter().filter(|&&o|o!=v).map(|&o| p!(p:v <!> p:o)).collect());
        m.add_product_invar(&format!("unique_{}", v.leaf()), &p!([p: v != 0] => [pp: ne]))
    }

    let pp3 = &dorna2["prev_pos"];

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (pp, pt.to_spvalue()), // TODO: move to measured in robot driver?
        (pp3, pt.to_spvalue()),
        (&dorna_holding, 0.to_spvalue()),
        (&dorna2_holding, 0.to_spvalue()),
        (&shelf1, 0.to_spvalue()), //SPValue::Unknown),
        (&shelf2, 0.to_spvalue()),
        (&shelf3, 0.to_spvalue()),
        (&product_1_kind, 100.to_spvalue()), // unknown products
        (&product_2_kind, 100.to_spvalue()),
        (&product_3_kind, 100.to_spvalue()),
        (&conveyor, 0.to_spvalue()),
        (&conveyor2, 0.to_spvalue()),
        (&poison, false.to_spvalue()),
        (gripper_fc, 0.to_spvalue()),
    ]);

    println!("MAKING MODEL");
    m.make_model()
}