use crate::mecademic::*;
use sp_domain::*;
use sp_runner::*;

pub fn cubes() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cubes");

    let h = "home";
    let t1 = "table1";
    let t2 = "table2";
    let b1 = "buffer1";
    let b2 = "buffer2";

    let r1 = m.use_resource(make_mecademic("r1", &[h, t1, t2, b1]));
    let r2 = m.use_resource(make_mecademic("r2", &[h, t1, t2, b2]));

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

    m.add_invar("table_zone_1", &p!(!([p: r1act == t1] && [p: r2act == t1])));
    m.add_invar("table_zone_2", &p!(!([p: r1act == t2] && [p: r2act == t2])));
    m.add_invar("table_zone_3", &p!(!([p: r1act == t1] && [p: r2act == t2])));
    m.add_invar("table_zone_4", &p!(!([p: r1act == t2] && [p: r2act == t1])));

    // special case at table 2
    m.add_invar(
        "table_zone_2_2",
        &p!(!([p: r1prev == t2] && [p: r1ref != t2] && [p: r2ref == t2])),
    );
    m.add_invar(
        "table_zone_2_2",
        &p!(!([p: r2prev == t2] && [p: r2ref != t2] && [p: r1ref == t2])),
    );

    // must go to table positions via the home pose
    // this leads to RIDICULOUSLY long plans (52 steps for the long operation below) :)
    m.add_invar(
        "via_home_r1_table1",
        &p!([p:r1act == t1] => [[p:r1prev == t1] || [p:r1prev == h]]),
    );
    m.add_invar(
        "via_home_r1_table2",
        &p!([p:r1act == t2] => [[p:r1prev == t2] || [p:r1prev == h]]),
    );
    m.add_invar(
        "via_home_r2_table1",
        &p!([p:r2act == t1] => [[p:r2prev == t1] || [p:r2prev == h]]),
    );
    m.add_invar(
        "via_home_r2_table2",
        &p!([p:r2act == t2] => [[p:r2prev == t2] || [p:r2prev == h]]),
    );

    // same for buffers
    m.add_invar(
        "via_home_buffer1",
        &p!([p:r1act == b1] => [[p:r1prev == b1] || [p:r1prev == h]]),
    );
    m.add_invar(
        "via_home_buffer2",
        &p!([p:r2act == b2] => [[p:r2prev == b2] || [p:r2prev == h]]),
    );

    // robot take/leave products
    let rs = vec![
        ("r1", r1act, r1_holding.clone()),
        ("r2", r2act, r2_holding.clone()),
    ];
    let pos = vec![
        (b1, buffer1_holding.clone()),
        (b2, buffer2_holding.clone()),
        (t1, table1_holding.clone()),
        (t2, table2_holding.clone()),
    ];
    for (r_name, act, holding) in rs {
        for (pos_name, pos) in pos.iter() {
            // r1 cannot pick at buffer2 and vice versa
            if r_name == "r1" && pos.leaf() == "buffer2_holding" {
                continue;
            }
            if r_name == "r2" && pos.leaf() == "buffer1_holding" {
                continue;
            }

            m.add_delib(
                &format!("{}_take_{}", r_name, pos_name),
                &p!([p: act == pos_name] && [p: pos != 0] && [p: holding == 0]),
                &[a!(p:holding <- p:pos), a!(p: pos = 0)],
            );

            m.add_delib(
                &format!("{}_leave_{}", r_name, pos_name),
                &p!([p: act == pos_name] && [p: holding != 0] && [p: pos == 0]),
                &[a!(p:pos <- p:holding), a!(p: holding = 0)],
            );
        }
    }

    // let g = p!([p:buffer1_holding == 1] && [p:buffer2_holding == 2]);
    let g = p!([p: buffer1_holding == 1]);

    // HIGH LEVEL OPS

    m.add_hl_op(
        "swap_parts",
        true,
        &p!([p: buffer1_holding == 2] && [p: buffer2_holding == 1]),
        &p!([p: buffer1_holding == 1] && [p: buffer2_holding == 2]),
        &[],
        None,
    );

    m.add_hl_op(
        "swap_parts_again",
        true,
        &p!([p: buffer1_holding == 1] && [p: buffer2_holding == 2]),
        &p!([p: buffer1_holding == 2] && [p: buffer2_holding == 1]),
        &[],
        None,
    );

    // OPERATIONS

    let prods = vec![1, 2, 3];
    let rs = vec![("r1", r1_holding.clone()), ("r2", r2_holding.clone())];
    let pos = vec![
        buffer1_holding.clone(),
        buffer2_holding.clone(),
        table1_holding.clone(),
        table2_holding.clone(),
    ];

    for (r_name, holding) in rs {
        for pos in pos.iter() {
            for p in prods.iter() {
                // r1 cannot pick at buffer2 and vice versa
                if r_name == "r1" && pos.leaf() == "buffer2_holding" {
                    continue;
                }
                if r_name == "r2" && pos.leaf() == "buffer1_holding" {
                    continue;
                }

                m.add_op(
                    &format!("{}_{}_pick_{}", p, r_name, pos.leaf()),
                    true,
                    &p!([p: pos == p] && [p: holding == 0]),
                    &p!([p: pos == 0] && [p: holding == p]),
                    &[a!(p: pos = 0), a!(p: holding = p)],
                    None,
                );

                m.add_op(
                    &format!("{}_{}_place_{}", p, r_name, pos.leaf()),
                    true,
                    &p!([p: pos == 0] && [p: holding == p]),
                    &p!([p: pos == p] && [p: holding == 0]),
                    &[a!(p: pos = p), a!(p: holding = 0)],
                    None,
                );
            }
        }
    }

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (r1prev, h.to_spvalue()),
        (r2prev, h.to_spvalue()),
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

// Modelchecking the swap operation:
//
// This operation can always finish.
// CTLSPEC AG ( ( cubes#buffer1_holding = 1 )&( cubes#buffer2_holding = 2 ) -> EF (( cubes#buffer1_holding = 2 )&( cubes#buffer2_holding = 1 )) );
//  => TRUE

//
// This operation cannot actually never finish.
// CTLSPEC AG ( ( cubes#buffer1_holding = 1 )&( cubes#buffer2_holding = 2 ) -> EF (( cubes#buffer1_holding = 1 )&( cubes#buffer2_holding = 1 )) );
//  => FALSE

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
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
        let _initial = Predicate::AND(inits);

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
            new_specs.push(Spec::new(s.name(), refine_invariant(&m, s.invariant())));
        }
        ts_model.specs = new_specs;
        println!("INVAR REFINEMENT DONE IN {}ms", now.elapsed().as_millis());

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

        // assert!(false);
    }
}
