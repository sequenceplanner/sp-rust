use sp_smt::*;
fn main(){

    // problem descriprion:
    println!("\n Space colonies:
 There are 4 space colonies, each of which requires a certain number of
 plasma conduits. There are 3 starbases in the vicinity. Each of them
 has a total number of conduits they can spare and supply to the colonies.
 For each pair of starbase and colony, there is an associated cost for 
 sending a cargo ship (each of which carries one plasma conduit), as shown
 in the table below:
             
             | Triacus | New Berlin | Strnad | Vega |  SUPPLY
    -----------------------------------------------------------
    Farpoint |     6   |      9     |   10   |   8  |    35
    Yorktown |     9   |      5     |   16   |  14  |    40
    Earhart  |    12   |      7     |   13   |   9  |    50
    -----------------------------------------------------------
    DEMAND   |    20   |     30     |   30   |  45  | SUM = 125

 The goal is to supply the colonies with the plasma conduits they need
 at a minimum cost. \n");

    // setup the optimization context:
    let cfg = cfgz3!();
    let ctx = ctxz3!(&cfg);
    let opt = optz3!(&ctx);

    // data from the table:
    let supply = vec!(35, 40, 50);
    let starbases = vec!("farpoint", "yorktown", "earhart");
    let demand = vec!(20, 30, 30, 45);
    let colonies = vec!("triacus", "new_berlin", "strnad", "vega");
    let delivery_cost = vec!(
        vec!(6, 9, 10, 8),
        vec!(9, 5, 16, 14),
        vec!(12, 7, 13, 9)
    );

    // the first constraint is about limiting the possible amount of conduits that the
    // starbases can deliver to colonies:
    for starbase in &starbases {
        let mut variables = vec!();
        // extracting the index of the element in the vector
        let s_index = starbases.iter().position(|&r| r.to_string() == starbase.to_string()).unwrap();
        for colony in &colonies {
            // concatenating strings and taking a full slice of the string
            let slice: &str = &format!("{}{}{}", starbase, "_to_", colony).to_owned()[..];
            // populating the vector element by element
            variables.push(int_var_z3!(&ctx, slice));
        }

        // asserting the constraint:
        opt_assert_z3!(&ctx, &opt, 
            lez3!(&ctx, 
                addz3!(&ctx, variables.clone()), 
                int_z3!(&ctx, supply[s_index])
            )
        );
    }

    // the second constraint is about meeting the minimum requirements, i.e. delivering
    // at least the required amount of conduits to colonies:
    for colony in &colonies {
        let mut variables = vec!();
        let c_index = colonies.iter().position(|&r| r.to_string() == colony.to_string()).unwrap();
        for starbase in &starbases {
            let slice: &str = &format!("{}{}{}", starbase, "_to_", colony).to_owned()[..];
            variables.push(int_var_z3!(&ctx, slice));
        }

        opt_assert_z3!(&ctx, &opt, 
            gez3!(&ctx, 
                addz3!(&ctx, variables.clone()), 
                int_z3!(&ctx, demand[c_index])
            )
        );
    }

    let mut variables = vec!();
    let z = int_var_z3!(&ctx, "z");

    // the third constraint is a minimization constraint that tries to minimize the 
    // total cost of delivering conduits based on the given table:
    for starbase in &starbases {
        let s_index = starbases.iter().position(|&r| r.to_string() == starbase.to_string()).unwrap();
        for colony in &colonies {
            let c_index = colonies.iter().position(|&r| r.to_string() == colony.to_string()).unwrap();
            let slice: &str = &format!("{}{}{}", starbase, "_to_", colony).to_owned()[..];
            variables.push(mulz3!(&ctx, int_var_z3!(&ctx, slice), int_z3!(&ctx, delivery_cost[s_index][c_index])));

            // this constraint limiting the minimum amount of possibly delivered
            // conduits to 0
            opt_assert_z3!(&ctx, &opt,
                gez3!(&ctx,
                    int_var_z3!(&ctx, slice),
                    int_z3!(&ctx, 0)
                )
            );
        }
    }

    // asserting the minimization constraint
    opt_assert_z3!(&ctx, &opt,
        gez3!(&ctx,
            int_var_z3!(&ctx, "z"),
            addz3!(&ctx, variables.clone())
        )
    );

    // objective function:
    opt_minimize_z3!(&ctx, &opt, z);

    // check consistency and produce oprimal value:
    let res1 = opt_check_z3!(&ctx, &opt, );
    if res1 == 1 {
        println!("SAT");
    } else if res1 == -1 {
        println!("UNSAT");
    } else {
        println!("UNDEF");
    };

    // uncomment if you want to print the context:
    // println!("{}", opt_to_string_z3!(&ctx, &opt));

    // get the optimal value:
    let model = opt_get_model_z3!(&ctx, &opt);
    println!("{}", model_to_string_z3!(&ctx, model));
}