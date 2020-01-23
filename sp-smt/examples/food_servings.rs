use sp_smt::*;
fn main(){

    // problem descriprion:
    println!("\nFood servings:
 Given a set of foods, along with the nutrient information for each food and the 
 cost per serving of each food, the objective of the diet problem is to select 
 the number of servings of each food to purchase (and consume) so as to minimize
 the cost of the food while meeting the specified nutritional requirements. 
 Typically, the nutritional requirements are expressed as a minimum and a maximum 
 allowable level for each nutritional component. Other constraints such a minimum
 and/or maximum number of servings may be included to improve the quality of the menu.

 Consider the following simple example. Suppose there are three foods available, corn,
 milk, and bread, and there are restrictions on the number of calories (between 2000
 and 2250) and the amount of Vitamin A (between 5000 and 50,000). The first table lists,
 for each food, the cost per serving, the amount of Vitamin A per serving, and the
 number of calories per serving.
 
    Table 1. | Corn | Milk | Bread 
    -------------------------------
    Cost     |  180 |  230 |   50  
    Nutrient |  107 |  500 |    0  
    Calories |   72 |  121 |   65  
 
 Suppose that the maximum number of servings per food is 10.\n");

    // setup the optimization context:
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let opt = opt_z3!(&ctx);

    // data from the table:
    let foods = vec!("corn", "milk", "bread");
    let cost = vec!(180, 230, 50);
    let nutrient = vec!(107, 500, 0);
    let calories = vec!(72, 121, 65);

    // some constants:
    let min_nutrient = int_z3!(&ctx, 5000);
    let max_nutrient = int_z3!(&ctx, 50000);
    let min_calories = int_z3!(&ctx, 2000);
    let max_calories = int_z3!(&ctx, 2250);
    let max_servings = int_z3!(&ctx, 10);

    // assering constraints
    for food_init in &foods {
        let food: &str = &format!("{}_servings", food_init).to_owned()[..];

        // the first three constraints are about limiting the amount of servings per food
        opt_assert_z3!(&ctx, &opt, 
            le_z3!(&ctx, 
                int_var_z3!(&ctx, food), 
                max_servings
            )
        )        
    };

    let mut total_nutrient = Vec::new();
    let mut total_calories = Vec::new();
    let mut total_cost = Vec::new();

    // collecting stuff to make later assertions more convenient
    for food_init in &foods {
        let food: &str = &format!("{}_servings", food_init).to_owned()[..];
        let food_index = foods.iter().position(|&r| r.to_string() == food_init.to_string()).unwrap();
        &total_nutrient.push(
            mul_z3!(&ctx,
                int_var_z3!(&ctx, food),
                int_z3!(&ctx, nutrient[food_index])
            )
        );
        &total_calories.push(
            mul_z3!(&ctx,
                int_var_z3!(&ctx, food),
                int_z3!(&ctx, calories[food_index])
            )
        );
        &total_cost.push(
            mul_z3!(&ctx,
                int_var_z3!(&ctx, food),
                int_z3!(&ctx, cost[food_index])
            )
        );
    };
    
    let daily_cost = int_var_z3!(&ctx, "cost");
    opt_assert_z3!(&ctx, &opt, ge_z3!(&ctx, daily_cost, add_z3!(&ctx, total_cost)));

    let daily_nutrient = int_var_z3!(&ctx, "nutrient");
    opt_assert_z3!(&ctx, &opt, ge_z3!(&ctx, daily_nutrient, add_z3!(&ctx, total_nutrient.clone())));

    let daily_calories = int_var_z3!(&ctx, "calories");
    opt_assert_z3!(&ctx, &opt, ge_z3!(&ctx, daily_calories, add_z3!(&ctx, total_calories.clone())));

    // the sum of nutrients from all servings of different foods must be at least the minimum required
    opt_assert_z3!(&ctx, &opt,
        ge_z3!(&ctx, 
            add_z3!(&ctx, total_nutrient.clone()),
            min_nutrient
        )
    );

    // same story for calories
    opt_assert_z3!(&ctx, &opt,
        ge_z3!(&ctx, 
            add_z3!(&ctx, total_calories.clone()),
            min_calories
        )
    );

    // the sum of nutrients from all servings of different foods must not exceed the maximum allowed
    opt_assert_z3!(&ctx, &opt,
        le_z3!(&ctx, 
            add_z3!(&ctx, total_nutrient),
            max_nutrient
        )
    );

    // same story for calories
    opt_assert_z3!(&ctx, &opt,
        le_z3!(&ctx, 
            add_z3!(&ctx, total_calories),
            max_calories
        )
    );

    // the objective is to minimize the cost of food per day, meeting the req's from before
    opt_minimize_z3!(&ctx, &opt, daily_cost);

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