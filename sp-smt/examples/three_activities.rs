use sp_smt::*;
fn main(){

    // problem descriprion:
    println!("\n Three activities:
 - Decide about three activities (do or don't) and aim for maximum value
 - Need to choose at least activity 1 or 2 (or both)
 - The total time limit is 4 hours
    - Activity 1 takes 1 hour
    - Activity 2 takes 2 hours
    - Activity 3 takes 3 hours
 - Activity 3 is worth twice as much as activities 1 and 2 \n");

    // setup the optimization context:
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let opt = opt_z3!(&ctx);

    // variables:
    let x = int_var_z3!(&ctx, "x");
    let y = int_var_z3!(&ctx, "y");
    let z = int_var_z3!(&ctx, "z");

    // numerals:
    let i0 = int_z3!(&ctx, 0);
    let i1 = int_z3!(&ctx, 1);
    let i2 = int_z3!(&ctx, 2);
    let i3 = int_z3!(&ctx, 3);
    let i4 = int_z3!(&ctx, 4);

    // objective:
    opt_maximize_z3!(&ctx, &opt, add_z3!(&ctx, x, y, mul_z3!(&ctx, i2, z)));

    // constraints:
    let c1 = le_z3!(&ctx, add_z3!(&ctx, x, mul_z3!(&ctx, i2, y), mul_z3!(&ctx, i3, z)), i4);
    let c2 = ge_z3!(&ctx, add_z3!(&ctx, x, y), i1);
    let c3 = and_z3!(&ctx, ge_z3!(&ctx, x, i0), le_z3!(&ctx, x, i1));
    let c4 = and_z3!(&ctx, ge_z3!(&ctx, y, i0), le_z3!(&ctx, y, i1));
    let c5 = and_z3!(&ctx, ge_z3!(&ctx, z, i0), le_z3!(&ctx, z, i1));

    // assert constraints:
    opt_assert_z3!(&ctx, &opt, c1);
    opt_assert_z3!(&ctx, &opt, c2);
    opt_assert_z3!(&ctx, &opt, c3);
    opt_assert_z3!(&ctx, &opt, c4);
    opt_assert_z3!(&ctx, &opt, c5);

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