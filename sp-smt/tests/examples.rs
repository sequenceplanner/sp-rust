use sp_smt::*;
use std::collections::HashMap;

/// DESCRIPTION:
///  - Decide about three activities (do or don't) and aim for maximum value
///  - Need to choose at least activity 1 or 2 (or both)
///  - The total time limit is 4 hours
///     - Activity 1 takes 1 hour
///     - Activity 2 takes 2 hours
///     - Activity 3 takes 3 hours
///  - Activity 3 is worth twice as much as activities 1 and 2
///
/// MODEL:
///  - This can be modelled as a linear mixed-integer Problem
///     - Binary variables x, y, z for activities 1, 2, 3
///     - Linear constraint for time limit
///     - Linear constraint for condition (1 or 2)
///
///     max x + y + 2z
///     so that: x + 2y + 3z <= 4
///              x + y >= 1
///     where x, y, z in {0, 1}
#[test]
fn example_1(){
    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    let opt = OptimizerZ3::new(&ctx);

    let intsort = IntSortZ3::new(&ctx);
    let x = IntVarZ3::new(&ctx, &intsort, "x");
    let y = IntVarZ3::new(&ctx, &intsort, "y");
    let z = IntVarZ3::new(&ctx, &intsort, "z");
    let obj = IntVarZ3::new(&ctx, &intsort, "obj");

    let zero = IntZ3::new(&ctx, &intsort, 0);
    let one = IntZ3::new(&ctx, &intsort, 1);
    let two = IntZ3::new(&ctx, &intsort, 2);
    let three = IntZ3::new(&ctx, &intsort, 3);
    let four = IntZ3::new(&ctx, &intsort, 4);

    let twoy = MULZ3::new(&ctx, vec!(two.r, y.r));
    let threez = MULZ3::new(&ctx, vec!(three.r, z.r));
    let add1 = ADDZ3::new(&ctx, vec!(x.r, twoy.r, threez.r));

    let constr1 = LEZ3::new(&ctx, add1.r, four.r);
    let constr2 = GEZ3::new(&ctx, ADDZ3::new(&ctx, vec!(x.r, y.r)).r, one.r);
    let add2 = ADDZ3::new(&ctx, vec!(x.r, y.r, MULZ3::new(&ctx, vec!(two.r, z.r)).r));
    let constr3 = EQZ3::new(&ctx, obj.r, add2.r);    
    
    OptAssertZ3::new(&ctx, &opt, GEZ3::new(&ctx, x.r, zero.r).r);
    OptAssertZ3::new(&ctx, &opt, GEZ3::new(&ctx, y.r, zero.r).r);
    OptAssertZ3::new(&ctx, &opt, GEZ3::new(&ctx, z.r, zero.r).r);
    OptAssertZ3::new(&ctx, &opt, LEZ3::new(&ctx, x.r, one.r).r);
    OptAssertZ3::new(&ctx, &opt, LEZ3::new(&ctx, y.r, one.r).r);
    OptAssertZ3::new(&ctx, &opt, LEZ3::new(&ctx, z.r, one.r).r);
    
    OptAssertZ3::new(&ctx, &opt, constr1.r);
    OptAssertZ3::new(&ctx, &opt, constr2.r);
    OptAssertZ3::new(&ctx, &opt, constr3.r);
    
    OptMaximizeZ3::new(&ctx, &opt, obj.r);

    println!("Now we have an assert in the opt context, should print it.");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print empty string, no check yet.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);

    let res1 = OptCheckZ3::new(&ctx, &opt, vec!());
    println!("This is the return of the check:");
    println!("{}", res1.r);

    println!("This is the opt context with an assertion after the check:");
    println!("{}", GetOptStringZ3::new(&ctx, &opt).s);

    println!("Model: Should print the solution, we did a check.");
    println!("{}", GetOptModelZ3::new(&ctx, &opt).s);
}


// DESCRIPTION:
//  There are 4 space colonies, each of which requires a certain number of
//  plasma conduits. There are 3 starbases in the vicinity. Each of them
//  has a total number of conduits they can spare and supply to the colonies.
//  For each pair of starbase and colony, there is an associated cost for 
//  sending a cargo ship (each of which carries one plasma conduit), as shown
//  in the table below:
            //  
            //  | Triacus | New Berlin | Strnad | Vega |  SUPPLY
    // -----------------------------------------------------------
    // Farpoint |     6   |      9     |   10   |   8  |    35
    // Yorktown |     9   |      5     |   16   |  14  |    40
    // Earhart  |    12   |      7     |   13   |   9  |    50
    // -----------------------------------------------------------
    // DEMAND   |    20   |     30     |   30   |  45  | SUM = 125
// 
//  The goal is to supply the colonies with the plasma conduits they need
//  at a minimum cost.
#[test]
fn example_2(){
    let mut starbase_supply: HashMap<String, i32> = HashMap::new();
    starbase_supply.insert("farpoint".to_string(), 35);
    starbase_supply.insert("yorktown".to_string(), 40);
    starbase_supply.insert("earhart".to_string(), 50);

    let mut colony_demand: HashMap<String, i32> = HashMap::new();
    colony_demand.insert("triacus".to_string(), 20);
    colony_demand.insert("new_berlin".to_string(), 30);
    colony_demand.insert("strnad".to_string(), 30);
    colony_demand.insert("vega".to_string(), 45);

    let mut delivery_cost: HashMap<(String, String), i32> = HashMap::new();
    delivery_cost.insert(("farpoint".to_string(), "triacus".to_string()), 6);
    delivery_cost.insert(("farpoint".to_string(), "new_berlin".to_string()), 9);
    delivery_cost.insert(("farpoint".to_string(), "strnad".to_string()), 10);
    delivery_cost.insert(("farpoint".to_string(), "vega".to_string()), 8);
    delivery_cost.insert(("yorktown".to_string(), "triacus".to_string()), 9);
    delivery_cost.insert(("yorktown".to_string(), "new_berlin".to_string()), 5);
    delivery_cost.insert(("yorktown".to_string(), "strnad".to_string()), 16);
    delivery_cost.insert(("yorktown".to_string(), "vega".to_string()), 14);
    delivery_cost.insert(("earhart".to_string(), "triacus".to_string()), 12);
    delivery_cost.insert(("earhart".to_string(), "new_berlin".to_string()), 7);
    delivery_cost.insert(("earhart".to_string(), "strnad".to_string()), 13);
    delivery_cost.insert(("earhart".to_string(), "vega".to_string()), 9);
    
    println!("{:?}", starbase_supply);
    println!("{:?}", colony_demand);
    println!("{:?}", delivery_cost);

    let conf = ConfigZ3::new();
    let ctx = ContextZ3::new(&conf);
    OptimizerZ3::new(&ctx);

    IntSortZ3::new(&ctx);
    // let sup = IntVarZ3::new(&ctx, &intsort, "sup");
    // let dem = IntVarZ3::new(&ctx, &intsort, "dem");
    // let cst = IntVarZ3::new(&ctx, &intsort, "cst");

    // let sum1 = ADDZ3::new(&ctx, ) 

    // `HashMap::iter()` returns an iterator that yields 
//     // (&'a key, &'a value) pairs in arbitrary order.
//     for (contact, &number) in contacts.iter() {
//         println!("Calling {}: {}", contact, call(number)); 
//     }
// }

    for ((sup, dem), cost) in delivery_cost.iter() {
         println!("Calling {} {} {}", sup, dem, cost); 
    }

    // // m.addConstrs(sum(x[i,j] for j in colony_demand) <= starbase_supply[i] for i in starbase_supply)
    // // use std::collections::HashMap;

    //     println!(
    //         "{:?}",
    //         (1..6).map(|i| (i * i, i + i)).collect::<HashMap<_, _>>()
    //     );

    //     println!(
    //         "{:?}",
    //         vec!(1, 2, 3, 4, 5).into_iter().map(|i| (i * i, i + i)).collect::<HashMap<_, _>>()
    //     );

    // //     for pair in vec!['a', 'b', 'c'].into_iter()
    // //                            .map(|letter| { c += 1; (letter, c) }) {
    // // println!("{:?}", pair);

    //     let mut ints: HashMap<String, IntZ3> = HashMap::new();
    //     let num = vec![("a", 1), ("b", 2), ("c", 3)];
    //     num.into_iter().map(|i[0]| IntZ3::new(&ctx, &intsort, i[1]));
    //     // let m: HashMap<_, _> = tuples.into_iter().collect();
    //     println!("{:?}", num);

        // println!(
        //     "{:?}",
        //     vec!(("a", 1), ("b", 2), ("c", 3))).map(|i| ints.insert(i[0].to_string(), IntZ3::new(&ctx, &intsort, i[1])));

//         fn main() {
//     let v1 = (0u32..9).filter(|x| x % 2 == 0).map(|x| x.pow(2)).collect::<Vec<_>>();
//     let v2 = (1..10).filter(|x| x % 2 == 0).collect::<Vec<u32>>();

//     println!("{:?}", v1); // [0, 4, 16, 36, 64]
//     println!("{:?}", v2); // [2, 4, 6, 8]
// }

    // Is roughly equivalent to the Python

    // print({i+i: i*i for i in range(1, 5)})

    // Though conceptually, it's actually closer to

    // print("{!r}".format(dict(map(lambda i: (i+i, i*i), range(1, 5)))))

    // let mut ints: HashMap<String, i32> = HashMap::new();
    // ints.insert()
    // OptAssertZ3::new(&ctx, &opt, 
    //              LEZ3::new(&ctx, 
    //                    ADDZ3::new(&ctx, ))

    

    }
