use sp_smt::*;
use std::time::{Duration, Instant};
fn main(){

    // problem descriprion:
    println!("\n Finland's hardest sudoku:
 Applied mathematician Dr Arto Inkala, 41, from Finland, said: 
 I believe it is the most difficult puzzle I’ve ever created.

    - - 5 | 3 - - | - - -
    8 - - | - - - | - 2 -
    - 7 - | - 1 - | 5 - -
    ---------------------
    4 - - | - - 5 | 3 - -
    - 1 - | - 7 - | - - 6
    - - 3 | 2 - - | - 8 -
    ---------------------
    - 6 - | 5 - - | - - 9
    - - 4 | - - - | - 3 -
    - - - | - - 9 | 7 - - \n");

    // setup the solver context:
    let cfg = cfg_z3!();
    let ctx = ctx_z3!(&cfg);
    let slv = slv_z3!(&ctx);

    // variables:
    let row1 = vec!("x11", "x12", "x13", "x14", "x15", "x16", "x17", "x18", "x19");
    let row2 = vec!("x21", "x22", "x23", "x24", "x25", "x26", "x27", "x28", "x29");
    let row3 = vec!("x31", "x32", "x33", "x34", "x35", "x36", "x37", "x38", "x39");
    let row4 = vec!("x41", "x42", "x43", "x44", "x45", "x46", "x47", "x48", "x49");
    let row5 = vec!("x51", "x52", "x53", "x54", "x55", "x56", "x57", "x58", "x59");
    let row6 = vec!("x61", "x62", "x63", "x64", "x65", "x66", "x67", "x68", "x69");
    let row7 = vec!("x71", "x72", "x73", "x74", "x75", "x76", "x77", "x78", "x79");
    let row8 = vec!("x81", "x82", "x83", "x84", "x85", "x86", "x87", "x88", "x89");
    let row9 = vec!("x91", "x92", "x93", "x94", "x95", "x96", "x97", "x98", "x99");

    let rows = vec!(row1, row2, row3, row4, row5, row6, row7, row8, row9);

    let mut row1_z3 = vec!();
    let mut row2_z3 = vec!();
    let mut row3_z3 = vec!();
    let mut row4_z3 = vec!();
    let mut row5_z3 = vec!();
    let mut row6_z3 = vec!();
    let mut row7_z3 = vec!();
    let mut row8_z3 = vec!();
    let mut row9_z3 = vec!();

    let mut rows_z3 = vec!(row1_z3, row2_z3, row3_z3, row4_z3, row5_z3, row6_z3, row7_z3, row8_z3, row9_z3);

    let mut row_index = 0;
    for row in &rows {
        for elem in row {
            rows_z3[row_index].push(int_var_z3!(&ctx, elem));
            slv_assert_z3!(&ctx, &slv, gt_z3!(&ctx, int_var_z3!(&ctx, elem), int_z3!(&ctx, 0)));
            slv_assert_z3!(&ctx, &slv, lt_z3!(&ctx, int_var_z3!(&ctx, elem), int_z3!(&ctx, 10)));
        }
        row_index = row_index + 1;
    }

    //initial values from the table:
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x13"), int_z3!(&ctx, 5)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x14"), int_z3!(&ctx, 3)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x21"), int_z3!(&ctx, 8)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x28"), int_z3!(&ctx, 2)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x32"), int_z3!(&ctx, 7)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x35"), int_z3!(&ctx, 1)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x37"), int_z3!(&ctx, 5)));

    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x41"), int_z3!(&ctx, 4)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x46"), int_z3!(&ctx, 5)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x47"), int_z3!(&ctx, 3)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x52"), int_z3!(&ctx, 1)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x55"), int_z3!(&ctx, 7)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x59"), int_z3!(&ctx, 6)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x63"), int_z3!(&ctx, 3)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x64"), int_z3!(&ctx, 2)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x68"), int_z3!(&ctx, 8)));

    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x72"), int_z3!(&ctx, 6)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x74"), int_z3!(&ctx, 5)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x79"), int_z3!(&ctx, 9)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x83"), int_z3!(&ctx, 4)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x88"), int_z3!(&ctx, 3)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x96"), int_z3!(&ctx, 9)));
    slv_assert_z3!(&ctx, &slv, eq_z3!(&ctx, int_var_z3!(&ctx, "x97"), int_z3!(&ctx, 7)));

    // all different in a row constraints:
    for row in &rows_z3 {
        slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, row.to_vec()));
    }
    
    // all different in a column constraint:
    for i in 0..9 {
        let mut new_vec = vec!();
        for row in &rows_z3 {
            new_vec.push(row[i])
        }
        slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, new_vec.clone()));
    }

    // all different within the 3x3 cells:
    let mut cell11 = vec!();
    let mut cell12 = vec!();
    let mut cell13 = vec!();
    let mut cell21 = vec!();
    let mut cell22 = vec!();
    let mut cell23 = vec!();
    let mut cell31 = vec!();
    let mut cell32 = vec!();
    let mut cell33 = vec!();

    for i in 0..3 {
        for j in 0..3 {
            cell11.push(rows_z3[i][j]);
        }
        for j in 3..6 {
            cell12.push(rows_z3[i][j]);
        }
        for j in 6..9 {
            cell13.push(rows_z3[i][j]);
        }
    }

    for i in 3..6 {
        for j in 0..3 {
            cell21.push(rows_z3[i][j]);
        }
        for j in 3..6 {
            cell22.push(rows_z3[i][j]);
        }
        for j in 6..9 {
            cell23.push(rows_z3[i][j]);
        }
    }

    for i in 6..9 {
        for j in 0..3 {
            cell31.push(rows_z3[i][j]);
        }
        for j in 3..6 {
            cell32.push(rows_z3[i][j]);
        }
        for j in 6..9 {
            cell33.push(rows_z3[i][j]);
        }
    }

    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell11));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell12));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell13));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell21));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell22));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell23));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell31));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell32));
    slv_assert_z3!(&ctx, &slv, distinct_z3!(&ctx, cell33));

    let now = Instant::now();

    // check consistency:
    let res1 = slv_check_z3!(&ctx, &slv);
    if res1 == 1 {
        println!("SAT");
    } else if res1 == -1 {
        println!("UNSAT");
    } else {
        println!("UNDEF");
    };

    println!("Solving time: {} milliseconds", now.elapsed().as_millis());

    let model = slv_get_model_z3!(&ctx, &slv);
    println!("{}", model_to_string_z3!(&ctx, model));
}