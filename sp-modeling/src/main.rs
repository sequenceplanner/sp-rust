#[macro_use]
pub mod modeling;
pub mod testing_model;
pub mod planning;



#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (model, initial_state) = testing_model::cylinders::cylinders();

    let res = modeling::compile_model(&model, initial_state, true);

    let json = serde_json::to_string(&model)?;
    println!("{}", json);



    Ok(())

}



