#[macro_use]
mod modeling;
mod testing_model;
mod planning;


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (model, initial_state) = testing_model::cylinders::cylinders();

    let json = serde_json::to_string(&model)?;
    println!("{}", json);

    Ok(())

}



