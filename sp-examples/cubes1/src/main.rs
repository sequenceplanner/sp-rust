use failure::Error;
use sp_runner::*;

fn main() -> Result<(), Error> {
    let (model, initial_state, _) = cubes();

    launch_model(model, initial_state)?;

    Ok(())
}
