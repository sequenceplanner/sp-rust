use failure::Error;
use sp_runner::*;

mod mecademic;
mod cubes;

fn main() -> Result<(), Error> {
    let (model, initial_state, _) = cubes::cubes();

    launch_model(model, initial_state)?;

    Ok(())
}
