use failure::Error;
use sp_runner::*;

mod cubes;
mod mecademic;

fn main() -> Result<(), Error> {
    let (model, initial_state, _) = cubes::cubes();

    launch_model(model, initial_state)?;

    Ok(())
}
