use failure::Error;
use sp_runner::*;

mod mecademic;
mod simple_monolithic;

fn main() -> Result<(), Error> {
    let (model, initial_state, _) = simple_monolithic::simple_monolithic();

    launch_model(model, initial_state)?;

    Ok(())
}
