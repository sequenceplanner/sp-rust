use failure::Error;
use sp_runner::*;

mod cylinders;
mod dorna;
mod control_box;

fn main() -> Result<(), Error> {
    let (model, initial_state, _) = cylinders::cylinders();

    launch_model(model, initial_state)?;

    Ok(())
}
