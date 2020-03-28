use failure::Error;
use sp_runner::*;

mod camera;
mod control_box;
mod cylinders;
mod dorna;

fn main() -> Result<(), Error> {
    let (model, initial_state, _) = cylinders::cylinders();

    launch_model(model, initial_state)?;

    Ok(())
}
