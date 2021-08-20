use sp_runner::*;
use sp_domain::*;

#[tokio::main]
async fn main(){
    launch_model(Model::new("empty"), SPState::new()).await.unwrap();
}