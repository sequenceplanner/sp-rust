use sp_domain::*;
use serde::{Deserialize, Serialize};
use std::process::Stdio;
use std::process::Command;
use std::io::Write;

#[derive(Serialize, Deserialize)]
#[serde(tag = "type")]
enum Request {
    Refine { ts_model: TransitionSystemModel, pred: Predicate },
    Clean { ts_model: TransitionSystemModel, pred: Predicate },
}

fn do_request(cmd: &[u8]) -> std::io::Result<String> {
    let mut process = Command::new("sp-fm")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()?;

    let mut stdin = process.stdin.take().unwrap();
    stdin.write_all(cmd)?;
    // must close stdin to avoid blocking in the process
    drop(stdin);

    // block on result
    let result = process.wait_with_output()?;
    assert!(result.status.success());

    Ok(String::from_utf8(result.stdout)
       .expect("Check character encoding"))
}

pub fn refine_invariant(
    mut ts_model: TransitionSystemModel,
    invariant: Predicate
) -> std::io::Result<Predicate> {

    // clean out ts model to avoid serializing unneded things.
    ts_model
        .transitions
        .retain(|t| t.type_ == TransitionType::Auto ||
                t.type_ == TransitionType::Effect);
    ts_model.invariants.clear();

    let command = Request::Refine { ts_model, pred: invariant };
    let command = serde_json::to_string(&command)?;

    let str = do_request(command.as_bytes())?;

    let pred: Predicate = serde_json::from_str(&str)?;

    Ok(pred)
}
