// The intention of this build script is to make available any binary tools
// that SP depend on, for example SAT solvers.

// As this is currently not possible to do using just cargo,
// we use this temporary hack from:
// https://github.com/rust-lang/cargo/issues/2267
// while we wait for
// https://github.com/rust-lang/rfcs/blob/master/text/3028-cargo-binary-dependencies.md
// to be implemented.

// Each binary dependency should have a crate in the root of the
// workspace that prepare the needed binaries.

use std::process::Command;

fn visit_dirs(dir: std::path::PathBuf) {
    println!("cargo:rerun-if-changed={}", dir.as_os_str().to_str().unwrap());

    for entry in std::fs::read_dir(dir).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();

        if path.is_dir() {
            visit_dirs(path);
        } else {
            println!("cargo:rerun-if-changed={}", path.as_os_str().to_str().unwrap());
        }
    }
}

fn main() {
    let mdir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let path = std::path::Path::new(&mdir).join("..");

    Command::new("cargo").args(&["install", "--path=sp-fm"])
        .current_dir(path.clone())
        .spawn().unwrap().wait().unwrap();

    visit_dirs(path.join("sp-fm"));

    println!("cargo:rerun-if-changed=build.rs");
}
