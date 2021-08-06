use which;

pub fn find_nuxmv() -> &'static str {
    let supported = &[ "nuXmv", "nuxmv", "nusmv" ];
    for f in supported.iter() {
        if which::which(f).is_ok() {
            return f;
        }
    }
    panic!("nu[X|x|s]mv not found. check your path!");
}
