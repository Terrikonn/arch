use std::env;

fn main() {
    let target = env::var("TARGET").expect("TARGET environment variable must be set");

    // not supported on non-x86 platforms
    if !target.starts_with("x86_64") && !target.starts_with("i686") && !target.starts_with("i586") {
        return;
    }
}
