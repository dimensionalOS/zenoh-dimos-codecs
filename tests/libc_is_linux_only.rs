//! Fails if anything outside the core-pinning module reaches for `libc`.
//!
//! `libc` is declared under `[target.'cfg(target_os = "linux")'.dependencies]`, for one reason: putting a GStreamer
//! pipeline's streaming thread on a core of its own (`encoders::pin`). So on macOS the crate is built with no `libc`
//! at all, and any use of it outside that module is a compile error **only there**.
//!
//! That is a very expensive place for the error to surface. This crate has no CI, and the first thing that compiles it
//! for darwin is dimos-desktop's release build — so `libc::EAGAIN` in `utils::av1::try_picture` (added with the rav1d
//! decoder, correct on Linux, invisible to every test here) was first reported by a failed Desktop release, about 45
//! minutes after the tag was pushed, with three of four platforms red. A grep is a poor substitute for building for a
//! second target, but it costs nothing and it catches exactly the mistake that was made.
//!
//! Reaching for an errno is the tempting case, and it does not need `libc`: `std::io::Error::from_raw_os_error` knows
//! each platform's numbers, so compare its `.kind()` against the `ErrorKind` instead (EAGAIN is 11 on Linux and 35 on
//! macOS, and both map to `WouldBlock`).

use std::path::{Path, PathBuf};

/// The only module allowed to name `libc`, relative to `src/`.
const PINNING: &str = "encoders/pin.rs";

fn sources(directory: &Path, found: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(directory).expect("src/ is readable").flatten() {
        let path = entry.path();
        if path.is_dir() {
            sources(&path, found);
        } else if path.extension().is_some_and(|extension| extension == "rs") {
            found.push(path);
        }
    }
}

#[test]
fn only_the_core_pinning_module_uses_libc() {
    let src = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    sources(&src, &mut files);
    assert!(files.len() > 10, "found only {} source files under {}", files.len(), src.display());

    let mut offenders = Vec::new();
    for file in &files {
        let relative = file.strip_prefix(&src).expect("walked from src/");
        if relative == Path::new(PINNING) {
            continue;
        }
        let text = std::fs::read_to_string(file).expect("a source file is readable");
        for (number, line) in text.lines().enumerate() {
            if line.contains("libc::") {
                offenders.push(format!("{}:{}: {}", relative.display(), number + 1, line.trim()));
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "libc is a Linux-only dependency of this crate, so these uses do not compile for macOS (the only module that \
         may name it is {PINNING}); for an errno, compare `std::io::Error::from_raw_os_error(..).kind()` against the \
         `ErrorKind` instead:\n  {}",
        offenders.join("\n  ")
    );
}
