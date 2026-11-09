//! Fails if a `[patch.crates-io]` entry in this crate's manifest is **inert**.
//!
//! Cargo ignores a patch it cannot satisfy — the patched version has to match what something in the graph asks for —
//! and says so only as a *warning* ("patch ... was not used in the crate graph") in a build log nobody reads once the
//! build is green. This crate has been bitten twice, in adjacent lines of one file:
//!
//! - `zenoh-gateway = "0.5.37"` with the patch pointing at a local 0.6.x checkout: the patch was ignored, the registry
//!   0.5.x was built, and the result was a wall of "method is not a member of trait VideoEncoder" errors that read
//!   exactly like the gateway missing an API it has.
//! - `zenoh = { tag = "zenoh-1.6.2-patch.1" }` after the gateway moved to `=1.10.1`: the patch was ignored and every
//!   test here ran against *unpatched* zenoh. The patch exists for a routing fix (puts dropped after a publisher is
//!   undeclared), so the symptom of losing it is dropped samples — which reads as a gateway or a network problem, and
//!   would never have been traced to the manifest.
//!
//! The first cost an hour and the second went unnoticed for weeks. `Cargo.lock` is where the answer is: it records the
//! `source` each package really resolved to, so a patch that took effect leaves a non-registry source behind.

use std::path::PathBuf;

/// The names under `[patch.crates-io]` in Cargo.toml, in order.
fn patched_names(manifest: &str) -> Vec<String> {
    let mut names = Vec::new();
    let mut inside = false;
    for line in manifest.lines() {
        let line = line.trim();
        if line.starts_with('[') {
            inside = line == "[patch.crates-io]";
            continue;
        }
        if !inside || line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((name, _)) = line.split_once('=') {
            names.push(name.trim().trim_matches('"').to_owned());
        }
    }
    names
}

/// Every `source` line Cargo.lock records for the package called `name`.
fn locked_sources(lock: &str, name: &str) -> Vec<String> {
    let mut sources = Vec::new();
    for package in lock.split("[[package]]") {
        let field = |key: &str| {
            package
                .lines()
                .map(str::trim)
                .find_map(|line| line.strip_prefix(key)?.split_once('=').map(|(_, value)| value.trim().trim_matches('"').to_owned()))
        };
        // a package with no `source` is a path dependency, which is a patch that took effect
        if field("name").as_deref() == Some(name) {
            sources.push(field("source").unwrap_or_else(|| "path".to_owned()));
        }
    }
    sources
}

/// A patch that cargo could not apply is silently ignored, so each one has to be shown to have taken effect: the
/// package it names must resolve to something that is not the registry.
///
/// Read `Cargo.lock` rather than asking the crates at runtime, because the point is to catch a patch that *did not*
/// apply — and a crate built from the registry cannot be asked whether it should have come from somewhere else.
#[test]
fn every_declared_patch_actually_applied() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let manifest = std::fs::read_to_string(root.join("Cargo.toml")).expect("Cargo.toml");
    let lock = std::fs::read_to_string(root.join("Cargo.lock")).expect("Cargo.lock");
    let names = patched_names(&manifest);
    assert!(!names.is_empty(), "no [patch.crates-io] entries found: either the section moved or this test stopped reading it");
    for name in &names {
        let sources = locked_sources(&lock, name);
        assert!(!sources.is_empty(), "Cargo.lock has no package called {name:?}, which [patch.crates-io] patches: the patch cannot be doing anything");
        let patched: Vec<&String> = sources.iter().filter(|source| !source.starts_with("registry+")).collect();
        assert!(
            !patched.is_empty(),
            "[patch.crates-io] patches {name:?} but Cargo.lock resolved it only from the registry ({sources:?}), so the patch is inert and the \
             unpatched crate is what gets built. Cargo says this as a warning only. Usually the version requirement no longer matches what the \
             patch provides — check what asks for {name:?} and what the patch points at, then `cargo update -p {name}`."
        );
        // a patched name resolving from the registry *as well* is two copies of one crate in the graph, which is its
        // own bug: the patched one is not necessarily the one in use
        assert_eq!(
            sources.len(),
            patched.len(),
            "{name:?} resolved both from the registry and from the patch ({sources:?}): two copies of one crate are in the graph, so which one \
             a given dependency got is down to its version requirement"
        );
    }
}

#[test]
fn the_manifest_and_lock_readers_find_what_is_there() {
    let manifest = "[dependencies]\nserde = \"1\"\n\n[patch.crates-io]\n# a comment\nzenoh = { git = \"g\", tag = \"t\" }\nzenoh-gateway = { path = \"../x\" }\n\n[dev-dependencies]\ntokio = \"1\"\n";
    assert_eq!(patched_names(manifest), ["zenoh", "zenoh-gateway"], "only the patch section, and not the comment");
    let lock = "[[package]]\nname = \"serde\"\nversion = \"1.0\"\nsource = \"registry+https://x\"\n\n[[package]]\nname = \"zenoh\"\nversion = \"1.10.1\"\nsource = \"git+https://g?tag=t#abc\"\n\n[[package]]\nname = \"zenoh-gateway\"\nversion = \"0.6.8\"\n";
    assert_eq!(locked_sources(lock, "serde"), ["registry+https://x"]);
    assert_eq!(locked_sources(lock, "zenoh"), ["git+https://g?tag=t#abc"]);
    assert_eq!(locked_sources(lock, "zenoh-gateway"), ["path"], "a path dependency has no source line, and that counts as patched");
    assert!(locked_sources(lock, "absent").is_empty());
}
