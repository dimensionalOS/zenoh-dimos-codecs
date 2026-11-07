//! Putting a pipeline's streaming thread on a core of its own.
//!
//! GStreamer runs a pipeline's work on a thread it creates itself, named after the element it belongs to. Which core
//! that thread lands on is the scheduler's choice, and on a robot with four cores reserved for the gateway it put
//! every hot thread on **one** of them while three sat idle at full clock: 11 ms of decode took 30 ms of wall clock,
//! and a stream capable of 30 fps ran at 22.
//!
//! Pinning from outside cannot keep up, because the threads are not stable: an encoder's pipeline is rebuilt whenever
//! its picture changes size, which is a new thread each time, so a re-pinning loop always leaves a freshly rebuilt
//! encoder unpinned for as long as its interval — and an unpinned encoder contends, which costs frames, which drops
//! the budget, which changes the size, which rebuilds it again.
//!
//! So it is done here, in the moment between a pipeline reaching `PLAYING` and the first buffer being pushed into it:
//! the thread exists, has done no work yet, and a rebuild comes back through the same path.
//!
//! **Off unless `ZDC_PIN_PIPELINES` is set**, because which core a robot's video should run on is a decision for
//! whoever deploys it, not for a library. What this provides is the mechanism; the policy is the environment.

/// What `ZDC_PIN_PIPELINES` asks for.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum Scope {
    /// nothing is pinned; the scheduler places everything
    Off,
    /// decoders only, which is what `1` means — see [`scope`]
    Decode,
    /// decoders and encoders both (`all`)
    All,
}

/// What to pin, from `ZDC_PIN_PIPELINES`, read once.
///
/// **`1` means decoders only**, because that is the shape that was measured to work. A decode is the expensive stage
/// (12 ms of a 33 ms interval for 720p H.265 in software) and it is the one that needs a core to itself; an encode is
/// a few milliseconds and places itself perfectly well on whatever is left.
///
/// Pinning encoders as well was tried and is worse. A GStreamer encoder has **two** threads of its element's name —
/// `x264enc` keeps the appsrc's streaming task as well as its own, and `threads=1` caps only the latter — so the pin
/// confines both to one core, which took an encode from 7 ms to 24-55 ms and two cameras from 29 fps to 15-18. The
/// built-in software encoder, having no GStreamer pipeline at all, was never pinned and so never suffered that: the
/// run that passed was decoders pinned and encoders floating. `all` is kept for a machine where that turns out
/// differently, and says so in the log either way.
pub(crate) fn scope() -> Scope {
    static SCOPE: std::sync::OnceLock<Scope> = std::sync::OnceLock::new();
    *SCOPE.get_or_init(|| {
        let scope = std::env::var("ZDC_PIN_PIPELINES").map(|value| scope_from(value.trim())).unwrap_or(Scope::Off);
        if scope != Scope::Off {
            log::info!("ZDC_PIN_PIPELINES: pinning {}", if scope == Scope::All { "decoders and encoders" } else { "decoders only (`all` for encoders too)" });
        }
        scope
    })
}

/// `""`, `0`, `off`, `no`, `false` are off; `all`, `both`, `encode` include encoders; anything else is decoders only,
/// so `1` — the obvious thing to write — gets the shape that works rather than the one that does not.
fn scope_from(value: &str) -> Scope {
    match value.to_ascii_lowercase().as_str() {
        "" | "0" | "off" | "no" | "false" => Scope::Off,
        "all" | "both" | "encode" | "encoders" => Scope::All,
        _ => Scope::Decode,
    }
}

/// Whether `stage` is pinned, and so also whether it is given one thread (the two are only correct together).
pub(crate) fn pins(stage: Stage) -> bool {
    covers(scope(), stage)
}

fn covers(scope: Scope, stage: Stage) -> bool {
    match (scope, stage) {
        (Scope::Off, _) => false,
        (Scope::Decode, Stage::Encode) => false,
        _ => true,
    }
}

/// Which end of the reserved cores a stage takes, so decoders and encoders do not land on each other.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum Stage {
    /// from the front of the set
    Decode,
    /// from the back
    Encode,
}

/// The cores a stage may use: decoding takes the front of the set and encoding the back, **disjoint**, so a decoder
/// and an encoder can never be given the same core.
///
/// They were given the same core. The first version counted decoders up from the first core and encoders down from
/// the last, which only avoids a collision while the two stages' numbering is aligned — and it is not: encoder
/// instances are also made to *probe* an element and thrown away, so the encoder of a single camera was number 3
/// while its decoder was number 0, and both resolved to the same core. One camera starved itself with three cores
/// idle. Splitting the set makes that impossible rather than unlikely.
///
/// With an odd number of cores decoding takes the extra one (it is the more expensive stage); with one core there is
/// nothing to split and both stages share it, which is what a single core means.
pub(crate) fn cores_for(stage: Stage, cores: &[usize]) -> Vec<usize> {
    if cores.len() < 2 {
        return cores.to_vec();
    }
    let split = cores.len().div_ceil(2);
    match stage {
        Stage::Decode => cores[..split].to_vec(),
        // from the back, so the first encoder takes the last core
        Stage::Encode => cores[split..].iter().rev().copied().collect(),
    }
}

/// The core for the pipeline called `element`, which is the same core every time it is asked.
///
/// Assigned densely in the order the pipelines of a stage are first seen, not by their own numbers: those have gaps
/// (a probe takes one) and gaps put two streams on one core. Remembered by name, so an encoder rebuilt for a new
/// picture size comes back to the core it had — the whole point, since a rebuilt encoder landing somewhere new is
/// what the external pinning could not prevent.
pub(crate) fn core_for(stage: Stage, element: &str, cores: &[usize]) -> Option<usize> {
    static ASSIGNED: std::sync::OnceLock<std::sync::Mutex<Assignments>> = std::sync::OnceLock::new();
    let mut assigned = ASSIGNED.get_or_init(Default::default).lock().expect("the core assignments");
    assign(stage, element, cores, &mut assigned)
}

/// Which pipeline has which core, by element name.
type Assignments = std::collections::HashMap<String, usize>;

/// [`core_for`] against a given set of assignments rather than the process's, so what it decides can be tested
/// without the order other tests ran in deciding it too.
fn assign(stage: Stage, element: &str, cores: &[usize], assigned: &mut Assignments) -> Option<usize> {
    if let Some(core) = assigned.get(element) {
        return Some(*core);
    }
    let within = cores_for(stage, cores);
    // the next core of this stage that nothing has yet, counted among what this stage may use
    let core = *within.get(assigned.values().filter(|core| within.contains(core)).count() % within.len().max(1))?;
    assigned.insert(element.to_owned(), core);
    Some(core)
}

/// A thread's name as Linux reports it: `comm` keeps 15 characters, and GStreamer appends `:src` to the element's.
pub(crate) fn thread_name(element: &str) -> String {
    let full = format!("{element}:src");
    full.chars().take(15).collect()
}

/// Pins the streaming thread of the pipeline whose appsrc is called `element`, if pinning is on.
///
/// The core is remembered against `element`, so an encoder rebuilt for a new picture size comes back to the core it
/// had: a rebuilt encoder landing somewhere new is the thing this exists to prevent.
///
/// Called with the pipeline already `PLAYING` and nothing pushed into it yet. Reports what it did, and what it could
/// not: a thread that cannot be found or pinned is a stream that runs where the scheduler likes, which is worth
/// saying rather than leaving to be inferred from a frame rate.
pub(crate) fn pin(stage: Stage, element: &str) {
    if !pins(stage) {
        return;
    }
    let cores = reserved_cores();
    let Some(core) = core_for(stage, element, &cores) else {
        log::warn!("ZDC_PIN_PIPELINES is set but this process has no cores to choose from; {element} runs where it is put");
        return;
    };
    let name = thread_name(element);
    match wait_for_threads(&name) {
        found if found.is_empty() => log::warn!("ZDC_PIN_PIPELINES: no thread called {name} within {APPEAR_WITHIN:?}, so {element} was not put on core {core} and runs where the scheduler likes"),
        found => {
            // More than one thread of that name is a decoder that has made workers, one per core, which all end up
            // here: the ones already made are pinned now and the ones made later inherit it. Confining a whole pool
            // to one core is slower than leaving it alone, so this is worth saying rather than discovering as a frame
            // rate. Measured: `avdec_h265` with its default threading made 16 of them on a 16-core machine.
            if found.len() > 1 {
                log::warn!(
                    "{element} has {} threads, so pinning puts its whole worker pool on core {core} — that is slower than not pinning. Give the decoder one worker (`ZDC_H265_GST_DECODER=\"avdec_h265 max-threads=1\"`) or turn ZDC_PIN_PIPELINES off",
                    found.len()
                );
            }
            for tid in &found {
                match set_affinity(*tid, core) {
                    Ok(()) => log::info!("{element} runs on core {core} (thread {name}, tid {tid})"),
                    Err(error) => log::warn!("{element}: thread {name} (tid {tid}) could not be put on core {core}: {error}"),
                }
            }
        }
    }
}

/// How long to let a pipeline's streaming thread appear. A pipeline reaching `PLAYING` has started its task, but the
/// thread behind it is the scheduler's to create, and a state change can complete asynchronously — so this polls for
/// it rather than assuming. Nothing has been pushed into the pipeline yet, so the wait costs a few milliseconds once
/// per pipeline and buys the difference between a stream that is placed and one that silently is not.
const APPEAR_WITHIN: std::time::Duration = std::time::Duration::from_millis(50);

/// The threads called `name`, once there are any or `APPEAR_WITHIN` has passed.
fn wait_for_threads(name: &str) -> Vec<Thread> {
    let until = std::time::Instant::now() + APPEAR_WITHIN;
    loop {
        let found = threads_named(name);
        if !found.is_empty() || std::time::Instant::now() >= until {
            return found;
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

/// The cores this process may use, in order: its own affinity, so pinning can never reach for a core the service was
/// not given (a unit file's `CPUAffinity=4-7` is the set, and this stays inside it).
#[cfg(target_os = "linux")]
fn reserved_cores() -> Vec<usize> {
    // SAFETY: a zeroed set the kernel fills in, read back through libc's own accessor
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        if libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) != 0 {
            return Vec::new();
        }
        (0..libc::CPU_SETSIZE as usize).filter(|cpu| libc::CPU_ISSET(*cpu, &set)).collect()
    }
}

/// The threads of this process called `name`. More than one means a decoder has already made workers, which inherit
/// the affinity set on their parent.
/// A thread of this process, as the kernel numbers them.
#[cfg(target_os = "linux")]
type Thread = libc::pid_t;
#[cfg(not(target_os = "linux"))]
type Thread = i32;

#[cfg(target_os = "linux")]
fn threads_named(name: &str) -> Vec<Thread> {
    let Ok(entries) = std::fs::read_dir("/proc/self/task") else { return Vec::new() };
    entries
        .flatten()
        .filter_map(|entry| {
            let tid = entry.file_name().to_string_lossy().parse::<libc::pid_t>().ok()?;
            let comm = std::fs::read_to_string(entry.path().join("comm")).ok()?;
            (comm.trim() == name).then_some(tid)
        })
        .collect()
}

#[cfg(target_os = "linux")]
fn set_affinity(tid: Thread, core: usize) -> std::io::Result<()> {
    // SAFETY: a set with one core in it, applied to a thread of this process
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_SET(core, &mut set);
        if libc::sched_setaffinity(tid, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        Ok(())
    }
}

// Elsewhere there is nothing to do: `reserved_cores` is empty, so `pin` says so once and leaves the threads alone.
#[cfg(not(target_os = "linux"))]
fn reserved_cores() -> Vec<usize> {
    Vec::new()
}

#[cfg(not(target_os = "linux"))]
fn threads_named(_name: &str) -> Vec<Thread> {
    Vec::new()
}

#[cfg(not(target_os = "linux"))]
fn set_affinity(_tid: Thread, _core: usize) -> std::io::Result<()> {
    Err(std::io::Error::new(std::io::ErrorKind::Unsupported, "only Linux pins threads here"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `1` — the obvious thing to write — has to mean the shape that works: decoders pinned, encoders floating. A
    /// GStreamer encoder has two threads of its element's name (its own, capped by `threads=1`, and the appsrc's
    /// streaming task, which is not), so pinning it confines both to one core: an encode went from 7 ms to 24-55 ms
    /// and two cameras from 29 fps to 15-18. The run that passed had the encoders floating.
    #[test]
    fn asking_to_pin_pins_decoders_and_leaves_encoders_alone() {
        assert_eq!(scope_from("1"), Scope::Decode, "the obvious thing to write gets the shape that works");
        assert!(covers(Scope::Decode, Stage::Decode) && !covers(Scope::Decode, Stage::Encode));
        // off, in all the ways someone might write it
        for off in ["", "0", "off", "no", "false", "OFF"] {
            assert_eq!(scope_from(off), Scope::Off, "{off:?}");
        }
        assert!(!covers(Scope::Off, Stage::Decode) && !covers(Scope::Off, Stage::Encode));
        // and encoders only on an explicit ask
        for all in ["all", "both", "encode", "ALL"] {
            assert_eq!(scope_from(all), Scope::All, "{all:?}");
        }
        assert!(covers(Scope::All, Stage::Decode) && covers(Scope::All, Stage::Encode));
        // anything else is the safe shape rather than an error: a typo in a unit file should not decide this
        assert_eq!(scope_from("yes please"), Scope::Decode);
    }

    /// **A decoder and an encoder must never be given the same core.** They were: the first version counted decoders
    /// up from the first core and encoders down from the last, which collides as soon as the two stages' numbering
    /// differs — and it does, because encoder instances are also made to probe an element and thrown away. One
    /// camera's decoder (number 0) and its encoder (number 3) both landed on core 4 of 4-7, and the camera starved
    /// itself at 20.7 fps with three cores idle. Disjoint sets make it impossible instead of unlikely.
    #[test]
    fn a_decoder_and_an_encoder_never_share_a_core() {
        let cores = vec![4, 5, 6, 7];
        let (decode, encode) = (cores_for(Stage::Decode, &cores), cores_for(Stage::Encode, &cores));
        assert_eq!((decode.clone(), encode.clone()), (vec![4, 5], vec![7, 6]), "the front for decoding, the back for encoding");
        assert!(decode.iter().all(|core| !encode.contains(core)), "nothing in common, whatever the numbering");
    }

    /// Two cores is one each; one core cannot be split, and a set it was not given cannot be reached.
    #[test]
    fn it_stays_inside_the_cores_it_was_given() {
        assert_eq!(cores_for(Stage::Decode, &[4, 5]), vec![4], "two cores, one each");
        assert_eq!(cores_for(Stage::Encode, &[4, 5]), vec![5]);
        // an odd set gives decoding the extra core, it being the dearer stage
        assert_eq!((cores_for(Stage::Decode, &[4, 5, 6]), cores_for(Stage::Encode, &[4, 5, 6])), (vec![4, 5], vec![6]));
        // and with one core there is nothing to split
        assert_eq!((cores_for(Stage::Decode, &[6]), cores_for(Stage::Encode, &[6])), (vec![6], vec![6]));
        assert!(cores_for(Stage::Decode, &[]).is_empty());
    }

    /// Cores are handed out in the order pipelines are first seen, not by their own numbers, because those have gaps
    /// — and a gap put two streams on one core. The same element always gets the same core back, which is what an
    /// encoder rebuilt for a new picture size relies on.
    #[test]
    fn each_pipeline_keeps_the_core_it_was_given() {
        let cores = vec![4, 5, 6, 7];
        // its own assignments, so what another test did cannot change what this one sees
        let mut assigned = Assignments::default();
        // the numbers here are deliberately gappy, as a probe or a rebuild leaves them
        let front_decode = assign(Stage::Decode, "h265src0", &cores, &mut assigned);
        let front_encode = assign(Stage::Encode, "encsrc3", &cores, &mut assigned);
        let rear_decode = assign(Stage::Decode, "h265src1", &cores, &mut assigned);
        let rear_encode = assign(Stage::Encode, "encsrc9", &cores, &mut assigned);
        assert_eq!((front_decode, rear_decode), (Some(4), Some(5)), "a core each, densely, whatever the numbers say");
        assert_eq!((front_encode, rear_encode), (Some(7), Some(6)));
        let four = [front_decode, front_encode, rear_decode, rear_encode];
        assert_eq!(four.iter().flatten().collect::<std::collections::BTreeSet<_>>().len(), 4, "four pipelines, four cores");
        // asked again — a rebuilt encoder — it comes back to the same core
        assert_eq!(assign(Stage::Encode, "encsrc3", &cores, &mut assigned), front_encode);
        assert_eq!(assign(Stage::Decode, "h265src0", &cores, &mut assigned), front_decode);
        // and a third camera on four cores wraps within its stage rather than running off the end
        assert_eq!(assign(Stage::Decode, "h265src2", &cores, &mut assigned), Some(4));
    }

    /// Linux keeps 15 characters of a thread's name, so what to look for is the truncation and not the whole.
    #[test]
    fn the_thread_name_is_what_linux_would_report() {
        assert_eq!(thread_name("h265src0"), "h265src0:src");
        assert_eq!(thread_name("encsrc12"), "encsrc12:src");
        assert_eq!(thread_name("a_very_long_element_name").len(), 15, "15 characters, as `comm` keeps");
    }

    /// A thread that appears a moment after the pipeline started is still found, and one that never appears gives up
    /// rather than holding the stream: a pipeline reaching `PLAYING` has started its task, but the thread behind it is
    /// the scheduler's to make.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_thread_that_appears_late_is_still_found() {
        let (go, waiting) = std::sync::mpsc::channel::<()>();
        let late = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(10));
            std::thread::Builder::new().name("latepin:src".to_owned()).spawn(move || waiting.recv()).expect("a thread")
        });
        assert_eq!(wait_for_threads("latepin:src").len(), 1, "it waits for the thread to appear");
        drop(go);
        let _ = late.join().expect("the spawner").join();

        // and a name that never appears costs the wait once, not the stream
        let started = std::time::Instant::now();
        assert!(wait_for_threads("nosuchthread:s").is_empty());
        assert!(started.elapsed() < APPEAR_WITHIN * 3, "it gives up rather than waiting on nothing");
    }

    /// The lookup finds a thread by the name Linux reports, which is the half that cannot be reasoned about.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_named_thread_can_be_found_by_that_name() {
        let (go, waiting) = std::sync::mpsc::channel::<()>();
        let named = std::thread::Builder::new().name("pintest:src".to_owned()).spawn(move || waiting.recv()).expect("a thread");
        // it is running and has its name by the time it is waiting
        let mut found = Vec::new();
        for _ in 0..200 {
            found = threads_named("pintest:src");
            if !found.is_empty() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(found.len(), 1, "exactly the one thread called that");
        // and it has cores to be put on, this being a machine with a scheduler
        let cores = reserved_cores();
        assert!(!cores.is_empty(), "this process can run somewhere");

        // the half that cannot be reasoned about either: that the call lands. Put it on one core and read back what
        // the kernel thinks, since a pin that silently did nothing is the failure this whole thing exists to prevent
        let core = cores[cores.len() - 1];
        set_affinity(found[0], core).expect("a thread of this process can be pinned");
        assert_eq!(affinity_of(found[0]), vec![core], "the kernel agrees it is on that core alone");

        drop(go);
        let _ = named.join();
    }

    /// What the kernel says a thread's affinity is, for the test above to check against.
    #[cfg(target_os = "linux")]
    fn affinity_of(tid: libc::pid_t) -> Vec<usize> {
        // SAFETY: a zeroed set the kernel fills in for a thread of this process
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            assert_eq!(libc::sched_getaffinity(tid, std::mem::size_of::<libc::cpu_set_t>(), &mut set), 0, "reading a thread's affinity");
            (0..libc::CPU_SETSIZE as usize).filter(|cpu| libc::CPU_ISSET(*cpu, &set)).collect()
        }
    }
}
