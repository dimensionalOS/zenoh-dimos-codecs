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

/// Whether to pin at all (`ZDC_PIN_PIPELINES`), read once.
pub(crate) fn pinning() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("ZDC_PIN_PIPELINES").is_ok_and(|value| !value.trim().is_empty() && value.trim() != "0"))
}

/// Which end of the reserved cores a stage takes, so decoders and encoders do not land on each other.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum Stage {
    /// from the front of the set
    Decode,
    /// from the back
    Encode,
}

/// The core for the `index`th pipeline of `stage`, out of `cores` (the process's own, in order).
///
/// Decoders count up from the first and encoders down from the last, so two cameras on four cores put their decoders
/// on the first two and their encoders on the last two without either knowing about the other. With more pipelines
/// than cores they wrap, which is the scheduler's problem again but at least a predictable one.
pub(crate) fn core_for(stage: Stage, index: usize, cores: &[usize]) -> Option<usize> {
    let count = cores.len();
    if count == 0 {
        return None;
    }
    Some(match stage {
        Stage::Decode => cores[index % count],
        Stage::Encode => cores[count - 1 - (index % count)],
    })
}

/// A thread's name as Linux reports it: `comm` keeps 15 characters, and GStreamer appends `:src` to the element's.
pub(crate) fn thread_name(element: &str) -> String {
    let full = format!("{element}:src");
    full.chars().take(15).collect()
}

/// Pins the streaming thread of the pipeline whose appsrc is called `element`, if pinning is on.
///
/// `number` is the decoder's or encoder's own (the one in its element names), **not** a count of pipelines pinned: an
/// encoder is rebuilt whenever its picture changes size, and it has to come back to the same core each time or the
/// thing this exists to prevent happens on every resolution change.
///
/// Called with the pipeline already `PLAYING` and nothing pushed into it yet. Reports what it did, and what it could
/// not: a thread that cannot be found or pinned is a stream that runs where the scheduler likes, which is worth
/// saying rather than leaving to be inferred from a frame rate.
pub(crate) fn pin(stage: Stage, element: &str, number: u64) {
    if !pinning() {
        return;
    }
    let cores = reserved_cores();
    let Some(core) = core_for(stage, number as usize, &cores) else {
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

    /// Two cameras on four cores: each decoder on its own core from the front, each encoder from the back, and
    /// neither needing to know about the other. The numbers are the pipelines' own, so a rebuilt encoder comes back
    /// to the core it had.
    #[test]
    fn two_streams_on_four_cores_take_a_core_each() {
        let cores = vec![4, 5, 6, 7];
        assert_eq!(core_for(Stage::Decode, 0, &cores), Some(4));
        assert_eq!(core_for(Stage::Decode, 1, &cores), Some(5));
        assert_eq!(core_for(Stage::Encode, 0, &cores), Some(7));
        assert_eq!(core_for(Stage::Encode, 1, &cores), Some(6));
        // the same number gives the same core however many times it is asked, which is what a rebuild relies on
        assert_eq!(core_for(Stage::Encode, 0, &cores), Some(7));
    }

    /// It can only ever use what the process was given, and says nothing is possible rather than guessing.
    #[test]
    fn it_stays_inside_the_cores_it_was_given() {
        assert_eq!(core_for(Stage::Decode, 0, &[]), None, "no cores, no pinning");
        assert_eq!(core_for(Stage::Decode, 0, &[6]), Some(6));
        assert_eq!(core_for(Stage::Encode, 3, &[6]), Some(6), "one core takes everything");
        // more pipelines than cores wrap rather than running off the end
        let cores = vec![4, 5];
        assert_eq!((core_for(Stage::Decode, 2, &cores), core_for(Stage::Encode, 2, &cores)), (Some(4), Some(5)));
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
