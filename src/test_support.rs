//! Isolated filesystem fixtures, including when a test process reuses a PID.
use std::path::{Path, PathBuf};

pub struct TempDir(PathBuf);

impl TempDir {
    pub fn new(prefix: &str) -> Self {
        static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let id = NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        // Avoid reusing a dropped fixture's name while a failed test's child
        // process may still be unwinding. Atomic creation remains authoritative.
        Self::new_in(&std::env::temp_dir(), &format!("{prefix}-{time}-{id}"))
    }

    fn new_in(parent: &Path, prefix: &str) -> Self {
        for attempt in 0_u64.. {
            let path = parent.join(format!("{prefix}-{}-{attempt}", std::process::id()));
            // Claim the directory atomically. Existing files and directories
            // belong to another run, including one that may still be active.
            match std::fs::create_dir(&path) {
                Ok(()) => return Self(path),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("Cannot create test directory '{}': {error}", path.display()),
            }
        }
        unreachable!("exhausted temporary directory names")
    }

    pub fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        // Cleanup also runs during unwinding; never hide the original failure.
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn temporary_directories_skip_existing_files_and_directories() {
    let parent = TempDir::new("apex-temp-dir-test");
    let occupied = |attempt| {
        parent
            .path()
            .join(format!("fixture-{}-{attempt}", std::process::id()))
    };
    std::fs::create_dir(occupied(0)).unwrap();
    std::fs::write(occupied(0).join("sentinel"), "previous run").unwrap();
    std::fs::write(occupied(1), "previous file").unwrap();

    let first = TempDir::new_in(parent.path(), "fixture");
    let second = TempDir::new_in(parent.path(), "fixture");
    assert_eq!(first.path(), occupied(2));
    assert_eq!(second.path(), occupied(3));
    assert_eq!(std::fs::read_dir(first.path()).unwrap().count(), 0);
    assert_eq!(std::fs::read_dir(second.path()).unwrap().count(), 0);
    drop(first);
    assert!(!occupied(2).exists());
    assert!(second.path().is_dir());
    assert_eq!(
        std::fs::read_to_string(occupied(0).join("sentinel")).unwrap(),
        "previous run"
    );
    assert_eq!(
        std::fs::read_to_string(occupied(1)).unwrap(),
        "previous file"
    );
}

#[test]
fn temporary_directories_are_cleaned_up_after_a_panic() {
    let directory = TempDir::new("apex-temp-dir-unwind-test");
    let path = directory.path().to_owned();
    std::fs::create_dir(path.join("nested")).unwrap();
    std::fs::write(path.join("nested/file"), "fixture").unwrap();
    let result = std::panic::catch_unwind(move || {
        let _directory = directory;
        panic!("simulated test failure");
    });
    assert!(result.is_err());
    assert!(!path.exists());
}
