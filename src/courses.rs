//! Shared order for the bundled track switcher and full-course driving checks,
//! plus the game root that bundled tracks and saved records resolve against.
use std::io::ErrorKind;
use std::path::{Component, Path, PathBuf};

pub const BUNDLED_PATHS: &[&str] = &[
    "tracks/alpine.track",
    "tracks/club.track",
    "tracks/camber_loop.track",
    "tracks/ridgeway.track",
    "tracks/stone_gallery.track",
    "tracks/airfield.track",
    "tracks/skyline_eight.track",
];

/// Directories that may be the game root, in priority order: the working
/// directory, the executable's directory, then each of that directory's
/// ancestors (so `target/release/` and `target/<triple>/debug/` builds find
/// the checkout that built them).
pub fn root_candidates() -> Vec<PathBuf> {
    let mut candidates: Vec<PathBuf> = Vec::new();
    let mut add = |directory: &Path| {
        if !candidates.iter().any(|known| known == directory) {
            candidates.push(directory.to_owned());
        }
    };
    if let Ok(directory) = std::env::current_dir() {
        add(&directory);
    }
    if let Ok(executable) = std::env::current_exe() {
        let absolute = std::path::absolute(&executable).unwrap_or(executable.clone());
        absolute.ancestors().skip(1).for_each(&mut add);
        // A symlinked executable (for example in ~/bin) still finds its build.
        // Windows canonical paths carry a verbatim prefix; skip them there.
        #[cfg(unix)]
        if let Ok(target) = executable.canonicalize() {
            target.ancestors().skip(1).for_each(&mut add);
        }
    }
    // Keep tests working when CARGO_TARGET_DIR lives outside the checkout.
    // Release builds never embed the builder's source path.
    #[cfg(test)]
    add(Path::new(env!("CARGO_MANIFEST_DIR")));
    candidates
}

/// The first root candidate whose `tracks/` folder holds every bundled course,
/// or failing that, the first holding any of them: a copy missing a course
/// still keeps its records together. On failure, returns every location
/// that was tried.
pub fn game_root() -> Result<PathBuf, Vec<PathBuf>> {
    let candidates = root_candidates();
    match root_among(&candidates) {
        Some(root) => Ok(root.clone()),
        None => Err(candidates),
    }
}

fn root_among(candidates: &[PathBuf]) -> Option<&PathBuf> {
    // Any inspection error, including a file named `tracks`, is "no match".
    let found = |directory: &Path| -> Vec<bool> {
        BUNDLED_PATHS
            .iter()
            .map(|path| directory.join(path).is_file())
            .collect()
    };
    candidates
        .iter()
        .find(|directory| !found(directory).contains(&false))
        .or_else(|| {
            candidates
                .iter()
                .find(|directory| found(directory).contains(&true))
        })
}

/// The first bundled course from `start` onward, wrapping, that can be
/// found, so the switcher skips a course missing from this copy. If none
/// can, `start` itself, whose load then reports why.
fn first_available(start: usize) -> PathBuf {
    first_available_where(start, |path| {
        resolve_track(Path::new(path)).is_ok_and(|resolved| resolved.is_file())
    })
}

fn first_available_where(start: usize, available: impl Fn(&str) -> bool) -> PathBuf {
    let count = BUNDLED_PATHS.len();
    (0..count)
        .map(|offset| BUNDLED_PATHS[(start + offset) % count])
        .find(|path| available(path))
        .unwrap_or(BUNDLED_PATHS[start % count])
        .into()
}

/// The bundled entry a relative path names, such as `./tracks/club.track`.
fn bundled_name(path: &Path) -> Option<&'static str> {
    if !path.is_relative() {
        return None;
    }
    let normalized: PathBuf = path
        .components()
        .filter(|component| *component != Component::CurDir)
        .collect();
    BUNDLED_PATHS
        .iter()
        .copied()
        .find(|bundled| Path::new(bundled) == normalized)
}

fn absent(error: &std::io::Error) -> bool {
    matches!(error.kind(), ErrorKind::NotFound | ErrorKind::NotADirectory)
}

pub fn quoted_list(paths: &[PathBuf]) -> String {
    paths
        .iter()
        .map(|path| format!("'{}'", path.display()))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Select a source once, retaining its lexical absolute path for later reloads.
/// Do not canonicalize: an editor may replace a file or retarget its symlink.
///
/// Paths resolve against the working directory. Only when a relative path
/// names a bundled course (`tracks/club.track`) and nothing exists there does
/// it fall back to the same file in the game root.
pub fn resolve_track(path: &Path) -> Result<PathBuf, String> {
    resolve_track_in(path, game_root)
}

fn resolve_track_in(
    path: &Path,
    game_root: impl FnOnce() -> Result<PathBuf, Vec<PathBuf>>,
) -> Result<PathBuf, String> {
    let local = std::path::absolute(path)
        .map_err(|error| format!("Cannot resolve track '{}': {error}", path.display()))?;
    let Some(bundled) = bundled_name(path) else {
        // The loader reports a missing file, naming this exact location.
        return Ok(local);
    };
    // Keep an existing directory entry selected even when it is a symlink
    // whose target has gone missing. Following it here would silently replace
    // an unavailable local override with the bundled namesake. Likewise, an
    // inspection error such as permission denied must not make the requested
    // file look absent. A missing parent (or a file named `tracks`) is absent.
    match local.symlink_metadata() {
        Ok(_) => return Ok(local),
        Err(error) if absent(&error) => {}
        Err(error) => {
            return Err(format!(
                "Cannot inspect track '{}': {error}",
                local.display()
            ));
        }
    }
    // Collecting components uses the platform's separators throughout.
    let bundled_in = |root: PathBuf| root.join(bundled).components().collect();
    game_root().map(bundled_in).map_err(|tried| {
        format!(
            "Cannot find track '{}': it is not at '{}', and no folder containing \
             the bundled tracks/ was found in {}",
            path.display(),
            local.display(),
            quoted_list(&tried)
        )
    })
}

pub fn next_path(current: &Path) -> Result<PathBuf, String> {
    // Compare file identities, including aliases. Prefer the actual bundled
    // files so adding a local override cannot hide a loaded bundled course.
    // A custom course may have the same trailing filename as a bundled course.
    let identity = |path: PathBuf| path.canonicalize().unwrap_or(path);
    let Ok(current) = resolve_track(current).map(identity) else {
        // Resolving the first course reports why nothing could be found.
        return Ok(first_available(0));
    };
    // Anchor the local candidates before resolving identity. If a loaded
    // override has since disappeared, a relative candidate would fall back
    // to the bundled namesake and lose the selected file's cycle position.
    // An unavailable directory contributes no candidates.
    let directories = [game_root().ok(), std::env::current_dir().ok()];
    for directory in directories.iter().flatten() {
        for (index, path) in BUNDLED_PATHS.iter().enumerate() {
            if current == identity(directory.join(path)) {
                return Ok(first_available(index + 1));
            }
        }
    }
    Ok(first_available(0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::TempDir;
    use std::collections::HashSet;

    #[test]
    fn track_switcher_visits_every_course_and_wraps() {
        let first = PathBuf::from(BUNDLED_PATHS[0]);
        let mut current = first.clone();
        let mut visited = HashSet::new();
        for _ in BUNDLED_PATHS {
            assert!(
                visited.insert(current.clone()),
                "course appeared twice before completing the cycle"
            );
            current = next_path(&current).unwrap();
        }
        assert_eq!(current, first);
        assert_eq!(visited.len(), BUNDLED_PATHS.len());
        assert_eq!(
            next_path(&Path::new(env!("CARGO_MANIFEST_DIR")).join(BUNDLED_PATHS[0])).unwrap(),
            PathBuf::from(BUNDLED_PATHS[1])
        );
    }

    #[test]
    fn custom_course_returns_to_bundled_selection() {
        assert_eq!(
            next_path(Path::new("tracks/my_custom.track")).unwrap(),
            PathBuf::from(BUNDLED_PATHS[0])
        );
    }

    #[test]
    fn custom_course_with_a_bundled_filename_starts_the_bundled_cycle() {
        let directory = TempDir::new("apex-custom-course-test");
        let path = directory.path().join("tracks/club.track");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, "straight 40").unwrap();
        let next = next_path(&path).unwrap();
        assert_eq!(next, PathBuf::from(BUNDLED_PATHS[0]));
    }

    #[cfg(unix)]
    #[test]
    fn bundled_course_alias_continues_from_its_actual_position() {
        let directory = TempDir::new("apex-course-alias-test");
        let alias = directory.path().join("favorite.track");
        let bundled = Path::new(env!("CARGO_MANIFEST_DIR")).join(BUNDLED_PATHS[1]);
        std::os::unix::fs::symlink(bundled, &alias).unwrap();
        let next = next_path(&alias).unwrap();
        assert_eq!(next, PathBuf::from(BUNDLED_PATHS[2]));
    }

    /// Run the named test again in a child process whose working directory is
    /// a fresh fixture prepared by `setup`. Returns true inside that child.
    /// A separate process gives resolution a custom working directory without
    /// racing the other tests' relative file accesses.
    fn in_child_directory(test: &str, setup: impl FnOnce(&Path)) -> bool {
        const CHILD: &str = "APEX_COURSES_CHILD_DIRECTORY";
        if let Some(directory) = std::env::var_os(CHILD) {
            // macOS may report /private/var for a fixture created under /var.
            // Compare directory identities after resolving filesystem aliases.
            assert_eq!(
                std::env::current_dir().unwrap().canonicalize().unwrap(),
                PathBuf::from(directory).canonicalize().unwrap(),
                "fixture must run in its isolated temporary directory"
            );
            return true;
        }
        let directory = TempDir::new("apex-courses-directory-test");
        setup(directory.path());
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", &format!("courses::tests::{test}"), "--nocapture"])
            .env(CHILD, directory.path())
            .current_dir(directory.path())
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{test} failed:\n{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        false
    }

    fn manifest_directory() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .canonicalize()
            .unwrap()
    }

    #[test]
    fn bundled_names_ignore_separators_and_a_leading_dot() {
        assert_eq!(
            bundled_name(Path::new("./tracks/club.track")),
            Some("tracks/club.track")
        );
        #[cfg(windows)]
        assert_eq!(
            bundled_name(Path::new(r".\tracks\club.track")),
            Some("tracks/club.track")
        );
        for other in ["club.track", "tracks/my.track", "other/tracks/club.track"] {
            assert_eq!(bundled_name(Path::new(other)), None, "{other}");
        }
        let absolute = Path::new(env!("CARGO_MANIFEST_DIR")).join(BUNDLED_PATHS[1]);
        assert_eq!(bundled_name(&absolute), None);
    }

    #[test]
    fn the_checkout_is_the_game_root_for_tests() {
        assert_eq!(
            game_root().unwrap().canonicalize().unwrap(),
            manifest_directory()
        );
        let candidates = root_candidates();
        assert_eq!(candidates[0], std::env::current_dir().unwrap());
        let executable = std::env::current_exe().unwrap();
        assert!(
            candidates
                .iter()
                .any(|c| Some(c.as_path()) == executable.parent())
        );
    }

    #[test]
    fn relative_custom_paths_resolve_only_against_the_working_directory() {
        let directory = std::env::current_dir().unwrap();
        for typed in ["my.track", "tracks/my_custom.track"] {
            let resolved = resolve_track(Path::new(typed)).unwrap();
            assert_eq!(resolved, directory.join(typed));
            let error = crate::track::Track::load(&resolved).unwrap_err();
            assert!(error.contains(&resolved.display().to_string()), "{error}");
        }
    }

    #[test]
    fn a_missing_game_root_names_every_location_tried() {
        let tried = vec![PathBuf::from("first-place"), PathBuf::from("second-place")];
        let typed = Path::new("./missing-apex-dir/../tracks/../tracks/alpine.track");
        // Not a bundled name: the path is returned for the loader to report.
        assert!(resolve_track_in(typed, || Err(tried.clone())).is_ok());

        let fixture = TempDir::new("apex-missing-root-test");
        let typed = fixture.path().join("absent");
        // Absolute paths are never redirected to the game root.
        assert_eq!(resolve_track_in(&typed, || unreachable!()).unwrap(), typed);

        let typed = Path::new("tracks/not-bundled.track");
        assert!(resolve_track_in(typed, || unreachable!()).is_ok());

        // A bundled name only consults the game root when absent locally,
        // which is never the case in the checkout used for tests. Run the
        // lookup from a working directory without bundled tracks instead.
        if in_child_directory("a_missing_game_root_names_every_location_tried", |_| {}) {
            let error = resolve_track_in(Path::new(BUNDLED_PATHS[1]), || Err(tried)).unwrap_err();
            let local = std::path::absolute(BUNDLED_PATHS[1]).unwrap();
            for named in ["first-place", "second-place", &local.display().to_string()] {
                assert!(error.contains(named), "{error}");
            }
        }
    }

    #[test]
    fn a_copy_missing_a_bundled_course_is_still_the_game_root() {
        let fixture = TempDir::new("apex-partial-root-test");
        let [empty, partial, complete] =
            ["empty", "partial", "complete"].map(|name| fixture.path().join(name));
        for (index, path) in BUNDLED_PATHS.iter().enumerate() {
            for (root, skip) in [(&partial, index == 2), (&complete, false)] {
                if !skip {
                    let file = root.join(path);
                    std::fs::create_dir_all(file.parent().unwrap()).unwrap();
                    std::fs::write(file, "straight 40").unwrap();
                }
            }
        }
        std::fs::create_dir_all(&empty).unwrap();
        // A complete copy wins wherever it is; a partial one beats nothing.
        let candidates = [empty.clone(), partial.clone(), complete.clone()];
        assert_eq!(root_among(&candidates), Some(&complete));
        assert_eq!(root_among(&candidates[..2]), Some(&partial));
        assert_eq!(root_among(&candidates[..1]), None);
    }

    #[test]
    fn the_switcher_skips_a_missing_bundled_course() {
        let missing = BUNDLED_PATHS[2];
        let available = |path: &str| path != missing;
        assert_eq!(
            first_available_where(2, available),
            PathBuf::from(BUNDLED_PATHS[3])
        );
        assert_eq!(
            first_available_where(1, available),
            PathBuf::from(BUNDLED_PATHS[1])
        );
        let last = BUNDLED_PATHS.len() - 1;
        assert_eq!(
            first_available_where(last + 1, available),
            PathBuf::from(BUNDLED_PATHS[0])
        );
        // With nothing available, the requested course reports the problem.
        assert_eq!(
            first_available_where(4, |_| false),
            PathBuf::from(BUNDLED_PATHS[4])
        );
    }

    #[test]
    fn a_file_named_tracks_in_the_working_directory_is_not_a_match() {
        let child = in_child_directory(
            "a_file_named_tracks_in_the_working_directory_is_not_a_match",
            |directory| std::fs::write(directory.join("tracks"), "not a folder").unwrap(),
        );
        if !child {
            return;
        }
        let root = game_root().unwrap();
        assert_eq!(root.canonicalize().unwrap(), manifest_directory());
        assert_eq!(
            resolve_track(Path::new(BUNDLED_PATHS[1])).unwrap(),
            root.join(BUNDLED_PATHS[1])
        );
        assert_eq!(
            next_path(Path::new(BUNDLED_PATHS[1])).unwrap(),
            PathBuf::from(BUNDLED_PATHS[2])
        );
        assert_eq!(
            next_path(Path::new("tracks/custom.track")).unwrap(),
            PathBuf::from(BUNDLED_PATHS[0])
        );
        // Custom relative paths never fall back to the game root.
        assert_eq!(
            resolve_track(Path::new("tracks/custom.track")).unwrap(),
            std::env::current_dir().unwrap().join("tracks/custom.track")
        );
    }

    #[test]
    fn reload_keeps_the_selected_file_when_local_overrides_change() {
        if !in_child_directory(
            "reload_keeps_the_selected_file_when_local_overrides_change",
            |_| {},
        ) {
            return;
        }

        let requested = Path::new(BUNDLED_PATHS[0]);
        std::fs::create_dir_all(requested.parent().unwrap()).unwrap();
        std::fs::write(requested, "name Custom\nstraight 40").unwrap();
        let selected = resolve_track(requested).unwrap();
        assert_eq!(selected, std::env::current_dir().unwrap().join(requested));
        assert_eq!(crate::track::Track::load(&selected).unwrap().name, "Custom");
        assert_eq!(
            next_path(&selected).unwrap(),
            PathBuf::from(BUNDLED_PATHS[1]),
            "local overrides must retain their place in the course cycle"
        );

        // Renaming an authored file should report a reload error, preserving
        // the active world, rather than silently loading the bundled namesake.
        std::fs::rename(requested, "tracks/renamed.track").unwrap();
        assert!(crate::track::Track::load(&selected).is_err());
        assert_eq!(resolve_track(&selected).unwrap(), selected);
        assert_eq!(
            next_path(&selected).unwrap(),
            PathBuf::from(BUNDLED_PATHS[1]),
            "removing the selected local override must not reset the course cycle"
        );

        // Conversely, creating a local override must not redirect the reload
        // of a bundled file selected before that override existed.
        let bundled = resolve_track(requested).unwrap();
        let original = crate::track::Track::load(&bundled).unwrap();
        std::fs::write(requested, "name Replacement\nstraight 60").unwrap();
        assert_eq!(resolve_track(&bundled).unwrap(), bundled);
        assert_eq!(
            crate::track::Track::load(&bundled).unwrap().source_hash(),
            original.source_hash()
        );
        assert_ne!(resolve_track(requested).unwrap(), bundled);
        assert_eq!(
            next_path(&bundled).unwrap(),
            PathBuf::from(BUNDLED_PATHS[1]),
            "a newly created local override must not hide the loaded bundled course"
        );

        #[cfg(unix)]
        {
            let alias = Path::new("favorite.track");
            std::os::unix::fs::symlink(requested, alias).unwrap();
            let selected_alias = resolve_track(alias).unwrap();
            assert_eq!(selected_alias, std::env::current_dir().unwrap().join(alias));

            // A dangling local symlink is still an explicit override. Report
            // its load error instead of silently choosing the bundled file.
            std::fs::remove_file(requested).unwrap();
            std::os::unix::fs::symlink("missing.track", requested).unwrap();
            assert_eq!(resolve_track(requested).unwrap(), selected);
            assert!(crate::track::Track::load(resolve_track(requested).unwrap()).is_err());
            assert_eq!(
                next_path(&selected).unwrap(),
                PathBuf::from(BUNDLED_PATHS[1])
            );
        }
    }
}
