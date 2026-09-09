//! Shared order for the bundled track switcher and full-course driving checks.
use std::path::{Path, PathBuf};

pub const BUNDLED_PATHS: &[&str] = &[
    "tracks/alpine.track",
    "tracks/club.track",
    "tracks/camber_loop.track",
    "tracks/ridgeway.track",
    "tracks/stone_gallery.track",
    "tracks/airfield.track",
    "tracks/skyline_eight.track",
];

/// Select a source once, retaining its lexical absolute path for later reloads.
/// Do not canonicalize: an editor may replace a file or retarget its symlink.
pub fn resolve_track(path: &Path) -> Result<PathBuf, String> {
    // Keep an existing directory entry selected even when it is a symlink
    // whose target has gone missing. Following it here would silently replace
    // an unavailable local override with the bundled namesake. Likewise, an
    // inspection error must not make the requested file look absent.
    let local_exists = match path.symlink_metadata() {
        Ok(_) => true,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => false,
        Err(error) => {
            return Err(format!(
                "Cannot inspect track '{}': {error}",
                path.display()
            ));
        }
    };
    let selected = if local_exists {
        path.to_owned()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    };
    std::path::absolute(selected)
        .map_err(|error| format!("Cannot resolve track '{}': {error}", path.display()))
}

pub fn next_path(current: &Path) -> Result<PathBuf, String> {
    // Compare file identities, including aliases. Prefer the actual bundled
    // files so adding a local override cannot hide a loaded bundled course.
    // A custom course may have the same trailing filename as a bundled course.
    let identity = |path: &Path| -> Result<PathBuf, String> {
        let resolved = resolve_track(path)?;
        Ok(resolved.canonicalize().unwrap_or(resolved))
    };
    let current = identity(current)?;
    // Anchor the local candidates before resolving identity. If a loaded
    // override has since disappeared, a relative candidate would fall back
    // to the bundled namesake and lose the selected file's cycle position.
    let local_directory = std::env::current_dir()
        .map_err(|error| format!("Cannot resolve the local track directory: {error}"))?;
    for directory in [Path::new(env!("CARGO_MANIFEST_DIR")), &local_directory] {
        for (index, path) in BUNDLED_PATHS.iter().enumerate() {
            if current == identity(&directory.join(path))? {
                return Ok(BUNDLED_PATHS[(index + 1) % BUNDLED_PATHS.len()].into());
            }
        }
    }
    Ok(BUNDLED_PATHS[0].into())
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

    #[test]
    fn reload_keeps_the_selected_file_when_local_overrides_change() {
        // A separate process gives resolution a custom working directory
        // without racing the other tests' relative file accesses.
        const CHILD: &str = "APEX_RELOAD_PATH_TEST_CHILD";
        if std::env::var_os(CHILD).is_none() {
            let directory = TempDir::new("apex-reload-path-test");
            let result = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "courses::tests::reload_keeps_the_selected_file_when_local_overrides_change",
                    "--nocapture",
                ])
                .env(CHILD, directory.path())
                .current_dir(directory.path())
                .output();
            let output = result.unwrap();
            assert!(
                output.status.success(),
                "reload regression failed:\n{}\n{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            return;
        }
        assert_eq!(
            std::env::current_dir().unwrap(),
            PathBuf::from(std::env::var_os(CHILD).unwrap()),
            "reload fixture must run in its isolated temporary directory"
        );

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
