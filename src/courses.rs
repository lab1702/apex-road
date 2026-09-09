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

pub fn resolve_track(path: &Path) -> PathBuf {
    if path.exists() {
        path.into()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}

pub fn next_path(current: &Path) -> PathBuf {
    // Compare the files that loading resolves, including aliases. A custom
    // course may have the same trailing filename as a bundled course.
    let identity = |path: &Path| {
        let resolved = resolve_track(path);
        resolved.canonicalize().unwrap_or(resolved)
    };
    let current = identity(current);
    let next = BUNDLED_PATHS
        .iter()
        .position(|path| current == identity(Path::new(path)))
        .map_or(0, |index| (index + 1) % BUNDLED_PATHS.len());
    BUNDLED_PATHS[next].into()
}

#[cfg(test)]
mod tests {
    use super::*;
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
            current = next_path(&current);
        }
        assert_eq!(current, first);
        assert_eq!(visited.len(), BUNDLED_PATHS.len());
        assert_eq!(
            next_path(&Path::new(env!("CARGO_MANIFEST_DIR")).join(BUNDLED_PATHS[0])),
            PathBuf::from(BUNDLED_PATHS[1])
        );
    }

    #[test]
    fn custom_course_returns_to_bundled_selection() {
        assert_eq!(
            next_path(Path::new("tracks/my_custom.track")),
            PathBuf::from(BUNDLED_PATHS[0])
        );
    }

    #[test]
    fn custom_course_with_a_bundled_filename_starts_the_bundled_cycle() {
        let directory =
            std::env::temp_dir().join(format!("apex-custom-course-test-{}", std::process::id()));
        let path = directory.join("tracks/club.track");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, "straight 40").unwrap();
        let next = next_path(&path);
        std::fs::remove_dir_all(directory).unwrap();
        assert_eq!(next, PathBuf::from(BUNDLED_PATHS[0]));
    }

    #[cfg(unix)]
    #[test]
    fn bundled_course_alias_continues_from_its_actual_position() {
        let directory =
            std::env::temp_dir().join(format!("apex-course-alias-test-{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let alias = directory.join("favorite.track");
        let bundled = Path::new(env!("CARGO_MANIFEST_DIR")).join(BUNDLED_PATHS[1]);
        std::os::unix::fs::symlink(bundled, &alias).unwrap();
        let next = next_path(&alias);
        std::fs::remove_dir_all(directory).unwrap();
        assert_eq!(next, PathBuf::from(BUNDLED_PATHS[2]));
    }
}
