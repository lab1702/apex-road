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

pub fn next_path(current: &Path) -> PathBuf {
    let next = BUNDLED_PATHS
        .iter()
        .position(|path| current.ends_with(path))
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
}
