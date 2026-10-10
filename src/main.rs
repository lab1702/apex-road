mod courses;
mod hud;
mod input;
mod race;
#[cfg(test)]
mod test_support;
mod track;
mod vehicle;
mod view;
mod world;

use courses::resolve_track;
use macroquad::prelude::*;
use race::Race;
use std::ffi::{OsStr, OsString};
use std::path::{Path, PathBuf};
use track::Track;
use vehicle::{Car, Control};

struct Options {
    path: PathBuf,
    validate: bool,
    frames: Option<u64>,
    capture: Option<PathBuf>,
    autodrive: bool,
    demo: bool,
    at: Option<f32>,
}
const SMOKE_TEST_FRAMES: u64 = 240;
const CAPTURE_FRAMES: u64 = 30;
const HELP: &str = "APEX / ROAD

apex-road [PATH | --track PATH] [options]
  PATH, --track PATH  Drive a .track file (default tracks/alpine.track)
  --validate PATH     Check a track without opening a window
  --smoke-test        Auto-drive, exiting after 240 frames unless --frames is given
  --autodrive         Run the demonstration driver
  --demo              Auto-play every bundled track in a repeating loop
  --frames N          Exit after N rendered frames
  --capture PATH      Save the final frame as PNG (after 30 frames unless --frames is given)
  --at METERS         Preview a position on the track

Relative track paths are read from the current directory. Bundled
tracks/*.track paths fall back to the game folder: the first of the current
directory, the executable's directory, or one of its parents that contains
the bundled tracks/. Records are saved in that folder's data/.

Enter start · WASD / arrows drive · Space handbrake · R restart
Right click toggles mouse driving: left/right steer, up adds throttle, down adds brake
U toggles US / metric HUD units (default: US)
Esc closes help, otherwise pauses · F1 help · F5 reload · Tab track · F11 fullscreen";

fn starts_with_dash(arg: &OsStr) -> bool {
    arg.as_encoded_bytes().starts_with(b"-")
}
fn path_argument(args: &mut impl Iterator<Item = OsString>, flag: &str) -> Result<PathBuf, String> {
    args.next()
        .filter(|value| !starts_with_dash(value))
        .map(PathBuf::from)
        .ok_or_else(|| format!("{flag} needs a path"))
}
fn number_argument<T: std::str::FromStr>(
    args: &mut impl Iterator<Item = OsString>,
    missing: &str,
    invalid: &str,
) -> Result<T, String> {
    let value = args.next().ok_or(missing)?;
    value
        .to_str()
        .and_then(|value| value.parse().ok())
        .ok_or_else(|| format!("{invalid}: {}", value.to_string_lossy()))
}
/// Parse command-line arguments, which need not be valid Unicode paths.
fn options<T: Into<OsString>>(args: impl IntoIterator<Item = T>) -> Result<Options, String> {
    let mut out = Options {
        path: courses::BUNDLED_PATHS[0].into(),
        validate: false,
        frames: None,
        capture: None,
        autodrive: false,
        demo: false,
        at: None,
    };
    let mut path: Option<PathBuf> = None;
    // `--validate a --track b` must not silently check only b.
    let mut set_path = |next: PathBuf| {
        if let Some(previous) = &path {
            return Err(format!(
                "Only one track path can be given; got '{}' and '{}'",
                previous.display(),
                next.display()
            ));
        }
        path = Some(next);
        Ok(())
    };
    let mut smoke_test = false;
    let mut args = args.into_iter().map(Into::into);
    while let Some(arg) = args.next() {
        let Some(flag) = arg.to_str() else {
            if starts_with_dash(&arg) {
                return Err(format!(
                    "Unknown option: {}. Use --help.",
                    arg.to_string_lossy()
                ));
            }
            set_path(arg.into())?;
            continue;
        };
        match flag {
            "--track" => set_path(path_argument(&mut args, "--track")?)?,
            "--validate" => {
                out.validate = true;
                set_path(path_argument(&mut args, "--validate")?)?;
            }
            "--frames" => {
                out.frames = Some(number_argument(
                    &mut args,
                    "--frames needs a positive integer",
                    "invalid frame count",
                )?)
            }
            "--capture" => out.capture = Some(path_argument(&mut args, "--capture")?),
            "--autodrive" => out.autodrive = true,
            "--demo" => {
                out.demo = true;
                out.autodrive = true;
            }
            "--smoke-test" => {
                smoke_test = true;
                out.autodrive = true;
            }
            "--at" => {
                out.at = Some(number_argument(
                    &mut args,
                    "--at needs a distance in meters",
                    "invalid distance",
                )?)
            }
            "--help" | "-h" => {
                println!("{HELP}");
                std::process::exit(0);
            }
            other if !other.starts_with('-') => set_path(arg.into())?,
            _ => return Err(format!("Unknown option: {flag}. Use --help.")),
        }
    }
    if let Some(path) = path {
        out.path = path;
    }
    if out.frames == Some(0) {
        return Err("--frames must be positive".into());
    }
    if out.at.is_some_and(|v| !v.is_finite() || v < 0.) {
        return Err("--at must be finite and nonnegative".into());
    }
    // An explicit --frames wins regardless of its position.
    if smoke_test && out.frames.is_none() {
        out.frames = Some(SMOKE_TEST_FRAMES);
    }
    if out.capture.is_some() && out.frames.is_none() {
        out.frames = Some(CAPTURE_FRAMES);
    }
    Ok(out)
}
fn config() -> macroquad::conf::Conf {
    macroquad::conf::Conf {
        miniquad_conf: Conf {
            window_title: "APEX / ROAD — Time Attack".into(),
            window_width: 1440,
            window_height: 900,
            high_dpi: true,
            sample_count: 4,
            ..Default::default()
        },
        draw_call_vertex_capacity: 60000,
        draw_call_index_capacity: 120000,
        ..Default::default()
    }
}
fn main() {
    let mut opts = options(std::env::args_os().skip(1)).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2)
    });
    let track = resolve_track(&opts.path)
        .and_then(|path| {
            opts.path = path;
            Track::load(&opts.path)
        })
        .unwrap_or_else(|e| {
            eprintln!("Track error: {e}");
            std::process::exit(1)
        });
    if opts.validate {
        println!(
            "OK: {} | {} | {:.0} m | {} samples | {} checkpoints | {}",
            opts.path.display(),
            track.name,
            track.length,
            track.samples().len(),
            track.checkpoints.len(),
            if track.closed {
                "circuit"
            } else {
                "point to point"
            }
        );
        return;
    }
    macroquad::Window::from_config(config(), async move {
        if let Err(error) = game(opts, track).await {
            eprintln!("{error}");
            std::process::exit(1);
        }
    });
}
/// Version of the rules that make best times comparable. Bump it whenever
/// vehicle handling, the physics step, or timing changes enough that older
/// records would be unfair, so new runs start a fresh record history.
/// Version 1 files were named `{hash}.best`; they are left on disk unread.
const RECORD_VERSION: u32 = 2;
/// Records live in the game root's `data/`, beside the bundled `tracks/`.
/// Without a game root (only possible with an explicit track path), fall back
/// to the executable's directory rather than wherever the game was launched.
fn records_dir() -> PathBuf {
    courses::game_root()
        .ok()
        .or_else(|| Some(std::env::current_exe().ok()?.parent()?.to_owned()))
        .unwrap_or_default()
        .join("data")
}
fn record_path(track: &Track) -> PathBuf {
    record_path_in(&records_dir(), track)
}
fn record_path_in(directory: &Path, track: &Track) -> PathBuf {
    directory.join(format!(
        "{:016x}-v{RECORD_VERSION}.best",
        track.source_hash()
    ))
}
fn read_record(path: &Path) -> Option<f32> {
    try_read_record(path).ok().flatten()
}
fn try_read_record(path: &Path) -> std::io::Result<Option<f32>> {
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(error),
    };
    // Invalid encoding is corrupt record data, just like an invalid number.
    // Keep filesystem errors distinct so only successfully read data is replaced.
    Ok(std::str::from_utf8(&bytes)
        .ok()
        .and_then(|text| text.trim().parse::<f32>().ok())
        .filter(|x| x.is_finite() && *x > 0.))
}
#[derive(Debug)]
struct SavedRecord {
    best: f32,
    improved: bool,
}
fn save_record(path: &Path, time: f32) -> std::io::Result<SavedRecord> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    // The record itself is replaced by rename. Keep a separate, stable lock
    // file so every running game serializes its read/compare/write sequence.
    // Never remove this sidecar: another process may already be waiting on it.
    // The lock is released when `lock` is dropped at the end of this function.
    let lock = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(path.with_extension("lock"))?;
    let locked = lock_record(&lock)?;
    if let Some(best) = try_read_record(path)?
        && best <= time
    {
        return Ok(SavedRecord {
            best,
            improved: false,
        });
    }
    // All locked writers for this record share its temporary file. Without
    // a lock, a per-process name keeps concurrent writers from interleaving.
    // The same-directory rename keeps readers atomic.
    let temp = if locked {
        path.with_extension("tmp")
    } else {
        path.with_extension(format!("{}.tmp", std::process::id()))
    };
    let written = write_synced(&temp, format!("{time}\n").as_bytes())
        .and_then(|()| rename_with_retry(&temp, path));
    if let Err(error) = written {
        // Do not leave a partial record behind. This cannot remove a
        // directory or another writer's file: the name is ours while locked.
        let _ = std::fs::remove_file(&temp);
        return Err(error);
    }
    Ok(SavedRecord {
        best: time,
        improved: true,
    })
}
/// Bounded so a stuck writer cannot freeze the render thread, which saves
/// records; contended saves report an error and retry after the next run.
const RECORD_LOCK_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(250);
/// Take the record lock, waiting briefly for another game to finish saving.
/// Returns false when the filesystem cannot lock files at all; saving then
/// proceeds unserialized, as it would on a single running game.
fn lock_record(lock: &std::fs::File) -> std::io::Result<bool> {
    let deadline = std::time::Instant::now() + RECORD_LOCK_TIMEOUT;
    let mut delay = std::time::Duration::from_millis(1);
    loop {
        match lock.try_lock() {
            Ok(()) => return Ok(true),
            Err(std::fs::TryLockError::Error(error)) if locking_unsupported(&error) => {
                return Ok(false);
            }
            Err(std::fs::TryLockError::Error(error)) => return Err(error),
            Err(std::fs::TryLockError::WouldBlock) => {
                let now = std::time::Instant::now();
                if now >= deadline {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::TimedOut,
                        "another game is still saving this record",
                    ));
                }
                std::thread::sleep(delay.min(deadline - now));
                delay = (delay * 2).min(std::time::Duration::from_millis(16));
            }
        }
    }
}
fn locking_unsupported(error: &std::io::Error) -> bool {
    // ENOLCK (for example, NFS without a lock daemon), or Windows
    // ERROR_INVALID_FUNCTION / ERROR_NOT_SUPPORTED from some network shares.
    const NO_LOCKS: &[i32] = if cfg!(target_os = "linux") {
        &[37]
    } else if cfg!(target_os = "macos") {
        &[77]
    } else if cfg!(windows) {
        &[1, 50]
    } else {
        &[]
    };
    error.kind() == std::io::ErrorKind::Unsupported
        || error
            .raw_os_error()
            .is_some_and(|code| NO_LOCKS.contains(&code))
}
/// Write a complete file and flush it to disk before it can replace a record,
/// so a crash cannot leave a renamed but empty record.
fn write_synced(path: &Path, contents: &[u8]) -> std::io::Result<()> {
    let mut file = std::fs::File::create(path)?;
    std::io::Write::write_all(&mut file, contents)?;
    file.sync_all()
}
/// Antivirus and indexing services on Windows can briefly hold the old record
/// open, failing the rename with access denied. Retry for at most ~75 ms.
fn rename_with_retry(from: &Path, to: &Path) -> std::io::Result<()> {
    let mut delay = std::time::Duration::from_millis(5);
    for _ in 0..4 {
        match std::fs::rename(from, to) {
            Err(error) if cfg!(windows) && error.kind() == std::io::ErrorKind::PermissionDenied => {
                std::thread::sleep(delay);
                delay *= 2;
            }
            result => return result,
        }
    }
    std::fs::rename(from, to)
}
fn save_capture(path: &Path, screenshot: &Image) -> Result<(), image::ImageError> {
    let mut pixels = image::RgbaImage::from_raw(
        u32::from(screenshot.width),
        u32::from(screenshot.height),
        screenshot.bytes.clone(),
    )
    .ok_or_else(|| {
        std::io::Error::new(std::io::ErrorKind::InvalidData, "Invalid screenshot pixels")
    })?;
    // Screen readback starts at the bottom row; PNGs start at the top.
    image::imageops::flip_vertical_in_place(&mut pixels);
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    let mut output = std::io::BufWriter::new(std::fs::File::create(path)?);
    pixels.write_to(&mut output, image::ImageOutputFormat::Png)?;
    // BufWriter discards flush failures on drop, including a full destination.
    std::io::Write::flush(&mut output)?;
    Ok(())
}
fn input() -> Control {
    controls_from_keys(is_key_down)
}
fn controls_from_keys(key_down: impl Fn(KeyCode) -> bool) -> Control {
    let down = |a, b| key_down(a) || key_down(b);
    Control {
        throttle: if down(KeyCode::W, KeyCode::Up) {
            1.
        } else {
            0.
        },
        brake: if down(KeyCode::S, KeyCode::Down) {
            1.
        } else {
            0.
        },
        steer: (if down(KeyCode::D, KeyCode::Right) {
            1.
        } else {
            0.
        }) - (if down(KeyCode::A, KeyCode::Left) {
            1.
        } else {
            0.
        }),
        handbrake: key_down(KeyCode::Space),
    }
}
fn demo_control(car: &Car, track: &Track) -> Control {
    let speed = car.speed_kmh() / 3.6;
    let target = track.sample_at(car.distance + 12. + speed * 0.55);
    let to = target.pos - car.position;
    let desired = to.x.atan2(to.z);
    let angle = (desired - car.heading + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;
    let corner = (target.forward.x.atan2(target.forward.z) - car.heading + std::f32::consts::PI)
        .rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;
    let target_speed = (27. - corner.abs() * 28.).clamp(12., 27.);
    Control {
        throttle: if speed < target_speed { 0.7 } else { 0. },
        brake: if speed > target_speed + 2. { 0.35 } else { 0. },
        steer: (angle * 2.4).clamp(-1., 1.),
        handbrake: false,
    }
}
/// Keep legitimate records separate from the best time displayed during a preview.
struct RunRecords {
    best: Option<f32>,
    eligible: bool,
}

impl RunRecords {
    fn new(best: Option<f32>) -> Self {
        Self {
            best,
            eligible: false,
        }
    }

    fn start_race(&mut self, distance: f32, eligible: bool) -> Race {
        self.eligible = eligible;
        Race::new(self.best, distance)
    }

    fn accept(&mut self, candidate: Option<f32>) -> Option<f32> {
        if !self.eligible {
            return None;
        }
        if let Some(best) = candidate {
            self.best = Some(best);
        }
        candidate
    }

    fn save_candidate(
        &mut self,
        path: &Path,
        race: &mut Race,
        candidate: Option<f32>,
    ) -> std::io::Result<Option<SavedRecord>> {
        let previous_best = self.best;
        let Some(candidate) = self.accept(candidate) else {
            return Ok(None);
        };
        let saved = match save_record(path, candidate) {
            Ok(saved) => saved,
            Err(error) => {
                // Race already promoted the candidate. Keep both best-time
                // thresholds at the saved value so a failed write cannot
                // suppress later records, including after a restart.
                self.best = previous_best;
                race.best = previous_best;
                return Err(error);
            }
        };
        self.best = Some(saved.best);
        race.best = Some(saved.best);
        Ok(Some(saved))
    }
}

fn restart_run(
    car: &mut Car,
    track: &Track,
    records: &mut RunRecords,
    race: &mut Race,
    autodrive: bool,
) {
    let last = race.last;
    car.reset_to_grid(track);
    *race = records.start_race(car.distance, !autodrive);
    race.last = last;
    race.started = true;
}

/// Keys that act once on the frame they are pressed, rather than while held.
const COMMAND_KEYS: [KeyCode; 9] = [
    KeyCode::F11,
    KeyCode::F1,
    KeyCode::U,
    KeyCode::Escape,
    KeyCode::Q,
    KeyCode::Enter,
    KeyCode::R,
    KeyCode::F5,
    KeyCode::Tab,
];
/// The simulation's fixed physics step, in seconds.
const PHYSICS_STEP: f32 = 1. / 120.;
/// Frame-limited auto-driven runs advance this much per rendered frame, so
/// smoke tests and captures do not depend on the machine's frame rate.
const FIXED_FRAME_TIME: f32 = 1. / 60.;
/// Seconds during which a right click may still be the click that activated
/// the window, rather than a request to toggle mouse driving.
const ACTIVATION_GUARD: f32 = 0.25;
/// Seconds stopped off the road before the restart hint appears.
const OFF_COURSE_HINT_DELAY: f32 = 3.;
/// A car this far below the lowest road surface is falling forever.
const FALL_DEPTH: f32 = 30.;

/// Everything one frame reads from the window, so the game's rules can run
/// without one.
#[derive(Clone, Debug)]
struct FrameInput {
    /// Seconds since the previous frame, as reported by the window.
    frame_time: f32,
    /// Focus was lost at some point since the previous frame.
    focus_lost: bool,
    /// The window became focused since the previous frame.
    focus_gained: bool,
    /// The window has focus now.
    focused: bool,
    /// Command keys pressed this frame.
    pressed: Vec<KeyCode>,
    right_click: bool,
    /// Keyboard driving controls held this frame, including the handbrake.
    keys: Control,
    /// This frame's mouse motion events in order, in logical pixels.
    mouse_motion: Vec<Vec2>,
    /// The final mouse position this frame, in logical pixels.
    mouse_position: Vec2,
}

impl FrameInput {
    /// Gather this frame's input after its events were replayed into `focus`.
    fn read(focus: &mut input::WindowInput) -> Self {
        let focus_lost = focus.take_loss();
        let frame_time = get_frame_time();
        let focus_gained = focus.take_gain();
        // Event coordinates are physical pixels; `mouse_position` is logical.
        let dpi_scale = macroquad::miniquad::window::dpi_scale();
        Self {
            frame_time,
            focus_lost,
            focus_gained,
            focused: focus.focused,
            pressed: COMMAND_KEYS
                .into_iter()
                .filter(|key| is_key_pressed(*key))
                .collect(),
            right_click: is_mouse_button_pressed(MouseButton::Right),
            keys: input(),
            mouse_motion: focus
                .drain_mouse_motion()
                .map(|position| position / dpi_scale)
                .collect(),
            mouse_position: mouse_position().into(),
        }
    }

    fn pressed(&self, key: KeyCode) -> bool {
        self.pressed.contains(&key)
    }
}

/// Window side effects requested by a frame's update, in order.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Action {
    /// Leave the game. Nothing else in the frame was updated.
    Quit,
    SetFullscreen(bool),
    /// Grab and hide the cursor for mouse driving, or release and show it.
    GrabCursor(bool),
    /// The track changed, so its scenery must be rebuilt.
    RebuildWorld,
}

/// The whole game apart from the window: course, car, run, records and UI
/// modes. `update` applies one frame of input and simulation.
struct GameState {
    track: Track,
    path: PathBuf,
    /// Where records are kept. `None` uses the game root's `data/`,
    /// resolved again for each loaded course.
    records_dir: Option<PathBuf>,
    record_file: PathBuf,
    records: RunRecords,
    car: Car,
    race: Race,
    attitude: CameraAttitude,
    /// The controls applied on the latest frame, as shown by the HUD.
    control: Control,
    paused: bool,
    help: bool,
    units: hud::Units,
    fullscreen: bool,
    autodrive: bool,
    /// `--demo`: cycle through the courses while the auto driver drives.
    demo: bool,
    /// `--frames`: rendered frames before the game exits.
    frame_limit: Option<u64>,
    frames: u64,
    accumulator: f32,
    note: String,
    note_time: f32,
    /// Seconds the car has been stopped off the road.
    respawn_timer: f32,
    /// Seconds left in which a right click is treated as window activation.
    activation_guard: f32,
    mouse: input::MouseDriving,
    cursor_grabbed: bool,
}

fn record_path_for(directory: Option<&Path>, track: &Track) -> PathBuf {
    match directory {
        Some(directory) => record_path_in(directory, track),
        None => record_path(track),
    }
}

impl GameState {
    fn new(opts: &Options, track: Track, records_dir: Option<PathBuf>) -> Self {
        let mut car = Car::new(&track);
        if let Some(at) = opts.at {
            car.reset(&track, at.min(track.length - 5.));
        }
        let record_file = record_path_for(records_dir.as_deref(), &track);
        let mut records = RunRecords::new(read_record(&record_file));
        let mut race = records.start_race(car.distance, !opts.autodrive && opts.at.is_none());
        race.started = opts.autodrive;
        if opts.demo {
            println!("Demo: {}", track.name);
        }
        Self {
            attitude: CameraAttitude::new(&car),
            track,
            path: opts.path.clone(),
            records_dir,
            record_file,
            records,
            car,
            race,
            control: Control::default(),
            paused: false,
            help: false,
            units: hud::Units::default(),
            fullscreen: false,
            autodrive: opts.autodrive,
            demo: opts.demo,
            frame_limit: opts.frames,
            frames: 0,
            accumulator: 0.,
            note: String::new(),
            note_time: 0.,
            respawn_timer: 0.,
            activation_guard: 0.,
            mouse: input::MouseDriving::default(),
            cursor_grabbed: false,
        }
    }

    fn notify(&mut self, note: impl Into<String>, seconds: f32) {
        self.note = note.into();
        self.note_time = seconds;
    }

    /// The notification currently on screen, if any.
    fn notification(&self) -> Option<&str> {
        (self.note_time > 0.).then_some(self.note.as_str())
    }

    /// Back to the grid for a new run, with the view snapped to match.
    fn restart(&mut self) {
        restart_run(
            &mut self.car,
            &self.track,
            &mut self.records,
            &mut self.race,
            self.autodrive,
        );
        self.attitude.snap(&self.car);
        self.mouse.reset();
    }

    /// Apply one frame of input, then advance the simulation. Returns the
    /// window side effects for the caller to perform in order.
    fn update(&mut self, input: &FrameInput) -> Vec<Action> {
        let mut actions = Vec::new();
        if input.focus_lost {
            // Keys and mouse motion stop arriving without focus. Hold any
            // timed run still until the player resumes it deliberately.
            if self.race.started && !self.race.finished && !self.autodrive {
                self.paused = true;
            }
            self.mouse.reset();
        }
        let dt = input.frame_time.clamp(0., 0.1);
        self.activation_guard = if input.focus_gained {
            ACTIVATION_GUARD
        } else {
            (self.activation_guard - dt).max(0.)
        };
        self.frames += 1;
        self.note_time = (self.note_time - dt).max(0.);
        if input.pressed(KeyCode::F11) {
            self.fullscreen = !self.fullscreen;
            actions.push(Action::SetFullscreen(self.fullscreen));
            self.mouse.reset();
        }
        if input.pressed(KeyCode::F1) {
            self.help = !self.help;
        }
        if input.pressed(KeyCode::U) {
            self.units.toggle();
        }
        if input.pressed(KeyCode::Escape) {
            if self.help {
                self.help = false;
            } else {
                self.paused = !self.paused;
            }
        }
        if self.paused && input.pressed(KeyCode::Q) {
            actions.push(Action::Quit);
            return actions;
        }
        // Replayed events report the activating click as focused input, so
        // ignore right clicks while the window is just regaining focus.
        if input.focused && !input.focus_gained && self.activation_guard <= 0. && input.right_click
        {
            self.mouse.toggle();
            // A manual takeover ends the demonstration for this session.
            self.autodrive = false;
        }
        if input.pressed(KeyCode::Enter) {
            if self.race.finished {
                self.restart();
            }
            self.race.started = true;
            self.paused = false;
            self.help = false;
        }
        if input.pressed(KeyCode::R) {
            self.restart();
            self.accumulator = 0.;
            self.respawn_timer = 0.;
            self.notify("NEW RUN", 2.);
        }
        let demo = self.demo && self.autodrive;
        let reload = input.pressed(KeyCode::F5);
        let advance_demo =
            demo && self.race.completed_runs > 0 && !self.paused && !self.help && !reload;
        let switch = input.pressed(KeyCode::Tab) || advance_demo;
        if (switch || reload) && self.change_course(switch, advance_demo) {
            actions.push(Action::RebuildWorld);
        }
        let driving = self.race.started
            && !self.paused
            && !self.help
            && !self.race.finished
            && (input.focused || self.autodrive);
        let capture_mouse = self.mouse.enabled() && driving;
        if capture_mouse != self.cursor_grabbed {
            actions.push(Action::GrabCursor(capture_mouse));
            self.cursor_grabbed = capture_mouse;
            self.mouse.reset();
        }
        self.mouse.update_frame(
            input.mouse_motion.iter().copied(),
            input.mouse_position,
            capture_mouse,
        );
        self.control = if self.autodrive {
            demo_control(&self.car, &self.track)
        } else if self.mouse.enabled() {
            self.mouse.control(input.keys.handbrake)
        } else {
            input.keys
        };
        if driving {
            self.simulate(dt, demo);
        } else {
            self.accumulator = 0.;
        }
        self.attitude.follow(&self.car, dt);
        actions
    }

    /// Load the next course (`switch`) or reload the current one. A failed
    /// load keeps the current course. Returns whether the track changed.
    fn change_course(&mut self, switch: bool, automatic: bool) -> bool {
        let next = if switch {
            courses::next_path(&self.path).and_then(|path| resolve_track(&path))
        } else {
            Ok(self.path.clone())
        };
        match next.and_then(|path| Track::load(&path).map(|track| (path, track))) {
            Ok((path, track)) => {
                self.track = track;
                self.path = path;
                self.car = Car::new(&self.track);
                self.record_file = record_path_for(self.records_dir.as_deref(), &self.track);
                self.records = RunRecords::new(read_record(&self.record_file));
                self.race = self.records.start_race(self.car.distance, !self.autodrive);
                self.race.started = self.autodrive;
                if self.demo && self.autodrive {
                    println!("Demo: {}", self.track.name);
                }
                self.attitude.snap(&self.car);
                self.accumulator = 0.;
                self.paused = false;
                self.respawn_timer = 0.;
                self.mouse.reset();
                self.notify(
                    if switch {
                        "COURSE LOADED"
                    } else {
                        "TRACK RELOADED"
                    },
                    3.,
                );
                true
            }
            Err(e) => {
                // Stop automatic retries until the user resumes or reloads.
                if automatic {
                    self.paused = true;
                }
                self.notify(format!("RELOAD FAILED: {e}"), 15.);
                eprintln!("{}", self.note);
                false
            }
        }
    }

    /// Run the physics steps covered by this frame, saving records and
    /// recovering a car that has fallen off the world.
    fn simulate(&mut self, dt: f32, demo: bool) {
        // Frame-limited auto-driven runs advance a fixed time per frame, so
        // everything timed in simulation follows that clock, not the window's.
        let dt = if self.frame_limit.is_some() && self.autodrive {
            FIXED_FRAME_TIME
        } else {
            dt
        };
        self.accumulator += dt;
        while self.accumulator >= PHYSICS_STEP {
            self.car.update(&self.track, self.control, PHYSICS_STEP);
            let candidate = self.race.update(
                &self.track,
                PHYSICS_STEP,
                self.car.distance,
                self.car.on_course(),
            );
            match self
                .records
                .save_candidate(&self.record_file, &mut self.race, candidate)
            {
                Err(e) => self.notify(format!("Record could not be saved: {e}"), 8.),
                Ok(Some(saved)) => self.notify(
                    if saved.improved {
                        "NEW PERSONAL BEST"
                    } else {
                        "PERSONAL BEST SYNCED"
                    },
                    5.,
                ),
                Ok(None) => {}
            }
            if !self.car.position.is_finite()
                || self.car.position.y < self.track.ground_height() - FALL_DEPTH
            {
                self.restart();
                self.notify("BACK ON THE GRID", 3.);
                self.accumulator = 0.;
                break;
            }
            self.accumulator -= PHYSICS_STEP;
            if self.race.finished || (demo && self.race.completed_runs > 0) {
                self.accumulator = 0.;
                break;
            }
        }
        if self.car.offroad && self.car.speed_kmh() < 4. {
            self.respawn_timer += dt;
        } else {
            self.respawn_timer = 0.;
        }
        if self.respawn_timer > OFF_COURSE_HINT_DELAY {
            self.notify("OFF COURSE · PRESS R TO RESTART", 2.);
        }
    }

    /// Whether `--frames` has been reached and the game should exit after
    /// drawing (and capturing) this frame.
    fn frame_limit_reached(&self) -> bool {
        self.frame_limit.is_some_and(|limit| self.frames >= limit)
    }

    fn driver_camera(&self, aspect: f32) -> view::DriverCamera {
        view::DriverCamera::new(
            self.car.position,
            self.car.heading,
            self.attitude.pitch,
            self.attitude.roll,
            self.car.speed_kmh(),
            aspect,
        )
    }

    fn hud(&self, fps: i32) -> hud::HudState<'_> {
        let speed = self.car.speed_kmh();
        let gear = hud::gear(speed, self.car.reversing);
        hud::HudState {
            track_name: &self.track.name,
            speed,
            units: self.units,
            rpm: hud::engine_rpm(speed, gear, self.control.throttle),
            gear,
            throttle: self.car.throttle,
            brake: self.car.brake,
            // The input, not the wheel angle: lock narrows with speed.
            steering: self.control.steer,
            slip: self.car.slip,
            elapsed: self.race.elapsed as f32,
            best: self.race.best,
            last: self.race.last,
            checkpoint: self.race.next_checkpoint,
            checkpoint_count: self.track.checkpoints.len(),
            progress: self.car.distance / self.track.length,
            paused: self.paused,
            started: self.race.started,
            finished: self.race.finished,
            invalid: self.race.invalid_reason,
            airborne: !self.car.grounded,
            offroad: self.car.offroad,
            help: self.help,
            mouse_enabled: self.mouse.enabled(),
            autodrive: self.autodrive,
            notification: self.notification(),
            fps,
        }
    }
}

async fn game(opts: Options, track: Track) -> Result<(), String> {
    hud::init();
    prevent_quit();
    let mut state = GameState::new(&opts, track, None);
    let mut world = world::World::new(&state.track);
    let mut focus = input::WindowInput::new();
    let input_subscriber = macroquad::input::utils::register_input_subscriber();
    let mut result = Ok(());
    'frames: loop {
        if is_quit_requested() {
            break;
        }
        macroquad::input::utils::repeat_all_miniquad_input(&mut focus, input_subscriber);
        let input = FrameInput::read(&mut focus);
        for action in state.update(&input) {
            match action {
                Action::Quit => break 'frames,
                Action::SetFullscreen(fullscreen) => set_fullscreen(fullscreen),
                Action::GrabCursor(grab) => {
                    set_cursor_grab(grab);
                    show_mouse(!grab);
                }
                Action::RebuildWorld => world.rebuild(&state.track),
            }
        }
        world.draw(&state.driver_camera(screen_width() / screen_height().max(1.)));
        hud::draw(
            &state.hud(get_fps()),
            &state.track,
            state.car.position,
            state.car.heading,
        );
        if state.frame_limit_reached() {
            if let Some(ref dest) = opts.capture {
                if let Err(error) = save_capture(dest, &get_screen_data()) {
                    result = Err(format!("Capture error for '{}': {error}", dest.display()));
                    break;
                }
                println!("Captured {}", dest.display());
            }
            println!(
                "Graphics smoke passed: {} frames, {:.1} km/h, {:.1} m progress, position {:?}",
                state.frames,
                state.car.speed_kmh(),
                state.car.distance,
                state.car.position
            );
            break;
        }
        next_frame().await;
    }
    set_cursor_grab(false);
    show_mouse(true);
    hud::shutdown();
    result
}

/// The driver's view follows the car's pitch and roll with a little lag.
struct CameraAttitude {
    pitch: f32,
    roll: f32,
}

impl CameraAttitude {
    fn new(car: &Car) -> Self {
        let mut attitude = Self {
            pitch: 0.,
            roll: 0.,
        };
        attitude.snap(car);
        attitude
    }

    /// Jump straight to the car's attitude after a restart or respawn,
    /// instead of swinging over from where the previous run ended.
    fn snap(&mut self, car: &Car) {
        self.pitch = smooth_toward(f32::NAN, car.pitch, 0., 0.);
        self.roll = smooth_toward(f32::NAN, car.roll, 0., 0.);
    }

    fn follow(&mut self, car: &Car, dt: f32) {
        self.pitch = smooth_toward(self.pitch, car.pitch, 8., dt);
        self.roll = smooth_toward(self.roll, car.roll, 7., dt);
    }
}

/// Exponential smoothing at `rate` per second, independent of frame rate.
/// A non-finite value never reaches the camera: an invalid current value
/// adopts the target, and an invalid target holds the current value.
fn smooth_toward(current: f32, target: f32, rate: f32, dt: f32) -> f32 {
    match (current.is_finite(), target.is_finite()) {
        (true, true) => {
            let blend = 1. - (-rate * dt.max(0.)).exp();
            current + (target - current) * blend
        }
        (false, true) => target,
        (true, false) => current,
        (false, false) => 0.,
    }
}

#[cfg(test)]
mod camera_checks {
    use super::*;

    #[test]
    fn smoothing_is_independent_of_frame_rate() {
        let mut fast = 0.;
        for _ in 0..240 {
            fast = smooth_toward(fast, 1., 8., 1. / 240.);
        }
        let mut slow = 0.;
        for _ in 0..30 {
            slow = smooth_toward(slow, 1., 8., 1. / 30.);
        }
        assert!((fast - slow).abs() < 1e-4, "{fast} vs {slow}");
        assert!((fast - (1. - (-8f32).exp())).abs() < 1e-4);
        // A long frame approaches the target without overshooting it.
        assert!(smooth_toward(0., 1., 8., 0.5) <= 1.);
        assert_eq!(smooth_toward(0.25, 1., 8., 0.), 0.25);
    }

    #[test]
    fn non_finite_attitudes_never_reach_the_camera() {
        assert_eq!(smooth_toward(f32::NAN, 0.3, 8., 0.01), 0.3);
        assert_eq!(smooth_toward(0.2, f32::NAN, 8., 0.01), 0.2);
        assert_eq!(smooth_toward(0.2, f32::INFINITY, 8., 0.01), 0.2);
        assert_eq!(smooth_toward(f32::NAN, f32::NAN, 8., 0.01), 0.);
        assert_eq!(smooth_toward(0.2, 0.3, 8., f32::NAN), 0.2);
    }

    #[test]
    fn restarts_snap_the_view_to_the_grid_attitude() {
        let track = Track::parse("straight 40 rise 6\nstraight 40 bank 10\nstraight 40").unwrap();
        let mut car = Car::new(&track);
        let mut attitude = CameraAttitude::new(&car);
        assert_eq!((attitude.pitch, attitude.roll), (car.pitch, car.roll));
        car.reset(&track, 60.);
        attitude.follow(&car, 1. / 60.);
        assert_ne!(attitude.roll, car.roll);
        let mut records = RunRecords::new(None);
        let mut race = records.start_race(car.distance, true);
        restart_run(&mut car, &track, &mut records, &mut race, false);
        attitude.snap(&car);
        assert_eq!((attitude.pitch, attitude.roll), (car.pitch, car.roll));
        attitude.pitch = f32::NAN;
        attitude.follow(&car, 1. / 60.);
        assert_eq!(attitude.pitch, car.pitch);
    }
}

#[cfg(test)]
mod driving_checks {
    use super::*;
    use crate::test_support::TempDir;
    use macroquad::camera::Camera;

    #[test]
    fn path_options_reject_missing_paths_without_consuming_other_options() {
        for flag in ["--track", "--validate", "--capture"] {
            for args in [vec![flag], vec![flag, "--autodrive", "tracks/club.track"]] {
                assert_eq!(
                    options(args.into_iter().map(String::from)).err(),
                    Some(format!("{flag} needs a path"))
                );
            }
        }
        let opts = options(["--validate", "tracks/club.track"].map(String::from)).unwrap();
        assert!(opts.validate);
        assert_eq!(opts.path, PathBuf::from("tracks/club.track"));
    }

    #[test]
    fn frame_counts_must_be_positive_integers() {
        for args in [
            vec!["--frames", "0"],
            vec!["--smoke-test", "--frames", "0"],
            vec!["--capture", "x.png", "--frames", "0"],
        ] {
            assert_eq!(
                options(args).err(),
                Some("--frames must be positive".into())
            );
        }
        assert!(options(["--frames", "-3"]).is_err());
        assert!(options(["--frames", "many"]).is_err());
        assert!(options(["--frames"]).is_err());
        assert_eq!(options(["--frames", "1"]).unwrap().frames, Some(1));
    }

    #[test]
    fn explicit_frames_override_the_smoke_test_default_in_any_order() {
        for args in [
            vec!["--smoke-test", "--frames", "5"],
            vec!["--frames", "5", "--smoke-test"],
        ] {
            let opts = options(args).unwrap();
            assert_eq!(opts.frames, Some(5));
            assert!(opts.autodrive);
        }
        let opts = options(["--smoke-test"]).unwrap();
        assert_eq!(opts.frames, Some(SMOKE_TEST_FRAMES));
        let opts = options(["--smoke-test", "--capture", "drive.png"]).unwrap();
        assert_eq!(opts.frames, Some(SMOKE_TEST_FRAMES));
    }

    #[test]
    fn capture_without_frames_defaults_to_thirty() {
        let opts = options(["--capture", "shot.png"]).unwrap();
        assert_eq!(opts.capture, Some(PathBuf::from("shot.png")));
        assert_eq!(opts.frames, Some(30));
        let opts = options(["--frames", "7", "--capture", "shot.png"]).unwrap();
        assert_eq!(opts.frames, Some(7));
        assert_eq!(options(["--autodrive"]).unwrap().frames, None);
    }

    #[test]
    fn only_one_track_path_is_accepted() {
        for args in [
            vec!["--validate", "a.track", "--track", "b.track"],
            vec!["--track", "a.track", "--validate", "b.track"],
            vec!["--track", "a.track", "--track", "b.track"],
            vec!["a.track", "b.track"],
            vec!["a.track", "--validate", "b.track"],
        ] {
            let error = options(args.clone()).err().unwrap();
            assert!(
                error.contains("a.track") && error.contains("b.track"),
                "{args:?}: {error}"
            );
        }
        let opts = options(["my.track"]).unwrap();
        assert_eq!(opts.path, PathBuf::from("my.track"));
        assert!(!opts.validate);
        let opts = options(["--validate", "b.track", "--autodrive"]).unwrap();
        assert!(opts.validate && opts.autodrive);
        assert_eq!(opts.path, PathBuf::from("b.track"));
    }

    #[test]
    fn unknown_options_are_reported() {
        for flag in ["--trak", "-x", "--"] {
            assert_eq!(
                options(["--autodrive", flag]).err(),
                Some(format!("Unknown option: {flag}. Use --help."))
            );
        }
    }

    #[cfg(unix)]
    #[test]
    fn non_unicode_arguments_are_paths_or_clean_errors() {
        use std::os::unix::ffi::OsStringExt;
        let path = OsString::from_vec(b"caf\xe9.track".to_vec());
        assert_eq!(
            options([path.clone()]).unwrap().path,
            PathBuf::from(path.clone())
        );
        let opts = options([OsString::from("--capture"), path.clone()]).unwrap();
        assert_eq!(opts.capture, Some(PathBuf::from(path)));
        let flag = OsString::from_vec(b"--fr\xe9mes".to_vec());
        assert!(options([flag]).err().unwrap().starts_with("Unknown option"));
        let number = OsString::from_vec(b"1\xe9".to_vec());
        assert!(
            options([OsString::from("--frames"), number])
                .err()
                .unwrap()
                .starts_with("invalid frame count")
        );
    }

    #[cfg(windows)]
    #[test]
    fn non_unicode_arguments_are_paths_or_clean_errors() {
        use std::os::windows::ffi::OsStringExt;
        // An unpaired surrogate cannot be represented as UTF-8.
        let wide = |text: &str| {
            let mut units: Vec<u16> = text.encode_utf16().collect();
            units.insert(2, 0xd800);
            OsString::from_wide(&units)
        };
        let path = wide("my.track");
        assert_eq!(options([path.clone()]).unwrap().path, PathBuf::from(path));
        assert!(
            options([wide("--frames")])
                .err()
                .unwrap()
                .starts_with("Unknown option")
        );
        assert!(
            options([OsString::from("--at"), wide("12")])
                .err()
                .unwrap()
                .starts_with("invalid distance")
        );
    }

    #[test]
    fn records_are_keyed_by_track_contents_and_record_version() {
        let directory = Path::new("records");
        let track = Track::parse("straight 40").unwrap();
        let path = record_path_in(directory, &track);
        assert_eq!(
            path,
            directory.join(format!(
                "{:016x}-v{RECORD_VERSION}.best",
                track.source_hash()
            ))
        );
        assert_ne!(
            path,
            directory.join(format!("{:016x}.best", track.source_hash()))
        );
        assert_ne!(
            record_path_in(directory, &Track::parse("straight 41").unwrap()),
            path
        );
        // Records sit beside the bundled tracks, wherever the game started.
        assert_eq!(
            record_path(&track),
            courses::game_root()
                .unwrap()
                .join("data")
                .join(path.file_name().unwrap())
        );
    }

    #[test]
    fn a_held_record_lock_fails_quickly_without_changing_the_record() {
        let directory = TempDir::new("apex-record-lock-timeout-test");
        let path = directory.path().join("course.best");
        save_record(&path, 60.0).unwrap();
        // A second handle conflicts even within one process.
        let holder = std::fs::File::create(path.with_extension("lock")).unwrap();
        holder.lock().unwrap();
        let started = std::time::Instant::now();
        let error = save_record(&path, 40.0).unwrap_err();
        assert_eq!(error.kind(), std::io::ErrorKind::TimedOut);
        assert!(started.elapsed() < std::time::Duration::from_secs(2));
        assert_eq!(read_record(&path), Some(60.0));
        assert!(!path.with_extension("tmp").exists());
        holder.unlock().unwrap();
        assert!(save_record(&path, 40.0).unwrap().improved);
        assert_eq!(read_record(&path), Some(40.0));
        assert!(!path.with_extension("tmp").exists());
    }

    #[test]
    fn screenshot_export_is_png_or_reports_io_errors_without_panicking() {
        let dir = TempDir::new("apex-capture-test");
        let path = dir.path().join("nested/capture.without-png-extension");
        let screenshot = Image {
            bytes: vec![255, 0, 0, 255, 0, 0, 255, 255],
            width: 1,
            height: 2,
        };
        save_capture(&path, &screenshot).unwrap();
        let encoded = std::fs::read(&path).unwrap();
        assert_eq!(
            image::guess_format(&encoded).unwrap(),
            image::ImageFormat::Png
        );
        let decoded = image::load_from_memory(&encoded).unwrap().into_rgba8();
        assert_eq!(decoded.dimensions(), (1, 2));
        assert_eq!(decoded.get_pixel(0, 0).0, [0, 0, 255, 255]);
        assert_eq!(decoded.get_pixel(0, 1).0, [255, 0, 0, 255]);
        assert!(save_capture(dir.path(), &screenshot).is_err());
        assert!(save_capture(&path.join("blocked.png"), &screenshot).is_err());
        #[cfg(target_os = "linux")]
        assert!(save_capture(Path::new("/dev/full"), &screenshot).is_err());
    }

    #[test]
    fn a_stale_session_cannot_replace_a_faster_saved_record() {
        let directory = TempDir::new("apex-stale-record-test");
        let path = directory.path().join("course.best");
        let mut fast_records = RunRecords::new(None);
        let mut slow_records = RunRecords::new(None);
        let mut fast_race = fast_records.start_race(0.0, true);
        let mut slow_race = slow_records.start_race(0.0, true);
        assert!(
            fast_records
                .save_candidate(&path, &mut fast_race, Some(40.0))
                .unwrap()
                .unwrap()
                .improved
        );
        let saved = slow_records
            .save_candidate(&path, &mut slow_race, Some(60.0))
            .unwrap()
            .unwrap();
        assert!(!saved.improved);
        assert_eq!(saved.best, 40.0);
        assert_eq!(slow_race.best, Some(40.0));
        assert_eq!(slow_records.start_race(0.0, true).best, Some(40.0));
        assert_eq!(read_record(&path), Some(40.0));
        assert!(!save_record(&path, 40.0).unwrap().improved);

        let saved = slow_records
            .save_candidate(&path, &mut slow_race, Some(35.0))
            .unwrap()
            .unwrap();
        assert!(saved.improved);
        assert_eq!(saved.best, 35.0);
        assert_eq!(slow_race.best, Some(35.0));
        assert_eq!(slow_records.best, Some(35.0));
        assert_eq!(read_record(&path), Some(35.0));
    }

    #[test]
    fn failed_record_saves_do_not_suppress_later_persistent_records() {
        let directory = TempDir::new("apex-record-recovery-test");
        let path = directory.path().join("course.best");
        let track = Track::parse("straight 40").unwrap();
        for previous_best in [None, Some(60.0)] {
            if let Some(best) = previous_best {
                save_record(&path, best).unwrap();
            }
            let mut records = RunRecords::new(previous_best);
            let mut race = records.start_race(track.finish_distance() - 1.0, true);
            race.started = true;
            race.next_checkpoint = track.checkpoints.len();
            let candidate = race.update(&track, 40.0, track.finish_distance(), true);
            assert_eq!(candidate, Some(40.0));
            // Block the temporary write regardless of test-user permissions.
            let blocked = path.with_extension("tmp");
            std::fs::create_dir(&blocked).unwrap();
            assert!(records.save_candidate(&path, &mut race, candidate).is_err());
            assert_eq!(race.last, Some(40.0));
            assert_eq!(race.best, previous_best);
            assert_eq!(records.best, previous_best);
            assert_eq!(read_record(&path), previous_best);

            // Once the destination is repaired, even a slower run can beat the
            // persistent record. The failed write must not hide that candidate.
            std::fs::remove_dir(&blocked).unwrap();
            race = records.start_race(track.finish_distance() - 1.0, true);
            race.started = true;
            race.next_checkpoint = track.checkpoints.len();
            let candidate = race.update(&track, 50.0, track.finish_distance(), true);
            assert_eq!(candidate, Some(50.0));
            let saved = records
                .save_candidate(&path, &mut race, candidate)
                .unwrap()
                .unwrap();
            assert!(saved.improved);
            assert_eq!(read_record(&path), Some(50.0));
            assert_eq!(race.best, Some(50.0));
            assert_eq!(records.best, Some(50.0));
            std::fs::remove_file(&path).unwrap();
        }
    }

    #[test]
    fn record_writers_share_one_lock_across_processes() {
        const CHILD: &str = "APEX_RECORD_PROCESS_TEST";
        const WRITER: &str = "APEX_RECORD_PROCESS_WRITER";
        fn wait_for_files(paths: &[PathBuf]) {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
            while !paths.iter().all(|path| path.exists()) {
                assert!(
                    std::time::Instant::now() < deadline,
                    "record writer timed out"
                );
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        }
        if let Some(directory) = std::env::var_os(CHILD) {
            let directory = PathBuf::from(directory);
            let writer: u32 = std::env::var(WRITER).unwrap().parse().unwrap();
            std::fs::write(directory.join(format!("ready-{writer}")), []).unwrap();
            wait_for_files(&[directory.join("go")]);
            for step in 0..32 {
                let time = (1000 - step * 4 - writer) as f32;
                // The game gives up on a contended lock after a short wait
                // and retries after a later run; do the same here.
                let saved = loop {
                    match save_record(&directory.join("course.best"), time) {
                        Err(error) if error.kind() == std::io::ErrorKind::TimedOut => {}
                        result => break result.unwrap(),
                    }
                };
                assert!(saved.best <= time);
                std::thread::yield_now();
            }
            return;
        }

        let fixture = TempDir::new("apex-concurrent-record-test");
        let directory = fixture.path();
        let children: Vec<_> = (0..4)
            .map(|writer| {
                std::process::Command::new(std::env::current_exe().unwrap())
                    .args([
                        "--exact",
                        "driving_checks::record_writers_share_one_lock_across_processes",
                        "--nocapture",
                    ])
                    .env(CHILD, directory)
                    .env(WRITER, writer.to_string())
                    .stdout(std::process::Stdio::piped())
                    .stderr(std::process::Stdio::piped())
                    .spawn()
                    .unwrap()
            })
            .collect();
        let ready: Vec<_> = (0..4)
            .map(|writer| directory.join(format!("ready-{writer}")))
            .collect();
        wait_for_files(&ready);
        std::fs::write(directory.join("go"), []).unwrap();
        let outputs: Vec<_> = children
            .into_iter()
            .map(|child| child.wait_with_output().unwrap())
            .collect();
        let best = read_record(&directory.join("course.best"));
        assert!(directory.join("course.lock").is_file());
        assert!(!directory.join("course.tmp").exists());
        for output in outputs {
            assert!(
                output.status.success(),
                "record writer failed:\n{}\n{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
        }
        assert_eq!(best, Some(873.0));
    }

    #[test]
    fn record_saving_preserves_read_errors_and_recovers_invalid_numbers() {
        let directory = TempDir::new("apex-record-errors-test");
        let path = directory.path().join("course.best");
        std::fs::create_dir(&path).unwrap();
        assert!(save_record(&path, 60.0).is_err());
        assert!(path.is_dir());
        std::fs::remove_dir(&path).unwrap();
        std::fs::write(&path, "NaN\n").unwrap();
        let saved = save_record(&path, 60.0).unwrap();
        assert!(saved.improved);
        assert_eq!(saved.best, 60.0);
        assert_eq!(read_record(&path), Some(60.0));
    }

    #[test]
    fn corrupt_record_encoding_can_be_replaced_by_a_completed_run() {
        let directory = TempDir::new("apex-record-encoding-test");
        let path = directory.path().join("course.best");
        std::fs::write(&path, [0xff, 0xfe, b'6', b'0', b'\n']).unwrap();
        let track = Track::parse("straight 40").unwrap();
        let mut records = RunRecords::new(read_record(&path));
        let mut race = records.start_race(track.finish_distance() - 1.0, true);
        race.started = true;
        race.next_checkpoint = track.checkpoints.len();
        let candidate = race.update(&track, 40.0, track.finish_distance(), true);
        let saved = records
            .save_candidate(&path, &mut race, candidate)
            .unwrap()
            .unwrap();
        assert!(saved.improved);
        assert_eq!(saved.best, 40.0);
        assert_eq!(read_record(&path), Some(40.0));
        assert_eq!(race.best, Some(40.0));
        assert_eq!(records.start_race(0.0, true).best, Some(40.0));
    }

    #[test]
    fn demo_option_starts_autodrive_and_preserves_preview_options() {
        for args in [
            vec!["--demo", "--track", "tracks/club.track", "--frames", "180"],
            vec!["--track", "tracks/club.track", "--frames", "180", "--demo"],
        ] {
            let opts = options(args.into_iter().map(String::from)).unwrap();
            assert!(opts.demo && opts.autodrive);
            assert_eq!(opts.path, PathBuf::from("tracks/club.track"));
            assert_eq!(opts.frames, Some(180));
        }
        let opts = options(["--demo"]).unwrap();
        assert!(opts.demo && opts.autodrive);
        assert!(opts.frames.is_none());
        assert_eq!(opts.path, PathBuf::from(courses::BUNDLED_PATHS[0]));
        for args in [vec![], vec!["--autodrive"], vec!["--smoke-test"]] {
            assert!(!options(args.into_iter().map(String::from)).unwrap().demo);
        }
    }

    fn finish_straight_run(
        track: &Track,
        car: &mut Car,
        race: &mut Race,
        records: &mut RunRecords,
        throttle: f32,
    ) -> Option<f32> {
        let mut accepted = None;
        for _ in 0..1200 {
            car.update(
                track,
                Control {
                    throttle,
                    ..Control::default()
                },
                1. / 120.,
            );
            let candidate = race.update(track, 1. / 120., car.distance, car.on_course());
            accepted = records.accept(candidate).or(accepted);
            if race.finished {
                break;
            }
        }
        assert!(race.finished && !race.invalid());
        accepted
    }

    #[test]
    fn fresh_manual_runs_can_record_after_autodrive_or_position_previews() {
        let track = Track::parse("straight 40").unwrap();
        for (autodrive, at) in [(true, None), (false, Some(6.))] {
            for persisted_best in [None, Some(60.)] {
                let mut car = Car::new(&track);
                if let Some(at) = at {
                    car.reset(&track, at);
                }
                let mut records = RunRecords::new(persisted_best);
                let mut race = records.start_race(car.distance, !autodrive && at.is_none());
                race.started = true;
                assert_eq!(
                    finish_straight_run(&track, &mut car, &mut race, &mut records, 1.),
                    None
                );
                let preview_best = race.best.unwrap();
                assert_eq!(records.best, persisted_best);

                // Taking over does not retroactively make the mixed run eligible.
                assert_eq!(records.accept(Some(preview_best)), None);
                restart_run(&mut car, &track, &mut records, &mut race, false);
                assert_eq!(race.best, persisted_best);
                let manual_best =
                    finish_straight_run(&track, &mut car, &mut race, &mut records, 0.7).unwrap();
                assert!(manual_best > preview_best);
                assert_eq!(records.best, Some(manual_best));

                restart_run(&mut car, &track, &mut records, &mut race, false);
                assert_eq!(race.best, Some(manual_best));
            }
        }
    }

    #[test]
    fn restarting_a_finished_sprint_preserves_its_last_time() {
        let track = Track::parse("straight 40").unwrap();
        let mut car = Car::new(&track);
        let mut records = RunRecords::new(None);
        let mut race = records.start_race(car.distance, true);
        race.started = true;
        let completed = finish_straight_run(&track, &mut car, &mut race, &mut records, 0.7);
        assert!(completed.is_some());
        assert_eq!(race.last, completed);

        // Enter after finishing, R, and automatic recovery share this reset.
        // A second reset before finishing must retain the same completed time.
        for _ in 0..2 {
            restart_run(&mut car, &track, &mut records, &mut race, false);
            assert!(race.started && !race.finished && !race.invalid());
            assert_eq!(race.last, completed);
            assert_eq!(race.best, completed);
            assert_eq!(race.elapsed, 0.);
            assert_eq!(race.completed_runs, 0);
            assert_eq!(race.next_checkpoint, 0);
            assert_eq!(car.distance, Car::new(&track).distance);
            race.update(&track, 1. / 120., car.distance, true);
        }
    }

    #[test]
    fn restarting_an_active_demonstration_keeps_records_ineligible() {
        let track = Track::parse("straight 40").unwrap();
        let mut car = Car::new(&track);
        let mut records = RunRecords::new(Some(60.));
        let mut race = records.start_race(car.distance, false);
        for _ in 0..2 {
            restart_run(&mut car, &track, &mut records, &mut race, true);
            assert_eq!(
                finish_straight_run(&track, &mut car, &mut race, &mut records, 1.),
                None
            );
            assert_eq!(records.best, Some(60.));
        }
    }

    #[test]
    fn arrow_and_letter_keys_steer_in_their_screen_direction() {
        let track = Track::parse("width 40\nstraight 200").unwrap();
        for (key, direction) in [
            (KeyCode::Right, 1.),
            (KeyCode::D, 1.),
            (KeyCode::Left, -1.),
            (KeyCode::A, -1.),
        ] {
            let mut car = Car::new(&track);
            car.velocity = Vec3::Z * 15.;
            let camera = view::DriverCamera::new(car.position, 0., 0., 0., 54., 16. / 9.);
            let control = controls_from_keys(|pressed| pressed == key);
            for _ in 0..60 {
                car.update(&track, control, 1. / 120.);
            }
            assert!(car.heading * direction > 0.01, "{key:?}: yaw");
            assert!(
                camera.matrix().project_point3(car.position).x * direction > 0.01,
                "{key:?}: screen direction"
            );
        }
    }

    #[test]
    fn mouse_motion_steers_the_car_in_its_screen_direction() {
        let track = Track::parse("width 40\nstraight 200").unwrap();
        for direction in [-1., 1.] {
            let mut mouse = input::MouseDriving::default();
            mouse.toggle();
            mouse.update_frame([], Vec2::ZERO, true);
            let position = vec2(150. * direction, 0.);
            mouse.update_frame([position], position, true);
            let mut car = Car::new(&track);
            car.velocity = Vec3::Z * 15.;
            let camera = view::DriverCamera::new(car.position, 0., 0., 0., 54., 16. / 9.);
            for _ in 0..60 {
                car.update(&track, mouse.control(false), 1. / 120.);
            }
            assert!(car.heading * direction > 0.01);
            assert!(camera.matrix().project_point3(car.position).x * direction > 0.01);
        }
    }

    #[test]
    fn demo_drives_every_bundled_course_and_repeats_without_saving_records() {
        let opts = options(["--demo"]).unwrap();
        let mut path = opts.path.clone();
        for expected in courses::BUNDLED_PATHS
            .iter()
            .cycle()
            .take(courses::BUNDLED_PATHS.len() * 2)
        {
            assert_eq!(path, PathBuf::from(expected));
            let track = Track::load(resolve_track(&path).unwrap()).unwrap();
            let mut car = Car::new(&track);
            let mut records = RunRecords::new(None);
            let mut race = records.start_race(car.distance, !opts.autodrive);
            race.started = opts.autodrive;
            assert_eq!(race.completed_runs, 0);
            for _ in 0..(240 * 60) {
                // Match the game's 60 Hz control input and 120 Hz physics.
                let control = demo_control(&car, &track);
                for _ in 0..2 {
                    car.update(&track, control, 1. / 120.);
                    let candidate = race.update(&track, 1. / 120., car.distance, car.on_course());
                    assert!(records.accept(candidate).is_none());
                    if race.completed_runs > 0 {
                        break;
                    }
                }
                if race.completed_runs > 0 {
                    break;
                }
            }
            assert!(
                race.last.is_some(),
                "{}: stopped at {:.1}/{:.1}m; {:.1}km/h; offroad {}; position {:?}",
                track.name,
                car.distance,
                track.length,
                car.speed_kmh(),
                car.offroad,
                car.position
            );
            assert!(
                race.best.is_some(),
                "{}: invalid completion at {:.1}m; checkpoint {}",
                track.name,
                car.distance,
                race.next_checkpoint
            );
            assert_eq!(race.completed_runs, 1);
            assert!(records.best.is_none());
            path = courses::next_path(&path).unwrap();
        }
        assert_eq!(path, opts.path);
    }
}

#[cfg(test)]
mod frame_checks {
    use super::*;
    use crate::test_support::TempDir;

    const COURSE: &str = "name \"Test Straight\"\nwidth 12\nstraight 150\n";

    /// One 60 Hz frame in a focused window with these command keys pressed.
    fn frame(pressed: &[KeyCode]) -> FrameInput {
        FrameInput {
            frame_time: 1. / 60.,
            focus_lost: false,
            focus_gained: false,
            focused: true,
            pressed: pressed.to_vec(),
            right_click: false,
            keys: Control::default(),
            mouse_motion: Vec::new(),
            mouse_position: Vec2::ZERO,
        }
    }

    fn idle() -> FrameInput {
        frame(&[])
    }

    fn lasting(frame_time: f32) -> FrameInput {
        FrameInput {
            frame_time,
            ..idle()
        }
    }

    fn right_click() -> FrameInput {
        FrameInput {
            right_click: true,
            ..idle()
        }
    }

    fn throttle() -> FrameInput {
        FrameInput {
            keys: Control {
                throttle: 1.,
                ..Control::default()
            },
            ..idle()
        }
    }

    /// A game on `track_path`, keeping records in the fixture's `data/`.
    fn game_on(dir: &TempDir, track_path: &Path, args: &[&str]) -> GameState {
        let mut all = vec![OsString::from("--track"), track_path.into()];
        all.extend(args.iter().map(OsString::from));
        let opts = options(all).unwrap();
        let track = Track::load(&opts.path).unwrap();
        GameState::new(&opts, track, Some(dir.path().join("data")))
    }

    /// A game on a short custom straight in a fresh fixture directory.
    fn game(args: &[&str]) -> (TempDir, GameState) {
        let dir = TempDir::new("apex-game-state-test");
        let path = dir.path().join("course.track");
        std::fs::write(&path, COURSE).unwrap();
        let state = game_on(&dir, &path, args);
        (dir, state)
    }

    fn bundled(index: usize) -> PathBuf {
        resolve_track(Path::new(courses::BUNDLED_PATHS[index])).unwrap()
    }

    fn grid_distance(state: &GameState) -> f32 {
        Car::new(&state.track).distance
    }

    #[test]
    fn escape_closes_help_before_pausing() {
        let (_dir, mut state) = game(&[]);
        state.update(&frame(&[KeyCode::F1]));
        assert!(state.help && !state.paused);
        state.update(&frame(&[KeyCode::Escape]));
        assert!(!state.help && !state.paused);
        state.update(&frame(&[KeyCode::Escape]));
        assert!(state.paused && !state.help);
        // Help opened over the pause menu closes first, leaving it paused.
        state.update(&frame(&[KeyCode::F1]));
        assert!(state.help && state.paused);
        state.update(&frame(&[KeyCode::Escape]));
        assert!(!state.help && state.paused);
        state.update(&frame(&[KeyCode::Escape]));
        assert!(!state.help && !state.paused);
    }

    #[test]
    fn q_quits_only_while_paused() {
        let (_dir, mut state) = game(&[]);
        assert_eq!(state.update(&frame(&[KeyCode::Q])), []);
        // Escape is handled first: it resumes, so the same frame's Q does not quit.
        state.update(&frame(&[KeyCode::Escape]));
        assert!(state.paused);
        assert_eq!(state.update(&frame(&[KeyCode::Escape, KeyCode::Q])), []);
        assert!(!state.paused);
        state.update(&frame(&[KeyCode::Escape]));
        // Earlier side effects in the frame still happen; nothing after Q does.
        assert_eq!(
            state.update(&frame(&[KeyCode::F11, KeyCode::Q, KeyCode::Enter])),
            [Action::SetFullscreen(true), Action::Quit]
        );
        assert!(state.paused && !state.race.started);
    }

    #[test]
    fn enter_starts_a_run_and_restarts_a_finished_one() {
        let (_dir, mut state) = game(&[]);
        let grid = grid_distance(&state);
        state.update(&throttle());
        assert!(!state.race.started);
        assert_eq!(state.car.distance, grid);
        assert_eq!(state.race.elapsed, 0.);

        // The run starts and is simulated on the frame Enter is pressed.
        state.update(&frame(&[KeyCode::Enter]));
        assert!(state.race.started && !state.paused);
        assert!(state.race.elapsed > 0.);

        // Enter also resumes a paused run in place, closing help.
        state.update(&throttle());
        state.update(&frame(&[KeyCode::Escape]));
        state.update(&frame(&[KeyCode::F1]));
        let distance = state.car.distance;
        assert!(distance > grid);
        state.update(&frame(&[KeyCode::Enter]));
        assert!(!state.paused && !state.help);
        assert!(state.car.distance >= distance);

        for _ in 0..3000 {
            if state.race.finished {
                break;
            }
            state.update(&throttle());
        }
        assert!(state.race.finished && !state.race.invalid());
        let finish_time = state.race.last.unwrap();
        assert_eq!(state.notification(), Some("NEW PERSONAL BEST"));
        assert_eq!(read_record(&state.record_file), Some(finish_time));
        // A finished run stays put.
        let finished_at = state.car.distance;
        state.update(&throttle());
        assert_eq!(state.car.distance, finished_at);

        state.update(&frame(&[KeyCode::Enter]));
        assert!(state.race.started && !state.race.finished);
        assert_eq!(state.race.last, Some(finish_time));
        assert_eq!(state.race.best, Some(finish_time));
        assert!(state.race.elapsed < 0.1);
        assert!(state.car.distance < grid + 1.);
    }

    #[test]
    fn r_restarts_the_run_and_snaps_the_camera() {
        let (_dir, mut state) = game(&[]);
        let grid = grid_distance(&state);
        state.update(&frame(&[KeyCode::Enter]));
        for _ in 0..60 {
            state.update(&throttle());
        }
        assert!(state.car.distance > grid + 1.);
        state.update(&frame(&[KeyCode::Escape]));
        state.attitude.pitch = 0.5;
        state.attitude.roll = -0.3;
        state.respawn_timer = 2.;

        state.update(&frame(&[KeyCode::R]));
        assert_eq!(state.car.distance, grid);
        assert!(state.race.started && state.race.elapsed == 0.);
        assert_eq!(
            (state.attitude.pitch, state.attitude.roll),
            (state.car.pitch, state.car.roll)
        );
        assert_eq!((state.accumulator, state.respawn_timer), (0., 0.));
        assert_eq!(state.notification(), Some("NEW RUN"));
        // Restarting does not resume a paused run.
        assert!(state.paused);
        for _ in 0..121 {
            state.update(&idle());
        }
        assert_eq!(state.notification(), None);
    }

    #[test]
    fn a_failed_reload_keeps_the_current_track_and_shows_the_error() {
        let (dir, mut state) = game(&[]);
        let path = dir.path().join("course.track");
        let original = state.track.source_hash();
        state.update(&frame(&[KeyCode::Enter]));
        std::fs::write(&path, "straight banana\n").unwrap();
        let distance = state.car.distance;

        let actions = state.update(&FrameInput {
            pressed: vec![KeyCode::F5],
            ..throttle()
        });
        assert!(!actions.contains(&Action::RebuildWorld));
        assert_eq!(state.track.source_hash(), original);
        assert_eq!(state.path, path);
        let note = state.notification().unwrap();
        assert!(note.starts_with("RELOAD FAILED: "), "{note}");
        assert!(note.contains("banana"), "{note}");
        assert_eq!(state.note_time, 15.);
        // The current run carries on.
        assert!(state.race.started && !state.paused);
        assert!(state.car.distance > distance);

        std::fs::write(&path, "name \"Edited\"\nstraight 90\n").unwrap();
        let actions = state.update(&frame(&[KeyCode::F5]));
        assert_eq!(actions, [Action::RebuildWorld]);
        assert_eq!(state.track.name, "Edited");
        assert_ne!(state.track.source_hash(), original);
        assert_eq!(state.notification(), Some("TRACK RELOADED"));
        assert!(!state.race.started);
    }

    #[test]
    fn course_changes_reset_the_run_and_records_but_keep_units_and_mouse_mode() {
        let (dir, mut state) = game(&[]);
        let next = Track::load(bundled(0)).unwrap();
        let next_record = record_path_in(&dir.path().join("data"), &next);
        save_record(&next_record, 123.).unwrap();
        state.update(&frame(&[KeyCode::U]));
        state.update(&right_click());
        state.update(&frame(&[KeyCode::Enter]));
        state.update(&frame(&[KeyCode::Escape]));
        state.update(&frame(&[KeyCode::F1]));
        state.attitude.pitch = 0.5;
        assert!(state.mouse.enabled() && state.paused && state.help);

        let actions = state.update(&frame(&[KeyCode::Tab]));
        assert!(actions.contains(&Action::RebuildWorld));
        assert_eq!(state.path, bundled(0));
        assert_eq!(state.track.source_hash(), next.source_hash());
        assert_eq!(state.record_file, next_record);
        assert_eq!(state.race.best, Some(123.));
        assert_eq!(state.race.elapsed, 0.);
        assert!(!state.race.started && !state.paused);
        assert_eq!(state.car.distance, grid_distance(&state));
        assert_eq!(
            (state.attitude.pitch, state.attitude.roll),
            (state.car.pitch, state.car.roll)
        );
        assert_eq!(state.notification(), Some("COURSE LOADED"));
        // Player preferences and the help screen survive the change.
        assert!(matches!(state.units, hud::Units::Metric));
        assert!(state.mouse.enabled() && state.help);

        // Reloading rereads the record, which may have changed on disk.
        save_record(&next_record, 100.).unwrap();
        state.update(&frame(&[KeyCode::F1]));
        state.update(&frame(&[KeyCode::Enter]));
        state.update(&frame(&[KeyCode::Escape]));
        state.update(&frame(&[KeyCode::F5]));
        assert_eq!(state.path, bundled(0));
        assert_eq!(state.race.best, Some(100.));
        assert!(!state.race.started && !state.paused);
        assert_eq!(state.notification(), Some("TRACK RELOADED"));
        assert!(matches!(state.units, hud::Units::Metric) && state.mouse.enabled());
    }

    #[test]
    fn losing_focus_pauses_only_an_active_manual_run() {
        let lose_focus = FrameInput {
            focus_lost: true,
            focused: false,
            ..idle()
        };
        let (_dir, mut state) = game(&[]);
        state.update(&lose_focus);
        assert!(!state.paused, "the start screen is not a run");

        state.update(&frame(&[KeyCode::Enter]));
        state.update(&right_click());
        assert!(state.mouse.enabled() && state.cursor_grabbed);
        assert_eq!(state.update(&lose_focus), [Action::GrabCursor(false)]);
        assert!(state.paused && state.mouse.enabled());
        let distance = state.car.distance;
        // Without focus the run stays still even once resumed.
        state.update(&FrameInput {
            pressed: vec![KeyCode::Escape],
            focused: false,
            ..throttle()
        });
        assert!(!state.paused);
        assert_eq!(state.car.distance, distance);

        state.race.finished = true;
        state.update(&lose_focus);
        assert!(!state.paused, "a finished run has nothing to hold");

        let (_dir, mut state) = game(&["--autodrive"]);
        state.update(&lose_focus);
        assert!(!state.paused, "the auto driver keeps driving");
        let distance = state.car.distance;
        state.update(&FrameInput {
            focused: false,
            ..idle()
        });
        assert!(state.car.distance > distance);
    }

    #[test]
    fn the_click_that_refocuses_the_window_does_not_toggle_mouse_driving() {
        let (_dir, mut state) = game(&["--autodrive"]);
        state.update(&FrameInput {
            focus_lost: true,
            focused: false,
            ..idle()
        });
        state.update(&FrameInput {
            focused: false,
            ..right_click()
        });
        assert!(!state.mouse.enabled() && state.autodrive);
        state.update(&FrameInput {
            focus_gained: true,
            ..right_click()
        });
        assert!(!state.mouse.enabled() && state.autodrive);
        // Clicks replayed shortly after activation are ignored too.
        state.update(&FrameInput {
            frame_time: 0.1,
            ..right_click()
        });
        assert!(!state.mouse.enabled());
        state.update(&lasting(0.1));
        state.update(&lasting(0.1));
        state.update(&right_click());
        assert!(state.mouse.enabled() && !state.autodrive);
        state.update(&right_click());
        assert!(!state.mouse.enabled() && !state.autodrive);
    }

    #[test]
    fn a_car_falling_below_the_course_returns_to_the_grid() {
        let (_dir, mut state) = game(&[]);
        let grid = grid_distance(&state);
        state.update(&frame(&[KeyCode::Enter]));
        for _ in 0..60 {
            state.update(&throttle());
        }
        let floor = state.track.ground_height() - FALL_DEPTH;
        // Just above the threshold is still a fall in progress.
        state.car.position.y = floor + 1.;
        state.car.grounded = false;
        state.update(&idle());
        assert_ne!(state.notification(), Some("BACK ON THE GRID"));
        assert!(state.car.distance > grid + 1.);

        state.car.position.y = floor - 5.;
        state.attitude.pitch = 0.5;
        state.update(&idle());
        assert_eq!(state.notification(), Some("BACK ON THE GRID"));
        assert_eq!(state.car.distance, grid);
        assert!(state.race.started && state.race.elapsed < 0.01);
        assert_eq!(state.accumulator, 0.);
        assert!(state.attitude.pitch.abs() < 0.01);

        state.car.position.x = f32::NAN;
        state.update(&idle());
        assert!(state.car.position.is_finite());
        assert_eq!(state.car.distance, grid);
    }

    #[test]
    fn a_car_stopped_off_the_road_is_told_how_to_restart() {
        let (_dir, mut state) = game(&["--at", "60"]);
        state.update(&frame(&[KeyCode::Enter]));
        state.car.position.x += 25.;
        state.update(&idle());
        assert!(state.car.offroad && state.car.speed_kmh() < 4.);
        for _ in 0..28 {
            state.update(&lasting(0.1));
        }
        assert_eq!(state.notification(), None);
        for _ in 0..3 {
            state.update(&lasting(0.1));
        }
        assert_eq!(
            state.notification(),
            Some("OFF COURSE · PRESS R TO RESTART")
        );
        // Pausing stops the hint's timer rather than resetting it.
        state.update(&frame(&[KeyCode::Escape]));
        assert!(state.respawn_timer > OFF_COURSE_HINT_DELAY);
        state.update(&frame(&[KeyCode::R]));
        assert_eq!(state.respawn_timer, 0.);
        assert_eq!(state.notification(), Some("NEW RUN"));
    }

    #[test]
    fn the_off_course_timer_follows_the_fixed_frame_clock() {
        // Smoke tests and captures must not depend on the machine's speed.
        let (_dir, mut state) = game(&["--autodrive", "--frames", "100", "--at", "60"]);
        state.car.position.x += 25.;
        state.update(&idle());
        assert!(state.car.offroad && state.car.speed_kmh() < 4.);
        let before = state.respawn_timer;
        state.update(&lasting(0.1));
        assert!(state.car.offroad && state.car.speed_kmh() < 4.);
        assert!((state.respawn_timer - before - FIXED_FRAME_TIME).abs() < 1e-6);
    }

    #[test]
    fn demo_mode_moves_on_after_a_completed_run() {
        let dir = TempDir::new("apex-demo-state-test");
        let mut state = game_on(&dir, &bundled(0), &["--demo"]);
        assert!(state.race.started && state.autodrive);
        state.update(&idle());
        assert_eq!(state.path, bundled(0));

        state.race.completed_runs = 1;
        // Not while paused or reading help, and a reload takes precedence.
        state.update(&frame(&[KeyCode::Escape]));
        assert!(state.paused);
        assert_eq!(state.path, bundled(0));
        state.update(&frame(&[KeyCode::Escape, KeyCode::F5]));
        assert!(!state.paused);
        assert_eq!(state.path, bundled(0));
        assert_eq!(state.notification(), Some("TRACK RELOADED"));
        assert_eq!(state.race.completed_runs, 0);
        state.race.completed_runs = 1;
        state.update(&frame(&[KeyCode::F1]));
        assert_eq!(state.path, bundled(0));

        let actions = state.update(&frame(&[KeyCode::F1]));
        assert!(actions.contains(&Action::RebuildWorld));
        assert_eq!(state.path, bundled(1));
        assert_eq!(state.notification(), Some("COURSE LOADED"));
        assert!(state.race.started && state.race.completed_runs == 0);

        // A manual takeover ends the cycle.
        state.update(&right_click());
        state.race.completed_runs = 1;
        state.update(&idle());
        assert_eq!(state.path, bundled(1));
    }

    #[test]
    fn frame_limits_end_the_game_and_fix_the_auto_driver_step() {
        let (_dir, mut state) = game(&["--frames", "3"]);
        for _ in 0..2 {
            state.update(&idle());
            assert!(!state.frame_limit_reached());
        }
        state.update(&idle());
        assert!(state.frame_limit_reached());

        let (_dir, mut state) = game(&["--capture", "shot.png"]);
        for _ in 0..CAPTURE_FRAMES {
            assert!(!state.frame_limit_reached());
            state.update(&idle());
        }
        assert!(state.frame_limit_reached());
        let (_dir, state) = game(&[]);
        assert!(!state.frame_limit_reached());

        // Smoke tests advance a fixed 1/60 s per frame, however long it took.
        let (_dir, mut state) = game(&["--smoke-test"]);
        state.update(&lasting(0.1));
        assert!((state.race.elapsed - 1. / 60.).abs() < 1e-6);
        // Interactive frames, and frames after a takeover, use real time.
        let (_dir, mut state) = game(&["--frames", "100"]);
        state.update(&FrameInput {
            pressed: vec![KeyCode::Enter],
            ..lasting(0.1)
        });
        assert!(state.race.elapsed > 0.09);
    }
}
