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
            track.samples.len(),
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

async fn game(opts: Options, mut track: Track) -> Result<(), String> {
    hud::init();
    prevent_quit();
    let mut path = opts.path.clone();
    let mut world = world::World::new(&track);
    let mut car = Car::new(&track);
    if let Some(at) = opts.at {
        car.reset(&track, at.min(track.length - 5.));
    }
    let mut record_file = record_path(&track);
    let mut records = RunRecords::new(read_record(&record_file));
    let mut race = records.start_race(car.distance, !opts.autodrive && opts.at.is_none());
    race.started = opts.autodrive;
    let mut paused = false;
    let mut help = false;
    let mut units = hud::Units::default();
    let mut fullscreen = false;
    let mut accumulator = 0.;
    let mut frames = 0;
    let mut note = String::new();
    let mut note_time: f32 = 0.;
    let mut camera_pitch = car.pitch;
    let mut camera_roll = car.roll;
    let mut respawn_timer = 0.;
    let mut mouse = input::MouseDriving::default();
    let mut cursor_grabbed = false;
    let mut focus = input::WindowInput::new();
    let input_subscriber = macroquad::input::utils::register_input_subscriber();
    let mut autodrive = opts.autodrive;
    if opts.demo {
        println!("Demo: {}", track.name);
    }
    let fixed = 1. / 120.;
    let mut result = Ok(());
    loop {
        if is_quit_requested() {
            break;
        }
        macroquad::input::utils::repeat_all_miniquad_input(&mut focus, input_subscriber);
        if focus.take_loss() && mouse.enabled() {
            paused = true;
            mouse.reset();
        }
        let dt = get_frame_time().clamp(0., 0.1);
        frames += 1;
        note_time = (note_time - dt).max(0.);
        if is_key_pressed(KeyCode::F11) {
            fullscreen = !fullscreen;
            set_fullscreen(fullscreen);
            mouse.reset();
        }
        if is_key_pressed(KeyCode::F1) {
            help = !help;
        }
        if is_key_pressed(KeyCode::U) {
            units.toggle();
        }
        if is_key_pressed(KeyCode::Escape) {
            if help {
                help = false;
            } else {
                paused = !paused;
            }
        }
        if paused && is_key_pressed(KeyCode::Q) {
            break;
        }
        if focus.focused && is_mouse_button_pressed(MouseButton::Right) {
            mouse.toggle();
            // A manual takeover ends the demonstration for this session.
            autodrive = false;
        }
        if is_key_pressed(KeyCode::Enter) {
            if race.finished {
                restart_run(&mut car, &track, &mut records, &mut race, autodrive);
                mouse.reset();
            }
            race.started = true;
            paused = false;
            help = false;
        }
        if is_key_pressed(KeyCode::R) {
            restart_run(&mut car, &track, &mut records, &mut race, autodrive);
            accumulator = 0.;
            respawn_timer = 0.;
            mouse.reset();
            note = "NEW RUN".into();
            note_time = 2.;
        }
        let demo = opts.demo && autodrive;
        let reload = is_key_pressed(KeyCode::F5);
        let advance_demo = demo && race.completed_runs > 0 && !paused && !help && !reload;
        let switch = is_key_pressed(KeyCode::Tab) || advance_demo;
        if switch || reload {
            let next = if switch {
                courses::next_path(&path).and_then(|path| resolve_track(&path))
            } else {
                Ok(path.clone())
            };
            match next.and_then(|path| Track::load(&path).map(|track| (path, track))) {
                Ok((next, new_track)) => {
                    track = new_track;
                    world.rebuild(&track);
                    path = next;
                    car = Car::new(&track);
                    record_file = record_path(&track);
                    records = RunRecords::new(read_record(&record_file));
                    race = records.start_race(car.distance, !autodrive);
                    race.started = autodrive;
                    if demo {
                        println!("Demo: {}", track.name);
                    }
                    camera_pitch = car.pitch;
                    camera_roll = car.roll;
                    accumulator = 0.;
                    paused = false;
                    respawn_timer = 0.;
                    mouse.reset();
                    note = if switch {
                        "COURSE LOADED"
                    } else {
                        "TRACK RELOADED"
                    }
                    .into();
                    note_time = 3.;
                }
                Err(e) => {
                    // Stop automatic retries until the user resumes or reloads.
                    if advance_demo {
                        paused = true;
                    }
                    note = format!("RELOAD FAILED: {e}");
                    eprintln!("{note}");
                    note_time = 15.;
                }
            }
        }
        let driving =
            race.started && !paused && !help && !race.finished && (focus.focused || autodrive);
        let capture_mouse = mouse.enabled() && driving;
        if capture_mouse != cursor_grabbed {
            set_cursor_grab(capture_mouse);
            show_mouse(!capture_mouse);
            cursor_grabbed = capture_mouse;
            mouse.reset();
        }
        let (mouse_x, mouse_y) = mouse_position();
        let dpi_scale = macroquad::miniquad::window::dpi_scale();
        mouse.update_frame(
            focus
                .drain_mouse_motion()
                .map(|position| position / dpi_scale),
            vec2(mouse_x, mouse_y),
            capture_mouse,
        );
        let control = if autodrive {
            demo_control(&car, &track)
        } else if mouse.enabled() {
            mouse.control(is_key_down(KeyCode::Space))
        } else {
            input()
        };
        if driving {
            accumulator += if opts.frames.is_some() && autodrive {
                1. / 60.
            } else {
                dt
            };
            while accumulator >= fixed {
                car.update(&track, control, fixed);
                let candidate = race.update(&track, fixed, car.distance, !car.offroad);
                match records.save_candidate(&record_file, &mut race, candidate) {
                    Err(e) => {
                        note = format!("Record could not be saved: {e}");
                        note_time = 8.;
                    }
                    Ok(Some(saved)) => {
                        note = if saved.improved {
                            "NEW PERSONAL BEST"
                        } else {
                            "PERSONAL BEST SYNCED"
                        }
                        .into();
                        note_time = 5.;
                    }
                    Ok(None) => {}
                }
                if !car.position.is_finite() || car.position.y < track.ground_height() - 30. {
                    restart_run(&mut car, &track, &mut records, &mut race, autodrive);
                    mouse.reset();
                    note = "BACK ON THE GRID".into();
                    note_time = 3.;
                    accumulator = 0.;
                    break;
                }
                accumulator -= fixed;
                if race.finished || (demo && race.completed_runs > 0) {
                    accumulator = 0.;
                    break;
                }
            }
            if car.offroad && car.speed_kmh() < 4. {
                respawn_timer += dt;
            } else {
                respawn_timer = 0.;
            }
            if respawn_timer > 3. {
                note = "OFF COURSE · PRESS R TO RESTART".into();
                note_time = 2.;
            }
        } else {
            accumulator = 0.;
        }
        camera_pitch += (car.pitch - camera_pitch) * (dt * 8.).min(1.);
        camera_roll += (car.roll - camera_roll) * (dt * 7.).min(1.);
        let camera = view::DriverCamera::new(
            car.position,
            car.heading,
            camera_pitch,
            camera_roll,
            car.speed_kmh(),
            screen_width() / screen_height().max(1.),
        );
        world.draw(&camera);
        let speed = car.speed_kmh();
        let gear = if car.reversing {
            7
        } else if speed < 1. {
            0
        } else {
            (1. + (speed / 42.).floor()).min(6.) as u8
        };
        let rpm = if gear == 0 {
            950. + control.throttle * 1200.
        } else {
            1800. + (speed % 42.) / 42. * 4300. + control.throttle * 350.
        };
        hud::draw(
            &hud::HudState {
                track_name: &track.name,
                speed,
                units,
                rpm,
                gear,
                throttle: car.throttle,
                brake: car.brake,
                steering: car.steering / 0.58,
                slip: car.slip,
                elapsed: race.elapsed as f32,
                best: race.best,
                last: race.last,
                checkpoint: race.next_checkpoint,
                checkpoint_count: track.checkpoints.len(),
                progress: car.distance / track.length,
                paused,
                started: race.started,
                finished: race.finished,
                invalid: race.invalid,
                airborne: !car.grounded,
                offroad: car.offroad,
                help,
                mouse_enabled: mouse.enabled(),
                autodrive,
                notification: if note_time > 0. { Some(&note) } else { None },
                fps: get_fps(),
            },
            &track,
            car.position,
            car.heading,
        );
        if let Some(limit) = opts.frames
            && frames >= limit
        {
            if let Some(ref dest) = opts.capture {
                if let Err(error) = save_capture(dest, &get_screen_data()) {
                    result = Err(format!("Capture error for '{}': {error}", dest.display()));
                    break;
                }
                println!("Captured {}", dest.display());
            }
            println!(
                "Graphics smoke passed: {frames} frames, {:.1} km/h, {:.1} m progress, position {:?}",
                speed, car.distance, car.position
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
            let candidate = race.update(track, 1. / 120., car.distance, !car.offroad);
            accepted = records.accept(candidate).or(accepted);
            if race.finished {
                break;
            }
        }
        assert!(race.finished && !race.invalid);
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
            assert!(race.started && !race.finished && !race.invalid);
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
                    let candidate = race.update(&track, 1. / 120., car.distance, !car.offroad);
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
