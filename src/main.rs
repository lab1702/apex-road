mod courses;
mod hud;
mod input;
mod race;
mod track;
mod vehicle;
mod view;
mod world;

use macroquad::prelude::*;
use race::Race;
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
fn path_argument(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<PathBuf, String> {
    args.next()
        .filter(|value| !value.starts_with('-'))
        .map(PathBuf::from)
        .ok_or_else(|| format!("{flag} needs a path"))
}
fn options(args: impl IntoIterator<Item = String>) -> Result<Options, String> {
    let mut out = Options {
        path: "tracks/alpine.track".into(),
        validate: false,
        frames: None,
        capture: None,
        autodrive: false,
        demo: false,
        at: None,
    };
    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--track" => out.path = path_argument(&mut args, "--track")?,
            "--validate" => {
                out.validate = true;
                out.path = path_argument(&mut args, "--validate")?;
            }
            "--frames" => {
                out.frames = Some(
                    args.next()
                        .ok_or("--frames needs a positive integer")?
                        .parse()
                        .map_err(|_| "invalid frame count")?,
                )
            }
            "--capture" => out.capture = Some(path_argument(&mut args, "--capture")?),
            "--autodrive" => out.autodrive = true,
            "--demo" => {
                out.demo = true;
                out.autodrive = true;
            }
            "--smoke-test" => {
                out.frames = Some(240);
                out.autodrive = true;
            }
            "--at" => {
                out.at = Some(
                    args.next()
                        .ok_or("--at needs a distance in meters")?
                        .parse()
                        .map_err(|_| "invalid distance")?,
                )
            }
            "--help" | "-h" => {
                println!(
                    "APEX / ROAD\n\ncargo run --release -- [--track tracks/alpine.track]\n  --validate PATH   Check a track without opening a window\n  --smoke-test      Render 240 frames of automated driving\n  --autodrive       Run the demonstration driver\n  --demo            Auto-play every bundled track in a repeating loop\n  --frames N        Exit after N rendered frames\n  --capture PATH    Save the final frame as PNG (pair with --frames)\n  --at METERS       Preview a position on the track\n\nEnter start · WASD / arrows drive · Space handbrake · R restart\nRight click toggles mouse driving: left/right steer, up adds throttle, down adds brake\nEsc pause · F1 help · F5 reload · Tab track · F11 fullscreen"
                );
                std::process::exit(0);
            }
            other if !other.starts_with('-') => out.path = other.into(),
            _ => return Err(format!("Unknown option: {arg}. Use --help.")),
        }
    }
    if out.frames == Some(0) {
        return Err("--frames must be positive".into());
    }
    if out.at.is_some_and(|v| !v.is_finite() || v < 0.) {
        return Err("--at must be finite and nonnegative".into());
    }
    if out.capture.is_some() && out.frames.is_none() {
        out.frames = Some(30);
    }
    Ok(out)
}
fn resolve_track(path: &Path) -> PathBuf {
    if path.exists() {
        path.into()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
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
    let opts = options(std::env::args().skip(1)).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2)
    });
    let track = Track::load(resolve_track(&opts.path)).unwrap_or_else(|e| {
        eprintln!("Track error: {e}");
        std::process::exit(1)
    });
    if opts.validate {
        println!(
            "OK: {} | {:.0} m | {} samples | {} checkpoints | {}",
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
fn record_path(track: &Track) -> PathBuf {
    PathBuf::from(format!("data/{:016x}.best", track.source_hash()))
}
fn read_record(path: &Path) -> Option<f32> {
    std::fs::read_to_string(path)
        .ok()?
        .trim()
        .parse::<f32>()
        .ok()
        .filter(|x| x.is_finite() && *x > 0.)
}
fn save_record(path: &Path, time: f32) -> std::io::Result<()> {
    std::fs::create_dir_all("data")?;
    let p = path;
    let temp = p.with_extension("tmp");
    std::fs::write(&temp, format!("{time:.6}\n"))?;
    std::fs::rename(temp, p)
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
}

fn restart_run(car: &mut Car, track: &Track, records: &mut RunRecords, autodrive: bool) -> Race {
    car.reset_to_grid(track);
    let mut race = records.start_race(car.distance, !autodrive);
    race.started = true;
    race
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
                race = restart_run(&mut car, &track, &mut records, autodrive);
                mouse.reset();
            }
            race.started = true;
            paused = false;
            help = false;
        }
        if is_key_pressed(KeyCode::R) {
            let last = race.last;
            race = restart_run(&mut car, &track, &mut records, autodrive);
            race.last = last;
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
                courses::next_path(&path)
            } else {
                path.clone()
            };
            match Track::load(resolve_track(&next)) {
                Ok(new_track) => {
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
                if let Some(best) = records.accept(candidate) {
                    if let Err(e) = save_record(&record_file, best) {
                        note = format!("Record could not be saved: {e}");
                        note_time = 8.;
                    } else {
                        note = "NEW PERSONAL BEST".into();
                        note_time = 5.;
                    }
                }
                if !car.position.is_finite() || car.position.y < track.ground_height() - 30. {
                    race = restart_run(&mut car, &track, &mut records, autodrive);
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
    fn screenshot_export_is_png_or_reports_io_errors_without_panicking() {
        let dir = std::env::temp_dir().join(format!("apex-capture-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("nested/capture.without-png-extension");
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
        assert!(save_capture(&dir, &screenshot).is_err());
        assert!(save_capture(&path.join("blocked.png"), &screenshot).is_err());
        #[cfg(target_os = "linux")]
        assert!(save_capture(Path::new("/dev/full"), &screenshot).is_err());
        std::fs::remove_dir_all(dir).unwrap();
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
        let opts = options(["--demo".into()]).unwrap();
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
                race = restart_run(&mut car, &track, &mut records, false);
                assert_eq!(race.best, persisted_best);
                let manual_best =
                    finish_straight_run(&track, &mut car, &mut race, &mut records, 0.7).unwrap();
                assert!(manual_best > preview_best);
                assert_eq!(records.best, Some(manual_best));

                race = restart_run(&mut car, &track, &mut records, false);
                assert_eq!(race.best, Some(manual_best));
            }
        }
    }

    #[test]
    fn restarting_an_active_demonstration_keeps_records_ineligible() {
        let track = Track::parse("straight 40").unwrap();
        let mut car = Car::new(&track);
        let mut records = RunRecords::new(Some(60.));
        for _ in 0..2 {
            let mut race = restart_run(&mut car, &track, &mut records, true);
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
        let opts = options(["--demo".into()]).unwrap();
        let mut path = opts.path.clone();
        for expected in courses::BUNDLED_PATHS
            .iter()
            .cycle()
            .take(courses::BUNDLED_PATHS.len() * 2)
        {
            assert_eq!(path, PathBuf::from(expected));
            let track = Track::load(resolve_track(&path)).unwrap();
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
            path = courses::next_path(&path);
        }
        assert_eq!(path, opts.path);
    }
}
