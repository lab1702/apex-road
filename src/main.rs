mod courses;
mod hud;
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
    at: Option<f32>,
}
fn options() -> Result<Options, String> {
    let mut out = Options {
        path: "tracks/alpine.track".into(),
        validate: false,
        frames: None,
        capture: None,
        autodrive: false,
        at: None,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--track" => out.path = args.next().ok_or("--track needs a path")?.into(),
            "--validate" => {
                out.validate = true;
                if let Some(path) = args.next() {
                    out.path = path.into();
                }
            }
            "--frames" => {
                out.frames = Some(
                    args.next()
                        .ok_or("--frames needs a positive integer")?
                        .parse()
                        .map_err(|_| "invalid frame count")?,
                )
            }
            "--capture" => {
                out.capture = Some(args.next().ok_or("--capture needs a PNG path")?.into())
            }
            "--autodrive" => out.autodrive = true,
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
                    "APEX / ROAD\n\ncargo run --release -- [--track tracks/alpine.track]\n  --validate PATH   Check a track without opening a window\n  --smoke-test      Render 240 frames of automated driving\n  --autodrive       Run the demonstration driver\n  --frames N        Exit after N rendered frames\n  --capture PATH    Save the final frame as PNG (pair with --frames)\n  --at METERS       Preview a position on the track\n\nEnter start · WASD / arrows drive · Space handbrake · R restart\nEsc pause · F1 help · F5 reload · Tab track · F11 fullscreen"
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
    let opts = options().unwrap_or_else(|e| {
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
    macroquad::Window::from_config(config(), game(opts, track));
}
fn record_path(path: &Path) -> PathBuf {
    let bytes = std::fs::read(resolve_track(path)).unwrap_or_default();
    let hash = bytes.iter().fold(0xcbf29ce484222325u64, |h, b| {
        (h ^ *b as u64).wrapping_mul(0x100000001b3)
    });
    PathBuf::from(format!("data/{hash:016x}.best"))
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
fn grid_distance(track: &Track) -> f32 {
    if track.closed { 0. } else { 5. }
}

async fn game(opts: Options, mut track: Track) {
    hud::init();
    prevent_quit();
    let mut path = opts.path.clone();
    let mut world = world::World::new(&track);
    let mut car = Car::new(&track);
    if let Some(at) = opts.at {
        car.reset(&track, at.min(track.length - 5.));
    }
    let mut record_file = record_path(&path);
    let mut race = Race::new(read_record(&record_file), car.distance);
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
    let preview = opts.at.is_some() || opts.autodrive;
    let fixed = 1. / 120.;
    loop {
        if is_quit_requested() {
            break;
        }
        let dt = get_frame_time().clamp(0., 0.1);
        frames += 1;
        note_time = (note_time - dt).max(0.);
        if is_key_pressed(KeyCode::F11) {
            fullscreen = !fullscreen;
            set_fullscreen(fullscreen);
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
        if is_key_pressed(KeyCode::Enter) {
            if race.finished {
                car.reset(&track, grid_distance(&track));
                race = Race::new(race.best, car.distance);
            }
            race.started = true;
            paused = false;
            help = false;
        }
        if is_key_pressed(KeyCode::R) {
            car.reset(&track, grid_distance(&track));
            let last = race.last;
            race = Race::new(race.best, car.distance);
            race.last = last;
            race.started = true;
            accumulator = 0.;
            respawn_timer = 0.;
            note = "NEW RUN".into();
            note_time = 2.;
        }
        let switch = is_key_pressed(KeyCode::Tab);
        let reload = is_key_pressed(KeyCode::F5);
        if switch || reload {
            let next = if switch {
                courses::next_path(&path)
            } else {
                path.clone()
            };
            match Track::load(resolve_track(&next)) {
                Ok(new_track) => {
                    track = new_track;
                    world = world::World::new(&track);
                    path = next;
                    car = Car::new(&track);
                    record_file = record_path(&path);
                    race = Race::new(read_record(&record_file), car.distance);
                    race.started = opts.autodrive;
                    camera_pitch = car.pitch;
                    camera_roll = car.roll;
                    accumulator = 0.;
                    paused = false;
                    respawn_timer = 0.;
                    note = if reload {
                        "TRACK RELOADED"
                    } else {
                        "COURSE LOADED"
                    }
                    .into();
                    note_time = 3.;
                }
                Err(e) => {
                    note = format!("RELOAD FAILED: {e}");
                    eprintln!("{note}");
                    note_time = 15.;
                }
            }
        }
        let control = if opts.autodrive {
            demo_control(&car, &track)
        } else {
            input()
        };
        if race.started && !paused && !help && !race.finished {
            accumulator += if opts.frames.is_some() && opts.autodrive {
                1. / 60.
            } else {
                dt
            };
            while accumulator >= fixed {
                car.update(&track, control, fixed);
                if let Some(best) = race.update(&track, fixed, car.distance, !car.offroad)
                    && !preview
                {
                    if let Err(e) = save_record(&record_file, best) {
                        note = format!("Record could not be saved: {e}");
                        note_time = 8.;
                    } else {
                        note = "NEW PERSONAL BEST".into();
                        note_time = 5.;
                    }
                }
                if !car.position.is_finite() || car.position.y < track.ground_height() - 30. {
                    car.reset(&track, grid_distance(&track));
                    race = Race::new(race.best, car.distance);
                    race.started = true;
                    note = "BACK ON THE GRID".into();
                    note_time = 3.;
                }
                accumulator -= fixed;
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
                elapsed: race.elapsed,
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
                if let Some(parent) = dest.parent() {
                    let _ = std::fs::create_dir_all(parent);
                }
                get_screen_data().export_png(dest.to_str().unwrap_or("capture.png"));
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
    hud::shutdown();
}

#[cfg(test)]
mod driving_checks {
    use super::*;
    use macroquad::camera::Camera;

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
    fn demonstration_driver_completes_every_bundled_course() {
        for path in courses::BUNDLED_PATHS {
            let track = Track::load(Path::new(env!("CARGO_MANIFEST_DIR")).join(path)).unwrap();
            let mut car = Car::new(&track);
            let mut race = Race::new(None, car.distance);
            race.started = true;
            for _ in 0..(240 * 120) {
                let control = demo_control(&car, &track);
                car.update(&track, control, 1. / 120.);
                race.update(&track, 1. / 120., car.distance, !car.offroad);
                if race.finished || race.last.is_some() {
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
        }
    }
}
