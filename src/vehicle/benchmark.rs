//! Fixed autodrive runs that measure physics cost and prove that refactors keep
//! every step bit-identical. Run with
//! `cargo test --release autodrive_benchmark -- --ignored --nocapture`
//! and compare the printed digests before and after a change. The digests are
//! platform-specific (track frames use the platform's sin and cos).

use super::*;

const STEP: f32 = 1.0 / 120.0;

/// The game's demonstration driver, with optional seeded disturbances that
/// push the car into rails, off the road and through jumps.
fn control(car: &Car, track: &Track, seed: &mut u64, noisy: bool, step: usize) -> Control {
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
    let mut control = Control {
        throttle: if speed < target_speed { 0.7 } else { 0. },
        brake: if speed > target_speed + 2. { 0.35 } else { 0. },
        steer: (angle * 2.4).clamp(-1., 1.),
        handbrake: false,
    };
    if noisy && (step / 90) % 3 == 1 {
        // A new disturbance every 0.75 s, held for that interval.
        let mut state = *seed ^ ((step / 90) as u64).wrapping_mul(0x9e3779b97f4a7c15);
        state ^= state >> 33;
        state = state.wrapping_mul(0xff51afd7ed558ccd);
        state ^= state >> 29;
        let pick = |shift: u32| ((state >> shift) & 0xff) as f32 / 255.0;
        control.steer = pick(0) * 2.0 - 1.0;
        control.throttle = pick(8);
        control.brake = if pick(16) > 0.8 { pick(24) } else { 0.0 };
        control.handbrake = pick(32) > 0.9;
    }
    if noisy && step % 2400 == 2399 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
    }
    control
}

fn digest_car(hash: &mut u64, car: &Car) {
    let words = [
        car.position.x.to_bits(),
        car.position.y.to_bits(),
        car.position.z.to_bits(),
        car.velocity.x.to_bits(),
        car.velocity.y.to_bits(),
        car.velocity.z.to_bits(),
        car.heading.to_bits(),
        car.yaw_rate.to_bits(),
        car.steering.to_bits(),
        car.slip.to_bits(),
        car.distance.to_bits(),
        car.pitch.to_bits(),
        car.roll.to_bits(),
        car.impact.to_bits(),
        car.longitudinal_acceleration.to_bits(),
        car.reverse_hold.to_bits(),
        car.road_index as u32,
        u32::from(car.grounded) | u32::from(car.offroad) << 1 | u32::from(car.reversing) << 2,
    ];
    for word in words {
        for byte in word.to_le_bytes() {
            *hash = (*hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
}

/// About 25,000 samples: the densest course the format allows, with banks,
/// bridges, tunnels and jumps.
pub(super) fn long_course() -> Track {
    let mut source = String::from("name Long\nwidth 14\n");
    for lap in 0..21 {
        let turn = if lap % 2 == 0 { "right" } else { "left" };
        source.push_str(&format!(
            "straight 300\n{turn} 120 radius 160 rise 6 bank 8\nbridge 200 rise -6\n\
             straight 120 bank 0\ntunnel 160\n{turn} 60 radius 200\nramp 30 rise 3\n\
             gap 16 rise -3\nstraight 120\n{turn} 180 radius 220 bank -6\nstraight 100 bank 0\n"
        ));
    }
    Track::parse(&source).unwrap()
}

#[test]
#[ignore = "benchmark; run in --release with --ignored --nocapture"]
fn autodrive_benchmark() {
    let mut tracks: Vec<(String, Track)> = crate::courses::BUNDLED_PATHS
        .iter()
        .map(|path| (path.to_string(), Track::load(path).unwrap()))
        .collect();
    tracks.push(("long (generated)".into(), long_course()));
    let mut total_hash = 0xcbf29ce484222325u64;
    let mut total_time = std::time::Duration::ZERO;
    for (name, track) in &tracks {
        let start = std::time::Instant::now();
        let mut hash = 0xcbf29ce484222325u64;
        for (run, noisy) in [(0_u64, false), (1, true), (2, true), (3, true)] {
            let mut seed = run.wrapping_mul(0x2545f4914f6cdd1d) ^ 0x1234_5678;
            let mut car = Car::new(track);
            for step in 0..(120 * 90) {
                let control = control(&car, track, &mut seed, noisy, step);
                car.update(track, control, STEP);
                digest_car(&mut hash, &car);
                if noisy && step % 3600 == 3599 {
                    // Restart mid-course so every run covers more track.
                    car.reset(track, (seed % 997) as f32 / 997.0 * track.length);
                }
            }
        }
        let elapsed = start.elapsed();
        total_time += elapsed;
        total_hash = (total_hash ^ hash).wrapping_mul(0x100000001b3);
        println!(
            "{name:>28}: {:>6} samples, {:>9.1} ms, digest {hash:016x}",
            track.samples.len(),
            elapsed.as_secs_f64() * 1000.0
        );
    }
    println!(
        "{:>28}: {:>6} {:>17.1} ms, digest {total_hash:016x}",
        "total",
        "",
        total_time.as_secs_f64() * 1000.0
    );
}
