//! Deterministic, fixed-step vehicle dynamics and elevation-aware road contact.
//!
//! The chassis reference is 0.48 m above its contact patch. Horizontal tire
//! forces use a two-axle bicycle model; each axle shares its grip between
//! acceleration/braking and cornering. Road contact is separate so a ramp can
//! launch the car without an invisible force pulling it back onto the track.
//!
//! [`Car::update`] runs one fixed step in phases, each in its own module:
//!
//! - [`dynamics`]: driver inputs, tire forces, drive, brakes, load transfer
//!   and airborne motion;
//! - [`contact`]: road queries (the segment, deck or shoulder beneath a
//!   position), route walks between road points, and the decision whether
//!   the car stays on, lands on or leaves a surface;
//! - [`landing`]: swept airborne contact with finite decks and shoulders,
//!   and the velocity response at landings and creases;
//! - [`collision`]: guardrails and the tunnel arch, including sliding along
//!   them for the rest of a step.

mod collision;
mod contact;
mod dynamics;
mod landing;

use macroquad::prelude::*;

use crate::track::{RoadKind, SHOULDER_WIDTH, Track};
use contact::{RoadPoint, RoadSweep, Surface, contact_surface, nearest_road, road_surface};

const GRAVITY: f32 = 9.81;
const RIDE_HEIGHT: f32 = 0.48;
// Keep shared segment edges watertight despite rounded track coordinates.
const SEGMENT_TOLERANCE: f32 = 0.01;
const CONTACT_TOLERANCE: f32 = 0.30;

#[derive(Default, Clone, Copy, Debug)]
pub struct Control {
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
    pub handbrake: bool,
}

#[derive(Clone, Debug)]
pub struct Car {
    pub position: Vec3,
    pub velocity: Vec3,
    pub heading: f32,
    pub yaw_rate: f32,
    /// Actual front wheel angle in radians, after input smoothing.
    pub steering: f32,
    pub grounded: bool,
    /// Smoothed tire-slip intensity, from zero to one.
    pub slip: f32,
    pub road_index: usize,
    /// Distance along the nearest road segment, wrapped for a closed track.
    pub distance: f32,
    pub offroad: bool,
    /// Nose-up angle in radians.
    pub pitch: f32,
    /// Positive roll follows positive bank: the right side is raised.
    pub roll: f32,
    /// Smoothed pedal values, also useful for gauges and engine sound.
    pub throttle: f32,
    pub brake: f32,
    /// A short visual feedback envelope for landings and barrier impacts.
    pub impact: f32,
    /// Hold the brake at rest to select reverse; throttle returns to forward.
    pub reversing: bool,
    longitudinal_acceleration: f32,
    reverse_hold: f32,
}

/// Values fixed for the whole of one [`Car::update`] step.
#[derive(Clone, Copy)]
struct Step<'a> {
    track: &'a Track,
    dt: f32,
    ground_height: f32,
    /// Horizontal speed at the start of the step.
    speed: f32,
    /// Chassis position at the start of the step.
    before: Vec3,
    /// The road beneath `before`, and the surface the car stood on there.
    old_road: Option<RoadPoint>,
    old_surface: Surface,
    was_grounded: bool,
}

impl Car {
    pub fn new(track: &Track) -> Self {
        let mut car = Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            heading: 0.0,
            yaw_rate: 0.0,
            steering: 0.0,
            grounded: true,
            slip: 0.0,
            road_index: 0,
            distance: 0.0,
            offroad: false,
            pitch: 0.0,
            roll: 0.0,
            throttle: 0.0,
            brake: 0.0,
            impact: 0.0,
            reversing: false,
            longitudinal_acceleration: 0.0,
            reverse_hold: 0.0,
        };
        car.reset_to_grid(track);
        car
    }

    /// Use the same starting grid for the first run, restarts, and track changes.
    pub fn reset_to_grid(&mut self, track: &Track) {
        let distance = if track.closed {
            0.0
        } else {
            5.0_f32.min(track.length * 0.1)
        };
        self.reset(track, distance);
    }

    pub fn reset(&mut self, track: &Track, distance: f32) {
        if track.samples.is_empty() {
            self.position = vec3(0.0, track.ground_height() + RIDE_HEIGHT, 0.0);
            self.velocity = Vec3::ZERO;
            return;
        }
        let distance = if track.closed && track.length > 0.0 {
            distance.rem_euclid(track.length)
        } else {
            distance.clamp(0.0, track.length.max(0.0))
        };
        let mut sample = track.sample_at(distance);
        // A reset in a jump returns to its last solid approach.
        if matches!(sample.kind, RoadKind::Gap)
            && let Some(solid) = track
                .samples
                .iter()
                .rev()
                .find(|s| s.distance <= distance && !matches!(s.kind, RoadKind::Gap))
        {
            sample = *solid;
        }
        self.position = sample.pos + Vec3::Y * RIDE_HEIGHT;
        self.velocity = Vec3::ZERO;
        self.heading = sample.forward.x.atan2(sample.forward.z);
        self.yaw_rate = 0.0;
        self.steering = 0.0;
        self.grounded = !matches!(sample.kind, RoadKind::Gap);
        self.slip = 0.0;
        self.distance = sample.distance;
        self.road_index = track
            .samples
            .partition_point(|s| s.distance <= sample.distance)
            .saturating_sub(1)
            .min(track.samples.len().saturating_sub(2));
        self.offroad = false;
        self.pitch = sample.forward.y.clamp(-1.0, 1.0).asin();
        self.roll = sample.bank;
        self.throttle = 0.0;
        self.brake = 0.0;
        self.impact = 0.0;
        self.reversing = false;
        self.longitudinal_acceleration = 0.0;
        self.reverse_hold = 0.0;
    }

    /// Road speed: along the surface while grounded (the velocity is tangent
    /// to it), and horizontal in the air so a fall does not read as speed.
    pub fn speed_kmh(&self) -> f32 {
        if self.grounded {
            self.velocity.length() * 3.6
        } else {
            horizontal(self.velocity).length() * 3.6
        }
    }

    /// Advance one small fixed step (the game uses 1/120 s).
    pub fn update(&mut self, track: &Track, control: Control, dt: f32) {
        if !dt.is_finite() || dt <= 0.0 {
            return;
        }
        // Bound public calls as well as the application's fixed-step loop.
        let dt = dt.min(1.0 / 30.0);
        let ground_height = contact::ground_height(track);
        self.impact = (self.impact - dt * 2.8).max(0.0);
        let speed = horizontal(self.velocity).length();
        self.apply_controls(control, speed, dt);

        let before = self.position;
        let old_road = nearest_road(
            track,
            before,
            self.distance,
            before.y - RIDE_HEIGHT + 0.25,
            ground_height,
            None,
        );
        let old_surface = contact_surface(old_road, before.y - RIDE_HEIGHT + 0.25, ground_height);
        let was_grounded =
            self.grounded && (before.y - RIDE_HEIGHT - old_surface.height).abs() < 0.55;
        self.grounded = was_grounded;
        let step = Step {
            track,
            dt,
            ground_height,
            speed,
            before,
            old_road,
            old_surface,
            was_grounded,
        };

        self.integrate(&step, control.handbrake);

        let mut sweep = RoadSweep {
            before,
            support: old_road.filter(|road| {
                road_surface(*road, ground_height)
                    .is_some_and(|surface| before.y - RIDE_HEIGHT >= surface.height - 0.001)
            }),
        };
        let mut max_surface_height =
            (before.y - RIDE_HEIGHT).max(self.position.y - RIDE_HEIGHT) + CONTACT_TOLERANCE;
        let mut new_road = nearest_road(
            track,
            self.position,
            self.distance,
            max_surface_height,
            ground_height,
            Some(sweep),
        );
        if !was_grounded {
            self.land_before_exit(&step, &mut sweep, &mut max_surface_height, &mut new_road);
        }
        self.follow_progress(new_road);
        self.resolve_guardrail(
            track,
            new_road,
            old_road,
            before,
            max_surface_height,
            ground_height,
        );
        // Do not attach to an overpass that is above the car. The preceding
        // position provides a swept, one-sided contact test at landings.
        let new_road = nearest_road(
            track,
            self.position,
            self.distance,
            max_surface_height,
            ground_height,
            Some(sweep),
        );
        self.follow_progress(new_road);
        self.settle_on_surface(&step, new_road, max_surface_height);
        self.resolve_tunnel_roof(&step, max_surface_height);
    }

    /// Timing progress follows the road found beneath the car, if any.
    fn follow_progress(&mut self, road: Option<RoadPoint>) {
        if let Some(road) = road {
            self.road_index = road.index;
            self.distance = road.sample.distance;
        }
    }
}

fn horizontal(v: Vec3) -> Vec3 {
    vec3(v.x, 0.0, v.z)
}

fn approach(current: f32, target: f32, max_change: f32) -> f32 {
    current + (target - current).clamp(-max_change, max_change)
}

fn wrap_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

#[cfg(test)]
mod benchmark;

/// Helpers shared by the tests of every vehicle module.
#[cfg(test)]
mod test_util {
    use super::*;

    pub(super) const STEP: f32 = 1.0 / 120.0;

    pub(super) fn advance(car: &mut Car, track: &Track, control: Control, seconds: f32) {
        for _ in 0..(seconds / STEP).round() as usize {
            car.update(track, control, STEP);
            assert!(car.position.is_finite());
            assert!(car.velocity.is_finite());
            assert!(car.heading.is_finite());
        }
    }

    pub(super) fn wide_straight() -> Track {
        Track::parse("width 40\nstraight 1000").unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_util::*;

    #[test]
    fn grid_restart_matches_initial_run_on_short_sprints_and_circuits() {
        for (source, expected_distance) in [
            ("straight 20", 2.0),
            ("straight 40", 4.0),
            ("straight 100", 5.0),
            (include_str!("../tracks/club.track"), 0.0),
        ] {
            let track = Track::parse(source).unwrap();
            let mut initial = Car::new(&track);
            let mut restarted = initial.clone();
            advance(
                &mut restarted,
                &track,
                Control {
                    throttle: 1.0,
                    steer: 0.7,
                    ..Control::default()
                },
                1.0,
            );
            restarted.reset_to_grid(&track);
            assert_eq!(initial.distance, expected_distance);
            assert_eq!(restarted.distance, expected_distance);
            for _ in 0..120 {
                let control = Control {
                    throttle: 1.0,
                    ..Control::default()
                };
                initial.update(&track, control, STEP);
                restarted.update(&track, control, STEP);
                assert_eq!(restarted.position, initial.position);
                assert_eq!(restarted.velocity, initial.velocity);
                assert_eq!(restarted.distance, initial.distance);
            }
        }
    }

    #[test]
    fn closed_course_starts_at_the_line_for_a_full_first_lap() {
        let track = Track::parse(include_str!("../tracks/club.track")).unwrap();
        let car = Car::new(&track);
        assert_eq!(car.distance, 0.0);
        assert!(
            car.position
                .distance(track.samples[0].pos + Vec3::Y * RIDE_HEIGHT)
                < 0.001
        );
    }

    #[test]
    fn speed_reading_ignores_a_fall_but_follows_a_slope() {
        let track = wide_straight();
        let mut car = Car::new(&track);
        car.velocity = vec3(0.0, -30.0, 10.0);
        car.grounded = false;
        assert!((car.speed_kmh() - 36.0).abs() < 0.001);
        car.velocity = vec3(0.0, 3.0, 4.0);
        car.grounded = true;
        assert!((car.speed_kmh() - 18.0).abs() < 0.001);
    }
}
