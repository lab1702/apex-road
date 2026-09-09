//! Deterministic, fixed-step vehicle dynamics and elevation-aware road contact.
//!
//! The chassis reference is 0.48 m above its contact patch. Horizontal tire
//! forces use a two-axle bicycle model; each axle shares its grip between
//! acceleration/braking and cornering. Road contact is separate so a ramp can
//! launch the car without an invisible force pulling it back onto the track.

use macroquad::prelude::*;

use crate::track::{RoadKind, RoadSample, Track};

const GRAVITY: f32 = 9.81;
const RIDE_HEIGHT: f32 = 0.48;
const SHOULDER_WIDTH: f32 = 12.0;
const FRONT_ARM: f32 = 1.36;
const REAR_ARM: f32 = 1.29;
const INERTIA_PER_MASS: f32 = 2.45;
const CAR_HALF_WIDTH: f32 = 0.85;
// Keep shared segment edges watertight despite rounded track coordinates.
const SEGMENT_TOLERANCE: f32 = 0.01;
const ROAD_FRICTION: f32 = 1.18;
const FRONT_CORNERING_STIFFNESS: f32 = 56.0;
const REAR_CORNERING_STIFFNESS: f32 = 62.0;
// Keep a small cornering reserve under full keyboard pedals. Longitudinal
// and lateral forces still share the same friction circle.
const MAX_LONGITUDINAL_GRIP: f32 = 0.95;

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

#[derive(Clone, Copy)]
struct RoadPoint {
    sample: RoadSample,
    index: usize,
    lateral: f32,
    plane_height: f32,
    /// The motion crossed a rising surface from above during this step.
    swept_contact: bool,
}

#[derive(Clone, Copy)]
struct RoadSweep {
    before: Vec3,
    /// A deck actually supporting the car before the move, never an overpass.
    support: Option<RoadPoint>,
}

#[derive(Clone, Copy)]
struct Surface {
    height: f32,
    normal: Vec3,
    offroad: bool,
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

    pub fn speed_kmh(&self) -> f32 {
        self.velocity.length() * 3.6
    }

    /// Advance one small fixed step (the game uses 1/120 s).
    pub fn update(&mut self, track: &Track, control: Control, dt: f32) {
        if !dt.is_finite() || dt <= 0.0 {
            return;
        }
        // Bound public calls as well as the application's fixed-step loop.
        let dt = dt.min(1.0 / 30.0);
        let ground_height = track.ground_height();
        self.impact = (self.impact - dt * 2.8).max(0.0);
        let speed = horizontal(self.velocity).length();
        self.throttle = approach(self.throttle, control.throttle.clamp(0.0, 1.0), dt * 3.3);
        self.brake = approach(self.brake, control.brake.clamp(0.0, 1.0), dt * 5.5);
        if control.throttle > 0.1 {
            self.reversing = false;
            self.reverse_hold = 0.0;
        } else if control.brake > 0.5 && speed < 0.35 && self.grounded {
            self.reverse_hold += dt;
            if self.reverse_hold >= 0.5 {
                self.reversing = true;
            }
        } else if !self.reversing {
            self.reverse_hold = 0.0;
        }
        // Keyboard taps remain useful at speed without removing the ability
        // to ask more of the tires than they can deliver.
        let max_steer = 0.58 / (1.0 + speed * 0.085);
        let desired_steer = control.steer.clamp(-1.0, 1.0) * max_steer;
        let steering_rate = if desired_steer.abs() < self.steering.abs() {
            1.9
        } else {
            1.15
        };
        self.steering = approach(self.steering, desired_steer, steering_rate * dt);

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

        let (sin_heading, cos_heading) = self.heading.sin_cos();
        let forward = vec3(sin_heading, 0.0, cos_heading);
        let right = vec3(cos_heading, 0.0, -sin_heading);

        if was_grounded {
            self.ground_dynamics(forward, right, old_surface, control.handbrake, dt);
            self.velocity.y = surface_vertical_speed(self.velocity, old_surface.normal);
        } else {
            self.velocity.y -= GRAVITY * dt;
            let air_drag = horizontal(self.velocity) * (0.0015 * speed * dt);
            self.velocity -= air_drag;
            self.yaw_rate *= (-0.30 * dt).exp();
            self.slip += (0.0 - self.slip) * (1.0 - (-3.0 * dt).exp());
        }
        self.heading = wrap_angle(self.heading + self.yaw_rate * dt);
        self.position += self.velocity * dt;

        let sweep = RoadSweep {
            before,
            support: old_road.filter(|_| was_grounded && !old_surface.offroad),
        };
        let max_surface_height = (before.y - RIDE_HEIGHT).max(self.position.y - RIDE_HEIGHT) + 0.30;
        let new_road = nearest_road(
            track,
            self.position,
            self.distance,
            max_surface_height,
            ground_height,
            Some(sweep),
        );
        if let Some(road) = new_road {
            self.road_index = road.index;
            self.distance = road.sample.distance;
            self.resolve_guardrail(road, before);
        }
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
        let surface = contact_surface(new_road, max_surface_height, ground_height);
        let required_vertical = surface_vertical_speed(self.velocity, surface.normal);
        let current_bottom = self.position.y - RIDE_HEIGHT;
        let previous_clearance =
            before.y - RIDE_HEIGHT - plane_height_at(surface, self.position, before);
        // A proven sweep has penetrated a rising surface. Its recovery takes
        // precedence over the cross-section velocity test, which does not
        // include longitudinal changes in bank. Falling-away surfaces still
        // use the ordinary clearance/velocity check and can launch the car.
        let on_same_surface = was_grounded
            && (new_road.is_some_and(|road| road.swept_contact)
                || ((current_bottom - surface.height).abs() < 0.30
                    && self.velocity.y - required_vertical <= GRAVITY * dt + 0.025));
        // A step can leave one supporting surface and strike another: a steep
        // downhill shoulder can cross the terrain before the car is airborne.
        // Keep the one-sided sweep valid for those transitions as well.
        let landed = !on_same_surface
            && current_bottom <= surface.height
            && previous_clearance >= -0.08
            && self.velocity.y <= required_vertical + 0.3;

        self.grounded = on_same_surface || landed;
        self.offroad = surface.offroad;
        if self.grounded {
            if landed {
                let vertical_impact = (self.velocity.y - required_vertical).abs();
                self.impact = (vertical_impact / 11.0).clamp(0.0, 1.0);
                // A firm landing scrubs speed, but does not reset steering or
                // horizontal momentum and therefore still rewards alignment.
                let landing_loss = (vertical_impact - 2.5).max(0.0) * 0.014;
                self.velocity.x *= 1.0 - landing_loss.min(0.24);
                self.velocity.z *= 1.0 - landing_loss.min(0.24);
            }
            self.position.y = surface.height + RIDE_HEIGHT;
            self.velocity.y = surface_vertical_speed(self.velocity, surface.normal);
            let heading_forward = vec3(self.heading.sin(), 0.0, self.heading.cos());
            let heading_right = vec3(self.heading.cos(), 0.0, -self.heading.sin());
            let pitch = (-surface.normal.dot(heading_forward) / surface.normal.y.max(0.2)).atan();
            let bank = (-surface.normal.dot(heading_right) / surface.normal.y.max(0.2)).atan();
            let lean = self.yaw_rate * speed * 0.0025;
            self.pitch += (pitch - self.pitch) * (1.0 - (-12.0 * dt).exp());
            self.roll +=
                (bank + lean.clamp(-0.055, 0.055) - self.roll) * (1.0 - (-10.0 * dt).exp());
        } else {
            let flight_pitch = self.velocity.y.atan2(speed.max(8.0));
            self.pitch += (flight_pitch - self.pitch) * (1.0 - (-1.5 * dt).exp());
            self.roll *= (-0.35 * dt).exp();
        }

        // The tunnel roof is also a physical boundary; falling onto a roof
        // from outside is intentionally not treated as driving on its deck.
        if let Some(road) = new_road
            && let Some(roof_height) = tunnel_roof_height(road.sample, self.position)
            && let Some(previous_roof) = old_road
                .and_then(|road| tunnel_roof_height(road.sample, before))
                .or_else(|| tunnel_roof_height(road.sample, before))
            && before.y + 0.7 <= previous_roof + 0.001
            && self.position.y + 0.7 > roof_height
        {
            self.position.y = roof_height - 0.7;
            self.velocity.y = self.velocity.y.min(0.0);
            self.impact = 0.6;
        }
    }

    fn ground_dynamics(
        &mut self,
        forward: Vec3,
        right: Vec3,
        surface: Surface,
        handbrake: bool,
        dt: f32,
    ) {
        let longitudinal = self.velocity.dot(forward);
        let lateral = self.velocity.dot(right);
        let speed = horizontal(self.velocity).length();
        let friction = if surface.offroad { 0.61 } else { ROAD_FRICTION };
        let normal_load = GRAVITY * surface.normal.y.max(0.45);
        let transfer = self.longitudinal_acceleration * 0.54 / (FRONT_ARM + REAR_ARM);
        let front_load =
            (normal_load * 0.49 - transfer).clamp(normal_load * 0.24, normal_load * 0.78);
        let rear_load = normal_load - front_load;
        let front_grip = front_load * friction;
        let rear_grip = rear_load * friction;

        let engine = if self.reversing {
            -self.brake * 3.5 * (1.0 - (-longitudinal / 12.0).clamp(0.0, 1.0))
        } else {
            self.throttle * 7.0 / (1.0 + longitudinal.max(0.0) * 0.018)
        };
        // Prevent braking from becoming an unsolicited reverse gear at rest.
        let braking = (if self.reversing {
            0.0
        } else {
            self.brake * 13.4
        })
        .min(longitudinal.abs() / dt);
        let brake_sign = longitudinal.signum();
        let front_demand = -braking * 0.65 * brake_sign;
        let rear_demand = engine
            - braking * 0.35 * brake_sign
            - if handbrake {
                (longitudinal.abs() / dt).min(8.0) * brake_sign
            } else {
                0.0
            };
        let front_long = front_demand.clamp(
            -front_grip * MAX_LONGITUDINAL_GRIP,
            front_grip * MAX_LONGITUDINAL_GRIP,
        );
        let rear_long = rear_demand.clamp(
            -rear_grip * MAX_LONGITUDINAL_GRIP,
            rear_grip * MAX_LONGITUDINAL_GRIP,
        );
        let front_lateral_grip = (front_grip * front_grip - front_long * front_long)
            .max(0.0)
            .sqrt();
        let rear_lateral_grip = (rear_grip * rear_grip - rear_long * rear_long)
            .max(0.0)
            .sqrt()
            * if handbrake { 0.35 } else { 1.0 };

        let denominator = longitudinal.abs().max(2.5);
        let rolling = (longitudinal.abs() / 2.5).min(1.0) * longitudinal.signum();
        let front_angle =
            ((lateral + self.yaw_rate * FRONT_ARM) / denominator).atan() - self.steering * rolling;
        let rear_angle = ((lateral - self.yaw_rate * REAR_ARM) / denominator).atan();
        let front_lateral = tire_force(front_angle, FRONT_CORNERING_STIFFNESS, front_lateral_grip);
        let rear_lateral = tire_force(rear_angle, REAR_CORNERING_STIFFNESS, rear_lateral_grip);
        let (sin_steer, cos_steer) = self.steering.sin_cos();
        let front_side = front_lateral * cos_steer + front_long * sin_steer;
        let total_long = front_long * cos_steer - front_lateral * sin_steer + rear_long;
        let total_side = front_side + rear_lateral;

        let drag = longitudinal * (0.0023 * speed + if surface.offroad { 0.095 } else { 0.007 });
        let rolling_resistance = longitudinal.signum()
            * (if surface.offroad { 0.6 } else { 0.16 })
            * (longitudinal.abs() / 0.6).min(1.0);
        let gravity = vec3(0.0, -GRAVITY, 0.0);
        let slope_gravity = gravity - surface.normal * gravity.dot(surface.normal);
        let acceleration = forward * (total_long - drag - rolling_resistance)
            + right * total_side
            + horizontal(slope_gravity);
        self.velocity += acceleration * dt;
        self.longitudinal_acceleration += (acceleration.dot(forward)
            - self.longitudinal_acceleration)
            * (1.0 - (-9.0 * dt).exp());

        let yaw_acceleration =
            (front_side * FRONT_ARM - rear_lateral * REAR_ARM) / INERTIA_PER_MASS;
        // Small chassis damping makes a keyboard correction recoverable;
        // tire saturation and countersteering still determine the actual arc.
        self.yaw_rate += (yaw_acceleration - self.yaw_rate * 0.46) * dt;
        self.yaw_rate = self.yaw_rate.clamp(-2.8, 2.8);
        if speed < 0.15 && self.throttle < 0.02 && self.brake > 0.2 && !self.reversing {
            self.velocity.x = 0.0;
            self.velocity.z = 0.0;
            self.yaw_rate *= (-14.0 * dt).exp();
        }

        let slip_angle = front_angle.abs().max(rear_angle.abs());
        let combined_front =
            ((front_angle * FRONT_CORNERING_STIFFNESS).powi(2) + front_demand.powi(2)).sqrt()
                / front_grip.max(0.1);
        let combined_rear = ((rear_angle * REAR_CORNERING_STIFFNESS).powi(2) + rear_demand.powi(2))
            .sqrt()
            / rear_grip.max(0.1);
        let sliding = ((slip_angle - 0.045) / 0.22)
            .max((combined_front.max(combined_rear) - 0.92) * 0.7)
            .clamp(0.0, 1.0);
        let sliding = sliding * (speed / 3.0).min(1.0);
        self.slip += (sliding - self.slip) * (1.0 - (-8.0 * dt).exp());
    }

    fn resolve_guardrail(&mut self, road: RoadPoint, before: Vec3) {
        if !matches!(road.sample.kind, RoadKind::Bridge | RoadKind::Tunnel) {
            return;
        }
        let clearance = self.position.y - RIDE_HEIGHT - road.plane_height;
        // Ground contact may belong to terrain beneath an overpass; only a
        // car at the deck's height can hit its guardrails.
        if !(-0.45..0.85).contains(&clearance) {
            return;
        }
        let limit = (road.sample.width * 0.5 - CAR_HALF_WIDTH).max(0.5);
        if road.lateral.abs() <= limit {
            return;
        }
        // Cars approaching the side from below/outside should not teleport
        // through an entire bridge to its inside edge.
        let old_lateral = (before - Vec3::Y * RIDE_HEIGHT - road.sample.pos).dot(road.sample.right);
        if old_lateral.abs() > road.sample.width * 0.5 + 1.0 {
            return;
        }
        let side = road.lateral.signum();
        // The barrier follows the banked road frame. Both its correction and
        // impulse must stay in the road plane, or an uphill impact launches
        // the car by retaining its old upward velocity after the rebound.
        let normal = road.sample.up;
        let lateral_axis = (road.sample.right - normal * road.sample.right.dot(normal)).normalize();
        self.position +=
            lateral_axis * ((side * limit - road.lateral) / lateral_axis.dot(road.sample.right));
        let outward_speed = self.velocity.dot(lateral_axis) * side;
        if outward_speed > 0.0 {
            self.velocity -= lateral_axis * side * outward_speed * 1.12;
            let scrub = (outward_speed * 0.012).min(0.16);
            let tangent_velocity = self.velocity - normal * self.velocity.dot(normal);
            self.velocity -= tangent_velocity * scrub;
            self.yaw_rate *= 0.68;
            self.impact = self.impact.max((outward_speed / 10.0).clamp(0.08, 1.0));
        }
    }
}

fn tire_force(slip_angle: f32, stiffness: f32, available_grip: f32) -> f32 {
    if available_grip <= 0.001 {
        return 0.0;
    }
    let force = -available_grip * (slip_angle * stiffness / available_grip).tanh();
    // A sliding tire retains most of its force, but loses the sharp response
    // around peak grip. Recovery follows as its slip angle comes back down.
    let slide_loss = ((slip_angle.abs() - 0.18) * 0.60).clamp(0.0, 0.18);
    force * (1.0 - slide_loss)
}

fn nearest_road(
    track: &Track,
    position: Vec3,
    previous_distance: f32,
    max_surface_height: f32,
    ground_height: f32,
    sweep: Option<RoadSweep>,
) -> Option<RoadPoint> {
    let mut best: Option<(f32, RoadPoint)> = None;
    for (index, pair) in track.samples.windows(2).enumerate() {
        let a = pair[0];
        let b = pair[1];
        let chord = horizontal(b.pos - a.pos);
        let chord_length_squared = chord.length_squared();
        if chord_length_squared < 0.00001 {
            continue;
        }
        // Bound every segment by its rendered cross-sections, including the
        // edges of gaps. Chord projections alone leave holes on the outside
        // of bends; the banked right vectors also account for sloped joins.
        let start_normal = horizontal(a.right).cross(Vec3::Y).normalize();
        let end_normal = horizontal(b.right).cross(Vec3::Y).normalize();
        let start_offset = horizontal(position - a.pos).dot(start_normal);
        let end_offset = horizontal(position - b.pos).dot(end_normal);
        if start_offset < -SEGMENT_TOLERANCE || end_offset > SEGMENT_TOLERANCE {
            continue;
        }
        let projection = horizontal(position - a.pos).dot(chord) / chord_length_squared;
        let along = if start_offset.abs() <= SEGMENT_TOLERANCE {
            0.0
        } else if end_offset.abs() <= SEGMENT_TOLERANCE {
            1.0
        } else {
            projection.clamp(0.0, 1.0)
        };
        let pos = a.pos.lerp(b.pos, along);
        let forward = a.forward.lerp(b.forward, along).normalize_or_zero();
        let right = a.right.lerp(b.right, along).normalize_or_zero();
        let up = a.up.lerp(b.up, along).normalize_or_zero();
        let width = a.width + (b.width - a.width) * along;
        let distance = a.distance + (b.distance - a.distance) * along;
        let sample = RoadSample {
            pos,
            forward,
            right,
            up,
            width,
            bank: a.bank + (b.bank - a.bank) * along,
            distance,
            kind: if end_offset >= -SEGMENT_TOLERANCE {
                b.kind
            } else {
                a.kind
            },
        };
        let plane_height = pos.y - horizontal(position - pos).dot(up) / up.y.max(0.15);
        let contact = vec3(position.x, plane_height, position.z);
        let lateral = (contact - pos).dot(right);
        let delta = horizontal(position - pos);
        let vertical = position.y - RIDE_HEIGHT - plane_height;
        let mut progress_delta = (distance - previous_distance).abs();
        if track.closed && track.length > 0.0 {
            progress_delta = progress_delta.min((track.length - progress_delta).abs());
        }
        // Height separates crossing bridges. A small continuity preference
        // disambiguates joins, parallel lanes, and paths through a jump.
        let score = delta.length_squared()
            + vertical * vertical * 3.0
            + ((progress_delta - 12.0).max(0.0) * 0.025)
                .powi(2)
                .min(180.0);
        let mut point = RoadPoint {
            sample,
            index,
            lateral,
            plane_height,
            swept_contact: false,
        };
        // An overhead deck cannot become the route merely because the car
        // rises closer to it during a jump. Use the same swept height bound as
        // contact, including the actual shoulder height for terrain re-entry.
        // Gap centerlines remain eligible: they only guide airborne progress.
        let surface = road_surface(point, ground_height);
        let surface_height = surface.map_or(plane_height, |surface| surface.height);
        if !matches!(sample.kind, RoadKind::Gap) && surface_height > max_surface_height {
            // A steep surface can rise past an airborne car in one step. Its
            // plane provides a one-sided sweep for both decks and shoulders.
            // Changing bank can also lift a deck beyond the height allowance,
            // but its cross-section normal omits that longitudinal rise. That
            // recovery needs a supported, connected chain of solid sections.
            point.swept_contact = surface.is_some_and(|surface| {
                position.y - RIDE_HEIGHT <= surface.height
                    && sweep.is_some_and(|sweep| {
                        (sweep.before.y - RIDE_HEIGHT
                            >= plane_height_at(surface, position, sweep.before) - 0.08
                            && (surface.offroad
                                || crosses_finite_deck(
                                    track,
                                    point,
                                    surface,
                                    sweep.before,
                                    position,
                                    ground_height,
                                )))
                            || (!surface.offroad
                                && sweep.support.is_some_and(|support| {
                                    crosses_connected_deck(
                                        track,
                                        support,
                                        point,
                                        sweep.before,
                                        position,
                                    )
                                }))
                    })
            });
            if !point.swept_contact {
                continue;
            }
        }
        if best
            .as_ref()
            .is_none_or(|(best_score, _)| score < *best_score)
        {
            best = Some((score, point));
        }
    }
    best.map(|(_, point)| point)
}

/// Crossing an extended deck plane outside the road is not a landing. Locate
/// the actual contact point and require a solid route from there to this step's
/// endpoint, including any intervening sample boundaries.
fn crosses_finite_deck(
    track: &Track,
    to: RoadPoint,
    surface: Surface,
    before: Vec3,
    after: Vec3,
    ground_height: f32,
) -> bool {
    let start_clearance = before.y - RIDE_HEIGHT - plane_height_at(surface, after, before);
    let end_clearance = after.y - RIDE_HEIGHT - surface.height;
    let fraction = (start_clearance / (start_clearance - end_clearance)).clamp(0.0, 1.0);
    let crossing = before.lerp(after, fraction);
    nearest_road(
        track,
        crossing,
        to.sample.distance,
        crossing.y - RIDE_HEIGHT + 0.08,
        ground_height,
        None,
    )
    .is_some_and(|from| {
        road_surface(from, ground_height).is_some_and(|surface| {
            !surface.offroad
                && (crossing.y - RIDE_HEIGHT - surface.height).abs() <= 0.08
                && crosses_connected_deck(track, from, to, crossing, after)
        })
    })
}

/// Check the actual swept path through intervening road cross-sections, rather
/// than treating proximity in world space as evidence of a connected deck.
fn crosses_connected_deck(
    track: &Track,
    from: RoadPoint,
    to: RoadPoint,
    before: Vec3,
    after: Vec3,
) -> bool {
    let mut delta = to.sample.distance - from.sample.distance;
    if track.closed {
        if delta > track.length * 0.5 {
            delta -= track.length;
        } else if delta < -track.length * 0.5 {
            delta += track.length;
        }
    }
    let forward = delta >= 0.0;
    let direction = if forward { 1.0 } else { -1.0 };
    let segments = track.samples.len() - 1;
    let mut index = from.index;
    while index != to.index {
        let boundary = track.samples[index + usize::from(forward)];
        // At a shared endpoint, the preceding segment can have zero length
        // in this sweep (for example immediately after landing over a gap).
        let starts_at_boundary = index == from.index
            && (from.sample.distance - boundary.distance).abs() <= SEGMENT_TOLERANCE;
        if track.samples[index].kind == RoadKind::Gap && !starts_at_boundary {
            return false;
        }
        let normal = horizontal(boundary.right).cross(Vec3::Y).normalize();
        let start = horizontal(before - boundary.pos).dot(normal);
        let end = horizontal(after - boundary.pos).dot(normal);
        if direction * start > SEGMENT_TOLERANCE
            || direction * end < -SEGMENT_TOLERANCE
            || direction * (end - start) <= 0.0
        {
            return false;
        }
        let crossing = before.lerp(after, (start / (start - end)).clamp(0.0, 1.0));
        let right = horizontal(boundary.right);
        let lateral = horizontal(crossing - boundary.pos).dot(right) / right.length_squared();
        if lateral.abs() > boundary.width * 0.5 + SEGMENT_TOLERANCE {
            return false;
        }
        index = if forward {
            (index + 1) % segments
        } else {
            (index + segments - 1) % segments
        };
    }
    // A sweep may not recover onto a gap, including its exact starting edge.
    to.sample.kind != RoadKind::Gap
}

fn contact_surface(
    road: Option<RoadPoint>,
    max_surface_height: f32,
    ground_height: f32,
) -> Surface {
    let ground = Surface {
        height: ground_height,
        normal: Vec3::Y,
        offroad: true,
    };
    road.and_then(|road| {
        road_surface(road, ground_height)
            .filter(|surface| surface.height <= max_surface_height || road.swept_contact)
    })
    .unwrap_or(ground)
}

fn road_surface(road: RoadPoint, ground_height: f32) -> Option<Surface> {
    if matches!(road.sample.kind, RoadKind::Gap) {
        return None;
    }
    let half_width = road.sample.width * 0.5;
    let surface = if road.lateral.abs() <= half_width {
        Surface {
            height: road.plane_height,
            normal: road.sample.up,
            offroad: false,
        }
    } else if matches!(road.sample.kind, RoadKind::Road | RoadKind::Ramp)
        && road.lateral.abs() < half_width + SHOULDER_WIDTH
    {
        let side = road.lateral.signum();
        let edge = road.sample.pos + road.sample.right * half_width * side;
        let fraction = ((road.lateral.abs() - half_width) / SHOULDER_WIDTH).clamp(0.0, 1.0);
        // Banking shortens and can skew the shoulder's horizontal span. Use
        // its actual tangents so the normal agrees with the surface height.
        let across = horizontal(road.sample.right) * SHOULDER_WIDTH
            + Vec3::Y * (ground_height - edge.y) * side;
        let along =
            horizontal(road.sample.forward) + Vec3::Y * road.sample.forward.y * (1.0 - fraction);
        let normal = along.cross(across).normalize();
        Surface {
            height: edge.y + (ground_height - edge.y) * fraction,
            normal,
            offroad: true,
        }
    } else {
        return None;
    };
    Some(surface)
}

fn plane_height_at(surface: Surface, origin: Vec3, point: Vec3) -> f32 {
    surface.height - horizontal(point - origin).dot(surface.normal) / surface.normal.y.max(0.15)
}

/// Intersect a vertical line with the same ten arch panels drawn by the world.
/// Working in the road frame keeps roof clearance correct on banks and hills.
fn tunnel_roof_height(sample: RoadSample, position: Vec3) -> Option<f32> {
    if sample.kind != RoadKind::Tunnel {
        return None;
    }
    let mut height: Option<f32> = None;
    let arch = |index: usize| {
        let angle = index as f32 / 10.0 * std::f32::consts::PI;
        sample.pos
            + sample.right * angle.cos() * (sample.width * 0.5 + 0.65)
            + sample.up * (1.0 + angle.sin() * 5.2)
    };
    for index in 0..10 {
        let a = arch(index);
        let tangent = arch(index + 1) - a;
        let normal = tangent.cross(sample.forward);
        if normal.y <= 0.0001 {
            continue;
        }
        let panel_height = a.y - horizontal(position - a).dot(normal) / normal.y;
        let hit = vec3(position.x, panel_height, position.z);
        let fraction = (hit - a).dot(tangent) / tangent.length_squared();
        if (-0.0001..=1.0001).contains(&fraction) {
            height = Some(height.map_or(panel_height, |height| height.max(panel_height)));
        }
    }
    height
}

fn surface_vertical_speed(velocity: Vec3, normal: Vec3) -> f32 {
    -horizontal(velocity).dot(normal) / normal.y.max(0.15)
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
mod tests {
    use super::*;

    const STEP: f32 = 1.0 / 120.0;

    fn advance(car: &mut Car, track: &Track, control: Control, seconds: f32) {
        for _ in 0..(seconds / STEP).round() as usize {
            car.update(track, control, STEP);
            assert!(car.position.is_finite());
            assert!(car.velocity.is_finite());
            assert!(car.heading.is_finite());
        }
    }

    fn wide_straight() -> Track {
        Track::parse("width 40\nstraight 1000").unwrap()
    }

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
    fn steering_at_rest_does_not_create_motion() {
        let track = wide_straight();
        let mut car = Car::new(&track);
        let start = car.position;
        advance(
            &mut car,
            &track,
            Control {
                steer: 1.0,
                ..Control::default()
            },
            2.0,
        );
        assert!(car.position.distance(start) < 0.001);
        assert!(car.heading.abs() < 0.001);
        assert!(car.grounded);
    }

    #[test]
    fn acceleration_then_braking_stops_without_instant_reverse() {
        let track = wide_straight();
        let mut car = Car::new(&track);
        advance(
            &mut car,
            &track,
            Control {
                throttle: 1.0,
                ..Control::default()
            },
            5.0,
        );
        assert!(car.speed_kmh() > 75.0, "speed {}", car.speed_kmh());
        let before = car.speed_kmh();
        advance(
            &mut car,
            &track,
            Control {
                brake: 1.0,
                ..Control::default()
            },
            2.0,
        );
        assert!(car.speed_kmh() < before * 0.35, "speed {}", car.speed_kmh());
        assert!(car.velocity.z >= -0.02);
        assert!(!car.reversing);
    }

    #[test]
    fn holding_brake_selects_reverse_and_throttle_recovers() {
        let track = wide_straight();
        let mut car = Car::new(&track);
        car.reset(&track, 100.0);
        advance(
            &mut car,
            &track,
            Control {
                brake: 1.0,
                ..Control::default()
            },
            2.0,
        );
        assert!(car.reversing);
        assert!(car.velocity.z < -2.0);
        advance(
            &mut car,
            &track,
            Control {
                throttle: 1.0,
                ..Control::default()
            },
            2.0,
        );
        assert!(!car.reversing);
        assert!(car.velocity.z > 0.0);
    }

    #[test]
    fn turning_brakes_share_the_available_tire_grip() {
        let track = wide_straight();
        let mut coast = Car::new(&track);
        coast.velocity = vec3(0.0, 0.0, 28.0);
        let mut braking = coast.clone();
        advance(
            &mut coast,
            &track,
            Control {
                steer: 0.4,
                ..Control::default()
            },
            0.65,
        );
        advance(
            &mut braking,
            &track,
            Control {
                steer: 0.4,
                brake: 1.0,
                ..Control::default()
            },
            0.65,
        );
        assert!(coast.heading > 0.025);
        assert!(braking.speed_kmh() < coast.speed_kmh());
        assert!(
            braking.slip > coast.slip + 0.05,
            "coasting slip {}, braking slip {}",
            coast.slip,
            braking.slip
        );
        assert!(
            braking.heading < coast.heading,
            "braking should reduce initial cornering authority: coast {}, brake {}",
            coast.heading,
            braking.heading
        );
    }

    #[test]
    fn too_much_steering_produces_a_slide_instead_of_unlimited_cornering() {
        let track = wide_straight();
        let mut car = Car::new(&track);
        car.velocity = vec3(0.0, 0.0, 38.0);
        advance(
            &mut car,
            &track,
            Control {
                steer: 1.0,
                ..Control::default()
            },
            1.0,
        );
        let forward = vec3(car.heading.sin(), 0.0, car.heading.cos());
        let right = vec3(car.heading.cos(), 0.0, -car.heading.sin());
        let body_slip = (car.velocity.dot(right) / car.velocity.dot(forward))
            .atan()
            .abs();
        assert!(car.slip > 0.3, "slip {}", car.slip);
        assert!(body_slip > 0.025, "body slip {body_slip}");
        let path_heading = car.velocity.x.atan2(car.velocity.z);
        assert!(
            path_heading.abs() * 38.0 < GRAVITY * 1.2,
            "path heading {path_heading}, body yaw {}",
            car.yaw_rate
        );
    }

    #[test]
    fn full_throttle_uses_rear_grip_while_cornering() {
        let track = wide_straight();
        let mut coast = Car::new(&track);
        coast.velocity = vec3(0.0, 0.0, 22.0);
        let mut accelerating = coast.clone();
        advance(
            &mut coast,
            &track,
            Control {
                steer: 0.45,
                ..Control::default()
            },
            0.8,
        );
        advance(
            &mut accelerating,
            &track,
            Control {
                steer: 0.45,
                throttle: 1.0,
                ..Control::default()
            },
            0.8,
        );
        assert!(
            accelerating.slip > coast.slip + 0.05,
            "coast {}, accelerating {}",
            coast.slip,
            accelerating.slip
        );
        assert!(accelerating.speed_kmh() > coast.speed_kmh());
    }

    fn body_slip_degrees(car: &Car) -> f32 {
        let forward = vec3(car.heading.sin(), 0., car.heading.cos());
        let right = vec3(car.heading.cos(), 0., -car.heading.sin());
        car.velocity
            .dot(right)
            .atan2(car.velocity.dot(forward))
            .to_degrees()
    }

    #[test]
    fn moderate_powered_corner_stays_composed() {
        let track = wide_straight();
        let mut car = Car::new(&track);
        car.velocity = Vec3::Z * 22.;
        let control = Control {
            throttle: 1.,
            steer: 0.3,
            ..Control::default()
        };
        let mut peak_slip: f32 = 0.;
        for _ in 0..144 {
            car.update(&track, control, STEP);
            peak_slip = peak_slip.max(body_slip_degrees(&car).abs());
        }
        assert!(
            peak_slip < 6.,
            "moderate corner drifted {peak_slip} degrees"
        );
        assert!(car.speed_kmh() > 22. * 3.6);
        assert!(car.heading > 0.1);
    }

    #[test]
    fn lifting_after_a_moderate_corner_restores_grip() {
        let track = wide_straight();
        let mut car = Car::new(&track);
        car.velocity = Vec3::Z * 22.;
        advance(
            &mut car,
            &track,
            Control {
                throttle: 1.,
                steer: 0.3,
                ..Control::default()
            },
            1.2,
        );
        let initial_slip = body_slip_degrees(&car).abs();
        advance(&mut car, &track, Control::default(), 1.);
        let recovered_slip = body_slip_degrees(&car).abs();
        assert!(
            recovered_slip < 1.5,
            "still drifting {recovered_slip} degrees after lifting"
        );
        assert!(recovered_slip < initial_slip * 0.3);
        assert!(
            car.yaw_rate.abs() < 0.08,
            "continued rotating at {} rad/s",
            car.yaw_rate
        );
    }

    #[test]
    fn bridge_and_tunnel_guardrails_rebound_without_teleporting_or_stopping_forward_motion() {
        for kind in ["bridge", "tunnel"] {
            let track = Track::parse(&format!("start 0 8 0\n{kind} 160")).unwrap();
            let mut car = Car::new(&track);
            car.reset(&track, 20.0);
            car.position.x = 4.9;
            car.velocity = vec3(8.0, 0.0, 22.0);
            advance(&mut car, &track, Control::default(), 0.15);
            assert!(car.position.x <= 6.0 - CAR_HALF_WIDTH + 0.01);
            assert!(car.velocity.x < 0.0);
            assert!(car.velocity.z > 17.0);
            assert!(car.impact > 0.1);
            assert!((car.position.y - 8.0 - RIDE_HEIGHT).abs() < 0.03);
        }
    }

    #[test]
    fn banked_guardrail_impacts_preserve_ground_contact_on_both_sides_and_slopes() {
        for kind in ["bridge", "tunnel"] {
            for bank in [-60, -30, -14, 14, 30, 60] {
                for rise in [-20, 0, 20] {
                    for side in [-1.0, 1.0] {
                        let track = Track::parse(&format!(
                            "straight 50 bank {bank}\n{kind} 200 rise {rise}"
                        ))
                        .unwrap();
                        let mut car = Car::new(&track);
                        car.reset(&track, 80.0);
                        let sample = track.sample_at(car.distance);
                        car.position =
                            sample.pos + sample.right * (side * 5.0) + Vec3::Y * RIDE_HEIGHT;
                        car.velocity = sample.right * (side * 8.0) + sample.forward * 22.0;
                        let mut rebounded = false;
                        for _ in 0..18 {
                            car.update(&track, Control::default(), STEP);
                            assert!(
                                car.grounded && !car.offroad,
                                "lost road contact on {kind}, bank {bank}, rise {rise}, side {side}: {car:?}"
                            );
                            if car.impact > 0.0 && !rebounded {
                                let road = track.sample_at(car.distance);
                                assert!(car.velocity.dot(road.right) * side < 0.0);
                                rebounded = true;
                            }
                        }
                        assert!(rebounded);
                        assert!(car.velocity.dot(sample.forward) > 17.0);
                        assert!(car.impact > 0.1);
                    }
                }
            }
        }
    }

    #[test]
    fn car_on_terrain_passes_under_bridge_and_tunnel_guardrails() {
        for kind in ["bridge", "tunnel"] {
            let track = Track::parse(&format!("start 0 8 0\n{kind} 160")).unwrap();
            let mut car = Car::new(&track);
            car.reset(&track, 20.0);
            car.position = vec3(4.9, track.ground_height() + RIDE_HEIGHT, 20.0);
            car.velocity = vec3(8.0, 0.0, 22.0);
            advance(&mut car, &track, Control::default(), 0.25);
            assert!(car.position.x > 6.0, "blocked below {kind}: {car:?}");
            assert!(car.velocity.x > 0.0);
            assert_eq!(car.impact, 0.0);
            assert!(car.grounded && car.offroad);
            assert!((car.position.y - track.ground_height() - RIDE_HEIGHT).abs() < 0.01);
        }
    }

    #[test]
    fn moving_sideways_into_a_tunnel_arch_hits_the_roof() {
        let track = Track::parse("tunnel 100").unwrap();
        let mut car = Car::new(&track);
        car.position = vec3(4.0, 4.3, 20.0);
        car.velocity = Vec3::X * 30.0;
        car.grounded = false;
        car.update(&track, Control::default(), 1.0 / 30.0);
        let roof_height = 1.0 + 5.2 * (1.0 - (car.position.x / 6.65).powi(2)).sqrt();
        assert!(
            car.position.y + 0.7 <= roof_height + 0.001,
            "passed through tunnel roof: {car:?}"
        );
        assert!(car.impact > 0.0);
    }

    #[test]
    fn tunnel_roof_contact_follows_the_banked_arch() {
        for bank in [-60, -30, 0, 30, 60] {
            let track = Track::parse(&format!("straight 50 bank {bank}\ntunnel 100")).unwrap();
            let sample = track.sample_at(80.0);
            // This is the top vertex of the rendered arch in the road frame.
            let roof = sample.pos + sample.up * 6.2;
            let mut car = Car::new(&track);
            car.position = roof - Vec3::Y * 0.75;
            car.velocity = Vec3::Y * 20.0;
            car.grounded = false;
            car.distance = sample.distance;
            car.update(&track, Control::default(), STEP);
            assert!(
                (car.position.y + 0.7 - roof.y).abs() < 0.001,
                "wrong roof contact at bank {bank}: {car:?}, roof {roof:?}"
            );
            assert!(car.velocity.y <= 0.0 && car.impact > 0.0);
        }
    }

    #[test]
    fn tunnel_roof_query_matches_arch_vertices_on_hills() {
        for bank in [-60, -30, 0, 30, 60] {
            for rise in [-60, 60] {
                let track =
                    Track::parse(&format!("straight 50 bank {bank}\ntunnel 100 rise {rise}"))
                        .unwrap();
                let sample = track.sample_at(80.0);
                let roof = sample.pos + sample.up * 6.2;
                let height = tunnel_roof_height(sample, roof).unwrap();
                assert!(
                    (height - roof.y).abs() < 0.001,
                    "wrong arch height on bank {bank}, rise {rise}: {height}, expected {}",
                    roof.y
                );
            }
        }
    }

    #[test]
    fn falling_onto_a_tunnel_from_above_is_not_pulled_inside() {
        let track = Track::parse("tunnel 100").unwrap();
        let mut car = Car::new(&track);
        car.position = vec3(0.0, 6.0, 20.0);
        car.velocity = -Vec3::Y * 20.0;
        car.grounded = false;
        car.update(&track, Control::default(), STEP);
        assert_eq!(car.impact, 0.0);
        assert!(!car.grounded);
        assert!(car.position.y > 5.8);
    }

    #[test]
    fn open_road_does_not_extend_its_collider_past_the_finish() {
        let track = Track::parse("straight 40").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 38.0);
        car.velocity = vec3(0.0, 0.0, 15.0);
        advance(&mut car, &track, Control::default(), 0.5);
        assert!(!car.grounded);
        assert!(car.position.z > 40.0);
        assert!(car.position.y < RIDE_HEIGHT - 0.2);
    }

    #[test]
    fn falling_short_of_a_gap_lands_on_terrain_before_the_landing_road() {
        let track =
            Track::parse("start 0 12 0\nstraight 40\ngap 20 rise -12\nstraight 100").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 38.0);
        car.velocity = Vec3::Z * 3.0;
        let mut airborne = false;
        for _ in 0..600 {
            let previous_distance = car.distance;
            car.update(&track, Control::default(), STEP);
            assert!(
                (car.distance - previous_distance).abs() < 1.0,
                "progress jumped to a distant segment: {car:?}"
            );
            airborne |= !car.grounded;
            if airborne && car.grounded {
                break;
            }
        }
        assert!(airborne && car.grounded);
        assert!(car.position.z > 40.0 && car.position.z < 60.0);
        assert!(
            car.offroad,
            "landed on invisible road inside the gap: {car:?}"
        );
        assert!((car.position.y - track.ground_height() - RIDE_HEIGHT).abs() < 0.01);
    }

    #[test]
    fn curved_and_banked_gap_edges_follow_the_rendered_cross_section() {
        for source in [
            "right 45 radius 30\nright 45 radius 30 kind gap\nright 45 radius 30",
            "straight 30\nramp 30 rise 9 bank 35\nright 45 radius 30 rise -9 bank -20 kind gap\nright 45 radius 30 bank 0",
        ] {
            let track = Track::parse(source).unwrap();
            for pair in track
                .samples
                .windows(2)
                .filter(|pair| (pair[0].kind == RoadKind::Gap) != (pair[1].kind == RoadKind::Gap))
            {
                let gate = pair[1];
                let normal = horizontal(gate.right).cross(Vec3::Y).normalize();
                for side in [-4.5, 0.0, 4.5] {
                    for offset in [-0.1, 0.0, 0.1] {
                        let position =
                            gate.pos + gate.right * side + normal * offset + Vec3::Y * RIDE_HEIGHT;
                        let road = nearest_road(
                            &track,
                            position,
                            gate.distance,
                            position.y + 1.0,
                            track.ground_height(),
                            None,
                        )
                        .expect("shared cross-section must not leave a contact hole");
                        let expected = if offset < 0.0 {
                            pair[0].kind
                        } else {
                            gate.kind
                        };
                        assert_eq!(
                            road.sample.kind, expected,
                            "wrong surface at distance {}, side {side}, offset {offset}",
                            gate.distance,
                        );
                        if offset == 0.0 {
                            assert!((road.sample.distance - gate.distance).abs() < 0.01);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn ramp_launches_ballistically_and_lands_on_following_road() {
        let track =
            Track::parse("straight 40\nramp 20 rise 3\ngap 14 rise -3\nstraight 180").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 35.0);
        car.velocity = vec3(0.0, 0.0, 29.0);
        let mut airtime = 0.0;
        let mut launched_upward = false;
        let mut landed = false;
        for _ in 0..600 {
            let was_grounded = car.grounded;
            car.update(&track, Control::default(), STEP);
            if !car.grounded {
                airtime += STEP;
                launched_upward |= car.velocity.y > 1.0;
            }
            if !was_grounded && car.grounded && car.position.z > 74.0 {
                landed = true;
            }
        }
        assert!(launched_upward, "ramp must preserve upward momentum");
        assert!(airtime > 0.4, "airtime {airtime}");
        assert!(landed, "car did not land: {:?}", car.position);
        assert!((car.position.y - RIDE_HEIGHT).abs() < 0.05);
    }

    #[test]
    fn airborne_car_cannot_pass_through_a_rising_road() {
        for (rise, direction) in [(60, 1.0), (-60, -1.0)] {
            let track = Track::parse(&format!("straight 100 rise {rise}")).unwrap();
            for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                let mut car = Car::new(&track);
                car.reset(&track, 50.0);
                car.position.y += 0.01;
                car.grounded = false;
                car.velocity = Vec3::Z * 50.0 * direction;
                car.update(&track, Control::default(), dt);
                assert!(
                    car.grounded && !car.offroad,
                    "missed uphill landing at dt {dt}, direction {direction}: {car:?}"
                );
            }
        }
    }

    #[test]
    fn airborne_car_below_a_rising_road_is_not_pulled_through_it() {
        let track = Track::parse("straight 100 rise 60").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 50.0);
        car.position.y -= 1.0;
        car.grounded = false;
        car.velocity = Vec3::Z * 50.0;
        let before = car.position;
        car.update(&track, Control::default(), 1.0 / 30.0);
        assert!(!car.grounded && car.offroad);
        assert!(car.position.y < before.y);
    }

    #[test]
    fn crossing_a_deck_plane_outside_the_bridge_cannot_pull_a_car_inside() {
        let track = Track::parse("bridge 100 rise 60").unwrap();
        for (dt, position, velocity) in [
            (STEP, vec3(6.15, 21.62, 40.0), vec3(-30.0, 0.0, 50.0)),
            (1.0 / 30.0, vec3(8.0, 21.8, 40.0), vec3(-90.0, 0.0, 60.0)),
        ] {
            let mut car = Car::new(&track);
            car.position = position;
            car.grounded = false;
            car.velocity = velocity;
            car.update(&track, Control::default(), dt);
            assert!(
                !car.grounded && car.offroad,
                "pulled through bridge edge at dt {dt}: {car:?}"
            );
            assert!(car.position.x < 6.0);
            assert!(car.position.y < position.y);
        }
    }

    #[test]
    fn bundled_alpine_gap_is_passable_at_ninety_kmh() {
        let track = Track::parse(include_str!("../tracks/alpine.track")).unwrap();
        let ramp = track
            .samples
            .iter()
            .find(|s| s.kind == RoadKind::Ramp)
            .unwrap();
        let landing = track
            .samples
            .iter()
            .find(|s| s.distance > ramp.distance + 25.0 && s.kind == RoadKind::Road)
            .unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, ramp.distance - 10.0);
        car.velocity = track.sample_at(car.distance).forward * 25.0;
        let mut airborne = false;
        let mut landed = false;
        for _ in 0..600 {
            let was_grounded = car.grounded;
            car.update(&track, Control::default(), STEP);
            airborne |= !car.grounded;
            landed |= !was_grounded && car.grounded && car.distance >= landing.distance;
        }
        assert!(
            airborne && landed,
            "airborne {airborne}, landed {landed}, distance {}",
            car.distance
        );
        assert!(car.grounded && !car.offroad);
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
    fn bank_sets_contact_height_and_gravity_points_downhill() {
        let track = Track::parse("straight 40 bank 15\nstraight 120 bank 15").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 60.0);
        car.position += track.sample_at(60.0).right * 2.0;
        let initial_x = car.position.x;
        advance(&mut car, &track, Control::default(), 1.0);
        assert!(car.grounded);
        assert!(car.position.x < initial_x);
        assert!(car.roll > 0.2);
        let expected = car.position.x * 15.0_f32.to_radians().tan() + RIDE_HEIGHT;
        assert!((car.position.y - expected).abs() < 0.03);
    }

    #[test]
    fn rapid_banking_transitions_cannot_swallow_a_supported_car() {
        for bank in [-30.0_f32, 30.0] {
            for length in [1.0, 2.0, 4.0] {
                let track = Track::parse(&format!(
                    "width 20\nstraight 100\nstraight {length} bank {bank}\nstraight 100"
                ))
                .unwrap();
                for direction in [-1.0, 1.0] {
                    for speed in [20.0, 40.0] {
                        for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                            let mut car = Car::new(&track);
                            let start = if direction > 0.0 {
                                99.0
                            } else {
                                101.0 + length
                            };
                            car.reset(&track, start);
                            let sample = track.sample_at(start);
                            car.position += sample.right * (5.0 * bank.signum() * direction);
                            car.heading = if direction > 0.0 {
                                0.0
                            } else {
                                std::f32::consts::PI
                            };
                            car.velocity = Vec3::Z * speed * direction;
                            for _ in 0..(0.5 / dt).ceil() as usize {
                                car.update(&track, Control::default(), dt);
                                let sample = track.sample_at(car.position.z);
                                let deck = sample.pos.y
                                    - horizontal(car.position - sample.pos).dot(sample.up)
                                        / sample.up.y;
                                assert!(
                                    car.position.y - RIDE_HEIGHT >= deck - 0.05,
                                    "penetrated bank {bank}, length {length}, direction {direction}, speed {speed}, dt {dt}: {car:?}"
                                );
                                assert!(car.grounded && !car.offroad);
                            }
                            assert!((car.distance - start) * direction > length + 1.0);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn changing_bank_does_not_pull_a_car_up_from_beneath_the_deck() {
        for bank in [-30, 30] {
            let track = Track::parse(&format!(
                "width 20\nstraight 100\nstraight 2 bank {bank}\nstraight 100"
            ))
            .unwrap();
            let mut car = Car::new(&track);
            car.reset(&track, 99.0);
            car.position = vec3(
                5.0 * (bank as f32).signum(),
                track.ground_height() + RIDE_HEIGHT,
                99.0,
            );
            car.velocity = Vec3::Z * 40.0;
            for _ in 0..60 {
                car.update(&track, Control::default(), STEP);
                assert!((car.position.y - track.ground_height() - RIDE_HEIGHT).abs() < 0.01);
                assert!(car.grounded && car.offroad);
            }
            assert!(car.position.z > 102.0);
        }
    }

    #[test]
    fn bank_contact_recovery_crosses_the_circuit_seam_in_both_directions() {
        for bank in [-30.0_f32, 30.0] {
            let track = Track::parse(&format!(
                "width 20\nstraight 100\nright 180 radius 30\nstraight 100\nright 177 radius 30 bank {bank}\nright 3 radius 30 bank 0\nclose"
            ))
            .unwrap();
            for direction in [-1.0, 1.0] {
                let start = if direction > 0.0 {
                    track.length - 0.2
                } else {
                    0.1
                };
                let mut car = Car::new(&track);
                car.reset(&track, start);
                let sample = track.sample_at(start);
                car.position += sample.right * (-5.0 * bank.signum() * direction);
                car.velocity = sample.forward * (40.0 * direction);
                car.heading = car.velocity.x.atan2(car.velocity.z);
                car.update(&track, Control::default(), 1.0 / 30.0);
                assert!(
                    car.grounded && !car.offroad,
                    "lost contact across the seam: {car:?}"
                );
                if direction > 0.0 {
                    assert!(car.distance < 5.0);
                } else {
                    assert!(car.distance > track.length - 5.0);
                }
            }
        }
    }

    #[test]
    fn deck_recovery_cannot_snap_across_a_gap_to_a_higher_landing() {
        let track = Track::parse("straight 100\ngap 1 rise 0.6\nstraight 100").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 99.9);
        car.velocity = Vec3::Z * 40.0;
        car.update(&track, Control::default(), 1.0 / 30.0);
        assert!(car.position.z > 101.0, "the step must cross the whole gap");
        assert!(!car.grounded && car.offroad);
        assert!((car.position.y - RIDE_HEIGHT).abs() < 0.01);
    }

    #[test]
    fn overpass_query_respects_height_and_does_not_snap_upward() {
        let mut track = Track::parse("straight 200").unwrap();
        let mut high = track.samples[0];
        high.pos = vec3(-100.0, 8.0, 100.0);
        high.forward = Vec3::X;
        high.right = -Vec3::Z;
        high.up = Vec3::Y;
        high.distance = 500.0;
        high.kind = RoadKind::Bridge;
        track.samples.push(high);
        high.pos.x = 100.0;
        high.distance = 700.0;
        track.samples.push(high);
        track.length = 700.0;
        let lower = nearest_road(
            &track,
            vec3(0.0, RIDE_HEIGHT, 100.0),
            100.0,
            0.30,
            track.ground_height(),
            None,
        )
        .unwrap();
        let upper = nearest_road(
            &track,
            vec3(0.0, 8.0 + RIDE_HEIGHT, 100.0),
            600.0,
            8.30,
            track.ground_height(),
            None,
        )
        .unwrap();
        assert!(lower.sample.pos.y.abs() < 0.001);
        assert!((upper.sample.pos.y - 8.0).abs() < 0.001);
        let mut car = Car::new(&track);
        car.reset(&track, 85.0);
        car.velocity = vec3(0.0, 0.0, 15.0);
        advance(&mut car, &track, Control::default(), 2.0);
        assert!(car.position.z > 105.0);
        assert!((car.position.y - RIDE_HEIGHT).abs() < 0.01);
    }

    #[test]
    fn jump_under_bridge_preserves_lower_route_progress_and_landing() {
        let lower_route = "straight 40\ncheckpoint\nramp 20 rise 4\ngap 20 rise -4\nstraight 20";
        let lower = Track::parse(lower_route).unwrap();
        let crossing = Track::parse(&format!(
            "{lower_route}\nright 270 radius 20 rise 10 kind bridge\nbridge 40"
        ))
        .unwrap();
        let mut reference = Car::new(&lower);
        let mut car = Car::new(&crossing);
        let mut race = crate::race::Race::new(None, car.distance);
        race.started = true;
        let control = Control {
            throttle: 1.0,
            ..Control::default()
        };
        let mut passed_under_bridge = false;
        let mut landed = false;
        for _ in 0..1_200 {
            reference.update(&lower, control, STEP);
            let was_grounded = car.grounded;
            car.update(&crossing, control, STEP);
            race.update(&crossing, STEP, car.distance, !car.offroad);
            assert!(
                (car.distance - reference.distance).abs() < 0.01,
                "overhead deck changed progress: lower {}, crossing {} at {:?}",
                reference.distance,
                car.distance,
                car.position
            );
            assert!(!race.invalid);
            assert!(car.position.distance(reference.position) < 0.01);
            if (74.0..86.0).contains(&car.position.z) && !car.grounded {
                assert!(car.position.y + 0.7 < 9.0);
                passed_under_bridge = true;
            }
            landed |= !was_grounded && car.grounded && car.position.z > 80.0;
            if car.position.z >= 96.0 {
                break;
            }
        }
        assert!(passed_under_bridge && landed);
        assert!(car.position.z >= 96.0 && car.grounded && !car.offroad);
    }

    #[test]
    fn car_can_climb_a_shoulder_from_below_the_road_deck() {
        let track = Track::parse("straight 100").unwrap();
        let mut car = Car::new(&track);
        car.position = vec3(18.0, track.ground_height() + RIDE_HEIGHT, 50.0);
        car.heading = -std::f32::consts::FRAC_PI_2;
        car.velocity = -Vec3::X * 10.0;
        for _ in 0..600 {
            car.update(
                &track,
                Control {
                    throttle: 1.0,
                    ..Control::default()
                },
                STEP,
            );
            if car.grounded && !car.offroad {
                break;
            }
        }
        assert!(
            car.position.x.abs() < 6.0,
            "could not re-enter road: {car:?}"
        );
        assert!(car.grounded && !car.offroad);
        assert!((car.position.y - RIDE_HEIGHT).abs() < 0.01);
    }

    #[test]
    fn fast_shoulder_entry_cannot_pass_beneath_a_raised_road() {
        for bank in [-60, -30, -14, 0, 14, 30, 60] {
            let track =
                Track::parse(&format!("straight 50 rise 20 bank {bank}\nstraight 200")).unwrap();
            let sample = track.sample_at(70.0);
            let ground = track.ground_height();
            let half_width = sample.width * 0.5;
            let outer_edge = half_width + SHOULDER_WIDTH;
            for side in [-1.0, 1.0] {
                for speed in [10.0, 20.0, 40.0] {
                    for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                        let mut car = Car::new(&track);
                        car.reset(&track, sample.distance);
                        car.position = sample.pos + sample.right * side * (outer_edge + 1.0);
                        car.position.y = ground + RIDE_HEIGHT;
                        car.heading = -side * std::f32::consts::FRAC_PI_2;
                        car.velocity = -Vec3::X * side * speed;
                        let mut climbed = false;
                        for _ in 0..(2.0 / dt) as usize {
                            car.update(
                                &track,
                                Control {
                                    throttle: 1.0,
                                    ..Control::default()
                                },
                                dt,
                            );
                            let lateral = car.position.x / sample.right.x;
                            if lateral.abs() <= outer_edge {
                                let edge_height =
                                    sample.pos.y + lateral.signum() * sample.right.y * half_width;
                                let fraction =
                                    ((lateral.abs() - half_width) / SHOULDER_WIDTH).clamp(0.0, 1.0);
                                let surface_height = if lateral.abs() <= half_width {
                                    sample.pos.y + lateral * sample.right.y
                                } else {
                                    edge_height + (ground - edge_height) * fraction
                                };
                                assert!(
                                    car.position.y - RIDE_HEIGHT >= surface_height - 0.05,
                                    "passed through shoulder: bank {bank}, side {side}, speed {speed}, dt {dt}, car {car:?}"
                                );
                            }
                            climbed |= car.position.y > ground + RIDE_HEIGHT + 1.0;
                        }
                        assert!(
                            climbed,
                            "did not enter shoulder: bank {bank}, side {side}, speed {speed}, dt {dt}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn descending_a_steep_shoulder_lands_on_terrain_without_falling_through() {
        let track = Track::parse("straight 50 rise 20\nstraight 200").unwrap();
        let ground = track.ground_height();
        for side in [-1.0, 1.0] {
            for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                let mut car = Car::new(&track);
                car.reset(&track, 70.0);
                car.position.x = side * 17.9;
                car.position.y = ground + 23.0 * (18.0 - 17.9) / 12.0 + RIDE_HEIGHT;
                car.heading = side * std::f32::consts::FRAC_PI_2;
                car.velocity = Vec3::X * side * 40.0;
                for _ in 0..10 {
                    car.update(&track, Control::default(), dt);
                    assert!(
                        car.position.y >= ground + RIDE_HEIGHT - 0.01,
                        "fell below terrain at side {side}, dt {dt}: {car:?}"
                    );
                }
                assert!(car.grounded && car.offroad);
            }
        }
    }

    #[test]
    fn car_already_below_a_raised_shoulder_is_not_pulled_through_it() {
        let track = Track::parse("straight 50 rise 20\nstraight 200").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 70.0);
        car.position.x = 12.0;
        car.position.y = track.ground_height() + RIDE_HEIGHT;
        car.heading = -std::f32::consts::FRAC_PI_2;
        car.velocity = -Vec3::X * 20.0;
        for _ in 0..60 {
            car.update(&track, Control::default(), STEP);
            assert!((car.position.y - track.ground_height() - RIDE_HEIGHT).abs() < 0.01);
            assert!(car.grounded && car.offroad);
        }
    }

    #[test]
    fn exact_gap_boundaries_use_the_outgoing_surface() {
        let track =
            Track::parse("straight 90\ncheckpoint\nstraight 10\ngap 10\nstraight 3").unwrap();
        for (distance, kind, offroad) in
            [(100.0, RoadKind::Gap, true), (110.0, RoadKind::Road, false)]
        {
            let road = nearest_road(
                &track,
                vec3(0.0, 1.0, distance),
                distance - 0.25,
                1.0,
                track.ground_height(),
                None,
            )
            .unwrap();
            assert_eq!(road.sample.kind, kind);
            assert_eq!(
                contact_surface(Some(road), 1.0, track.ground_height()).offroad,
                offroad
            );
        }

        // Reach the first landing sample exactly in one airborne step. It is
        // also the sprint finish, so using the preceding gap invalidates a run.
        let finish = track.finish_distance();
        let speed = 30.0;
        let post_drag_speed = speed - speed * (0.0015 * speed * STEP);
        let mut car = Car::new(&track);
        car.position = vec3(0.0, 1.0, finish - post_drag_speed * STEP);
        car.distance = car.position.z;
        car.velocity = Vec3::Z * speed;
        car.grounded = false;
        let mut race = crate::race::Race::new(None, car.distance);
        race.started = true;
        race.next_checkpoint = track.checkpoints.len();
        car.update(&track, Control::default(), STEP);
        assert_eq!(car.distance, finish);
        assert!(
            race.update(&track, STEP, car.distance, !car.offroad)
                .is_some()
        );
        assert!(race.finished && !race.invalid);
    }

    #[test]
    fn negative_elevation_course_and_offroad_ground_remain_drivable() {
        let track = Track::parse("start 0 -100 0\nstraight 1000").unwrap();
        let mut car = Car::new(&track);
        advance(
            &mut car,
            &track,
            Control {
                throttle: 1.0,
                ..Control::default()
            },
            4.0,
        );
        assert!(car.grounded && !car.offroad);
        assert!(car.speed_kmh() > 60.0);
        assert!((car.position.y - (-100.0 + RIDE_HEIGHT)).abs() < 0.01);

        // Dropping from the edge lands on terrain below this course, rather
        // than falling forever beneath an unrelated, fixed world-zero floor.
        car.position.x = 30.0;
        car.velocity = Vec3::ZERO;
        car.grounded = false;
        advance(&mut car, &track, Control::default(), 2.0);
        assert!(car.grounded && car.offroad);
        assert!((car.position.y - (track.ground_height() + RIDE_HEIGHT)).abs() < 0.01);
        advance(
            &mut car,
            &track,
            Control {
                throttle: 1.0,
                ..Control::default()
            },
            2.0,
        );
        assert!(car.grounded && car.offroad);
        assert!(car.speed_kmh() > 10.0);
    }
}
