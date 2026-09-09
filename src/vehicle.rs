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
const CONTACT_TOLERANCE: f32 = 0.30;
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
    /// A deck or shoulder beneath the preceding position, never an overpass.
    support: Option<RoadPoint>,
}

#[derive(Clone, Copy)]
struct Surface {
    height: f32,
    normal: Vec3,
    offroad: bool,
    /// Decks and shoulders have finite footprints; the base terrain does not.
    finite: bool,
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
        // A landing can precede leaving a deck or shoulder in this same
        // step, including when the car first enters it during the step.
        // The final road lookup alone sees only the gap or
        // terrain beyond it. Resolve that earlier contact before continuing
        // the remaining motion, without applying input forces a second time.
        if !was_grounded
            && let Some(((crossing, surface, fraction), entering)) = old_road
                .or_else(|| {
                    // Just outside a shoulder, the height-filtered lookup can
                    // reject its elevated deck even though this step enters the
                    // shoulder. Use it only to bound the triangle sweep; a real
                    // one-sided surface crossing is still required for contact.
                    nearest_road(
                        track,
                        before,
                        self.distance,
                        f32::INFINITY,
                        ground_height,
                        None,
                    )
                })
                .and_then(|from| {
                    road_contact_before_exit(
                        track,
                        from,
                        new_road,
                        before,
                        self.position,
                        ground_height,
                    )
                    .map(|contact| (contact, false))
                    .or_else(|| {
                        road_entry_contact_before_exit(
                            track,
                            from,
                            new_road,
                            before,
                            self.position,
                            ground_height,
                        )
                        .map(|contact| (contact, true))
                    })
                })
                .or_else(|| {
                    // Outside an open start or finish there is no preceding
                    // road to seed the sweep. The crossed endpoint only
                    // bounds the search; finite triangles still prove a hit.
                    open_track_entry_road(track, before, self.position).and_then(|from| {
                        road_entry_contact_before_exit(
                            track,
                            from,
                            new_road,
                            before,
                            self.position,
                            ground_height,
                        )
                        .map(|contact| (contact, true))
                    })
                })
        {
            self.resolve_landing(surface);
            if entering {
                // A twisting deck can be nearly vertical along the approach.
                // Its local normal blocks motion into the deck without
                // converting all forward speed into an artificial launch.
                self.velocity -= surface.normal * self.velocity.dot(surface.normal);
                sweep = RoadSweep {
                    before: crossing,
                    support: nearest_road(
                        track,
                        crossing,
                        self.distance,
                        crossing.y - RIDE_HEIGHT + CONTACT_TOLERANCE,
                        ground_height,
                        None,
                    ),
                };
            } else {
                self.velocity.y = surface_vertical_speed(self.velocity, surface.normal);
            }
            self.position = crossing + self.velocity * (dt * (1.0 - fraction));
            max_surface_height =
                (before.y - RIDE_HEIGHT).max(self.position.y - RIDE_HEIGHT) + CONTACT_TOLERANCE;
            new_road = nearest_road(
                track,
                self.position,
                self.distance,
                max_surface_height,
                ground_height,
                Some(sweep),
            );
        }
        if let Some(road) = new_road {
            self.road_index = road.index;
            self.distance = road.sample.distance;
        }
        self.resolve_guardrail(track, new_road, old_road, before);
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
        if let Some(road) = new_road {
            self.road_index = road.index;
            self.distance = road.sample.distance;
        }
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
                || ((current_bottom - surface.height).abs() < CONTACT_TOLERANCE
                    && self.velocity.y - required_vertical <= GRAVITY * dt + 0.025
                    // Even a small height correction must follow a solid
                    // surface. A short gap can be crossed in a single step,
                    // and its landing can lie within the contact tolerance.
                    && (!surface.finite
                        || (old_surface.finite
                            && old_road.zip(new_road).is_some_and(|(from, to)| {
                                crosses_connected_surface(
                                    track,
                                    from,
                                    to,
                                    before,
                                    self.position,
                                )
                            })))));
        // A step can leave one supporting surface and strike another: a steep
        // downhill shoulder can cross the terrain before the car is airborne.
        // Keep the one-sided sweep valid for those transitions as well.
        let landed = !on_same_surface
            && current_bottom <= surface.height
            && (new_road.is_some_and(|road| road.swept_contact)
                || (previous_clearance >= -0.08
                    && self.velocity.y <= required_vertical + 0.3))
            // The height crossing must occur on the finite road or shoulder.
            // A car can fall below a landing while over a gap, then reach its
            // footprint later in this step without ever touching its top.
            && (!surface.finite
                || new_road.is_some_and(|road| {
                    road.swept_contact
                        || crosses_finite_road_surface(
                            track,
                            road,
                            surface,
                            before,
                            self.position,
                            ground_height,
                        )
                }));

        self.grounded = on_same_surface || landed;
        self.offroad = surface.offroad;
        if self.grounded {
            if landed {
                self.resolve_landing(surface);
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
        if let Some((hit, normal)) = tunnel_roof_sweep(track, before, self.position) {
            self.position = hit - Vec3::Y * 0.7 - normal * 0.0001;
            self.velocity -= normal * self.velocity.dot(normal).max(0.0);
            // Downward motion stays inside an upward-facing ceiling. On a
            // banked side panel that faces down, removing an upward tangent
            // would instead put outward velocity back into the wall.
            if normal.y >= 0.0 {
                self.velocity.y = self.velocity.y.min(0.0);
            }
            self.impact = 0.6;
            // The sweep can stop before a gate that the unconstrained step
            // crossed. Timing and surface state must follow the corrected car.
            let road = nearest_road(
                track,
                self.position,
                self.distance,
                max_surface_height,
                ground_height,
                None,
            );
            if let Some(road) = road {
                self.road_index = road.index;
                self.distance = road.sample.distance;
            }
            let surface = contact_surface(road, max_surface_height, ground_height);
            self.offroad = surface.offroad;
            self.grounded &= (self.position.y - RIDE_HEIGHT - surface.height).abs() < 0.55;
        }
    }

    fn resolve_landing(&mut self, surface: Surface) {
        let vertical_impact =
            (self.velocity.y - surface_vertical_speed(self.velocity, surface.normal)).abs();
        self.impact = self.impact.max((vertical_impact / 11.0).clamp(0.0, 1.0));
        // A firm landing scrubs speed, but does not reset steering or
        // horizontal momentum and therefore still rewards alignment.
        let landing_loss = (vertical_impact - 2.5).max(0.0) * 0.014;
        self.velocity.x *= 1.0 - landing_loss.min(0.24);
        self.velocity.z *= 1.0 - landing_loss.min(0.24);
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

    fn resolve_guardrail(
        &mut self,
        track: &Track,
        road: Option<RoadPoint>,
        previous_road: Option<RoadPoint>,
        before: Vec3,
    ) {
        // The barrier follows the banked road frame. Both its correction and
        // impulse must stay in the road plane, or an uphill impact launches
        // the car by retaining its old upward velocity after the rebound.
        let swept_hit = previous_road.and_then(|prior| {
            let destination =
                road.unwrap_or_else(|| guardrail_exit_road(track, prior, before, self.position));
            guardrail_sweep(track, prior, destination, before, self.position)
        });
        let (normal, outward) = if let Some((hit, normal, outward)) = swept_hit {
            // Stop at the wall that was crossed. On a taper, shunting all
            // the way to the final narrow cross-section would teleport
            // the car sideways and leave its forward speed unchanged.
            self.position = hit - outward * 0.001;
            (normal, outward)
        } else {
            let Some(road) = road else {
                return;
            };
            if !matches!(road.sample.kind, RoadKind::Bridge | RoadKind::Tunnel) {
                return;
            }
            let clearance = self.position.y - RIDE_HEIGHT - road.plane_height;
            // Terrain beneath an overpass cannot hit its guardrails.
            if !(-0.45..0.85).contains(&clearance) {
                return;
            }
            let limit = (road.sample.width * 0.5 - CAR_HALF_WIDTH).max(0.5);
            if road.lateral.abs() <= limit {
                return;
            }
            let side = road.lateral.signum();
            // Without a proven wall crossing, retain the local correction
            // for a car already touching the rail. An outside approach
            // must not teleport through the bridge to its inside edge.
            let old_lateral =
                (before - Vec3::Y * RIDE_HEIGHT - road.sample.pos).dot(road.sample.right);
            if old_lateral.abs() > road.sample.width * 0.5 + 1.0 {
                return;
            }
            let normal = road.sample.up;
            let lateral_axis =
                (road.sample.right - normal * road.sample.right.dot(normal)).normalize();
            self.position += lateral_axis
                * ((side * limit - road.lateral) / lateral_axis.dot(road.sample.right));
            (normal, lateral_axis * side)
        };
        let outward_speed = self.velocity.dot(outward);
        if outward_speed > 0.0 {
            self.velocity -= outward * outward_speed * 1.12;
            let scrub = (outward_speed * 0.012).min(0.16);
            let tangent_velocity = self.velocity - normal * self.velocity.dot(normal);
            self.velocity -= tangent_velocity * scrub;
            self.yaw_rate *= 0.68;
            self.impact = self.impact.max((outward_speed / 10.0).clamp(0.08, 1.0));
        }
    }
}

/// Seed an inward sweep at an open endpoint even when the preceding position
/// has no road lookup. This does not extend the endpoint's collision footprint.
fn open_track_entry_road(track: &Track, before: Vec3, after: Vec3) -> Option<RoadPoint> {
    if track.closed || track.samples.len() < 2 {
        return None;
    }
    let last = track.samples.len() - 1;
    [
        (0, track.samples[0], 1.0),
        (last - 1, track.samples[last], -1.0),
    ]
    .into_iter()
    .filter_map(|(index, sample, direction)| {
        let normal = horizontal(sample.right).cross(Vec3::Y).normalize() * direction;
        let start = horizontal(before - sample.pos).dot(normal);
        let end = horizontal(after - sample.pos).dot(normal);
        if start >= 0.0 || end < 0.0 {
            return None;
        }
        let fraction = -start / (end - start);
        let crossing = before.lerp(after, fraction);
        let plane_height =
            sample.pos.y - horizontal(crossing - sample.pos).dot(sample.up) / sample.up.y.max(0.15);
        let contact = vec3(crossing.x, plane_height, crossing.z);
        Some((
            fraction,
            RoadPoint {
                sample,
                index,
                lateral: (contact - sample.pos).dot(sample.right),
                plane_height,
                swept_contact: false,
            },
        ))
    })
    .min_by(|a, b| a.0.total_cmp(&b.0))
    .map(|(_, road)| road)
}

/// An endpoint outside every road footprint can still have crossed a rail.
/// Follow the swept cross-sections from its preceding road so the collision
/// query includes short final segments without extending the road collider.
fn guardrail_exit_road(track: &Track, mut road: RoadPoint, before: Vec3, after: Vec3) -> RoadPoint {
    let forward = horizontal(after - before).dot(road.sample.forward) >= 0.0;
    let direction = if forward { 1.0 } else { -1.0 };
    let segments = track.samples.len() - 1;
    for _ in 0..segments {
        let boundary = track.samples[road.index + usize::from(forward)];
        let normal = horizontal(boundary.right).cross(Vec3::Y).normalize();
        let start = horizontal(before - boundary.pos).dot(normal) * direction;
        let end = horizontal(after - boundary.pos).dot(normal) * direction;
        if start > SEGMENT_TOLERANCE || end < -SEGMENT_TOLERANCE || end <= start {
            break;
        }
        road.sample = boundary;
        if !track.closed
            && ((forward && road.index + 1 == segments) || (!forward && road.index == 0))
        {
            break;
        }
        road.index = if forward {
            (road.index + 1) % segments
        } else {
            (road.index + segments - 1) % segments
        };
    }
    road
}

/// Sweep the chassis center against the inset edges of the intervening road
/// segments. The wall tangent includes width changes as well as road curvature.
fn guardrail_sweep(
    track: &Track,
    from: RoadPoint,
    to: RoadPoint,
    before: Vec3,
    after: Vec3,
) -> Option<(Vec3, Vec3, Vec3)> {
    let mut delta = to.sample.distance - from.sample.distance;
    if track.closed {
        if delta > track.length * 0.5 {
            delta -= track.length;
        } else if delta < -track.length * 0.5 {
            delta += track.length;
        }
    }
    // Route selection can jump between nearby but disconnected roads. Such a
    // jump is not evidence that the intervening guardrails were traversed.
    if delta.abs() > 12.0 {
        return None;
    }
    let segments = track.samples.len() - 1;
    let forward = if delta == 0.0 && from.index != to.index {
        // The same cross-section may be reported from either adjacent
        // segment. At the circuit seam, +0 alone would choose a full lap.
        if (from.index + 1) % segments == to.index {
            true
        } else if (to.index + 1) % segments == from.index {
            false
        } else {
            return None;
        }
    } else {
        delta >= 0.0
    };
    let mut index = from.index;
    let motion = horizontal(after - before);
    let cross = |a: Vec3, b: Vec3| a.x * b.z - a.z * b.x;
    let mut hit: Option<(f32, Vec3, Vec3)> = None;
    loop {
        let a = track.samples[index];
        let b = track.samples[index + 1];
        if matches!(a.kind, RoadKind::Bridge | RoadKind::Tunnel) {
            for side in [-1.0, 1.0] {
                let edge = |sample: RoadSample| {
                    sample.pos
                        + sample.right * side * (sample.width * 0.5 - CAR_HALF_WIDTH).max(0.5)
                };
                let start = edge(a);
                let tangent = edge(b) - start;
                let wall = horizontal(tangent);
                let offset = horizontal(start - before);
                let denominator = cross(motion, wall);
                if denominator.abs() > 0.00001 {
                    let fraction = cross(offset, wall) / denominator;
                    let along = cross(offset, motion) / denominator;
                    if (0.0..=1.0).contains(&fraction)
                        && (0.0..=1.0).contains(&along)
                        && hit.is_none_or(|(best, _, _)| fraction < best)
                    {
                        let clearance = before.lerp(after, fraction).y
                            - RIDE_HEIGHT
                            - (start.y + tangent.y * along);
                        if (-0.45..0.85).contains(&clearance) {
                            let normal = a.up.lerp(b.up, along).normalize();
                            let outward = normal.cross(tangent).normalize() * side;
                            if motion.dot(outward) > 0.0 {
                                hit = Some((fraction, normal, outward));
                            }
                        }
                    }
                }
            }
        }
        if index == to.index {
            break;
        }
        index = if forward {
            (index + 1) % segments
        } else {
            (index + segments - 1) % segments
        };
    }
    hit.map(|(fraction, normal, outward)| (before.lerp(after, fraction), normal, outward))
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
    nearest_road_with_tolerance(
        track,
        position,
        previous_distance,
        max_surface_height,
        ground_height,
        sweep,
        SEGMENT_TOLERANCE,
    )
}

fn nearest_road_with_tolerance(
    track: &Track,
    position: Vec3,
    previous_distance: f32,
    max_surface_height: f32,
    ground_height: f32,
    sweep: Option<RoadSweep>,
    segment_tolerance: f32,
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
        // Pitch and bank can turn an offset cross-section against the
        // centerline's direction. Accept either sign change. Only snap a
        // point outside that bracket to a nearby seam: on dense transitions,
        // the full interval can be narrower than the usual seam tolerance.
        let bracketed = (start_offset >= 0.0 && end_offset <= 0.0)
            || (start_offset <= 0.0 && end_offset >= 0.0);
        // Arithmetic on a translated track can leave an exact shared edge a
        // few ULPs inside either neighbor. Preserve outgoing edge kinds there
        // without consuming a measurable part of a short segment.
        let roundoff =
            segment_tolerance.min(0.00001 + f32::EPSILON * position.x.abs().max(position.z.abs()));
        let along = if start_offset.abs() <= roundoff
            || (!bracketed && start_offset.abs() <= segment_tolerance)
        {
            0.0
        } else if end_offset.abs() <= roundoff
            || (!bracketed && end_offset.abs() <= segment_tolerance)
        {
            1.0
        } else if !bracketed {
            continue;
        } else {
            // Interpolate the cross-section that contains the car. On a
            // banked hill, the road's right vector has a longitudinal
            // horizontal component, so projecting onto the centerline can
            // move progress by metres just from steering across the deck.
            // The interpolated raw frame gives a quadratic in `along`;
            // select the stable root for either crossing direction, also
            // handling a linear segment.
            let start_normal = horizontal(a.right).cross(Vec3::Y);
            let normal_change = horizontal(b.right - a.right).cross(Vec3::Y);
            let offset = horizontal(position - a.pos);
            let quadratic = -chord.dot(normal_change);
            let linear = offset.dot(normal_change) - chord.dot(start_normal);
            let constant = offset.dot(start_normal);
            let discriminant = (linear * linear - 4.0 * quadratic * constant).max(0.0);
            let signed_root = discriminant.sqrt().copysign(start_offset);
            let along = if linear * constant > 0.0 {
                // Rationalizing the root would subtract nearly equal terms
                // in its denominator near a seam with this slope direction.
                (-linear - signed_root) / (2.0 * quadratic)
            } else {
                2.0 * constant / (-linear + signed_root)
            };
            let forward = a.forward.lerp(b.forward, along);
            let right = a.right.lerp(b.right, along);
            if (0.0..=1.0).contains(&along) && forward.dot(right).abs() <= 0.000001 {
                along
            } else {
                // sample_at removes the interpolated right axis's forward
                // component. On banked hills that changes the cross-section
                // direction, so the raw-axis quadratic is only an estimate.
                // Solve the same orthogonalized frame used by the renderer;
                // otherwise lateral motion alone changes timing progress.
                let mut lower = 0.0;
                let mut upper = 1.0;
                for _ in 0..16 {
                    let middle = (lower + upper) * 0.5;
                    let forward = a.forward.lerp(b.forward, middle);
                    let right = a.right.lerp(b.right, middle);
                    let right = right - forward * (forward.dot(right) / forward.length_squared());
                    let normal = horizontal(right).cross(Vec3::Y);
                    let offset = horizontal(position - a.pos.lerp(b.pos, middle));
                    if offset.dot(normal) * start_offset > 0.0 {
                        lower = middle;
                    } else {
                        upper = middle;
                    }
                }
                (lower + upper) * 0.5
            }
        };
        let pos = a.pos.lerp(b.pos, along);
        let forward = a.forward.lerp(b.forward, along).normalize_or_zero();
        let right = a.right.lerp(b.right, along);
        let right = (right - forward * right.dot(forward)).normalize_or_zero();
        // Use the same orthonormal frame as sample_at. Independently
        // blending up on a short banked hill would tilt the contact plane
        // away from the rendered cross-section.
        let up = forward.cross(right).normalize_or_zero();
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
            kind: if along == 1.0 { b.kind } else { a.kind },
        };
        let plane_height = pos.y - horizontal(position - pos).dot(up) / up.y.max(0.15);
        let contact = vec3(position.x, plane_height, position.z);
        let lateral = (contact - pos).dot(right);
        let mut progress_delta = (distance - previous_distance).abs();
        if track.closed && track.length > 0.0 {
            progress_delta = progress_delta.min((track.length - progress_delta).abs());
        }
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
        // Compare finite surfaces rather than centerlines: a narrow adjacent
        // deck or its lower shoulder must not hide the wide road under a car.
        // Gap footprints retain their virtual plane for airborne progress.
        let footprint_width = width * 0.5
            + if matches!(sample.kind, RoadKind::Road | RoadKind::Ramp) {
                SHOULDER_WIDTH
            } else {
                0.0
            };
        let outside = (lateral.abs() - footprint_width).max(0.0) * horizontal(right).length();
        let vertical = position.y - RIDE_HEIGHT - surface_height;
        // A tolerated seam extension must not tie with the real interior of
        // its neighbor on a level road and pin progress to the earlier edge.
        let seam_offset = if along == 0.0 {
            start_offset
        } else if along == 1.0 {
            end_offset
        } else {
            0.0
        };
        // Height separates crossing bridges. A small continuity preference
        // disambiguates joins, parallel lanes, and paths through a jump.
        let score = outside * outside
            + seam_offset * seam_offset
            + vertical * vertical * 3.0
            + ((progress_delta - 12.0).max(0.0) * 0.025)
                .powi(2)
                .min(180.0);
        // A changing bank has longitudinal height variation that the final
        // cross-section normal cannot represent. Compare the actual heights
        // at both ends of a connected sweep, including airborne approaches.
        point.swept_contact = surface.is_some_and(|surface| {
            position.y - RIDE_HEIGHT <= surface.height
                && sweep.is_some_and(|sweep| {
                    sweep.support.is_some_and(|support| {
                        crosses_connected_surface(track, support, point, sweep.before, position)
                    })
                })
        });
        if !matches!(sample.kind, RoadKind::Gap)
            && surface_height > max_surface_height.min(position.y - RIDE_HEIGHT + CONTACT_TOLERANCE)
        {
            // A steep surface can rise past an airborne car in one step. Its
            // plane provides a one-sided sweep for both decks and shoulders.
            point.swept_contact |= surface.is_some_and(|surface| {
                position.y - RIDE_HEIGHT <= surface.height
                    && sweep.is_some_and(|sweep| {
                        sweep.before.y - RIDE_HEIGHT
                            >= plane_height_at(surface, position, sweep.before) - 0.08
                            && crosses_finite_road_surface(
                                track,
                                point,
                                surface,
                                sweep.before,
                                position,
                                ground_height,
                            )
                    })
            });
            // Downward motion can exceed ordinary contact's current-position
            // tolerance before exceeding the previous-position height bound.
            // Run the sweep in that interval too, while keeping the same
            // one-sided bound for route selection.
            if !point.swept_contact && surface_height > max_surface_height {
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

/// Crossing an extended road or shoulder plane outside its footprint is not a
/// landing. Locate the actual contact point and require a solid route from there
/// to this step's endpoint, including any intervening sample boundaries.
fn crosses_finite_road_surface(
    track: &Track,
    to: RoadPoint,
    surface: Surface,
    before: Vec3,
    after: Vec3,
    ground_height: f32,
) -> bool {
    let start_clearance = before.y - RIDE_HEIGHT - plane_height_at(surface, after, before);
    let end_clearance = after.y - RIDE_HEIGHT - surface.height;
    // Clamping an intersection behind the motion to its starting point can
    // turn an approach already beneath the deck into a landing on its top.
    if start_clearance < -0.001 || end_clearance > 0.001 {
        return false;
    }
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
            (crossing.y - RIDE_HEIGHT - surface.height).abs() <= 0.08
                && crosses_connected_surface(track, from, to, crossing, after)
        })
    })
}

/// Detect a top-surface hit before this step leaves its finite footprint. The
/// usual contact path handles endpoints that remain on a connected surface.
fn road_contact_before_exit(
    track: &Track,
    from: RoadPoint,
    to: Option<RoadPoint>,
    before: Vec3,
    after: Vec3,
    ground_height: f32,
) -> Option<(Vec3, Surface, f32)> {
    let surface = road_surface(from, ground_height)?;
    let start_clearance = before.y - RIDE_HEIGHT - surface.height;
    let end_clearance = after.y - RIDE_HEIGHT - plane_height_at(surface, before, after);
    if start_clearance < 0.0 || end_clearance >= 0.0 {
        return None;
    }
    if to.is_some_and(|to| {
        road_surface(to, ground_height).is_some()
            && crosses_connected_surface(track, from, to, before, after)
    }) {
        return None;
    }
    let fraction = start_clearance / (start_clearance - end_clearance);
    let mut crossing = before.lerp(after, fraction);
    let road = nearest_road(
        track,
        crossing,
        from.sample.distance,
        crossing.y - RIDE_HEIGHT + 0.08,
        ground_height,
        None,
    )?;
    let surface = road_surface(road, ground_height)?;
    // The preceding plane is only a candidate. Its crossing must coincide
    // with an actual connected deck or shoulder, rather than its extension.
    if (crossing.y - RIDE_HEIGHT - surface.height).abs() > 0.08
        || !crosses_connected_surface(track, from, road, before, crossing)
    {
        return None;
    }
    crossing.y = surface.height + RIDE_HEIGHT;
    Some((crossing, surface, fraction))
}

/// A car can enter and leave a short deck, or cut across its corner, while
/// both endpoints remain outside its footprint. Sweep finite surface triangles
/// in only the intervening route segments. The local response normal includes
/// the longitudinal slope created by changing bank as well as ordinary grade.
fn road_entry_contact_before_exit(
    track: &Track,
    from: RoadPoint,
    to: Option<RoadPoint>,
    before: Vec3,
    after: Vec3,
    ground_height: f32,
) -> Option<(Vec3, Surface, f32)> {
    let destination = to.unwrap_or_else(|| guardrail_exit_road(track, from, before, after));
    let mut delta = destination.sample.distance - from.sample.distance;
    if track.closed {
        if delta > track.length * 0.5 {
            delta -= track.length;
        } else if delta < -track.length * 0.5 {
            delta += track.length;
        }
    }
    if delta.abs() > 12.0 {
        return None;
    }
    let segments = track.samples.len() - 1;
    let forward = if delta == 0.0 && from.index != destination.index {
        if (from.index + 1) % segments == destination.index {
            true
        } else if (destination.index + 1) % segments == from.index {
            false
        } else {
            return None;
        }
    } else {
        delta >= 0.0
    };
    let mut index = from.index;
    let mut hit: Option<(Vec3, Surface, f32)> = None;
    loop {
        let a = track.samples[index];
        let b = track.samples[index + 1];
        if a.kind != RoadKind::Gap {
            // Refine twisting strips so each triangle stays within 1 cm of
            // the interpolated road frame, including changing road width.
            let normal = (a.up + b.up).normalize();
            let twist = (b.right * b.width - a.right * a.width).dot(normal).abs() * 0.25;
            let frame_change = a.forward.distance(b.forward).max(a.right.distance(b.right));
            // A shoulder also twists on an ordinary hill: its inner edge
            // changes elevation while the terrain edge remains level.
            let shoulder_height_change = if matches!(a.kind, RoadKind::Road | RoadKind::Ramp) {
                [-1.0_f32, 1.0]
                    .into_iter()
                    .map(|side| {
                        ((b.pos.y - a.pos.y)
                            + (b.right.y * b.width - a.right.y * a.width) * side * 0.5)
                            .abs()
                    })
                    .fold(0.0_f32, f32::max)
            } else {
                0.0
            };
            // Curved cross-sections displace a shoulder triangle sideways.
            // On a tall embankment even a small displacement becomes a large
            // height error, so refine by the shoulder slope as the mesh does.
            let shoulder_curve_error = if matches!(a.kind, RoadKind::Road | RoadKind::Ramp) {
                let slope = [a, b]
                    .into_iter()
                    .flat_map(|sample| {
                        [-1.0, 1.0].map(|side| {
                            let edge_height =
                                sample.pos.y + sample.right.y * sample.width * 0.5 * side;
                            (edge_height - ground_height).abs()
                                / (SHOULDER_WIDTH * horizontal(sample.right).length())
                        })
                    })
                    .fold(0.0_f32, f32::max);
                frame_change
                    * frame_change
                    * (a.width.max(b.width) * 0.5 + SHOULDER_WIDTH)
                    * 0.25
                    * slope
            } else {
                0.0
            };
            let steps = (twist / 0.01)
                .ceil()
                .max((frame_change / 0.05).ceil())
                .max((shoulder_height_change / 0.04).ceil())
                .max((shoulder_curve_error / 0.01).sqrt().ceil())
                .max(1.0) as usize;
            let sample_at = |step| {
                if step == 0 {
                    a
                } else if step == steps {
                    b
                } else {
                    track.sample_at(
                        a.distance + (b.distance - a.distance) * step as f32 / steps as f32,
                    )
                }
            };
            for step in 0..steps {
                let start = sample_at(step);
                let end = sample_at(step + 1);
                let left_start = start.pos - start.right * start.width * 0.5;
                let right_start = start.pos + start.right * start.width * 0.5;
                let left_end = end.pos - end.right * end.width * 0.5;
                let right_end = end.pos + end.right * end.width * 0.5;
                let outer = |sample: RoadSample, side: f32| {
                    let mut point = sample.pos
                        + horizontal(sample.right) * side * (sample.width * 0.5 + SHOULDER_WIDTH);
                    point.y = ground_height;
                    point
                };
                let strips = [
                    (left_start, right_start, left_end, right_end),
                    (outer(start, -1.0), left_start, outer(end, -1.0), left_end),
                    (right_start, outer(start, 1.0), right_end, outer(end, 1.0)),
                ];
                let strip_count = if matches!(a.kind, RoadKind::Road | RoadKind::Ramp) {
                    strips.len()
                } else {
                    1
                };
                for &(left_start, right_start, left_end, right_end) in &strips[..strip_count] {
                    for [origin, second, third] in [
                        [left_start, right_end, right_start],
                        [left_start, left_end, right_end],
                    ] {
                        let u = second - origin;
                        let v = third - origin;
                        let normal = u.cross(v).normalize_or_zero();
                        let start = (before - Vec3::Y * RIDE_HEIGHT - origin).dot(normal);
                        let end = (after - Vec3::Y * RIDE_HEIGHT - origin).dot(normal);
                        if start < -0.001 || end >= 0.0 || end >= start {
                            continue;
                        }
                        let fraction = (start / (start - end)).max(0.0);
                        let crossing = before.lerp(after, fraction);
                        let offset = crossing - Vec3::Y * RIDE_HEIGHT - origin;
                        let uu = u.length_squared();
                        let vv = v.length_squared();
                        let uv = u.dot(v);
                        let denominator = uu * vv - uv * uv;
                        if denominator <= 0.0000001 {
                            continue;
                        }
                        let along_u = (offset.dot(u) * vv - offset.dot(v) * uv) / denominator;
                        let along_v = (offset.dot(v) * uu - offset.dot(u) * uv) / denominator;
                        if along_u < -0.0001 || along_v < -0.0001 || along_u + along_v > 1.0001 {
                            continue;
                        }
                        let Some((crossing, surface, fraction, road)) = refine_road_entry_contact(
                            track,
                            before,
                            after,
                            a.distance,
                            fraction,
                            ground_height,
                        ) else {
                            continue;
                        };
                        if hit.is_none_or(|(_, _, best)| fraction < best)
                            && !to.is_some_and(|to| {
                                road_surface(to, ground_height).is_some()
                                    && crosses_connected_surface(track, road, to, crossing, after)
                            })
                        {
                            hit = Some((crossing, surface, fraction));
                        }
                    }
                }
            }
        }
        if index == destination.index {
            break;
        }
        index = if forward {
            (index + 1) % segments
        } else {
            (index + segments - 1) % segments
        };
    }
    hit
}

/// Triangles provide a close candidate, but even a small height approximation
/// can collide with a car that remains above the actual road. Refine against
/// the actual ruled surface and require a one-sided crossing of its local plane.
fn refine_road_entry_contact(
    track: &Track,
    before: Vec3,
    after: Vec3,
    previous_distance: f32,
    mut fraction: f32,
    ground_height: f32,
) -> Option<(Vec3, Surface, f32, RoadPoint)> {
    for _ in 0..8 {
        let mut crossing = before.lerp(after, fraction);
        // Root refinement needs continuous height interpolation rather than
        // the broad cross-section seam tolerance used for ordinary contact.
        let road = nearest_road_with_tolerance(
            track,
            crossing,
            previous_distance,
            crossing.y - RIDE_HEIGHT + 0.08,
            ground_height,
            None,
            0.00001,
        )?;
        let mut surface = road_surface(road, ground_height)?;
        let clearance = crossing.y - RIDE_HEIGHT - surface.height;
        if clearance.abs() > 0.08 {
            return None;
        }
        // Differentiate at the contact's actual lateral coordinate. A triangle
        // spanning the full width has the slope of an outer edge, which can
        // launch a centerline approach even though its surface remains level.
        let a = track.samples[road.index];
        let b = track.samples[road.index + 1];
        let along =
            ((road.sample.distance - a.distance) / (b.distance - a.distance)).clamp(0.0, 1.0);
        let lower = (along - 0.01).max(0.0);
        let upper = (along + 0.01).min(1.0);
        let point_at = |fraction| {
            let forward = a.forward.lerp(b.forward, fraction).normalize();
            let right = a.right.lerp(b.right, fraction);
            let right = (right - forward * right.dot(forward)).normalize();
            // Work relative to the segment origin so short subsegments retain
            // precision even when the authored track is far from world zero.
            let pos = (b.pos - a.pos) * fraction;
            let mut point = pos + right * road.lateral;
            if surface.offroad {
                let half_width = (a.width + (b.width - a.width) * fraction) * 0.5;
                let edge_height = pos.y + right.y * road.lateral.signum() * half_width;
                let shoulder_fraction = (road.lateral.abs() - half_width) / SHOULDER_WIDTH;
                point.y = edge_height + (ground_height - a.pos.y - edge_height) * shoulder_fraction;
            }
            point
        };
        // Subtract endpoints before scaling to retain precision far from origin.
        let tangent = point_at(upper) - point_at(lower);
        let across = if surface.offroad {
            let side = road.lateral.signum();
            let edge_height =
                road.sample.pos.y + road.sample.right.y * side * road.sample.width * 0.5;
            horizontal(road.sample.right) * SHOULDER_WIDTH
                + Vec3::Y * (ground_height - edge_height) * side
        } else {
            road.sample.right
        };
        surface.normal = tangent.cross(across).normalize();
        crossing.y = surface.height + RIDE_HEIGHT;
        let start = (before - crossing).dot(surface.normal);
        let end = (after - crossing).dot(surface.normal);
        if start < 0.0 || end >= 0.0 || end >= start {
            return None;
        }
        if clearance.abs() <= 0.001 {
            return Some((crossing, surface, fraction, road));
        }
        fraction = start / (start - end);
    }
    None
}

/// Check the actual swept path through intervening road cross-sections, rather
/// than treating proximity in world space as evidence of a connected surface.
fn crosses_connected_surface(
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
        let next_index = if forward {
            (index + 1) % segments
        } else {
            (index + segments - 1) % segments
        };
        // A shoulder ends when either side of the join is a bridge, tunnel,
        // or gap. Recovery must not extend terrain support across that edge.
        let shoulder_width = if (starts_at_boundary
            || matches!(track.samples[index].kind, RoadKind::Road | RoadKind::Ramp))
            && matches!(
                track.samples[next_index].kind,
                RoadKind::Road | RoadKind::Ramp
            ) {
            SHOULDER_WIDTH
        } else {
            0.0
        };
        let right = horizontal(boundary.right);
        let lateral = horizontal(crossing - boundary.pos).dot(right) / right.length_squared();
        if lateral.abs() > boundary.width * 0.5 + shoulder_width + SEGMENT_TOLERANCE {
            return false;
        }
        index = next_index;
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
        finite: false,
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
            finite: true,
        }
    } else if matches!(road.sample.kind, RoadKind::Road | RoadKind::Ramp)
        && road.lateral.abs() <= half_width + SHOULDER_WIDTH + SEGMENT_TOLERANCE
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
            finite: true,
        }
    } else {
        return None;
    };
    Some(surface)
}

fn plane_height_at(surface: Surface, origin: Vec3, point: Vec3) -> f32 {
    surface.height - horizontal(point - origin).dot(surface.normal) / surface.normal.y.max(0.15)
}

/// Check the finite roof panels crossed by the chassis top. A lookup at only
/// the destination misses a roof impact followed by an exit in the same step.
/// Triangles also retain the tunnel's actual opening instead of extending its
/// ceiling beyond the final cross-section.
fn tunnel_roof_sweep(track: &Track, before: Vec3, after: Vec3) -> Option<(Vec3, Vec3)> {
    let before = before + Vec3::Y * 0.7;
    let after = after + Vec3::Y * 0.7;
    let path_min = before.min(after);
    let path_max = before.max(after);
    let mut hit: Option<(f32, Vec3)> = None;
    for pair in track.samples.windows(2) {
        let a = pair[0];
        let b = pair[1];
        if a.kind != RoadKind::Tunnel {
            continue;
        }
        // A conservative bound is cheap even for a long, densely sampled route.
        let radius = a.width.max(b.width) * 0.5 + 6.85;
        let bounds_min = a.pos.min(b.pos) - Vec3::splat(radius);
        let bounds_max = a.pos.max(b.pos) + Vec3::splat(radius);
        if path_max.cmplt(bounds_min).any() || path_min.cmpgt(bounds_max).any() {
            continue;
        }
        let arch = |sample: RoadSample, index: usize| {
            let angle = index as f32 / 10.0 * std::f32::consts::PI;
            sample.pos
                + sample.right * angle.cos() * (sample.width * 0.5 + 0.65)
                + sample.up * (1.0 + angle.sin() * 5.2)
        };
        for index in 0..10 {
            let aa = arch(a, index);
            let ab = arch(a, index + 1);
            let ba = arch(b, index);
            let bb = arch(b, index + 1);
            for [origin, second, third] in [[aa, ab, bb], [aa, bb, ba]] {
                let u = second - origin;
                let v = third - origin;
                let normal = u.cross(v).normalize_or_zero();
                // Winding points out of the tunnel even when banking tilts a
                // side panel below horizontal. World-up filtering would leave
                // those visible panels without a collider; the signed sweep
                // below already rejects approaches from outside the arch.
                if normal == Vec3::ZERO {
                    continue;
                }
                let start = (before - origin).dot(normal);
                let end = (after - origin).dot(normal);
                if start > 0.001 || end <= 0.0 || end <= start {
                    continue;
                }
                let fraction = (-start / (end - start)).max(0.0);
                if hit.is_some_and(|(best, _)| fraction >= best) {
                    continue;
                }
                let crossing = before.lerp(after, fraction) - origin;
                let uu = u.length_squared();
                let vv = v.length_squared();
                let uv = u.dot(v);
                let denominator = uu * vv - uv * uv;
                if denominator <= 0.0000001 {
                    continue;
                }
                let along_u = (crossing.dot(u) * vv - crossing.dot(v) * uv) / denominator;
                let along_v = (crossing.dot(v) * uu - crossing.dot(u) * uv) / denominator;
                if along_u >= -0.0001 && along_v >= -0.0001 && along_u + along_v <= 1.0001 {
                    hit = Some((fraction, normal));
                }
            }
        }
    }
    hit.map(|(fraction, normal)| (before.lerp(after, fraction), normal))
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
    fn interior_progress_remains_continuous_between_close_sample_edges() {
        for (origin, approach_segments) in [(0, 0), (10000, 0), (10000, 4), (10000, 8)] {
            let mut track = Track::parse(&format!(
                "start {origin} 0 {origin}\n{}straight 20",
                "straight 5000\n".repeat(approach_segments),
            ))
            .unwrap();
            // Dense tilt transitions can put both neighboring edges within
            // the 1 cm seam tolerance. Keep interior progress even far from
            // world zero, to within the precision of cumulative f32 distance.
            let start = approach_segments as f32 * 5000.0;
            let inserted = track.sample_at(start + 0.016);
            let index = track
                .samples
                .partition_point(|sample| sample.distance <= start);
            track.samples.insert(index, inserted);
            for offset in [0.004, 0.008, 0.012] {
                let distance = start + offset;
                let position = track.sample_at(distance).pos + Vec3::Y * RIDE_HEIGHT;
                let road = nearest_road(
                    &track,
                    position,
                    distance,
                    CONTACT_TOLERANCE,
                    track.ground_height(),
                    None,
                )
                .unwrap();
                let tolerance = (f32::EPSILON * distance.max(1.0)).max(0.000001);
                assert!(
                    (road.sample.distance - distance).abs() <= tolerance,
                    "origin {origin}, approach {start}, distance {distance}: got {}",
                    road.sample.distance,
                );
                assert!(road.plane_height.abs() < 0.000001);
            }
        }
    }

    #[test]
    fn reversed_cross_section_brackets_preserve_progress_and_contact() {
        let track = Track::parse(
            "width 40\nstraight 20 bank 60\nright 140 radius 22 rise -26.87807 bank -60\nstraight 20",
        )
        .unwrap();
        let (distance, contact) = track
            .samples
            .windows(2)
            .find_map(|pair| {
                let [a, b] = [pair[0], pair[1]];
                let distance = (a.distance + b.distance) * 0.5;
                let sample = track.sample_at(distance);
                let contact = sample.pos + sample.right * 16.0;
                let start =
                    horizontal(contact - a.pos).dot(horizontal(a.right).cross(Vec3::Y).normalize());
                let end =
                    horizontal(contact - b.pos).dot(horizontal(b.right).cross(Vec3::Y).normalize());
                (start < -0.05 && end > 0.05).then_some((distance, contact))
            })
            .expect("the banked hill must reverse its projected cross-sections");
        let road = nearest_road(
            &track,
            contact + Vec3::Y * RIDE_HEIGHT,
            distance,
            contact.y + CONTACT_TOLERANCE,
            track.ground_height(),
            None,
        )
        .unwrap();
        assert!((road.sample.distance - distance).abs() < 0.001);
        assert!((road.plane_height - contact.y).abs() < 0.001);
    }

    #[test]
    fn banking_hill_turns_match_rendered_cross_sections_across_the_deck() {
        for turn in ["right", "left"] {
            for rise in [-26.87807, 26.87807] {
                for bank in [-60, 60] {
                    let track = Track::parse(&format!(
                        "width 40\nstraight 20 bank {}\n{turn} 140 radius 22 rise {rise} bank {bank}\nstraight 20",
                        -bank
                    ))
                    .unwrap();
                    // This interior cross-section changes heading, grade,
                    // and bank together, well away from a segment boundary.
                    let distance = (track.samples[46].distance + track.samples[47].distance) * 0.5;
                    let sample = track.sample_at(distance);
                    for lateral in [-16.0, 0.0, 16.0] {
                        let contact = sample.pos + sample.right * lateral;
                        let road = nearest_road(
                            &track,
                            contact + Vec3::Y * RIDE_HEIGHT,
                            distance,
                            contact.y + CONTACT_TOLERANCE,
                            track.ground_height(),
                            None,
                        )
                        .unwrap();
                        assert!(
                            (road.sample.distance - distance).abs() < 0.001,
                            "lateral position changed progress on {turn}, rise {rise}, bank {bank}, lateral {lateral}: {} versus {distance}",
                            road.sample.distance
                        );
                        assert!(
                            (road.plane_height - contact.y).abs() < 0.001,
                            "contact differs from rendered deck on {turn}, rise {rise}, bank {bank}, lateral {lateral}: {} versus {}",
                            road.plane_height,
                            contact.y
                        );
                        assert!(road.sample.forward.dot(road.sample.right).abs() < 0.00001);
                    }
                }
            }
        }
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
    fn narrowing_guardrails_keep_cars_on_the_deck() {
        for kind in ["bridge", "tunnel"] {
            for bank in [-30, 0, 30] {
                for side in [-1.0, 1.0] {
                    for direction in [-1.0, 1.0] {
                        let (start_width, end_width, start) = if direction > 0.0 {
                            (40, 4, 29.9)
                        } else {
                            (4, 40, 31.1)
                        };
                        let track = Track::parse(&format!(
                            "width {start_width}\n{kind} 30 bank {bank}\nwidth {end_width}\n{kind} 1\n{kind} 100"
                        ))
                        .unwrap();
                        for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                            let mut car = Car::new(&track);
                            car.reset(&track, start);
                            let sample = track.sample_at(start);
                            car.position += sample.right * side * 10.0;
                            car.velocity = sample.forward * direction * 40.0;
                            let initial_x = car.position.x;
                            for _ in 0..10 {
                                car.update(&track, Control::default(), dt);
                                let sample = track.sample_at(car.distance);
                                let lateral = (car.position - Vec3::Y * RIDE_HEIGHT - sample.pos)
                                    .dot(sample.right);
                                assert!(
                                    lateral.abs() <= sample.width * 0.5 - CAR_HALF_WIDTH + 0.01,
                                    "escaped taper on {kind}, bank {bank}, side {side}, direction {direction}, dt {dt}: {car:?}"
                                );
                                assert!(car.grounded && !car.offroad);
                                if car.impact > 0.0 {
                                    break;
                                }
                            }
                            assert!(car.impact > 0.0);
                            assert!(car.velocity.z * direction < 0.0);
                            assert!((car.position.x - initial_x).abs() < 0.2);
                            assert!((car.position.z - car.distance).abs() < 0.01);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn taper_collision_precedes_an_open_road_or_gap_in_the_same_step() {
        for kind in ["bridge", "tunnel"] {
            for exit in ["straight 100", "gap 20\nstraight 100"] {
                let track =
                    Track::parse(&format!("width 40\n{kind} 30\nwidth 4\n{kind} 1\n{exit}"))
                        .unwrap();
                for side in [-1.0, 1.0] {
                    for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                        let mut car = Car::new(&track);
                        car.reset(&track, 30.8);
                        car.position.x = side * 4.0;
                        car.velocity = Vec3::Z * 40.0;
                        car.update(&track, Control::default(), dt);
                        assert!(car.position.z < 31.0);
                        assert!(car.velocity.z < 0.0 && car.impact > 0.0);
                        assert!(car.grounded && !car.offroad);
                    }
                }
            }
        }
    }

    #[test]
    fn taper_collision_precedes_leaving_the_track_in_the_same_step() {
        for kind in ["bridge", "tunnel"] {
            let track = Track::parse(&format!("width 40\n{kind} 30\nwidth 4\n{kind} 1")).unwrap();
            for side in [-1.0, 1.0] {
                for (start, dt) in [(30.8, STEP), (30.8, 1.0 / 60.0), (29.9, 1.0 / 30.0)] {
                    let mut car = Car::new(&track);
                    car.reset(&track, start);
                    car.position.x = side * 4.0;
                    car.velocity = Vec3::Z * 40.0;
                    car.update(&track, Control::default(), dt);
                    assert!(car.position.z < 31.0, "missed final {kind} rail: {car:?}");
                    assert!(car.velocity.z < 0.0 && car.impact > 0.0);
                    assert!(car.grounded && !car.offroad);
                }
            }
        }
    }

    #[test]
    fn endpoint_exits_hit_side_rails_in_both_directions_and_only_at_deck_height() {
        for kind in ["bridge", "tunnel"] {
            let track = Track::parse(&format!("{kind} 100")).unwrap();
            for direction in [-1.0, 1.0] {
                let start = if direction > 0.0 { 99.8 } else { 0.2 };
                for side in [-1.0, 1.0] {
                    for below_deck in [false, true] {
                        let mut car = Car::new(&track);
                        car.reset(&track, start);
                        car.position.x = side * 5.1;
                        if below_deck {
                            car.position.y = track.ground_height() + RIDE_HEIGHT;
                        }
                        car.velocity = vec3(side * 20.0, 0.0, direction * 40.0);
                        car.update(&track, Control::default(), STEP);
                        if below_deck {
                            assert!(car.position.x.abs() > 5.2);
                            assert!(!(0.0..=100.0).contains(&car.position.z));
                            assert_eq!(car.impact, 0.0);
                            assert!(car.grounded && car.offroad);
                        } else {
                            assert!(car.position.x.abs() <= 6.0 - CAR_HALF_WIDTH);
                            assert!((0.0..100.0).contains(&car.position.z));
                            assert!(car.velocity.x * side < 0.0 && car.impact > 0.0);
                            assert!(car.grounded && !car.offroad);
                        }
                    }
                }
                // The road still ends here: an unobstructed exit has no wall.
                let mut car = Car::new(&track);
                car.reset(&track, start);
                car.velocity = Vec3::Z * direction * 40.0;
                car.update(&track, Control::default(), STEP);
                assert!(!(0.0..=100.0).contains(&car.position.z));
                assert_eq!(car.impact, 0.0);
                assert!(!car.grounded);
            }
        }
    }

    #[test]
    fn guardrail_sweep_requires_the_height_of_the_crossed_deck() {
        let track = Track::parse("start 0 8 0\nbridge 100").unwrap();
        for side in [-1.0, 1.0] {
            let before = vec3(side * 4.8, 8.0 + RIDE_HEIGHT, 20.0);
            let after = vec3(side * 5.5, 8.0 + RIDE_HEIGHT, 20.5);
            let query = |position| {
                nearest_road(&track, position, 20.0, 9.0, track.ground_height(), None).unwrap()
            };
            let from = query(before);
            let to = query(after);
            assert!(guardrail_sweep(&track, from, to, before, after).is_some());
            assert!(
                guardrail_sweep(
                    &track,
                    from,
                    to,
                    before - Vec3::Y * 8.0,
                    after - Vec3::Y * 8.0
                )
                .is_none()
            );
        }
    }

    #[test]
    fn switching_to_a_disconnected_road_cannot_pull_a_car_inside_its_guardrail() {
        let track = Track::parse(
            "width 40\nbridge 100\nwidth 4\nbridge 1\nright 270 radius 30 kind bridge\nbridge 100",
        )
        .unwrap();
        let before = vec3(0.0, RIDE_HEIGHT, 67.0);
        let after = vec3(0.0, RIDE_HEIGHT, 67.5);
        let from = nearest_road(&track, before, 67.0, 1.0, track.ground_height(), None).unwrap();
        let to = nearest_road(
            &track,
            after,
            track.length - 70.0,
            1.0,
            track.ground_height(),
            None,
        )
        .unwrap();
        assert!(to.sample.distance - from.sample.distance > 12.0);
        assert!(guardrail_sweep(&track, from, to, before, after).is_none());
        let mut car = Car::new(&track);
        car.position = after;
        car.velocity = Vec3::Z * 20.0;
        car.resolve_guardrail(&track, Some(to), Some(from), before);
        assert_eq!(car.position, after);
        assert_eq!(car.velocity, Vec3::Z * 20.0);
        assert_eq!(car.impact, 0.0);
    }

    #[test]
    fn a_shared_seam_sweep_only_checks_its_adjacent_segments() {
        let mut track = Track::parse(
            "bridge 40\nright 180 radius 20 kind bridge\nbridge 40\nright 180 radius 20 kind bridge\nclose",
        )
        .unwrap();
        let before = vec3(4.8, RIDE_HEIGHT, 0.0);
        let after = vec3(5.5, RIDE_HEIGHT, 0.0);
        let from = nearest_road(&track, before, 0.0, 1.0, track.ground_height(), None).unwrap();
        assert_eq!(from.index, 0);
        let mut to = from;
        to.index = track.samples.len() - 2;
        to.sample.distance = track.length;
        // Put an unrelated rail just inside the actual seam rail. Traversing
        // the whole circuit on a zero-distance tie would hit this one first.
        track.samples[3].pos = vec3(-0.15, 0.0, -1.0);
        track.samples[4].pos = vec3(-0.15, 0.0, 1.0);
        let (hit, _, _) = guardrail_sweep(&track, from, to, before, after).unwrap();
        assert!((hit.x - (6.0 - CAR_HALF_WIDTH)).abs() < 0.001);
    }

    #[test]
    fn banked_hill_progress_matches_its_cross_section() {
        for bank in [-60, 60] {
            for rise in [-100, 100] {
                for heading in [0, 73] {
                    for road in ["bridge 200", "right 140 radius 90", "left 140 radius 90"] {
                        let track = Track::parse(&format!(
                            "start 0 0 0 {heading}\nstraight 50 bank {bank}\n{road} rise {rise}"
                        ))
                        .unwrap();
                        for distance in [120.2, 121.4, 130.5, track.samples[80].distance] {
                            let sample = track.sample_at(distance);
                            for lateral in [-5.0, 0.0, 5.0] {
                                let position =
                                    sample.pos + sample.right * lateral + Vec3::Y * RIDE_HEIGHT;
                                let road = nearest_road(
                                    &track,
                                    position,
                                    distance,
                                    position.y,
                                    track.ground_height(),
                                    None,
                                )
                                .unwrap();
                                assert!(
                                    (road.sample.distance - distance).abs() < 0.03,
                                    "bank {bank}, rise {rise}, heading {heading}, distance {distance}, lateral {lateral}, actual {}",
                                    road.sample.distance
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn short_banked_hill_contact_matches_its_cross_section() {
        for bank in [-60, 60] {
            let track = Track::parse(&format!(
                "width 40\nstraight 20 bank {bank}\nstraight 1 rise 0.6\nstraight 20"
            ))
            .unwrap();
            for pair in track.samples.windows(2) {
                for fraction in [0.25, 0.5, 0.75] {
                    let distance =
                        pair[0].distance + (pair[1].distance - pair[0].distance) * fraction;
                    let sample = track.sample_at(distance);
                    // Stay close to the centerline so neighboring road
                    // cross-sections cannot overlap this query's footprint.
                    for lateral in [-0.1, 0.1] {
                        let contact = sample.pos + sample.right * lateral;
                        let road = nearest_road(
                            &track,
                            contact + Vec3::Y * RIDE_HEIGHT,
                            distance,
                            contact.y + CONTACT_TOLERANCE,
                            track.ground_height(),
                            None,
                        )
                        .unwrap();
                        assert!((road.sample.distance - distance).abs() < 0.01);
                        assert!(
                            (road.plane_height - contact.y).abs() < 0.001,
                            "contact differs at bank {bank}, distance {distance}, lateral {lateral}: {} versus {}",
                            road.plane_height,
                            contact.y
                        );
                        assert!(road.sample.up.dot(road.sample.right).abs() < 0.0001);
                    }
                }
            }
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
    fn banked_tunnel_side_panels_block_outward_motion() {
        for (bank, panel) in [(-60, 1), (60, 8)] {
            let track = Track::parse(&format!("straight 50 bank {bank}\ntunnel 100")).unwrap();
            let sample = track.sample_at(80.0);
            let arch = |index: usize| {
                let angle = index as f32 / 10.0 * std::f32::consts::PI;
                sample.pos
                    + sample.right * angle.cos() * (sample.width * 0.5 + 0.65)
                    + sample.up * (1.0 + angle.sin() * 5.2)
            };
            let a = arch(panel);
            let b = arch(panel + 1);
            let roof = (a + b) * 0.5;
            let outward = (b - a).cross(sample.forward).normalize();
            assert!(outward.y < 0.0, "banking must tilt this panel downward");

            for tangential_speed in [0.0, 24.0] {
                let tangent = (Vec3::Y - outward * outward.y).normalize();
                let mut car = Car::new(&track);
                car.position = roof - Vec3::Y * 0.7 - outward * 0.1;
                car.velocity = outward * 24.0 + tangent * tangential_speed;
                car.grounded = false;
                car.distance = sample.distance;
                car.update(&track, Control::default(), STEP);
                assert!(
                    (car.position + Vec3::Y * 0.7 - roof).dot(outward) <= 0.001,
                    "escaped through the banked tunnel wall at bank {bank}: {car:?}"
                );
                assert!(car.impact > 0.0);
                assert!(
                    car.velocity.dot(outward) <= 0.001,
                    "collision response pushed back through the wall: {car:?}"
                );
                if tangential_speed > 0.0 {
                    assert!(
                        car.velocity.dot(tangent) > tangential_speed * 0.95,
                        "glancing collision scrubbed motion along the wall: {car:?}"
                    );
                }
            }

            // The same panel remains one-sided: an exterior approach must
            // not pull the chassis through the wall into the tunnel.
            assert!(
                tunnel_roof_sweep(
                    &track,
                    roof - Vec3::Y * 0.7 + outward * 0.1,
                    roof - Vec3::Y * 0.7 - outward * 0.1,
                )
                .is_none()
            );
        }
    }

    #[test]
    fn tunnel_roof_sweep_matches_arch_vertices_on_hills() {
        for bank in [-60, -30, 0, 30, 60] {
            for rise in [-60, 60] {
                let track =
                    Track::parse(&format!("straight 50 bank {bank}\ntunnel 100 rise {rise}"))
                        .unwrap();
                let sample = track.samples[40];
                let roof = sample.pos + sample.up * 6.2;
                let (hit, _) =
                    tunnel_roof_sweep(&track, roof - Vec3::Y * 0.75, roof - Vec3::Y * 0.65)
                        .unwrap();
                assert!(
                    (hit.y - roof.y).abs() < 0.001,
                    "wrong arch height on bank {bank}, rise {rise}: {}, expected {}",
                    hit.y,
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
    fn tunnel_roof_contact_precedes_leaving_its_final_segment() {
        for source in [
            "tunnel 100",
            "tunnel 100\nstraight 50",
            "tunnel 100\ngap 10\nstraight 50",
        ] {
            let track = Track::parse(source).unwrap();
            for direction in [-1.0, 1.0] {
                let mut car = Car::new(&track);
                car.reset(&track, if direction > 0.0 { 99.8 } else { 0.2 });
                car.position.y = 5.49;
                car.velocity = vec3(0.0, 20.0, 40.0 * direction);
                car.grounded = false;
                car.update(&track, Control::default(), STEP);
                assert!(
                    car.position.y + 0.7 <= 6.2 + 0.001,
                    "missed roof on exit: {car:?}"
                );
                assert!(car.velocity.y <= 0.0 && car.impact > 0.0);
                assert!((car.distance - car.position.z).abs() < 0.001);
            }
        }
    }

    #[test]
    fn tunnel_roof_does_not_extend_past_its_open_ends() {
        let track = Track::parse("tunnel 100").unwrap();
        for direction in [-1.0, 1.0] {
            let mut car = Car::new(&track);
            car.reset(&track, if direction > 0.0 { 99.99 } else { 0.01 });
            car.position.y = 5.4;
            car.velocity = vec3(0.0, 20.0, 40.0 * direction);
            car.grounded = false;
            car.update(&track, Control::default(), STEP);
            assert!(car.position.y + 0.7 > 6.2);
            assert_eq!(car.impact, 0.0);
        }
    }

    #[test]
    fn tunnel_roof_impact_cannot_award_an_unreached_finish() {
        let track = Track::parse("tunnel 100").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, track.finish_distance() - 0.1);
        car.position.y = 5.49;
        car.velocity = vec3(0.0, 20.0, 40.0);
        car.grounded = false;
        let mut race = crate::race::Race::new(None, car.distance);
        race.started = true;
        race.next_checkpoint = track.checkpoints.len();
        car.update(&track, Control::default(), STEP);
        assert!(car.impact > 0.0);
        assert!(car.position.z < track.finish_distance());
        assert!((car.distance - car.position.z).abs() < 0.001);
        assert!(
            race.update(&track, STEP, car.distance, !car.offroad)
                .is_none()
        );
        assert!(!race.finished);
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
    fn landing_precedes_leaving_a_deck_in_the_same_step() {
        for kind in ["bridge", "tunnel", "straight"] {
            for gap in [false, true] {
                let (source, start, end) = if gap {
                    (
                        format!("straight 20\ngap 10\n{kind} 100\ngap 10\nstraight 20"),
                        30.0,
                        130.0,
                    )
                } else {
                    (format!("{kind} 100"), 0.0, 100.0)
                };
                let track = Track::parse(&source).unwrap();
                for direction in [-1.0, 1.0] {
                    for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                        for lateral in [0.0, 10.0] {
                            if lateral != 0.0 && kind != "straight" {
                                continue;
                            }
                            let edge = if direction > 0.0 { end } else { start };
                            let mut car = Car::new(&track);
                            car.reset(&track, edge - direction * 0.1);
                            car.position.x = lateral;
                            let height = if lateral == 0.0 { 0.0 } else { -1.0 };
                            car.position.y = height + RIDE_HEIGHT + 0.01;
                            car.velocity = vec3(0.0, -5.0, direction * 40.0);
                            car.grounded = false;
                            car.update(&track, Control::default(), dt);
                            assert!((car.position.z - edge) * direction > 0.0);
                            assert!(!car.grounded && car.offroad);
                            assert!(
                                (car.position.y - height - RIDE_HEIGHT).abs() < 0.001,
                                "{source}, direction {direction}, dt {dt}, lateral {lateral}: {car:?}"
                            );
                            assert!(car.velocity.y.abs() < 0.001 && car.impact > 0.0);
                            assert!(
                                car.velocity.z.abs() > 37.0,
                                "landing must retain forward momentum"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn landing_precedes_crossing_a_short_deck_in_one_step() {
        let track = Track::parse("straight 20\ngap 10\nbridge 1\ngap 10\nstraight 20").unwrap();
        for direction in [-1.0, 1.0] {
            let mut car = Car::new(&track);
            car.position = vec3(
                0.0,
                RIDE_HEIGHT + 0.05,
                if direction > 0.0 { 29.9 } else { 31.1 },
            );
            car.distance = car.position.z;
            car.velocity = vec3(0.0, -5.0, direction * 40.0);
            car.grounded = false;
            car.update(&track, Control::default(), 1.0 / 30.0);
            let edge = if direction > 0.0 { 31.0 } else { 30.0 };
            assert!((car.position.z - edge) * direction > 0.0);
            assert!(!car.grounded && car.offroad);
            assert!((car.position.y - RIDE_HEIGHT).abs() < 0.001, "{car:?}");
            assert!(car.velocity.y.abs() < 0.001 && car.impact > 0.0);
        }
    }

    #[test]
    fn landing_precedes_crossing_a_short_shoulder_in_one_step() {
        let track = Track::parse("straight 20\ngap 10\nstraight 1\ngap 10\nstraight 20").unwrap();
        for side in [-1.0, 1.0] {
            for direction in [-1.0, 1.0] {
                let mut car = Car::new(&track);
                car.position = vec3(
                    side * 10.0,
                    -1.0 + RIDE_HEIGHT + 0.05,
                    if direction > 0.0 { 29.9 } else { 31.1 },
                );
                car.distance = car.position.z;
                car.velocity = vec3(0.0, -5.0, direction * 40.0);
                car.grounded = false;
                car.update(&track, Control::default(), 1.0 / 30.0);
                let edge = if direction > 0.0 { 31.0 } else { 30.0 };
                assert!((car.position.z - edge) * direction > 0.0);
                assert!(!car.grounded && car.offroad);
                assert!(
                    (car.position.y - (-1.0 + RIDE_HEIGHT)).abs() < 0.02,
                    "{car:?}"
                );
                assert!(car.velocity.y.abs() < 0.5 && car.impact > 0.0, "{car:?}");
            }
        }
    }

    #[test]
    fn shoulder_corner_landings_are_swept_at_the_game_step() {
        let track = Track::parse("straight 20\ngap 10\nstraight 1\ngap 10\nstraight 20").unwrap();
        for side in [-1.0, 1.0] {
            for direction in [-1.0, 1.0] {
                let edge = if direction > 0.0 { 31.0 } else { 30.0 };
                let mut car = Car::new(&track);
                car.position = vec3(
                    side * 18.05,
                    track.ground_height() + RIDE_HEIGHT + 0.02,
                    edge - direction * 0.2,
                );
                car.distance = car.position.z;
                car.velocity = vec3(-side * 28.0, -5.0, direction * 28.0);
                car.grounded = false;
                car.update(&track, Control::default(), STEP);
                assert!((car.position.z - edge) * direction > 0.0);
                assert!(!car.grounded && car.offroad, "{car:?}");
                assert!(car.impact > 0.0 && car.velocity.y > 0.0, "{car:?}");
                assert!(car.position.y > track.ground_height() + RIDE_HEIGHT + 0.03);
            }
        }
    }

    #[test]
    fn shoulder_entry_sweeps_require_a_top_crossing_on_banks_tapers_and_hills() {
        for bank in [-30, 0, 30] {
            for (solid, reach, center) in [
                ("width 16\nstraight 1", 0.6, 30.5),
                ("straight 2 rise 1.2", 2.0, 31.2),
            ] {
                if bank != 0 && center != 30.5 {
                    continue;
                }
                for origin in [Vec3::ZERO, vec3(9000.0, 500.0, 9000.0)] {
                    let track = Track::parse(&format!(
                    "start {} {} {}\nstraight 20 bank {bank}\ngap 10\n{solid}\ngap 10\nstraight 20",
                    origin.x, origin.y, origin.z
                ))
                .unwrap();
                    let ground = track.ground_height();
                    let sample = track.sample_at(center);
                    for side in [-1.0, 1.0] {
                        for direction in [-1.0, 1.0] {
                            let lateral = side * 12.0;
                            let mut crossing = sample.pos + sample.right * lateral;
                            let road = nearest_road(
                                &track,
                                crossing,
                                center,
                                origin.y + 10.0,
                                ground,
                                None,
                            )
                            .unwrap();
                            let surface = road_surface(road, ground).unwrap();
                            assert!(surface.offroad);
                            crossing.y = surface.height + RIDE_HEIGHT;
                            for offset in [-5.0, 0.0, 5.0] {
                                let before = crossing + vec3(0.0, 2.0 + offset, -direction * reach);
                                let after = crossing + vec3(0.0, -2.0 + offset, direction * reach);
                                let road_at = |position: Vec3| {
                                    nearest_road(
                                        &track,
                                        position,
                                        center,
                                        origin.y + 10.0,
                                        ground,
                                        None,
                                    )
                                };
                                let hit = road_entry_contact_before_exit(
                                    &track,
                                    road_at(before).unwrap(),
                                    road_at(after),
                                    before,
                                    after,
                                    ground,
                                );
                                assert_eq!(
                                    hit.is_some(),
                                    offset == 0.0,
                                    "solid {solid}, bank {bank}, origin {origin:?}, side {side}, direction {direction}, offset {offset}"
                                );
                                if let Some((point, surface, fraction)) = hit {
                                    assert!(surface.offroad);
                                    assert!(
                                        point.distance(crossing) < 0.02,
                                        "{point:?}, {crossing:?}"
                                    );
                                    assert!((fraction - 0.5).abs() < 0.02);
                                    assert!(surface.normal.is_normalized());
                                    assert!((after - before).dot(surface.normal) < 0.0);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn bridge_corner_landings_are_swept_at_the_game_step() {
        let track = Track::parse("straight 20\ngap 10\nbridge 1\ngap 10\nstraight 20").unwrap();
        for side in [-1.0, 1.0] {
            for direction in [-1.0, 1.0] {
                for clearance in [-0.02, 0.005, 0.02, 0.05] {
                    let mut car = Car::new(&track);
                    let edge = if direction > 0.0 { 31.0 } else { 30.0 };
                    car.position =
                        vec3(side * 6.05, RIDE_HEIGHT + clearance, edge - direction * 0.2);
                    car.distance = car.position.z;
                    car.velocity = vec3(-side * 28.0, -5.0, direction * 28.0);
                    car.grounded = false;
                    car.update(&track, Control::default(), STEP);
                    let crossed_top = clearance == 0.02;
                    assert_eq!(
                        car.impact > 0.0,
                        crossed_top,
                        "side {side}, direction {direction}, clearance {clearance}: {car:?}"
                    );
                    if crossed_top {
                        assert!((car.position.y - RIDE_HEIGHT).abs() < 0.001, "{car:?}");
                        assert!(car.velocity.y.abs() < 0.001);
                    } else {
                        assert!(car.velocity.y < -5.0);
                    }
                    assert!(!car.grounded && car.offroad);
                }
            }
        }
    }

    #[test]
    fn landing_precedes_crossing_an_open_endpoint_deck_in_one_step() {
        for direction in [-1.0, 1.0] {
            let track = Track::parse(if direction > 0.0 {
                "straight 1\ngap 10\nstraight 20"
            } else {
                "straight 20\ngap 1\nstraight 1"
            })
            .unwrap();
            for clearance in [-0.05, 0.01, 0.05, 0.2] {
                let mut car = Car::new(&track);
                let edge = if direction > 0.0 { 0.0 } else { 22.0 };
                car.position = vec3(0.0, RIDE_HEIGHT + clearance, edge - direction * 0.1);
                car.distance = edge;
                car.velocity = vec3(0.0, -5.0, direction * 40.0);
                car.grounded = false;
                car.update(&track, Control::default(), 1.0 / 30.0);
                let crossed_top = clearance == 0.05;
                assert_eq!(
                    car.impact > 0.0,
                    crossed_top,
                    "direction {direction}, clearance {clearance}: {car:?}"
                );
                if crossed_top {
                    assert!((car.position.y - RIDE_HEIGHT).abs() < 0.001, "{car:?}");
                    assert!(car.velocity.y.abs() < 0.001);
                } else {
                    assert!(car.velocity.y < -5.0);
                }
                assert!(!car.grounded && car.offroad);
            }
        }
    }

    #[test]
    fn fast_falls_hit_open_bridge_corners_at_the_game_step() {
        let track = Track::parse("bridge 100").unwrap();
        for side in [-1.0, 1.0] {
            for direction in [-1.0, 1.0] {
                let mut car = Car::new(&track);
                let edge = if direction > 0.0 { 0.0 } else { 100.0 };
                car.position = vec3(side * 5.85, RIDE_HEIGHT + 0.6, edge - direction * 0.05);
                car.distance = edge;
                // A long fall crosses the deck and its edge within 1/120 s.
                // Both endpoints also miss the guardrail's height interval.
                car.velocity = vec3(side * 28.0, -160.0, direction * 28.0);
                car.grounded = false;
                car.update(&track, Control::default(), STEP);
                assert!((car.position.y - RIDE_HEIGHT).abs() < 0.001, "{car:?}");
                assert!(car.velocity.y.abs() < 0.001 && car.impact > 0.0, "{car:?}");
            }
        }
    }

    #[test]
    fn rising_bank_decks_hit_between_airborne_step_endpoints_without_launching() {
        let track =
            Track::parse("straight 20\ngap 10\nbridge 1 bank 60\ngap 10\nstraight 20").unwrap();
        for (start_z, height, dt) in [
            (29.9, 0.5, 1.0 / 30.0),
            (29.9, 0.7, 1.0 / 30.0),
            (29.9, 1.0, 1.0 / 30.0),
            (29.9, 1.3, 1.0 / 30.0),
            (29.9, 1.5, 1.0 / 30.0),
            (29.9, 1.8, 1.0 / 30.0),
            (30.8, 3.0, STEP),
        ] {
            let mut car = Car::new(&track);
            car.position = vec3(2.0, height + RIDE_HEIGHT, start_z);
            car.distance = start_z;
            car.velocity = vec3(0.0, -0.1, 40.0);
            car.grounded = false;
            car.update(&track, Control::default(), dt);
            assert!(car.impact > 0.0, "missed rising deck: {car:?}");
            assert!(
                car.position.z < 31.0,
                "the steep bank must block the approach: {car:?}"
            );
            assert!(
                car.velocity.length() <= 40.0,
                "collision injected energy: {car:?}"
            );
            let road = nearest_road(
                &track,
                car.position,
                car.distance,
                car.position.y,
                track.ground_height(),
                None,
            )
            .unwrap();
            assert!(
                car.position.y - RIDE_HEIGHT >= road.plane_height - 0.05,
                "passed below deck: {car:?}"
            );
        }
    }

    #[test]
    fn a_changing_bank_does_not_launch_centerline_landings() {
        for origin in [Vec3::ZERO, vec3(9000.0, 500.0, 9000.0)] {
            let track = Track::parse(&format!(
                "start {} {} {}\nstraight 20\ngap 10\nbridge 1 bank 60\ngap 10\nstraight 20",
                origin.x, origin.y, origin.z
            ))
            .unwrap();
            for clearance in [0.003, 0.005, 0.01] {
                let mut car = Car::new(&track);
                car.position = origin + vec3(0.0, RIDE_HEIGHT + clearance, 29.9);
                car.distance = 29.9;
                car.velocity = vec3(0.0, -0.1, 40.0);
                car.grounded = false;
                car.update(&track, Control::default(), 1.0 / 30.0);
                assert!(car.impact > 0.0, "missed centerline landing: {car:?}");
                assert!(
                    car.velocity.y.abs() < 0.5,
                    "bank injected a centerline launch: {car:?}"
                );
                assert!(
                    car.velocity.z > 39.0,
                    "level centerline blocked forward motion: {car:?}"
                );
                assert!(
                    (car.position.y - origin.y - RIDE_HEIGHT).abs() < 0.02,
                    "centerline height changed: {car:?}"
                );
            }
        }
    }

    #[test]
    fn flying_above_a_twisting_deck_does_not_create_a_landing() {
        for bank in [-60, 60] {
            let track = Track::parse(&format!(
                "straight 20\ngap 10\nbridge 1 bank {bank}\ngap 10\nstraight 20"
            ))
            .unwrap();
            for start_z in [30.8, 30.9] {
                for clearance in [0.005, 0.01] {
                    for vertical_speed in [0.0, -0.1] {
                        let mut car = Car::new(&track);
                        car.position = vec3(0.0, RIDE_HEIGHT + clearance, start_z);
                        car.distance = start_z;
                        car.velocity = vec3(0.0, vertical_speed, 40.0);
                        car.grounded = false;
                        let expected_y = car.position.y + (vertical_speed - GRAVITY * STEP) * STEP;
                        car.update(&track, Control::default(), STEP);
                        assert_eq!(car.impact, 0.0, "false deck contact: {car:?}");
                        assert!(
                            (car.position.y - expected_y).abs() < 0.00001,
                            "flight changed: {car:?}"
                        );
                        assert!(!car.grounded);
                    }
                }
            }
        }
    }

    #[test]
    fn a_departure_landing_must_cross_the_top_before_the_edge() {
        let track = Track::parse("bridge 100\ngap 10\nstraight 100").unwrap();
        for clearance in [-0.01, 0.04] {
            let mut car = Car::new(&track);
            car.reset(&track, 99.9);
            // Already beneath the deck, or still above it when passing the
            // edge: neither motion hits the finite top before the gap.
            car.position.y += clearance;
            car.velocity = vec3(0.0, -5.0, 40.0);
            car.grounded = false;
            car.update(&track, Control::default(), STEP);
            assert!(car.position.z > 100.0);
            assert!(!car.grounded && car.offroad);
            assert!(car.velocity.y < -5.0 && car.impact == 0.0, "{car:?}");
        }
    }

    #[test]
    fn lateral_shoulder_departure_detects_only_hits_inside_its_footprint() {
        let track = wide_straight();
        let ground = track.ground_height();
        for side in [-1.0, 1.0] {
            for clearance in [0.01, 0.04] {
                // The shoulder ends at +/-32 m, slopes down at 1:4, and
                // loses support after one quarter of this lateral sweep.
                let before = vec3(side * 31.9, ground + 0.025 + RIDE_HEIGHT + clearance, 50.0);
                let after = before + vec3(side * 0.4, -0.15, 0.0);
                let road_at =
                    |point: Vec3| nearest_road(&track, point, 50.0, point.y, ground, None);
                let hit = road_contact_before_exit(
                    &track,
                    road_at(before).unwrap(),
                    road_at(after),
                    before,
                    after,
                    ground,
                );
                assert_eq!(hit.is_some(), clearance == 0.01);
                if let Some((crossing, _, fraction)) = hit {
                    assert!(crossing.x.abs() < 32.0);
                    assert!((0.0..0.25).contains(&fraction));
                }
            }
        }
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
    fn falling_short_of_a_flat_landing_cannot_snap_up_through_its_edge() {
        let track = Track::parse("straight 10\ngap 10\nbridge 100").unwrap();
        for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
            let mut car = Car::new(&track);
            // Cross deck height while still over the gap, then enter the
            // landing's footprint from below during the same physics step.
            car.position = vec3(0.0, RIDE_HEIGHT + 0.01, 20.0 - 45.0 * dt * 0.75);
            car.velocity = vec3(0.0, -15.0, 45.0);
            car.grounded = false;
            car.distance = car.position.z;
            car.update(&track, Control::default(), dt);
            assert!(car.position.z > 20.0);
            assert!(
                !car.grounded,
                "pulled through the landing edge at dt {dt}: {car:?}"
            );
            assert!(car.position.y < RIDE_HEIGHT);
        }
    }

    #[test]
    fn shoulder_landings_require_crossing_the_top_within_its_footprint() {
        for intervening in ["gap", "bridge", "tunnel"] {
            let track =
                Track::parse(&format!("straight 10\n{intervening} 10\nstraight 100")).unwrap();
            for direction in [-1.0, 1.0] {
                let edge = if direction > 0.0 { 20.0 } else { 10.0 };
                for side in [-1.0, 1.0] {
                    for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                        for lands_on_top in [false, true] {
                            let mut car = Car::new(&track);
                            // At x = +/-10 the shoulder is one metre below its
                            // deck. Enter its footprint partway through this
                            // step, either before or after crossing its height.
                            let clearance = if lands_on_top { 15.0 * dt * 0.9 } else { 0.01 };
                            car.position = vec3(
                                side * 10.0,
                                RIDE_HEIGHT - 1.0 + clearance,
                                edge - direction * 45.0 * dt * 0.75,
                            );
                            car.velocity = vec3(0.0, -15.0, direction * 45.0);
                            car.grounded = false;
                            car.distance = car.position.z;
                            car.update(&track, Control::default(), dt);
                            assert!((car.position.z - edge) * direction > 0.0);
                            assert_eq!(
                                car.grounded, lands_on_top,
                                "wrong shoulder contact after {intervening}, direction {direction}, side {side}, dt {dt}: {car:?}"
                            );
                            if lands_on_top {
                                assert!((car.position.y - (RIDE_HEIGHT - 1.0)).abs() < 0.001);
                            } else {
                                assert!(car.position.y < RIDE_HEIGHT - 1.0);
                            }
                        }
                    }
                }
            }
        }
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
    fn airborne_car_lands_on_a_rising_bank_transition() {
        for bank in [-30.0_f32, 30.0] {
            for direction in [-1.0, 1.0] {
                for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                    let track = Track::parse(&format!(
                        "width 20\nstraight 100\nstraight 4 bank {bank}\nstraight 100"
                    ))
                    .unwrap();
                    // Evaluate the straight road and shoulder heights directly
                    // so a missed collider cannot also hide a failed assertion.
                    let surface_height = |car: &Car| {
                        let sample = track.sample_at(car.position.z);
                        let lateral = car.position.x / sample.right.x;
                        let half_width = sample.width * 0.5;
                        if lateral.abs() <= half_width {
                            sample.pos.y + lateral * sample.right.y
                        } else {
                            let edge =
                                sample.pos.y + lateral.signum() * half_width * sample.right.y;
                            let fraction = (lateral.abs() - half_width) / SHOULDER_WIDTH;
                            edge + (track.ground_height() - edge) * fraction
                        }
                    };
                    for offset in [5.0, 12.0] {
                        let mut car = Car::new(&track);
                        let start = if direction > 0.0 { 100.0 } else { 104.0 };
                        car.reset(&track, start);
                        car.position +=
                            track.sample_at(start).right * (offset * bank.signum() * direction);
                        car.position.y = surface_height(&car) + RIDE_HEIGHT + 0.2;
                        car.velocity = Vec3::Z * 40.0 * direction;
                        car.heading = car.velocity.x.atan2(car.velocity.z);
                        car.grounded = false;
                        for _ in 0..(0.2 / dt).ceil() as usize {
                            car.update(&track, Control::default(), dt);
                            assert!(
                                car.position.y - RIDE_HEIGHT >= surface_height(&car) - 0.05,
                                "passed through rising bank {bank}, direction {direction}, offset {offset}, dt {dt}: {car:?}"
                            );
                        }
                        assert!(car.grounded);
                        assert_eq!(car.offroad, offset > 10.0);
                    }
                }
            }
        }
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
    fn rapid_banking_transitions_cannot_swallow_a_supported_shoulder_car() {
        for bank in [-30.0_f32, 30.0] {
            for length in [1.0, 2.0, 4.0] {
                let track = Track::parse(&format!(
                    "straight 100\nstraight {length} bank {bank}\nstraight 100"
                ))
                .unwrap();
                let ground = track.ground_height();
                for direction in [-1.0, 1.0] {
                    for speed in [20.0, 40.0] {
                        for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                            let start = if direction > 0.0 {
                                99.0
                            } else {
                                101.0 + length
                            };
                            let mut car = Car::new(&track);
                            car.reset(&track, start);
                            car.position.x = 10.0 * bank.signum() * direction;
                            let surface_at = |car: &Car| {
                                let road = nearest_road(
                                    &track,
                                    car.position,
                                    car.distance,
                                    10.0,
                                    ground,
                                    None,
                                )
                                .unwrap();
                                road_surface(road, ground).unwrap()
                            };
                            car.position.y = surface_at(&car).height + RIDE_HEIGHT;
                            car.heading = if direction > 0.0 {
                                0.0
                            } else {
                                std::f32::consts::PI
                            };
                            car.velocity = Vec3::Z * speed * direction;
                            for _ in 0..(0.5 / dt).ceil() as usize {
                                car.update(&track, Control::default(), dt);
                                let surface = surface_at(&car);
                                assert!(
                                    car.position.y - RIDE_HEIGHT >= surface.height - 0.05,
                                    "penetrated shoulder at bank {bank}, length {length}, direction {direction}, speed {speed}, dt {dt}: {car:?}"
                                );
                                assert!(car.grounded && car.offroad);
                            }
                            assert!((car.distance - start) * direction > length + 1.0);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn shoulder_recovery_cannot_snap_across_a_gap_or_bridge() {
        for intervening in ["gap", "bridge", "tunnel"] {
            let track = Track::parse(&format!(
                "straight 100\n{intervening} 1 rise 0.6\nstraight 100"
            ))
            .unwrap();
            let mut car = Car::new(&track);
            car.reset(&track, 99.9);
            car.position.x = 10.0;
            let road = nearest_road(
                &track,
                car.position,
                car.distance,
                10.0,
                track.ground_height(),
                None,
            )
            .unwrap();
            car.position.y =
                road_surface(road, track.ground_height()).unwrap().height + RIDE_HEIGHT;
            let initial_height = car.position.y;
            car.velocity = Vec3::Z * 40.0;
            car.update(&track, Control::default(), 1.0 / 30.0);
            assert!(car.position.z > 101.0);
            assert!(
                !car.grounded && car.offroad,
                "snapped across {intervening}: {car:?}"
            );
            assert!(car.position.y <= initial_height + 0.01);
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
    fn ordinary_contact_cannot_step_up_across_a_short_gap() {
        for rise in [0.05, 0.2] {
            let track =
                Track::parse(&format!("straight 100\ngap 1 rise {rise}\nstraight 100")).unwrap();
            for dt in [STEP, 1.0 / 30.0] {
                let mut car = Car::new(&track);
                car.reset(&track, 99.9);
                let initial_height = car.position.y;
                car.velocity = Vec3::Z * 40.0;
                for _ in 0..(0.3 / dt) as usize {
                    car.update(&track, Control::default(), dt);
                    assert!(
                        !car.grounded,
                        "stepped up {rise} m through the landing at dt {dt}: {car:?}"
                    );
                    assert!(car.position.y <= initial_height + 0.001);
                }
                assert!(car.position.z > 101.0, "the car must reach the landing");
            }
        }
    }

    #[test]
    fn deck_recovery_cannot_snap_across_a_gap_to_a_higher_landing() {
        let track = Track::parse("straight 100\ngap 1 rise 0.6\nstraight 100").unwrap();
        for airborne in [false, true] {
            let mut car = Car::new(&track);
            car.reset(&track, 99.9);
            if airborne {
                car.position.y += 0.1;
                car.grounded = false;
            }
            let initial_height = car.position.y;
            car.velocity = Vec3::Z * 40.0;
            car.update(&track, Control::default(), 1.0 / 30.0);
            assert!(car.position.z > 101.0, "the step must cross the whole gap");
            assert!(!car.grounded && car.offroad);
            let expected_height = initial_height - if airborne { GRAVITY / 900.0 } else { 0.0 };
            assert!((car.position.y - expected_height).abs() < 0.001);
        }
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
    fn a_nearby_narrow_route_does_not_hide_the_road_beneath_the_car() {
        for adjacent in [
            "straight 20\nright 180 radius 12 kind bridge\nbridge 140",
            "straight 20\nright 180 radius 12 kind tunnel\ntunnel 140",
            "straight 20\nright 180 radius 12\nstraight 140",
            "straight 20 rise 3\nright 180 radius 9 kind gap\ngap 140\nstraight 20",
        ] {
            let track =
                Track::parse(&format!("width 40\nstraight 100\nwidth 4\n{adjacent}")).unwrap();
            for speed in [0.0, 15.0] {
                let mut car = Car::new(&track);
                car.reset(&track, 50.0);
                car.position.x = 19.0;
                car.velocity = Vec3::Z * speed;
                advance(&mut car, &track, Control::default(), 0.5);
                assert!(
                    car.grounded && !car.offroad,
                    "lost supporting wide road beside {adjacent}, speed {speed}: {car:?}"
                );
                assert!((car.position.y - RIDE_HEIGHT).abs() < 0.001);
                assert!((car.distance - car.position.z).abs() < 0.01);
                assert!((car.velocity.z - speed).abs() < 0.5);
            }
        }
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
