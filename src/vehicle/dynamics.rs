//! Driver inputs and the forces that move the chassis: a two-axle bicycle
//! model whose tires share their grip between drive or braking and
//! cornering, load transfer between the axles, ballistic flight, and the
//! body's visual pitch and roll.

use super::landing::surface_vertical_speed;
use super::*;

const FRONT_ARM: f32 = 1.36;
const REAR_ARM: f32 = 1.29;
const INERTIA_PER_MASS: f32 = 2.45;
const ROAD_FRICTION: f32 = 1.18;
const FRONT_CORNERING_STIFFNESS: f32 = 56.0;
const REAR_CORNERING_STIFFNESS: f32 = 62.0;
// Keep a small cornering reserve under full keyboard pedals. Longitudinal
// and lateral forces still share the same friction circle.
const MAX_LONGITUDINAL_GRIP: f32 = 0.95;

impl Car {
    /// Smooth the pedals and steering toward the driver's inputs, and select
    /// reverse once the brake is held at rest. `speed` is the horizontal
    /// speed at the start of the step.
    pub(super) fn apply_controls(&mut self, control: Control, speed: f32, dt: f32) {
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
    }

    /// Apply this step's forces and move the car along its new velocity: tire
    /// forces on the surface it stood on, or gravity and drag in flight.
    pub(super) fn integrate(&mut self, step: &Step, handbrake: bool) {
        let dt = step.dt;
        let (sin_heading, cos_heading) = self.heading.sin_cos();
        let forward = vec3(sin_heading, 0.0, cos_heading);
        let right = vec3(cos_heading, 0.0, -sin_heading);

        if step.was_grounded {
            self.ground_dynamics(forward, right, step.old_surface, handbrake, dt);
            self.velocity.y = surface_vertical_speed(self.velocity, step.old_surface.normal);
        } else {
            self.velocity.y -= GRAVITY * dt;
            let air_drag = horizontal(self.velocity) * (0.0015 * step.speed * dt);
            self.velocity -= air_drag;
            self.yaw_rate *= (-0.30 * dt).exp();
            self.slip += (0.0 - self.slip) * (1.0 - (-3.0 * dt).exp());
        }
        self.heading = wrap_angle(self.heading + self.yaw_rate * dt);
        self.position += self.velocity * dt;
    }

    /// Ease the body's pitch and roll toward the contact plane, with a little
    /// cornering lean, or toward the flight path in the air. `speed` is the
    /// horizontal speed at the start of the step.
    pub(super) fn update_attitude(&mut self, surface: Surface, speed: f32, dt: f32) {
        if self.grounded {
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
        let gravity = vec3(0.0, -GRAVITY, 0.0);
        let slope_gravity = horizontal(gravity - surface.normal * gravity.dot(surface.normal));
        let front_demand = -braking * 0.65 * brake_sign;
        // Locked rear wheels resist the slope as well as the motion, so a
        // car parked on a hill stays put as far as the rear tires can hold.
        let rear_demand = engine
            - braking * 0.35 * brake_sign
            - if handbrake {
                (longitudinal / dt + slope_gravity.dot(forward)).clamp(-8.0, 8.0)
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
        let applied_long = total_long - drag - rolling_resistance;
        let acceleration = forward * applied_long + right * total_side + slope_gravity;
        self.velocity += acceleration * dt;
        // Load transfer follows the applied drive, brake and resistance
        // forces, not the kinematic acceleration. Gravity acts at the center
        // of mass and has no pitch moment about it, so coasting downhill does
        // not unload the rear; the tire force holding a car on a hill does.
        self.longitudinal_acceleration +=
            (applied_long - self.longitudinal_acceleration) * (1.0 - (-9.0 * dt).exp());

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::test_util::*;

    fn body_slip_degrees(car: &Car) -> f32 {
        let forward = vec3(car.heading.sin(), 0., car.heading.cos());
        let right = vec3(car.heading.cos(), 0., -car.heading.sin());
        car.velocity
            .dot(right)
            .atan2(car.velocity.dot(forward))
            .to_degrees()
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
    fn handbrake_holds_a_stopped_car_on_a_hill() {
        let track = Track::parse("straight 100\nstraight 100 rise 20\nstraight 100").unwrap();
        let sample = track.sample_at(150.0);
        assert!(sample.forward.y / horizontal(sample.forward).length() > 0.29);
        for heading in [0.0, std::f32::consts::PI] {
            let mut car = Car::new(&track);
            car.reset(&track, sample.distance);
            car.heading = heading;
            let start = car.position;
            advance(
                &mut car,
                &track,
                Control {
                    handbrake: true,
                    ..Control::default()
                },
                5.0,
            );
            assert!(
                car.position.distance(start) < 0.01,
                "crept {} m facing {heading}: {car:?}",
                car.position.distance(start)
            );
            assert!(car.grounded && !car.offroad);
        }
    }

    #[test]
    fn coasting_downhill_does_not_shift_load_like_braking() {
        let flat = Track::parse("straight 300").unwrap();
        let hill = Track::parse("straight 50\nstraight 200 rise -40\nstraight 50").unwrap();
        let coast = |track: &Track| {
            let mut car = Car::new(track);
            car.reset(track, 140.0);
            car.velocity = track.sample_at(140.0).forward * 15.0;
            advance(&mut car, track, Control::default(), 0.6);
            car
        };
        let on_flat = coast(&flat);
        let on_hill = coast(&hill);
        // Both cars feel only rolling and air resistance; gravity speeds the
        // hill car up but has no pitch moment about its center of mass.
        assert!(on_hill.velocity.length() > on_flat.velocity.length() + 1.0);
        assert!(
            (on_hill.longitudinal_acceleration - on_flat.longitudinal_acceleration).abs() < 0.1,
            "hill {} versus flat {}",
            on_hill.longitudinal_acceleration,
            on_flat.longitudinal_acceleration
        );
    }
}
