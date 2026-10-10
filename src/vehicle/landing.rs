//! Airborne contact with finite road surfaces, and the velocity response when
//! the car lands on a surface or crosses a crease between two of them.
//!
//! A single step can enter and leave a short deck or shoulder, so landings
//! are found by sweeping the motion against the surfaces of the intervening
//! route segments rather than by looking up the road beneath the endpoint.

use super::contact::{
    crosses_connected_surface, guardrail_exit_road, nearest_road_with_tolerance,
    open_track_entry_roads, plane_height_at, route_walk,
};
use super::*;
use crate::track::{RoadSample, deck_twist, frame_change, shoulder_curve_error};

// A change of contact plane at a shallower incidence than this is part of one
// continuous surface (sampled hill curvature): the car keeps its speed. From
// the sharper limit on, a crease is an impact, such as an embankment's toe or
// crest, and the velocity component into the new surface is lost.
const SMOOTH_CREASE_SIN: f32 = 0.087; // about 5 degrees
const SHARP_CREASE_SIN: f32 = 0.259; // about 15 degrees

impl Car {
    /// A landing can precede leaving a deck or shoulder in this same step,
    /// including when the car first enters it during the step. The final
    /// road lookup alone sees only the gap or terrain beyond it. Resolve that
    /// earlier contact before continuing the remaining motion, without
    /// applying input forces a second time.
    pub(super) fn land_before_exit(
        &mut self,
        step: &Step,
        sweep: &mut RoadSweep,
        max_surface_height: &mut f32,
        new_road: &mut Option<RoadPoint>,
    ) {
        let Step {
            track,
            dt,
            ground_height,
            before,
            old_road,
            ..
        } = *step;
        let Some(((crossing, surface, fraction), entering)) = earliest_contact_before_exit(
            track,
            old_road,
            *new_road,
            before,
            self.position,
            self.distance,
            ground_height,
        ) else {
            return;
        };
        self.resolve_landing(surface);
        if entering {
            // A twisting deck can be nearly vertical along the approach.
            // Its local normal blocks motion into the deck without
            // converting all forward speed into an artificial launch.
            self.velocity -= surface.normal * self.velocity.dot(surface.normal);
            *sweep = RoadSweep {
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
            self.velocity = landing_velocity(self.velocity, surface.normal);
        }
        self.position = crossing + self.velocity * (dt * (1.0 - fraction));
        *max_surface_height =
            (before.y - RIDE_HEIGHT).max(self.position.y - RIDE_HEIGHT) + CONTACT_TOLERANCE;
        *new_road = nearest_road(
            track,
            self.position,
            self.distance,
            *max_surface_height,
            ground_height,
            Some(*sweep),
        );
    }

    /// Move a grounded car onto a new contact plane without creating speed.
    /// The velocity is tangent to the previous plane; keeping its horizontal
    /// part and deriving a new vertical part would add v·tanθ of vertical
    /// speed at every upward crease (and v·tanθ rather than v·sinθ at a ramp
    /// lip). Gentle creases are sampled curvature and keep the speed; sharp
    /// ones are impacts and lose the component into the new surface.
    pub(super) fn follow_surface(&mut self, normal: Vec3) {
        let speed = self.velocity.length();
        let into = self.velocity.dot(normal);
        let tangent = tangent_component(self.velocity, normal);
        let tangent_speed = tangent.length();
        if speed <= 0.0001 || tangent_speed <= 0.0001 {
            self.velocity = tangent;
            return;
        }
        let incidence =
            ((into.abs() / speed) - SMOOTH_CREASE_SIN) / (SHARP_CREASE_SIN - SMOOTH_CREASE_SIN);
        let incidence = incidence.clamp(0.0, 1.0);
        let keep = 1.0 - incidence * incidence * (3.0 - 2.0 * incidence);
        let new_speed = tangent_speed + (speed - tangent_speed) * keep;
        self.velocity = tangent * (new_speed / tangent_speed);
        // Running into an embankment, or over its crest, is felt as a jolt.
        self.impact = self
            .impact
            .max(((speed - new_speed) / 11.0).clamp(0.0, 1.0));
    }

    pub(super) fn resolve_landing(&mut self, surface: Surface) {
        // Measure the speed into the surface, which is the vertical speed on
        // level ground. On a slope the vertical difference overstates it.
        let vertical_impact = self.velocity.dot(surface.normal).abs();
        self.impact = self.impact.max((vertical_impact / 11.0).clamp(0.0, 1.0));
        // A firm landing scrubs speed, but does not reset steering or
        // horizontal momentum and therefore still rewards alignment.
        let landing_loss = (vertical_impact - 2.5).max(0.0) * 0.014;
        self.velocity.x *= 1.0 - landing_loss.min(0.24);
        self.velocity.z *= 1.0 - landing_loss.min(0.24);
    }
}

/// The earliest proven top-surface contact between `before` and `after` on a
/// finite deck or shoulder that the motion leaves, or enters, during the
/// step; `true` marks an entry. `distance` is the car's current progress.
fn earliest_contact_before_exit(
    track: &Track,
    old_road: Option<RoadPoint>,
    new_road: Option<RoadPoint>,
    before: Vec3,
    after: Vec3,
    distance: f32,
    ground_height: f32,
) -> Option<((Vec3, Surface, f32), bool)> {
    old_road
        .or_else(|| {
            // Just outside a shoulder, the height-filtered lookup can
            // reject its elevated deck even though this step enters the
            // shoulder. Use it only to bound the triangle sweep; a real
            // one-sided surface crossing is still required for contact.
            nearest_road(track, before, distance, f32::INFINITY, ground_height, None)
        })
        .and_then(|from| {
            road_contact_before_exit(track, from, new_road, before, after, ground_height)
                .map(|contact| (contact, false))
                .or_else(|| {
                    road_entry_contact_before_exit(
                        track,
                        from,
                        new_road,
                        before,
                        after,
                        ground_height,
                    )
                    .map(|contact| (contact, true))
                })
        })
        .into_iter()
        .chain(
            open_track_entry_roads(track, before, after)
                .into_iter()
                .flatten()
                .filter_map(|from| {
                    // Outside an open start or finish there is no preceding
                    // road to seed the sweep. Either endpoint plane may be
                    // crossed away from its road, so select the earliest
                    // proven finite contact among both candidates.
                    road_entry_contact_before_exit(
                        track,
                        from,
                        new_road,
                        before,
                        after,
                        ground_height,
                    )
                    .map(|contact| (contact, true))
                }),
        )
        .min_by(|a, b| a.0.2.total_cmp(&b.0.2))
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
    let (delta, forward) = route_walk(track, from, destination)?;
    if delta.abs() > 12.0 {
        return None;
    }
    let segments = track.samples().len() - 1;
    let mut index = from.index;
    let mut hit: Option<(Vec3, Surface, f32)> = None;
    loop {
        let a = track.samples()[index];
        let b = track.samples()[index + 1];
        if a.kind != RoadKind::Gap {
            // Refine twisting strips so each triangle stays within 1 cm of
            // the interpolated road frame, including changing road width.
            let twist = deck_twist(a, b);
            let frame = frame_change(a, b);
            // A shoulder also twists on an ordinary hill: its inner edge
            // changes elevation while the terrain edge remains level. This
            // sweep bounds it more tightly than the rendered mesh does.
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
            // Refine curved shoulders by their slope, as the mesh does.
            let curve_error = if matches!(a.kind, RoadKind::Road | RoadKind::Ramp) {
                shoulder_curve_error(a, b, &[-1.0, 1.0], ground_height)
            } else {
                0.0
            };
            let steps = (twist / 0.01)
                .ceil()
                .max((frame / 0.05).ceil())
                .max((shoulder_height_change / 0.04).ceil())
                .max((curve_error / 0.01).sqrt().ceil())
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
        let a = track.samples()[road.index];
        let b = track.samples()[road.index + 1];
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

/// The vertical speed that keeps this horizontal velocity on a plane. Only
/// valid within one plane: at a change of plane it would keep the horizontal
/// speed and create vertical speed, so use `follow_surface` there.
pub(super) fn surface_vertical_speed(velocity: Vec3, normal: Vec3) -> f32 {
    // Tall embankments can be much steeper than the road's allowed grade.
    // Preserve their actual slope so tangential motion stays on the shoulder.
    -horizontal(velocity).dot(normal) / normal.y.max(0.0001)
}

/// Remove the velocity component along a contact normal, as an inelastic
/// impact does. The result is never faster than the input.
fn tangent_component(velocity: Vec3, normal: Vec3) -> Vec3 {
    velocity - normal * velocity.dot(normal)
}

/// The velocity after landing on a plane. A purely normal impulse would make
/// a car landing on a cross slope skid sideways; tire friction can instead
/// keep its horizontal velocity, as long as that is not the faster result
/// (keeping the horizontal speed on an upward slope would create speed).
pub(super) fn landing_velocity(velocity: Vec3, normal: Vec3) -> Vec3 {
    let projected = tangent_component(velocity, normal);
    let level = vec3(
        velocity.x,
        surface_vertical_speed(velocity, normal),
        velocity.z,
    );
    if level.length_squared() <= projected.length_squared() {
        level
    } else {
        projected
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::test_util::*;

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
    fn an_unrelated_endpoint_plane_cannot_hide_a_bridge_corner_landing() {
        for (turn, side) in [("left", -1.0), ("right", 1.0)] {
            for entering_finish in [false, true] {
                let final_length = if entering_finish { 99.95 } else { 100.05 };
                let track = Track::parse(&format!(
                    "bridge 100\n{turn} 180 radius 30 kind bridge\nbridge {final_length}"
                ))
                .unwrap();
                let endpoint = if entering_finish {
                    *track.samples().last().unwrap()
                } else {
                    track.samples()[0]
                };
                let mut car = Car::new(&track);
                car.position = endpoint.pos + vec3(side * 5.85, RIDE_HEIGHT + 0.6, -0.1);
                car.distance = endpoint.distance;
                car.velocity = vec3(side * 28.0, -160.0, 28.0);
                car.grounded = false;
                car.update(&track, Control::default(), STEP);
                assert!((car.position.y - RIDE_HEIGHT).abs() < 0.001, "{car:?}");
                assert!(car.velocity.y.abs() < 0.001 && car.impact > 0.0, "{car:?}");
            }
        }
    }

    #[test]
    fn an_overhead_endpoint_plane_cannot_hide_a_lower_bridge_corner_landing() {
        let track = Track::parse(
            "start 0 20 0\nbridge 20\nright 180 radius 8 kind bridge\nbridge 20\nright 180 radius 16 rise -20 kind bridge\nbridge 20\nright 180 radius 8 kind bridge\nbridge 19.95",
        ).unwrap();
        let endpoint = *track.samples().last().unwrap();
        let mut car = Car::new(&track);
        car.position = endpoint.pos + vec3(5.85, RIDE_HEIGHT + 0.6, -0.1);
        car.distance = endpoint.distance;
        car.velocity = vec3(28.0, -160.0, 28.0);
        car.grounded = false;
        car.update(&track, Control::default(), STEP);
        assert!((car.position.y - RIDE_HEIGHT).abs() < 0.01, "{car:?}");
        assert!(car.velocity.y.abs() < 0.01 && car.impact > 0.0, "{car:?}");
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
        let track = Track::parse(include_str!("../../tracks/alpine.track")).unwrap();
        let ramp = track
            .samples()
            .iter()
            .find(|s| s.kind == RoadKind::Ramp)
            .unwrap();
        let landing = track
            .samples()
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
                                // Driving on the embankment itself, not just
                                // beside it. A steep one is only climbed as
                                // far as the entry speed carries the car.
                                climbed |=
                                    car.grounded && car.position.y - RIDE_HEIGHT > ground + 0.1;
                            }
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
    fn coasting_up_a_shoulder_never_gains_speed_or_launches_off_the_deck() {
        for rise in [3, 6, 20] {
            for bank in [-30, 0, 30] {
                let track = Track::parse(&format!(
                    "straight 50 rise {rise} bank {bank}\nstraight 200"
                ))
                .unwrap();
                let sample = track.sample_at(70.0);
                let ground = track.ground_height();
                let half_width = sample.width * 0.5;
                for side in [-1.0, 1.0] {
                    for speed in [10.0, 20.0, 40.0] {
                        for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                            let mut car = Car::new(&track);
                            car.reset(&track, sample.distance);
                            car.position = sample.pos
                                + sample.right * side * (half_width + SHOULDER_WIDTH + 1.0);
                            car.position.y = ground + RIDE_HEIGHT;
                            car.heading = -side * std::f32::consts::FRAC_PI_2;
                            car.velocity = -Vec3::X * side * speed;
                            let energy = |car: &Car| {
                                0.5 * car.velocity.length_squared()
                                    + GRAVITY * (car.position.y - RIDE_HEIGHT - ground)
                            };
                            let initial_energy = energy(&car);
                            for _ in 0..(3.0 / dt) as usize {
                                car.update(&track, Control::default(), dt);
                                let context = format!(
                                    "rise {rise}, bank {bank}, side {side}, speed {speed}, dt {dt}: {car:?}"
                                );
                                // The car starts on the terrain, the lowest
                                // ground, so no reachable height allows a
                                // faster speed, nor any added energy.
                                assert!(
                                    car.velocity.length() <= speed + 0.01,
                                    "gained speed at {context}"
                                );
                                assert!(
                                    energy(&car) <= initial_energy + 0.5,
                                    "gained energy at {context}"
                                );
                                let lateral = car.position.x / sample.right.x;
                                if lateral.abs() <= half_width {
                                    let deck = sample.pos.y + lateral * sample.right.y;
                                    assert!(
                                        car.position.y - RIDE_HEIGHT - deck < 0.5,
                                        "launched off the shoulder crest at {context}"
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn a_shallow_shoulder_is_climbed_onto_the_deck_and_a_steep_one_rolls_back() {
        for (rise, reaches_deck) in [(3, true), (20, false)] {
            let track = Track::parse(&format!("straight 50 rise {rise}\nstraight 200")).unwrap();
            let sample = track.sample_at(70.0);
            let mut car = Car::new(&track);
            car.reset(&track, sample.distance);
            car.position = sample.pos + sample.right * (sample.width * 0.5 + SHOULDER_WIDTH + 1.0);
            car.position.y = track.ground_height() + RIDE_HEIGHT;
            car.heading = -std::f32::consts::FRAC_PI_2;
            car.velocity = -Vec3::X * 20.0;
            let mut on_deck = false;
            for _ in 0..360 {
                car.update(&track, Control::default(), STEP);
                on_deck |= car.grounded && !car.offroad;
                if on_deck {
                    break;
                }
                assert!(car.grounded, "left the embankment: {car:?}");
            }
            assert_eq!(on_deck, reaches_deck, "rise {rise}: {car:?}");
            if !reaches_deck {
                // Tire grip cannot hold the car on a steep embankment.
                assert!(car.position.x > sample.width * 0.5 + 2.0, "{car:?}");
            }
        }
    }

    #[test]
    fn ramp_take_off_follows_the_lip_without_gaining_energy() {
        let track =
            Track::parse("straight 40\nramp 20 rise 3\ngap 14 rise -3\nstraight 180").unwrap();
        for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
            let mut car = Car::new(&track);
            car.reset(&track, 35.0);
            car.velocity = vec3(0.0, 0.0, 29.0);
            let energy = |car: &Car| {
                0.5 * car.velocity.length_squared() + GRAVITY * (car.position.y - RIDE_HEIGHT)
            };
            let mut previous = energy(&car);
            for _ in 0..(1.5 / dt) as usize {
                let was_grounded = car.grounded;
                car.update(&track, Control::default(), dt);
                // Climbing the ramp only converts speed into height.
                assert!(energy(&car) <= previous + 0.05, "dt {dt}: {car:?}");
                previous = energy(&car);
                if was_grounded && !car.grounded {
                    // The take-off speed is the speed along the lip, |v|·sinθ
                    // upward, not the horizontal speed times tanθ.
                    let lip = track.sample_at(60.0).forward;
                    let direction = car.velocity.normalize();
                    assert!(direction.dot(lip) > 0.9995, "dt {dt}: {car:?}");
                    break;
                }
            }
            assert!(!car.grounded, "no take-off at dt {dt}");
        }
    }
}
