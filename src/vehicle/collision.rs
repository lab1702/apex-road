//! Physical boundaries: guardrails on bridge and tunnel decks, and the tunnel
//! arch. Both are swept along the step's motion, and a car that hits one
//! slides along it for the rest of the step instead of sticking.

use super::contact::{guardrail_exit_road, open_track_entry_roads, route_walk};
use super::*;
use crate::track::{
    RAIL_FACE_OFFSET, RoadSample, TUNNEL_ARCH_PANELS, TUNNEL_ARCH_REACH, TUNNEL_ARCH_RISE,
    TUNNEL_ARCH_SIDE_CLEARANCE, TUNNEL_ARCH_SPRING, tunnel_arch_angle,
};

const CAR_HALF_WIDTH: f32 = 0.85;

impl Car {
    /// The tunnel roof is also a physical boundary; falling onto a roof from
    /// outside is intentionally not treated as driving on its deck.
    pub(super) fn resolve_tunnel_roof(&mut self, step: &Step, max_surface_height: f32) {
        let Step {
            track,
            ground_height,
            before,
            ..
        } = *step;
        let Some((hit, normal)) = tunnel_roof_sweep(track, before, self.position) else {
            return;
        };
        let unconstrained = self.position;
        self.position = hit - Vec3::Y * 0.7 - normal * 0.0001;
        let first_stop = self.position;
        self.resolve_roof_velocity(normal);
        // Slide the rest of this step along the arch rather than stopping at
        // the hit, which would make it sticky. Each further panel crossed by
        // the slide redirects it; between two panels (the arch's crown is a
        // ridge) it follows their crease.
        let mut remaining = unconstrained - self.position;
        let mut normal = normal;
        let mut previous: Option<Vec3> = None;
        for _ in 0..3 {
            let mut slide = remaining - normal * remaining.dot(normal).max(0.0);
            if let Some(previous) = previous
                && slide.dot(previous) > 0.0
            {
                let crease = previous.cross(normal).normalize_or_zero();
                slide = crease * remaining.dot(crease);
            }
            let target = self.position + slide;
            let Some((hit, next)) = tunnel_roof_sweep(track, self.position, target) else {
                self.position = target;
                break;
            };
            let stop = hit - Vec3::Y * 0.7 - next * 0.0001;
            remaining = target - stop;
            self.position = stop;
            self.resolve_roof_velocity(next);
            previous = Some(normal);
            normal = next;
        }
        // A slide that starts exactly on a panel edge can pass between the
        // triangles' tolerances. Over one step the arch's inside is convex,
        // so a slid position outside it is crossed by the direct path from
        // the step's start; keep the first contact instead.
        if tunnel_roof_sweep(track, before, self.position).is_some() {
            self.position = first_stop;
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
        self.follow_progress(road);
        let surface = contact_surface(road, max_surface_height, ground_height);
        self.offroad = surface.offroad;
        self.grounded &= (self.position.y - RIDE_HEIGHT - surface.height).abs() < 0.55;
    }

    fn resolve_roof_velocity(&mut self, normal: Vec3) {
        self.velocity -= normal * self.velocity.dot(normal).max(0.0);
        // Downward motion stays inside an upward-facing ceiling. On a
        // banked side panel that faces down, removing an upward tangent
        // would instead put outward velocity back into the wall.
        if normal.y >= 0.0 {
            self.velocity.y = self.velocity.y.min(0.0);
        }
    }

    pub(super) fn resolve_guardrail(
        &mut self,
        track: &Track,
        road: Option<RoadPoint>,
        previous_road: Option<RoadPoint>,
        before: Vec3,
        max_surface_height: f32,
        ground_height: f32,
    ) {
        // The barrier follows the banked road frame. Both its correction and
        // impulse must stay in the road plane, or an uphill impact launches
        // the car by retaining its old upward velocity after the rebound.
        if let Some((hit, normal, outward)) =
            swept_guardrail_hit(track, previous_road, road, before, self.position)
        {
            // Stop at the wall that was crossed. On a taper, shunting all
            // the way to the final narrow cross-section would teleport
            // the car sideways and leave its forward speed unchanged.
            let unconstrained = self.position;
            self.position = hit - outward * 0.001;
            let first_stop = self.position;
            self.guardrail_impulse(normal, outward);
            // Then slide the rest of this step along that wall, so steering
            // into a rail does not make it sticky. A further rail crossed by
            // the slide (the outside of a bend) redirects it again; a corner
            // that the new wall would push back into the old one (a taper)
            // stops the car there.
            let distance = self.distance;
            let query = |position| {
                nearest_road(
                    track,
                    position,
                    distance,
                    max_surface_height,
                    ground_height,
                    None,
                )
            };
            let mut remaining = unconstrained - self.position;
            let mut outward = outward;
            let mut previous: Option<Vec3> = None;
            for _ in 0..3 {
                let slide = remaining - outward * remaining.dot(outward).max(0.0);
                if previous.is_some_and(|previous| slide.dot(previous) > 0.0) {
                    break;
                }
                let target = self.position + slide;
                let Some((hit, normal, next)) = swept_guardrail_hit(
                    track,
                    query(self.position),
                    query(target),
                    self.position,
                    target,
                ) else {
                    self.position = target;
                    break;
                };
                let stop = hit - next * 0.001;
                remaining = target - stop;
                self.position = stop;
                self.guardrail_impulse(normal, next);
                previous = Some(outward);
                outward = next;
            }
            // As for the tunnel arch, never let a slide end beyond a rail
            // that the direct path from the step's start would cross.
            if self.position != first_stop
                && swept_guardrail_hit(
                    track,
                    previous_road,
                    query(self.position),
                    before,
                    self.position,
                )
                .is_some()
            {
                self.position = first_stop;
            }
            return;
        }
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
        let limit = rail_limit(road.sample.width);
        if road.lateral.abs() <= limit {
            return;
        }
        let side = road.lateral.signum();
        // Without a proven wall crossing, retain the local correction
        // for a car already touching the rail. An outside approach
        // must not teleport through the bridge to its inside edge.
        // Measure the prior horizontal footprint on the deck plane.
        // Airborne clearance otherwise moves this coordinate inward on
        // the downhill side of a bank even while the car stays outside.
        let old_offset = horizontal(before - road.sample.pos);
        let old_contact =
            old_offset - Vec3::Y * old_offset.dot(road.sample.up) / road.sample.up.y.max(0.15);
        let old_lateral = old_contact.dot(road.sample.right);
        if old_lateral.abs() > road.sample.width * 0.5 + SEGMENT_TOLERANCE {
            return;
        }
        let normal = road.sample.up;
        let lateral_axis = (road.sample.right - normal * road.sample.right.dot(normal)).normalize();
        self.position +=
            lateral_axis * ((side * limit - road.lateral) / lateral_axis.dot(road.sample.right));
        self.guardrail_impulse(normal, lateral_axis * side);
    }

    fn guardrail_impulse(&mut self, normal: Vec3, outward: Vec3) {
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

/// The chassis center's lateral limit on a deck with guardrails, placing the
/// body's side against the rendered rail face (see `world.rs`).
fn rail_limit(width: f32) -> f32 {
    (width * 0.5 + RAIL_FACE_OFFSET - CAR_HALF_WIDTH).max(0.5)
}

/// The earliest guardrail crossed between two positions. An inward path can
/// traverse a short endpoint deck and its rail in one step while neither
/// endpoint has a road contact. Seed that sweep from the crossed open
/// endpoint, as for finite road-surface landings.
fn swept_guardrail_hit(
    track: &Track,
    previous_road: Option<RoadPoint>,
    road: Option<RoadPoint>,
    before: Vec3,
    after: Vec3,
) -> Option<(Vec3, Vec3, Vec3)> {
    previous_road
        .into_iter()
        .chain(
            open_track_entry_roads(track, before, after)
                .into_iter()
                .flatten(),
        )
        .filter_map(|prior| {
            let destination =
                road.unwrap_or_else(|| guardrail_exit_road(track, prior, before, after));
            guardrail_sweep(track, prior, destination, before, after)
        })
        .min_by(|a, b| {
            a.0.distance_squared(before)
                .total_cmp(&b.0.distance_squared(before))
        })
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
    let (delta, forward) = route_walk(track, from, to)?;
    // Route selection can jump between nearby but disconnected roads. Such a
    // jump is not evidence that the intervening guardrails were traversed.
    if delta.abs() > 12.0 {
        return None;
    }
    let segments = track.samples.len() - 1;
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
                    sample.pos + sample.right * side * rail_limit(sample.width)
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

/// Check the finite roof panels crossed by the chassis top. A lookup at only
/// the destination misses a roof impact followed by an exit in the same step.
/// Triangles also retain the tunnel's actual opening instead of extending its
/// ceiling beyond the final cross-section.
fn tunnel_roof_sweep(track: &Track, before: Vec3, after: Vec3) -> Option<(Vec3, Vec3)> {
    #[cfg(test)]
    {
        use super::contact::index_check::{self, Mode};
        match index_check::mode() {
            Mode::Scan => return roof_sweep(track, before, after, false),
            Mode::Verify => {
                let found = roof_sweep(track, before, after, true);
                let bits = |hit: Option<(Vec3, Vec3)>| {
                    hit.map(|(at, normal)| [at, normal].map(|v| v.to_array().map(f32::to_bits)))
                };
                assert_eq!(
                    bits(found),
                    bits(roof_sweep(track, before, after, false)),
                    "indexed roof sweep from {before:?} to {after:?} differs from the full scan"
                );
                return found;
            }
            Mode::Indexed => {}
        }
    }
    roof_sweep(track, before, after, true)
}

/// [`tunnel_roof_sweep`], through the spatial index if `indexed`.
fn roof_sweep(track: &Track, before: Vec3, after: Vec3, indexed: bool) -> Option<(Vec3, Vec3)> {
    let before = before + Vec3::Y * 0.7;
    let after = after + Vec3::Y * 0.7;
    let path_min = before.min(after);
    let path_max = before.max(after);
    let mut hit: Option<(f32, Vec3)> = None;
    let mut sweep_segment = |segment: usize| {
        let a = track.samples[segment];
        let b = track.samples[segment + 1];
        if a.kind != RoadKind::Tunnel {
            return;
        }
        // A conservative bound is cheap even for a long, densely sampled route.
        let radius = a.width.max(b.width) * 0.5 + TUNNEL_ARCH_REACH;
        let bounds_min = a.pos.min(b.pos) - Vec3::splat(radius);
        let bounds_max = a.pos.max(b.pos) + Vec3::splat(radius);
        if path_max.cmplt(bounds_min).any() || path_min.cmpgt(bounds_max).any() {
            return;
        }
        let arch = |sample: RoadSample, index: usize| {
            let angle = tunnel_arch_angle(index);
            sample.pos
                + sample.right * angle.cos() * (sample.width * 0.5 + TUNNEL_ARCH_SIDE_CLEARANCE)
                + sample.up * (TUNNEL_ARCH_SPRING + angle.sin() * TUNNEL_ARCH_RISE)
        };
        for index in 0..TUNNEL_ARCH_PANELS {
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
    };
    // Only segments near the path can pass the bound above. Visiting them
    // in index order keeps the earliest of equal hits, as a full scan does.
    match track.segment_index().filter(|_| indexed) {
        Some(index) if path_min.is_finite() && path_max.is_finite() => {
            let reach = index.max_half_width() + TUNNEL_ARCH_REACH;
            for segment in index.segments_near(path_min.xz(), path_max.xz(), reach) {
                sweep_segment(segment);
            }
        }
        _ => (0..track.samples.len().saturating_sub(1)).for_each(sweep_segment),
    }
    hit.map(|(fraction, normal)| (before.lerp(after, fraction), normal))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::test_util::*;

    #[test]
    fn bridge_and_tunnel_guardrails_rebound_without_teleporting_or_stopping_forward_motion() {
        for kind in ["bridge", "tunnel"] {
            let track = Track::parse(&format!("start 0 8 0\n{kind} 160")).unwrap();
            let mut car = Car::new(&track);
            car.reset(&track, 20.0);
            car.position.x = 4.9;
            car.velocity = vec3(8.0, 0.0, 22.0);
            advance(&mut car, &track, Control::default(), 0.15);
            assert!(car.position.x <= rail_limit(12.0) + 0.01);
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
                                    lateral.abs() <= rail_limit(sample.width) + 0.01,
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
                        car.position.x = side * (rail_limit(12.0) - 0.05);
                        if below_deck {
                            car.position.y = track.ground_height() + RIDE_HEIGHT;
                        }
                        car.velocity = vec3(side * 20.0, 0.0, direction * 40.0);
                        car.update(&track, Control::default(), STEP);
                        if below_deck {
                            assert!(car.position.x.abs() > rail_limit(12.0) + 0.05);
                            assert!(!(0.0..=100.0).contains(&car.position.z));
                            assert_eq!(car.impact, 0.0);
                            assert!(car.grounded && car.offroad);
                        } else {
                            assert!(car.position.x.abs() <= rail_limit(12.0));
                            assert!(car.velocity.x * side < 0.0 && car.impact > 0.0);
                            // The rest of the step slides along the rail, so
                            // the car keeps its forward travel past the end.
                            assert!(
                                (car.position.z - start) * direction > 40.0 * STEP * 0.95,
                                "{car:?}"
                            );
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
        car.resolve_guardrail(
            &track,
            Some(to),
            Some(from),
            before,
            1.0,
            track.ground_height(),
        );
        assert_eq!(car.position, after);
        assert_eq!(car.velocity, Vec3::Z * 20.0);
        assert_eq!(car.impact, 0.0);
    }

    #[test]
    fn flying_just_outside_a_guardrail_does_not_teleport_onto_the_deck() {
        for kind in ["bridge", "tunnel"] {
            for bank in [-60, 0, 60] {
                for rise in [-20, 0, 20] {
                    let track =
                        Track::parse(&format!("straight 50 bank {bank}\n{kind} 100 rise {rise}"))
                            .unwrap();
                    let sample = track.sample_at(80.0);
                    for side in [-1.0, 1.0] {
                        for offset in [0.25, 0.75] {
                            for clearance in [-0.25, 0.0, 0.5] {
                                let mut car = Car::new(&track);
                                car.reset(&track, 80.0);
                                car.position +=
                                    sample.right * side * (6.0 + offset) + Vec3::Y * clearance;
                                car.velocity = sample.forward * 25.0;
                                car.grounded = false;
                                let before = car.position;
                                car.update(&track, Control::default(), STEP);
                                assert!(
                                    (car.position.x - before.x).abs() < 0.001,
                                    "pulled inside {kind}, bank {bank}, rise {rise}, side {side}, clearance {clearance}: {car:?}"
                                );
                                assert!(car.position.z > before.z);
                                assert!(!car.grounded && car.offroad);
                                assert_eq!(car.impact, 0.0);
                            }
                        }
                    }
                }
            }
        }
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
        assert!((hit.x - rail_limit(12.0)).abs() < 0.001);
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
            // The chassis top stops on the rendered arch. A tilted panel
            // lets the rest of the step slide along it, so check every panel
            // of the cross-section rather than the crown's height alone.
            let top = car.position + Vec3::Y * 0.7 - sample.pos;
            let point = vec2(top.dot(sample.right), top.dot(sample.up));
            let arch = |index: usize| {
                let angle = tunnel_arch_angle(index);
                vec2(
                    angle.cos() * (sample.width * 0.5 + TUNNEL_ARCH_SIDE_CLEARANCE),
                    TUNNEL_ARCH_SPRING + angle.sin() * TUNNEL_ARCH_RISE,
                )
            };
            let outside = (0..TUNNEL_ARCH_PANELS)
                .map(|index| {
                    let edge = arch(index + 1) - arch(index);
                    (point - arch(index)).dot(vec2(edge.y, -edge.x).normalize())
                })
                .fold(f32::MIN, f32::max);
            assert!(
                (-0.001..=0.001).contains(&outside),
                "wrong roof contact at bank {bank} ({outside}): {car:?}, roof {roof:?}"
            );
            assert!((top - (roof - sample.pos)).length() < 20.0 * STEP);
            assert!(car.velocity.y <= 0.0 && car.impact > 0.0);
        }
    }

    #[test]
    fn banked_tunnel_side_panels_block_outward_motion() {
        for (bank, panel) in [(-60, 1), (60, 8)] {
            let track = Track::parse(&format!("straight 50 bank {bank}\ntunnel 100")).unwrap();
            let sample = track.sample_at(80.0);
            let arch = |index: usize| {
                let angle = tunnel_arch_angle(index);
                sample.pos
                    + sample.right * angle.cos() * (sample.width * 0.5 + TUNNEL_ARCH_SIDE_CLEARANCE)
                    + sample.up * (TUNNEL_ARCH_SPRING + angle.sin() * TUNNEL_ARCH_RISE)
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
                let start = if direction > 0.0 { 99.8 } else { 0.2 };
                car.reset(&track, start);
                car.position.y = 5.49;
                car.velocity = vec3(0.0, 20.0, 40.0 * direction);
                car.grounded = false;
                car.update(&track, Control::default(), STEP);
                assert!(
                    car.position.y + 0.7 <= 6.2 + 0.001,
                    "missed roof on exit: {car:?}"
                );
                assert!(car.velocity.y <= 0.0 && car.impact > 0.0);
                // The rest of the step slides along the roof and out of the
                // tunnel. Progress follows the car wherever a route continues.
                assert!((car.position.z - start) * direction > 40.0 * STEP * 0.95);
                if direction > 0.0 && source != "tunnel 100" {
                    assert!((car.distance - car.position.z).abs() < 0.001);
                } else {
                    // Beyond an open end, progress stays on the course.
                    assert!((0.0..=100.0).contains(&car.distance));
                }
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
        // The car slides along the roof for the rest of the step, so timing
        // must follow its corrected position: short of the line at a low
        // speed, across it at a high one.
        for (speed, reaches) in [(10.0, false), (40.0, true)] {
            let mut car = Car::new(&track);
            car.reset(&track, track.finish_distance() - 0.1);
            car.position.y = 5.49;
            car.velocity = vec3(0.0, 20.0, speed);
            car.grounded = false;
            let mut race = crate::race::Race::new(None, car.distance);
            race.started = true;
            race.next_checkpoint = track.checkpoints.len();
            car.update(&track, Control::default(), STEP);
            assert!(car.impact > 0.0);
            assert_eq!(car.position.z >= track.finish_distance(), reaches);
            assert!((car.distance - car.position.z).abs() < 0.001);
            assert_eq!(
                race.update(&track, STEP, car.distance, !car.offroad)
                    .is_some(),
                reaches
            );
            assert_eq!(race.finished, reaches);
        }
    }

    #[test]
    fn an_unrelated_endpoint_plane_cannot_hide_a_guardrail_impact() {
        for kind in ["bridge", "tunnel"] {
            for (turn, side) in [("left", -1.0), ("right", 1.0)] {
                for entering_finish in [false, true] {
                    let final_length = if entering_finish { 99.95 } else { 100.05 };
                    let track = Track::parse(&format!(
                        "{kind} 100\n{turn} 180 radius 30 kind {kind}\n{kind} {final_length}"
                    ))
                    .unwrap();
                    let endpoint = if entering_finish {
                        *track.samples.last().unwrap()
                    } else {
                        track.samples[0]
                    };
                    let mut car = Car::new(&track);
                    car.position = endpoint.pos
                        + vec3(side * (rail_limit(12.0) - 0.1), RIDE_HEIGHT + 0.2, -0.09);
                    car.distance = endpoint.distance;
                    car.velocity = vec3(side * 28.0, 0.0, 28.0);
                    car.grounded = false;
                    car.update(&track, Control::default(), STEP);
                    assert!(car.velocity.x * side < 0.0 && car.impact > 0.0, "{car:?}");
                    // The car is held inside the crossed wall and slides
                    // along it for the rest of the step.
                    assert!(
                        (car.position.x - endpoint.pos.x) * side <= rail_limit(12.0),
                        "{car:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn entering_an_open_endpoint_cannot_skip_its_short_deck_guardrails() {
        for kind in ["bridge", "tunnel"] {
            for direction in [-1.0, 1.0] {
                let track = Track::parse(&if direction > 0.0 {
                    format!("{kind} 1\ngap 10\nstraight 20")
                } else {
                    format!("straight 20\ngap 1\n{kind} 1")
                })
                .unwrap();
                let endpoint = if direction > 0.0 { 0.0 } else { 22.0 };
                for side in [-1.0, 1.0] {
                    for clearance in [-3.0, 0.2, 8.0] {
                        let mut car = Car::new(&track);
                        car.position = vec3(
                            side * 4.9,
                            RIDE_HEIGHT + clearance,
                            endpoint - direction * 0.05,
                        );
                        car.distance = endpoint;
                        car.velocity = vec3(side * 20.0, -10.0, direction * 40.0);
                        car.grounded = false;
                        car.update(&track, Control::default(), 1.0 / 30.0);
                        let hit = clearance == 0.2;
                        assert_eq!(
                            car.position.x.abs() <= rail_limit(12.0),
                            hit,
                            "{kind}, direction {direction}, side {side}, clearance {clearance}: {car:?}"
                        );
                        assert_eq!(
                            car.velocity.x * side < 0.0,
                            hit,
                            "{kind}, direction {direction}, side {side}, clearance {clearance}: {car:?}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn entering_a_tapered_finish_hits_its_guardrail_at_the_game_step() {
        for kind in ["bridge", "tunnel"] {
            let track = Track::parse(&format!("width 4\n{kind} 30\nwidth 40\n{kind} 1")).unwrap();
            for side in [-1.0, 1.0] {
                let mut car = Car::new(&track);
                car.position = vec3(side * 19.0, RIDE_HEIGHT + 0.2, 31.05);
                car.distance = track.length;
                car.velocity = vec3(0.0, -1.0, -40.0);
                car.grounded = false;
                car.update(&track, Control::default(), STEP);
                assert!(car.impact > 0.0, "{kind}, side {side}: {car:?}");
                assert!(car.velocity.z > -30.0, "{kind}, side {side}: {car:?}");
            }
        }
    }

    #[test]
    fn steering_into_a_guardrail_slides_along_it_against_the_visible_rail() {
        for kind in ["bridge", "tunnel"] {
            let track = Track::parse(&format!("start 0 8 0\n{kind} 160")).unwrap();
            for side in [-1.0, 1.0] {
                for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                    let mut car = Car::new(&track);
                    car.reset(&track, 40.0);
                    car.position.x = side * (rail_limit(12.0) - 0.02);
                    car.velocity = vec3(side * 3.0, 0.0, 30.0);
                    let start = car.position;
                    car.update(&track, Control::default(), dt);
                    assert!(car.impact > 0.0, "{kind}, side {side}, dt {dt}: {car:?}");
                    // The body's side rests against the rendered rail face.
                    assert!(
                        (car.position.x.abs() + CAR_HALF_WIDTH - (6.0 + RAIL_FACE_OFFSET)).abs()
                            < 0.01
                    );
                    // The whole step's forward travel continues along the
                    // wall instead of stopping at the contact point.
                    assert!(
                        car.position.z - start.z > 30.0 * dt * 0.97,
                        "{kind}, side {side}, dt {dt}: {car:?}"
                    );
                    assert!(car.grounded && !car.offroad);
                }
            }
        }
    }
}
