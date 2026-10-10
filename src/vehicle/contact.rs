//! Elevation-aware road contact: which road segment lies beneath a position,
//! the deck or shoulder surface there, route walks between road points, and
//! whether the car stays on, lands on or leaves a surface during a step.

use super::landing::{landing_velocity, surface_vertical_speed};
use super::*;
use crate::track::RoadSample;

/// The road cross-section through a position: the interpolated sample, the
/// segment that contains it, and the position's place on its plane.
#[derive(Clone, Copy)]
pub(super) struct RoadPoint {
    pub(super) sample: RoadSample,
    /// The segment from `samples[index]` to `samples[index + 1]`.
    pub(super) index: usize,
    /// Offset along the sample's right axis on the road plane.
    pub(super) lateral: f32,
    /// Height of the road plane, extended sideways, beneath the position.
    pub(super) plane_height: f32,
    /// The motion crossed a rising surface from above during this step.
    pub(super) swept_contact: bool,
}

/// The start of a step's motion, for one-sided swept contact tests.
#[derive(Clone, Copy)]
pub(super) struct RoadSweep {
    pub(super) before: Vec3,
    /// A deck or shoulder beneath the preceding position, never an overpass.
    pub(super) support: Option<RoadPoint>,
}

/// A supporting plane: a deck, a shoulder, or the base terrain.
#[derive(Clone, Copy)]
pub(super) struct Surface {
    pub(super) height: f32,
    pub(super) normal: Vec3,
    pub(super) offroad: bool,
    /// Decks and shoulders have finite footprints; the base terrain does not.
    pub(super) finite: bool,
}

impl Car {
    /// Decide whether the car stays on its surface, lands on a new one, or is
    /// airborne after this step's motion, then follow that surface.
    pub(super) fn settle_on_surface(
        &mut self,
        step: &Step,
        new_road: Option<RoadPoint>,
        max_surface_height: f32,
    ) {
        let Step {
            track,
            dt,
            ground_height,
            speed,
            before,
            old_road,
            old_surface,
            was_grounded,
        } = *step;
        let surface = contact_surface(new_road, max_surface_height, ground_height);
        let required_vertical = surface_vertical_speed(self.velocity, surface.normal);
        let current_bottom = self.position.y - RIDE_HEIGHT;
        let previous_clearance =
            before.y - RIDE_HEIGHT - plane_height_at(surface, self.position, before);
        // A proven sweep has penetrated a rising surface. Its recovery takes
        // precedence over the cross-section velocity test, which does not
        // include longitudinal changes in bank. Falling-away surfaces still
        // use the ordinary clearance/velocity check and can launch the car.
        let clearance = current_bottom - surface.height;
        // Even a small height correction must follow a solid surface. A short
        // gap can be crossed in a single step, and its landing can lie within
        // the contact tolerance.
        let connected = || {
            old_surface.finite
                && old_road.zip(new_road).is_some_and(|(from, to)| {
                    crosses_connected_surface(track, from, to, before, self.position)
                })
        };
        // An embankment's top edge is not a launch ramp: climbing a shoulder
        // onto its own deck keeps the tires on the road, and the crest
        // absorbs the climb's vertical motion (see `follow_surface`). The car
        // moved along the shoulder this step, so it can overshoot the deck
        // by at most that step's rise.
        let shoulder_crest = old_surface.finite
            && old_surface.offroad
            && surface.finite
            && !surface.offroad
            && clearance > -CONTACT_TOLERANCE
            && clearance < CONTACT_TOLERANCE + self.velocity.y.max(0.0) * dt
            && connected();
        let on_same_surface = was_grounded
            && (new_road.is_some_and(|road| road.swept_contact)
                || shoulder_crest
                || (clearance.abs() < CONTACT_TOLERANCE
                    // A crest launches once it falls away faster than
                    // gravity can follow; the slack is a rate, so the
                    // threshold matches across fixed-step sizes.
                    && self.velocity.y - required_vertical <= (GRAVITY + 3.0) * dt
                    && (!surface.finite || connected())));
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
                self.velocity = landing_velocity(self.velocity, surface.normal);
            } else {
                self.follow_surface(surface.normal);
            }
            self.position.y = surface.height + RIDE_HEIGHT;
        }
        self.update_attitude(surface, speed, dt);
    }
}

/// Seed inward sweeps at open endpoints even when the preceding position has
/// no road lookup. Both candidates need finite collision tests: an unrelated
/// endpoint's infinite plane may be crossed first, away from its actual road.
pub(super) fn open_track_entry_roads(
    track: &Track,
    before: Vec3,
    after: Vec3,
) -> [Option<RoadPoint>; 2] {
    if track.closed || track.samples.len() < 2 {
        return [None, None];
    }
    let last = track.samples.len() - 1;
    [
        (0, track.samples[0], 1.0),
        (last - 1, track.samples[last], -1.0),
    ]
    .map(|(index, sample, direction)| {
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
        Some(RoadPoint {
            sample,
            index,
            lateral: (contact - sample.pos).dot(sample.right),
            plane_height,
            swept_contact: false,
        })
    })
}

/// An endpoint outside every road footprint can still have crossed a rail.
/// Follow the swept cross-sections from its preceding road so the collision
/// query includes short final segments without extending the road collider.
pub(super) fn guardrail_exit_road(
    track: &Track,
    mut road: RoadPoint,
    before: Vec3,
    after: Vec3,
) -> RoadPoint {
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

/// The signed progress from one road point to another, wrapped on a circuit,
/// and whether to walk the intervening segments forward. The same
/// cross-section may be reported from either adjacent segment; at the circuit
/// seam (or any sample boundary) a zero delta alone would choose a full lap
/// instead of the single boundary between them. Such a tie between segments
/// that are not adjacent has no route.
pub(super) fn route_walk(track: &Track, from: RoadPoint, to: RoadPoint) -> Option<(f32, bool)> {
    let mut delta = to.sample.distance - from.sample.distance;
    if track.closed {
        if delta > track.length * 0.5 {
            delta -= track.length;
        } else if delta < -track.length * 0.5 {
            delta += track.length;
        }
    }
    let segments = track.samples.len() - 1;
    let forward = if delta == 0.0 && from.index != to.index {
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
    Some((delta, forward))
}

/// The road cross-section beneath `position`. Overlapping roads are ranked by
/// distance outside their deck and shoulder footprints, height above their
/// surface, and continuity with `previous_distance`. Surfaces above
/// `max_surface_height` are skipped unless `sweep` proves the step crossed
/// them from above.
pub(super) fn nearest_road(
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

/// [`nearest_road`] with an explicit tolerance for snapping a position just
/// outside a segment's end cross-sections onto that segment.
pub(super) fn nearest_road_with_tolerance(
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

/// Check the actual swept path through intervening road cross-sections, rather
/// than treating proximity in world space as evidence of a connected surface.
pub(super) fn crosses_connected_surface(
    track: &Track,
    from: RoadPoint,
    to: RoadPoint,
    before: Vec3,
    after: Vec3,
) -> bool {
    let Some((_, forward)) = route_walk(track, from, to) else {
        return false;
    };
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

pub(super) fn contact_surface(
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

pub(super) fn road_surface(road: RoadPoint, ground_height: f32) -> Option<Surface> {
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

pub(super) fn plane_height_at(surface: Surface, origin: Vec3, point: Vec3) -> f32 {
    surface.height - horizontal(point - origin).dot(surface.normal) / surface.normal.y.max(0.0001)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::test_util::*;

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
    fn tall_embankment_contact_preserves_the_actual_slope_in_both_directions() {
        for bank in [-60, 0, 60] {
            let track =
                Track::parse(&format!("straight 500 rise 200 bank {bank}\nstraight 200")).unwrap();
            let ground = track.ground_height();
            let sample = track.sample_at(track.length - 100.0);
            for side in [-1.0, 1.0] {
                let edge_height = sample.pos.y + sample.right.y * side * sample.width * 0.5;
                let height_at = |position: Vec3| {
                    let lateral = position.x.abs() / sample.right.x;
                    ground + (edge_height - ground) * (18.0 - lateral) / SHOULDER_WIDTH
                };
                for direction in [-1.0, 1.0] {
                    for dt in [STEP, 1.0 / 60.0, 1.0 / 30.0] {
                        let mut car = Car::new(&track);
                        car.reset(&track, sample.distance);
                        car.position.x = sample.right.x * side * 12.0;
                        car.position.y = height_at(car.position) + RIDE_HEIGHT;
                        car.heading = side * direction * std::f32::consts::FRAC_PI_2;
                        car.velocity = vec3(
                            side * direction * 5.0,
                            -(edge_height - ground) * direction * 5.0
                                / (SHOULDER_WIDTH * sample.right.x),
                            0.0,
                        );
                        let road = nearest_road(
                            &track,
                            car.position,
                            car.distance,
                            car.position.y,
                            ground,
                            None,
                        )
                        .unwrap();
                        let surface = road_surface(road, ground).unwrap();
                        let ahead = car.position + Vec3::X * side * direction * 0.25;
                        // The landing sweep must intersect this same physical
                        // plane, including when the endpoint leaves its footprint.
                        assert!(
                            (plane_height_at(surface, car.position, ahead) - height_at(ahead))
                                .abs()
                                < 0.001
                        );
                        for _ in 0..10 {
                            car.update(&track, Control::default(), dt);
                            assert!(
                                (car.position.y - RIDE_HEIGHT - height_at(car.position)).abs()
                                    < 0.02,
                                "lost the slope at bank {bank}, side {side}, direction {direction}, dt {dt}: {car:?}"
                            );
                            assert!(car.grounded && car.offroad);
                        }
                    }
                }
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

    #[test]
    fn route_walk_crosses_a_shared_boundary_in_either_direction() {
        let track = Track::parse(
            "straight 40\nright 180 radius 20\nstraight 40\nright 180 radius 20\nclose",
        )
        .unwrap();
        let segments = track.samples.len() - 1;
        let query = |position: Vec3, distance: f32| {
            nearest_road(&track, position, distance, 1.0, track.ground_height(), None).unwrap()
        };
        // The circuit seam: segment 0 at its start and the last segment at its
        // end report the same cross-section from either side.
        let seam = track.samples[0];
        let mut start = query(seam.pos + Vec3::Y * RIDE_HEIGHT, 0.0);
        start.index = 0;
        start.sample = seam;
        let mut end = start;
        end.index = segments - 1;
        end.sample.distance = track.length;
        // An interior sample boundary, reported by its two adjacent segments.
        let boundary = track.samples[5];
        let mut after_boundary = query(boundary.pos + Vec3::Y * RIDE_HEIGHT, boundary.distance);
        after_boundary.index = 5;
        after_boundary.sample = boundary;
        let mut before_boundary = after_boundary;
        before_boundary.index = 4;
        for (ahead, behind, sample) in [
            (start, end, seam),
            (after_boundary, before_boundary, boundary),
        ] {
            assert_eq!(route_walk(&track, behind, ahead), Some((0.0, true)));
            assert_eq!(route_walk(&track, ahead, behind), Some((0.0, false)));
            let forward = horizontal(sample.forward).normalize() * 0.3;
            let at = sample.pos + Vec3::Y * RIDE_HEIGHT;
            // Backing across the boundary walks one segment back rather than
            // a full lap forward, and moving ahead walks one segment forward.
            assert!(crosses_connected_surface(
                &track,
                ahead,
                behind,
                at + forward * 0.01,
                at - forward
            ));
            assert!(crosses_connected_surface(
                &track,
                behind,
                ahead,
                at - forward * 0.01,
                at + forward
            ));
            // The walk still requires the motion to cross that boundary.
            assert!(!crosses_connected_surface(
                &track,
                ahead,
                behind,
                at + forward,
                at + forward * 2.0
            ));
        }
        // A tie between segments that do not meet has no route.
        let mut distant = before_boundary;
        distant.index = 9;
        assert_eq!(route_walk(&track, after_boundary, distant), None);
    }
}
