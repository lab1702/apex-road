//! A uniform grid over a course's segments on the horizontal plane, so road
//! queries examine the segments near a position instead of the whole route.

use super::{RoadSample, SHOULDER_WIDTH, ground_height};
use macroquad::prelude::*;
use std::sync::OnceLock;

/// Cells start at this size and double until the grid stays small.
const MIN_CELL_SIZE: f32 = 16.0;
/// Rounding in cell coordinates is far below this distance.
const CELL_SLACK: f32 = 1.0;
/// How many cells beyond the grid a point's cell may lie. Far more than any
/// search walks, yet small enough that ring arithmetic never overflows.
const CELL_RANGE: i64 = 1 << 40;

/// One segment, from `samples[i]` to `samples[i + 1]`.
#[derive(Clone, Copy, Debug)]
struct Segment {
    /// Horizontal (x, z) bounds of the segment's centerline.
    min: Vec2,
    max: Vec2,
    /// Half the wider end's width plus a shoulder: the farthest any deck or
    /// shoulder of the segment reaches from its centerline.
    reach: f32,
    /// The inclusive range of grid cells that the centerline bounds overlap.
    cells: [usize; 4],
}

/// Course data derived once from the samples for fast physics queries: a
/// uniform grid of cells, each listing the segments whose centerline bounds
/// overlap it, plus per-sample values that queries would otherwise recompute.
#[derive(Debug)]
pub struct SegmentIndex {
    origin: Vec2,
    cell_size: f32,
    columns: usize,
    rows: usize,
    /// The segments overlapping cell `c` are `entries[starts[c]..starts[c + 1]]`.
    starts: Vec<u32>,
    entries: Vec<u32>,
    segments: Vec<Segment>,
    max_half_width: f32,
    max_reach: f32,
    /// Each sample's [`RoadSample::cross_section_normal`].
    normals: Vec<Vec3>,
    ground_height: f32,
}

impl SegmentIndex {
    pub fn new(samples: &[RoadSample]) -> Self {
        let mut segments: Vec<Segment> = samples
            .windows(2)
            .map(|pair| {
                let [a, b] = [pair[0], pair[1]];
                Segment {
                    min: a.pos.xz().min(b.pos.xz()),
                    max: a.pos.xz().max(b.pos.xz()),
                    reach: a.width.max(b.width) * 0.5 + SHOULDER_WIDTH,
                    cells: [0; 4],
                }
            })
            .collect();
        let (min, max) = segments.iter().fold(
            (Vec2::splat(f32::INFINITY), Vec2::splat(f32::NEG_INFINITY)),
            |(min, max), segment| (min.min(segment.min), max.max(segment.max)),
        );
        let (origin, extent) = if min.is_finite() && max.is_finite() {
            (min, max - min)
        } else {
            (Vec2::ZERO, Vec2::ZERO)
        };
        // Keep the cell count proportional to the route, however sparse.
        let cell_limit = (segments.len() * 4).max(4096);
        let dimensions = |cell_size: f32| {
            (
                (extent.x / cell_size) as usize + 1,
                (extent.y / cell_size) as usize + 1,
            )
        };
        let mut cell_size = MIN_CELL_SIZE;
        while {
            let (columns, rows) = dimensions(cell_size);
            columns.saturating_mul(rows) > cell_limit
        } {
            cell_size *= 2.0;
        }
        let (columns, rows) = dimensions(cell_size);
        let mut index = Self {
            origin,
            cell_size,
            columns,
            rows,
            starts: vec![0; columns * rows + 1],
            entries: Vec::new(),
            segments: Vec::new(),
            max_half_width: samples.iter().map(|s| s.width * 0.5).fold(0.0, f32::max),
            max_reach: segments.iter().map(|s| s.reach).fold(0.0, f32::max),
            normals: samples
                .iter()
                .map(RoadSample::cross_section_normal)
                .collect(),
            ground_height: ground_height(samples),
        };
        for segment in &mut segments {
            let [x0, z0] = index.clamped_cell(segment.min);
            let [x1, z1] = index.clamped_cell(segment.max);
            segment.cells = [x0, z0, x1, z1];
        }
        // Count, then fill, each cell's segment list.
        for segment in &segments {
            let [x0, z0, x1, z1] = segment.cells;
            for z in z0..=z1 {
                for x in x0..=x1 {
                    index.starts[z * columns + x + 1] += 1;
                }
            }
        }
        for cell in 0..columns * rows {
            index.starts[cell + 1] += index.starts[cell];
        }
        let mut next = index.starts.clone();
        index.entries = vec![0; index.starts[columns * rows] as usize];
        for (id, segment) in segments.iter().enumerate() {
            let [x0, z0, x1, z1] = segment.cells;
            for z in z0..=z1 {
                for x in x0..=x1 {
                    let slot = &mut next[z * columns + x];
                    index.entries[*slot as usize] = id as u32;
                    *slot += 1;
                }
            }
        }
        index.segments = segments;
        index
    }

    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    /// Each sample's [`RoadSample::cross_section_normal`], computed once.
    pub fn cross_section_normals(&self) -> &[Vec3] {
        &self.normals
    }

    /// [`Track::ground_height`](super::Track::ground_height), computed once.
    pub fn ground_height(&self) -> f32 {
        self.ground_height
    }

    /// Half the widest sample's width.
    pub fn max_half_width(&self) -> f32 {
        self.max_half_width
    }

    /// Call `visit` once for every segment whose centerline bounds lie
    /// within `margin` of the horizontal rectangle from `min` to `max`, and
    /// perhaps a few more, in no particular order. Nothing is allocated, so
    /// physics can call it every step.
    pub fn for_each_near(&self, min: Vec2, max: Vec2, margin: f32, mut visit: impl FnMut(usize)) {
        let margin = Vec2::splat(margin + CELL_SLACK);
        let [x0, z0] = self.clamped_cell(min - margin);
        let [x1, z1] = self.clamped_cell(max + margin);
        for z in z0..=z1 {
            for x in x0..=x1 {
                let cell = z * self.columns + x;
                let entries =
                    &self.entries[self.starts[cell] as usize..self.starts[cell + 1] as usize];
                for &id in entries {
                    // A segment spanning several cells is visited only from
                    // the lowest of its cells inside the rectangle.
                    let [sx, sz, _, _] = self.segments[id as usize].cells;
                    if [x, z] == [sx.max(x0), sz.max(z0)] {
                        visit(id as usize);
                    }
                }
            }
        }
    }

    /// The cell containing `point`, which may lie outside the grid. Cells
    /// farther than [`CELL_RANGE`] beyond either side of the grid are pulled
    /// in to that distance, so ring arithmetic around them cannot overflow.
    /// A pulled-in cell is no farther from any grid cell than the true one,
    /// so distance bounds derived from it stay conservative.
    fn cell(&self, point: Vec2) -> [i64; 2] {
        let local = (point - self.origin) / self.cell_size;
        [
            (local.x.floor() as i64).clamp(-CELL_RANGE, self.columns as i64 + CELL_RANGE),
            (local.y.floor() as i64).clamp(-CELL_RANGE, self.rows as i64 + CELL_RANGE),
        ]
    }

    fn clamped_cell(&self, point: Vec2) -> [usize; 2] {
        let [x, z] = self.cell(point);
        [
            x.clamp(0, self.columns as i64 - 1) as usize,
            z.clamp(0, self.rows as i64 - 1) as usize,
        ]
    }

    /// Visit segments in rings of cells around `point`, each segment at most
    /// once, until no unvisited segment can matter. `visit` receives a
    /// segment index and returns the current limit: the largest footprint
    /// gap (horizontal distance from `point` to the segment's centerline
    /// bounds, less its reach) that could still matter. Segments whose gap
    /// exceeds the limit are skipped, and the search ends once every
    /// unvisited segment's gap must exceed it. `limit` is its initial value.
    ///
    /// Far from every road (beyond an open course's end, say) the limit can
    /// stay large while ever larger rings hold nothing. Once walking cells
    /// would cost more than a pass over the segments, the search gives up
    /// and returns `false`: the caller must examine every segment itself.
    pub fn visit_nearby(
        &self,
        point: Vec2,
        mut limit: f32,
        mut visit: impl FnMut(usize) -> f32,
    ) -> bool {
        if self.segments.is_empty() {
            return true;
        }
        if !point.is_finite() {
            return false;
        }
        let [cx, cz] = self.cell(point);
        let (last_x, last_z) = (self.columns as i64 - 1, self.rows as i64 - 1);
        // Rings closer than this hold no cells of the grid; beyond the last
        // ring, every cell has been visited.
        let first_ring = (-cx).max(cx - last_x).max(-cz).max(cz - last_z).max(0);
        let last_ring = cx.max(last_x - cx).max(cz).max(last_z - cz);
        let cell_budget = self.segments.len() / 4 + 64;
        let mut visited_cells = 0;
        for ring in first_ring..=last_ring {
            // Every segment not yet visited lies outside the square of cells
            // within `ring - 1` of the point's cell.
            if ring > 0 && (ring - 1) as f32 * self.cell_size - self.max_reach - CELL_SLACK > limit
            {
                return true;
            }
            if visited_cells > cell_budget {
                return false;
            }
            let (x_min, x_max) = ((cx - ring).max(0), (cx + ring).min(last_x));
            let (z_min, z_max) = ((cz - ring).max(0), (cz + ring).min(last_z));
            for z in z_min..=z_max {
                if z == cz - ring || z == cz + ring {
                    for x in x_min..=x_max {
                        self.visit_cell(point, [cx, cz], ring, [x, z], &mut limit, &mut visit);
                    }
                    visited_cells += (x_max - x_min + 1) as usize;
                } else {
                    // Rows between the ring's top and bottom hold only its
                    // two side cells.
                    for x in [cx - ring, cx + ring] {
                        if (0..=last_x).contains(&x) {
                            self.visit_cell(point, [cx, cz], ring, [x, z], &mut limit, &mut visit);
                            visited_cells += 1;
                        }
                    }
                }
            }
        }
        true
    }

    fn visit_cell(
        &self,
        point: Vec2,
        [cx, cz]: [i64; 2],
        ring: i64,
        [x, z]: [i64; 2],
        limit: &mut f32,
        visit: &mut impl FnMut(usize) -> f32,
    ) {
        let cell = z as usize * self.columns + x as usize;
        let entries = &self.entries[self.starts[cell] as usize..self.starts[cell + 1] as usize];
        for &id in entries {
            let segment = &self.segments[id as usize];
            let [x0, z0, x1, z1] = segment.cells.map(|c| c as i64);
            // Visit each segment once: in the first ring that overlaps its
            // cells, from the lowest of those cells within that ring.
            let inner = ring - 1;
            let seen = ring > 0
                && x0 <= cx + inner
                && x1 >= cx - inner
                && z0 <= cz + inner
                && z1 >= cz - inner;
            if seen || [x, z] != [x0.max(cx - ring), z0.max(cz - ring)] {
                continue;
            }
            let gap = point.distance(point.clamp(segment.min, segment.max)) - segment.reach;
            if gap <= *limit {
                *limit = visit(id as usize);
            }
        }
    }
}

/// A lazily built [`SegmentIndex`]. A cloned course starts without one, so
/// a copy whose samples are then edited never sees a stale index.
#[derive(Default)]
pub(super) struct SegmentIndexCache(OnceLock<SegmentIndex>);

impl SegmentIndexCache {
    pub(super) fn get_or_build(&self, samples: &[RoadSample]) -> &SegmentIndex {
        self.0.get_or_init(|| SegmentIndex::new(samples))
    }
}

impl Clone for SegmentIndexCache {
    fn clone(&self) -> Self {
        Self::default()
    }
}

impl std::fmt::Debug for SegmentIndexCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SegmentIndexCache")
            .field("built", &self.0.get().is_some())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track::Track;

    #[test]
    fn searches_find_every_segment_within_their_bounds_once() {
        let mut state = 0x9e3779b97f4a7c15_u64;
        let mut random = move || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (state >> 40) as f32 / (1u64 << 24) as f32
        };
        for path in crate::courses::BUNDLED_PATHS {
            let track = Track::load(path).unwrap();
            let index = SegmentIndex::new(&track.samples);
            assert_eq!(index.segment_count(), track.samples.len() - 1);
            let low = index.origin - Vec2::splat(300.0);
            let size = Vec2::new(index.columns as f32, index.rows as f32) * index.cell_size + 600.0;
            for _ in 0..300 {
                let point = low + size * Vec2::new(random(), random());
                let gap = |segment: &Segment| {
                    point.distance(point.clamp(segment.min, segment.max)) - segment.reach
                };
                for limit in [-5.0, 0.0, 5.0, 40.0, 200.0, f32::INFINITY] {
                    let mut visits = vec![0; index.segment_count()];
                    let complete = index.visit_nearby(point, limit, |segment| {
                        visits[segment] += 1;
                        limit
                    });
                    for (segment, &count) in index.segments.iter().zip(&visits) {
                        assert!(count <= 1, "{path}: a segment was visited twice");
                        if complete {
                            assert_eq!(count == 1, gap(segment) <= limit, "{path}: {point}");
                        }
                    }
                }
                let margin = 50.0 * random();
                let mut near = vec![0; index.segment_count()];
                index.for_each_near(point, point, margin, |segment| near[segment] += 1);
                for (id, segment) in index.segments.iter().enumerate() {
                    assert!(near[id] <= 1, "{path}: a segment was visited twice");
                    if gap(segment) + segment.reach <= margin {
                        assert_eq!(near[id], 1, "{path}: {point}");
                    }
                }
            }
        }
    }

    #[test]
    fn searches_from_astronomically_distant_points_do_not_overflow() {
        let track = Track::parse("straight 100").unwrap();
        let index = SegmentIndex::new(&track.samples);
        for point in [
            Vec2::new(1e21, 0.0),
            Vec2::new(-1e21, 0.0),
            Vec2::new(0.0, 1e21),
            Vec2::new(0.0, -1e21),
            Vec2::splat(f32::MAX),
            Vec2::splat(f32::MIN),
        ] {
            for limit in [0.0, 1e30, f32::INFINITY] {
                let mut visits = vec![0; index.segment_count()];
                let complete = index.visit_nearby(point, limit, |segment| {
                    visits[segment] += 1;
                    limit
                });
                assert!(visits.iter().all(|&count| count <= 1));
                if complete {
                    for (segment, &count) in index.segments.iter().zip(&visits) {
                        let gap =
                            point.distance(point.clamp(segment.min, segment.max)) - segment.reach;
                        assert_eq!(count == 1, gap <= limit, "{point} {limit}");
                    }
                }
            }
            index.for_each_near(point, point, 10.0, |_| {});
        }
    }
}
