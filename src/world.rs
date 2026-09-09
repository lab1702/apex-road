//! Procedural, vertex-lit scenery. Every road mesh comes from the same samples as physics.
use crate::track::{RoadKind, RoadSample, Track};
use macroquad::miniquad::CullFace;
use macroquad::prelude::*;

const SKY: Color = Color::new(0.72, 0.84, 0.86, 1.0);
const ASPHALT: Color = Color::new(0.20, 0.25, 0.27, 1.0);
const GRASS: Color = Color::new(0.34, 0.48, 0.32, 1.0);
const CREAM: Color = Color::new(0.91, 0.91, 0.82, 1.0);
const RED: Color = Color::new(0.78, 0.28, 0.20, 1.0);
const TEAL: Color = Color::new(0.13, 0.67, 0.64, 1.0);

struct Builder {
    vertices: Vec<Vertex>,
    indices: Vec<u16>,
}
impl Builder {
    fn new() -> Self {
        Self {
            vertices: vec![],
            indices: vec![],
        }
    }
    fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3, color: Color) {
        let n = self.vertices.len() as u16;
        self.vertices
            .extend([a, b, c].map(|p| Vertex::new2(p, Vec2::ZERO, color)));
        self.indices.extend([n, n + 1, n + 2]);
    }
    fn quad(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, color: Color) {
        let n = self.vertices.len() as u16;
        self.vertices
            .extend([a, b, c, d].map(|p| Vertex::new2(p, Vec2::ZERO, color)));
        self.indices.extend([n, n + 1, n + 2, n, n + 2, n + 3]);
    }
    fn block(
        &mut self,
        center: Vec3,
        right: Vec3,
        up: Vec3,
        forward: Vec3,
        size: Vec3,
        color: Color,
    ) {
        let r = right * size.x * 0.5;
        let u = up * size.y * 0.5;
        let f = forward * size.z * 0.5;
        self.quad(
            center - r + u - f,
            center + r + u - f,
            center + r + u + f,
            center - r + u + f,
            tint(color, 1.15),
        );
        self.quad(
            center - r - u - f,
            center - r + u - f,
            center - r + u + f,
            center - r - u + f,
            tint(color, 0.78),
        );
        self.quad(
            center + r - u - f,
            center + r - u + f,
            center + r + u + f,
            center + r + u - f,
            tint(color, 1.0),
        );
        self.quad(
            center - r - u - f,
            center + r - u - f,
            center + r + u - f,
            center - r + u - f,
            tint(color, 0.86),
        );
        self.quad(
            center - r - u + f,
            center - r + u + f,
            center + r + u + f,
            center + r - u + f,
            tint(color, 0.90),
        );
    }
    fn cone(&mut self, base: Vec3, radius: f32, height: f32, color: Color, sides: usize) {
        for i in 0..sides {
            let a = i as f32 / sides as f32 * std::f32::consts::TAU;
            let b = (i + 1) as f32 / sides as f32 * std::f32::consts::TAU;
            self.tri(
                base + vec3(a.cos() * radius, 0., a.sin() * radius),
                base + Vec3::Y * height,
                base + vec3(b.cos() * radius, 0., b.sin() * radius),
                tint(color, 0.8 + 0.25 * a.cos()),
            );
        }
    }
    fn finish(self) -> Mesh {
        Mesh {
            vertices: self.vertices,
            indices: self.indices,
            texture: None,
        }
    }
}
fn tint(c: Color, v: f32) -> Color {
    Color::new((c.r * v).min(1.), (c.g * v).min(1.), (c.b * v).min(1.), c.a)
}
fn noise(n: u32) -> f32 {
    let mut x = n.wrapping_mul(747796405).wrapping_add(2891336453);
    x = ((x >> ((x >> 28) + 4)) ^ x).wrapping_mul(277803737);
    ((x >> 22) ^ x) as f32 / u32::MAX as f32
}
fn edge(s: RoadSample, x: f32, h: f32) -> Vec3 {
    s.pos + s.right * x + s.up * h
}

// A single diagonal across a changing bank can lift the middle of a wide
// road far above its driving surface. Refine only twisted sections until the
// diagonal's departure from the interpolated cross-section is below 2 cm.
fn surface_steps(a: RoadSample, b: RoadSample) -> usize {
    let normal = (a.up + b.up).normalize();
    let span_change = b.right * b.width - a.right * a.width;
    let twist = span_change.dot(normal).abs() * 0.25;
    ((twist / 0.02).ceil().max(1.0) as usize).max(frame_steps(a, b))
}

fn frame_steps(a: RoadSample, b: RoadSample) -> usize {
    // A banked hill can curve the road edges without twisting them relative
    // to the average normal. Bound frame changes too: otherwise a straight
    // chord cuts far inside the unit frame used by the driving surface.
    let change = a.forward.distance(b.forward).max(a.right.distance(b.right));
    (change / 0.05).ceil().max(1.0) as usize
}

fn surface_sample(track: &Track, a: RoadSample, b: RoadSample, t: f32) -> RoadSample {
    if t == 0.0 {
        a
    } else if t == 1.0 {
        b
    } else {
        track.sample_at(a.distance + (b.distance - a.distance) * t)
    }
}

fn road_segment(builder: &mut Builder, track: &Track, a: RoadSample, b: RoadSample, color: Color) {
    let steps = surface_steps(a, b);
    for step in 0..steps {
        let start = surface_sample(track, a, b, step as f32 / steps as f32);
        let end = surface_sample(track, a, b, (step + 1) as f32 / steps as f32);
        builder.quad(
            edge(start, -start.width * 0.5, 0.),
            edge(start, start.width * 0.5, 0.),
            edge(end, end.width * 0.5, 0.),
            edge(end, -end.width * 0.5, 0.),
            color,
        );
    }
}

fn shoulder_segment(
    builder: &mut Builder,
    track: &Track,
    samples: [RoadSample; 2],
    ground: f32,
    side: f32,
    color: Color,
) {
    let [a, b] = samples;
    // Even an unbanked hill twists a shoulder: its inner edge climbs while
    // its outer edge stays on the terrain. Bound that vertical error as well.
    let height_change =
        (edge(b, side * b.width * 0.5, 0.).y - edge(a, side * a.width * 0.5, 0.).y).abs();
    // Curved cross-sections bow away from the straight triangle edges.
    // On a high shoulder even a few millimetres of horizontal displacement
    // becomes a visible height error, proportional to its downhill slope.
    let frame_change = a.forward.distance(b.forward).max(a.right.distance(b.right));
    let shoulder_slope = [a, b]
        .into_iter()
        .map(|s| {
            (edge(s, side * s.width * 0.5, 0.).y - ground).abs() / (12.0 * s.right.xz().length())
        })
        .fold(0.0_f32, f32::max);
    let curve_error =
        frame_change.powi(2) * (a.width.max(b.width) * 0.5 + 12.0) * 0.25 * shoulder_slope;
    let steps = surface_steps(a, b)
        .max((height_change / 0.08).ceil() as usize)
        .max((curve_error / 0.01).sqrt().ceil() as usize);
    for step in 0..steps {
        let start = surface_sample(track, a, b, step as f32 / steps as f32);
        let end = surface_sample(track, a, b, (step + 1) as f32 / steps as f32);
        let mut far_start = edge(start, side * (start.width * 0.5 + 12.), 0.);
        let mut far_end = edge(end, side * (end.width * 0.5 + 12.), 0.);
        far_start.y = ground;
        far_end.y = ground;
        builder.quad(
            edge(start, side * start.width * 0.5, 0.) - Vec3::Y * 0.025,
            far_start,
            far_end,
            edge(end, side * end.width * 0.5, 0.) - Vec3::Y * 0.025,
            color,
        );
    }
}

fn checkerboard(track: &Track, distance: f32) -> Vec<Mesh> {
    let mut meshes = Vec::new();
    let mut builder = Builder::new();
    for pair in track.samples.windows(2) {
        let [a, b] = [pair[0], pair[1]];
        if b.distance <= distance || a.distance >= distance + 1.4 || a.kind == RoadKind::Gap {
            continue;
        }
        let steps = surface_steps(a, b);
        for step in 0..steps {
            let start = a.distance + (b.distance - a.distance) * step as f32 / steps as f32;
            let end = a.distance + (b.distance - a.distance) * (step + 1) as f32 / steps as f32;
            for row in 0..2 {
                let front = start.max(distance + row as f32 * 0.7);
                let back = end.min(distance + (row + 1) as f32 * 0.7);
                if back <= front {
                    continue;
                }
                let front = track.sample_at(front);
                let back = track.sample_at(back);
                // Rapid banks can require thousands of small squares. Keep
                // each mesh below both the u16 index and renderer batch limits.
                if builder.vertices.len() + 64 > 16_000 {
                    meshes.push(std::mem::replace(&mut builder, Builder::new()).finish());
                }
                for x in 0..16 {
                    let left = x as f32 / 16.0 - 0.5;
                    let right = (x + 1) as f32 / 16.0 - 0.5;
                    let color = if (x + row) % 2 == 0 { CREAM } else { ASPHALT };
                    builder.quad(
                        edge(front, left * front.width, 0.04),
                        edge(front, right * front.width, 0.04),
                        edge(back, right * back.width, 0.04),
                        edge(back, left * back.width, 0.04),
                        color,
                    );
                }
            }
        }
    }
    if !builder.vertices.is_empty() {
        meshes.push(builder.finish());
    }
    meshes
}

struct TreeExclusion<'a> {
    samples: &'a [RoadSample],
    min: Vec2,
    max: Vec2,
}
impl<'a> TreeExclusion<'a> {
    fn new(samples: &'a [RoadSample]) -> Self {
        let mut min = Vec2::splat(f32::INFINITY);
        let mut max = Vec2::splat(f32::NEG_INFINITY);
        let mut half_width = 0.0_f32;
        for sample in samples {
            min = min.min(sample.pos.xz());
            max = max.max(sample.pos.xz());
            half_width = half_width.max(sample.width * 0.5);
        }
        let padding = Vec2::splat(half_width + 12.0);
        Self {
            samples,
            min: min - padding,
            max: max + padding,
        }
    }

    fn allows_tree(&self, position: Vec2, radius: f32) -> bool {
        // Reject distant batches first, keeping full-segment clearance cheap
        // even for a 25,000-sample course. Include the crown and its shadow.
        if position.distance_squared(position.clamp(self.min, self.max)) > radius * radius {
            return true;
        }
        self.samples.windows(2).all(|pair| {
            let a = pair[0].pos.xz();
            let span = pair[1].pos.xz() - a;
            let t = ((position - a).dot(span) / span.length_squared().max(f32::EPSILON))
                .clamp(0.0, 1.0);
            let clearance = pair[0].width.max(pair[1].width) * 0.5 + 12.0 + radius;
            position.distance_squared(a + span * t) > clearance * clearance
        })
    }
}

struct Chunk {
    min: Vec3,
    max: Vec3,
    mesh: Mesh,
    distant: bool,
}
impl Chunk {
    fn new(mesh: Mesh, distant: bool) -> Self {
        let origin = mesh.vertices.first().map_or(Vec3::ZERO, |v| v.position);
        let (min, max) = mesh
            .vertices
            .iter()
            .fold((origin, origin), |(min, max), v| {
                (min.min(v.position), max.max(v.position))
            });
        Self {
            min,
            max,
            mesh,
            distant,
        }
    }

    fn visible_from(&self, position: Vec3) -> bool {
        // Tall supports and sloping shoulders can be close to the camera even
        // when the road deck is far away. Cull only when the entire bounds are.
        !self.mesh.vertices.is_empty()
            && (self.distant
                || position.distance_squared(position.clamp(self.min, self.max)) < 900. * 900.)
    }
}
pub struct World {
    chunks: Vec<Chunk>,
    material: Material,
}
impl World {
    pub fn new(track: &Track) -> Self {
        let material = load_material(
            ShaderSource::Glsl {
                vertex: VERTEX,
                fragment: FRAGMENT,
            },
            MaterialParams {
                pipeline_params: PipelineParams {
                    depth_write: true,
                    depth_test: Comparison::LessOrEqual,
                    cull_face: CullFace::Nothing,
                    ..Default::default()
                },
                uniforms: vec![UniformDesc::new("Eye", UniformType::Float3)],
                ..Default::default()
            },
        )
        .expect("OpenGL 2.1 world shader should compile");
        Self {
            chunks: Self::build_chunks(track),
            material,
        }
    }

    /// Course changes only replace geometry; the shared shader lives for the game.
    pub fn rebuild(&mut self, track: &Track) {
        self.chunks = Self::build_chunks(track);
    }

    fn build_chunks(track: &Track) -> Vec<Chunk> {
        let ground_y = track.ground_height();
        let tree_exclusions: Vec<_> = (0..track.samples.len() - 1)
            .step_by(48)
            .map(|first| {
                TreeExclusion::new(
                    &track.samples[first..=(first + 48).min(track.samples.len() - 1)],
                )
            })
            .collect();
        let mut chunks = vec![];
        // Road, shoulders and architecture are spatially batched for cheap distance culling.
        for (chunk_index, points) in track
            .samples
            .windows(2)
            .collect::<Vec<_>>()
            .chunks(48)
            .enumerate()
        {
            let mut b = Builder::new();
            for (local, pair) in points.iter().enumerate() {
                let i = chunk_index * 48 + local;
                let a = pair[0];
                let z = pair[1];
                let w = a.width * 0.5;
                let wz = z.width * 0.5;
                if a.kind == RoadKind::Gap {
                    continue;
                }
                let shade = if i % 2 == 0 { 1.0 } else { 1.015 };
                // Share the asphalt's twist and frame refinement with its
                // markings. Frame refinement alone can bury edge stripes in
                // widening banked hills. Collision architecture keeps its
                // source quads.
                let steps = surface_steps(a, z);
                for step in 0..steps {
                    let start = surface_sample(track, a, z, step as f32 / steps as f32);
                    let end = surface_sample(track, a, z, (step + 1) as f32 / steps as f32);
                    let [a, z] = [start, end];
                    let w = a.width * 0.5;
                    let wz = z.width * 0.5;
                    road_segment(&mut b, track, a, z, tint(ASPHALT, shade));
                    for side in [-1., 1.] {
                        // A generous shoulder matches the off-road contact surface.
                        if matches!(a.kind, RoadKind::Road | RoadKind::Ramp) {
                            shoulder_segment(
                                &mut b,
                                track,
                                [a, z],
                                ground_y,
                                side,
                                tint(GRASS, 0.96 + noise(i as u32) * 0.07),
                            );
                        } else {
                            // Deep fascia makes the bridge read as a solid structure from below.
                            b.quad(
                                edge(a, side * w, 0.),
                                edge(a, side * w, -0.85),
                                edge(z, side * wz, -0.85),
                                edge(z, side * wz, 0.),
                                Color::new(0.45, 0.51, 0.49, 1.),
                            );
                        }
                        b.quad(
                            edge(a, side * (w - 0.34), 0.024),
                            edge(a, side * (w - 0.18), 0.024),
                            edge(z, side * (wz - 0.18), 0.024),
                            edge(z, side * (wz - 0.34), 0.024),
                            CREAM,
                        );
                        let curb = if (a.distance / 4.).floor() as i32 % 2 == 0 {
                            CREAM
                        } else {
                            RED
                        };
                        b.quad(
                            edge(a, side * w, 0.035),
                            edge(a, side * (w + 0.43), 0.035),
                            edge(z, side * (wz + 0.43), 0.035),
                            edge(z, side * wz, 0.035),
                            curb,
                        );
                    }
                    // Subtle dashed road centerline.
                    if (a.distance / 6.).floor() as i32 % 2 == 0 {
                        b.quad(
                            edge(a, -0.075, 0.027),
                            edge(a, 0.075, 0.027),
                            edge(z, 0.075, 0.027),
                            edge(z, -0.075, 0.027),
                            tint(CREAM, 0.77),
                        );
                    }
                }
                if matches!(a.kind, RoadKind::Bridge | RoadKind::Tunnel) {
                    for side in [-1., 1.] {
                        b.quad(
                            edge(a, side * (w + 0.25), 0.2),
                            edge(a, side * (w + 0.25), 1.0),
                            edge(z, side * (wz + 0.25), 1.0),
                            edge(z, side * (wz + 0.25), 0.2),
                            Color::new(0.65, 0.70, 0.64, 1.),
                        );
                        b.quad(
                            edge(a, side * (w + 0.20), 1.0),
                            edge(a, side * (w + 0.35), 1.0),
                            edge(z, side * (wz + 0.35), 1.0),
                            edge(z, side * (wz + 0.20), 1.0),
                            CREAM,
                        );
                    }
                }
                if a.kind == RoadKind::Tunnel {
                    for j in 0..10 {
                        let t = j as f32 / 10. * std::f32::consts::PI;
                        let u = (j + 1) as f32 / 10. * std::f32::consts::PI;
                        let arch = |s: RoadSample, angle: f32| {
                            edge(
                                s,
                                angle.cos() * (s.width * 0.5 + 0.65),
                                1. + angle.sin() * 5.2,
                            )
                        };
                        let tone = 0.60 + 0.12 * t.sin();
                        b.quad(
                            arch(a, t),
                            arch(a, u),
                            arch(z, u),
                            arch(z, t),
                            Color::new(tone * 0.63, tone * 0.70, tone * 0.69, 1.),
                        );
                    }
                    if i % 5 == 0 {
                        for side in [-1., 1.] {
                            b.block(
                                edge(a, side * (w + 0.35), 2.1),
                                a.right,
                                a.up,
                                a.forward,
                                vec3(0.1, 0.12, 3.2),
                                Color::new(1., 0.80, 0.44, 1.),
                            );
                        }
                    }
                    if i % 8 == 0 {
                        arch_rib(&mut b, a);
                    }
                    if i == 0 || track.samples[i - 1].kind != RoadKind::Tunnel {
                        arch_rib(&mut b, a);
                    }
                }
                if a.kind == RoadKind::Bridge && i % 10 == 0 {
                    for side in [-1., 1.] {
                        let p = edge(a, side * (w - 0.9), -0.5);
                        let h = (p.y - ground_y).max(1.);
                        b.block(
                            vec3(p.x, p.y - h * 0.5, p.z),
                            a.right,
                            Vec3::Y,
                            a.forward,
                            vec3(1.4, h, 2.0),
                            Color::new(0.57, 0.61, 0.55, 1.),
                        );
                    }
                    b.block(
                        a.pos - a.up * 0.85,
                        a.right,
                        a.up,
                        a.forward,
                        vec3(a.width + 0.6, 1.1, 1.6),
                        Color::new(0.43, 0.49, 0.46, 1.),
                    );
                }
                if i % 12 == 0 && a.kind != RoadKind::Tunnel {
                    for side in [-1., 1.] {
                        let p = edge(a, side * (w + 0.8), 0.);
                        b.block(
                            p + a.up * 0.52,
                            a.right,
                            a.up,
                            a.forward,
                            vec3(0.12, 1.05, 0.14),
                            CREAM,
                        );
                        b.block(
                            p + a.up * 0.81,
                            a.right,
                            a.up,
                            a.forward,
                            vec3(0.14, 0.19, 0.16),
                            TEAL,
                        );
                    }
                }
                // Trees sit beyond drivable shoulders, with deterministic variation.
                if i % 4 == 0 && matches!(a.kind, RoadKind::Road | RoadKind::Ramp) {
                    for side in [-1., 1.] {
                        let seed = i as u32 * 17 + if side > 0. { 11 } else { 79 };
                        let p = edge(a, side * (w + 15. + noise(seed) * 34.), 0.);
                        let pos = vec3(p.x, ground_y, p.z);
                        let height = 5. + noise(seed + 7) * 8.;
                        // Keep the entire tree outside nearby road shoulders,
                        // including crossings between sampled center points.
                        if tree_exclusions
                            .iter()
                            .all(|area| area.allows_tree(pos.xz(), height * 0.34))
                        {
                            tree(&mut b, pos, height, seed);
                        }
                    }
                }
                if i % 40 == 0 && a.kind == RoadKind::Road {
                    let side = if i % 80 == 0 { 1. } else { -1. };
                    roadside_sign(&mut b, a, ground_y, side);
                }
                // Width changes on a bank can require many shoulder slices.
                // Split those dense runs before adding another source segment.
                if b.vertices.len() >= 16_000 {
                    chunks.push(Chunk::new(
                        std::mem::replace(&mut b, Builder::new()).finish(),
                        false,
                    ));
                }
            }
            chunks.push(Chunk::new(b.finish(), false));
        }
        let mut gates = Builder::new();
        // Circuits share the start gate at the seam; only sprints need a
        // separate finish gate before the endpoint.
        for (idx, d) in std::iter::once(0.)
            .chain(track.checkpoints.iter().copied())
            .chain((!track.closed).then_some(track.finish_distance()))
            .enumerate()
        {
            let s = track.sample_at(d);
            if s.kind == RoadKind::Gap {
                continue;
            }
            let width = s.width + 1.5;
            let color = if idx == 0 { CREAM } else { TEAL };
            for side in [-1., 1.] {
                gates.block(
                    edge(s, side * width * 0.5, 3.),
                    s.right,
                    s.up,
                    s.forward,
                    vec3(0.32, 6., 0.5),
                    tint(color, 0.85),
                );
            }
            gates.block(
                edge(s, 0., 6.0),
                s.right,
                s.up,
                s.forward,
                vec3(width + 0.32, 0.7, 0.6),
                Color::new(0.12, 0.24, 0.25, 1.),
            );
            gates.block(
                edge(s, 0., 6.01) - s.forward * 0.31,
                s.right,
                s.up,
                s.forward,
                vec3(width - 0.8, 0.15, 0.025),
                color,
            );
            if idx == 0 || d == track.finish_distance() {
                chunks.extend(
                    checkerboard(track, d)
                        .into_iter()
                        .map(|mesh| Chunk::new(mesh, true)),
                );
            }
            if idx % 32 == 31 {
                chunks.push(Chunk::new(gates.finish(), true));
                gates = Builder::new();
            }
        }
        chunks.push(Chunk::new(gates.finish(), true));
        let min = track
            .samples
            .iter()
            .fold(Vec3::splat(f32::MAX), |a, s| a.min(s.pos));
        let max = track
            .samples
            .iter()
            .fold(Vec3::splat(f32::MIN), |a, s| a.max(s.pos));
        let mid = (min + max) * 0.5;
        let mut ground = Builder::new();
        let extent = ((max - min).length() + 1600.).max(2000.);
        for x in -12i32..12 {
            for z in -12i32..12 {
                let cell = extent / 12.;
                let p = vec3(
                    mid.x + x as f32 * cell,
                    ground_y - 0.05,
                    mid.z + z as f32 * cell,
                );
                ground.quad(
                    p,
                    p + Vec3::X * cell,
                    p + (Vec3::X + Vec3::Z) * cell,
                    p + Vec3::Z * cell,
                    tint(GRASS, 0.95 + noise(((x + 12) * 31 + z + 12) as u32) * 0.08),
                );
            }
        }
        chunks.push(Chunk::new(ground.finish(), true));
        // Rounded overlapping alpine ridges; deliberately outside the driving area.
        for k in 0..28u32 {
            let angle = k as f32 / 28. * std::f32::consts::TAU;
            let radius = (max - min).length() * 0.55 + 400. + noise(k + 400) * 220.;
            let p = vec3(
                mid.x + angle.cos() * radius,
                ground_y,
                mid.z + angle.sin() * radius,
            );
            let mut b = Builder::new();
            mountain(
                &mut b,
                p,
                180. + noise(k + 800) * 180.,
                100. + noise(k + 600) * 240.,
                k,
            );
            chunks.push(Chunk::new(b.finish(), true));
        }
        chunks
    }
    pub fn draw(&self, camera: &crate::view::DriverCamera) {
        clear_background(SKY);
        // The gradient retains a soft, airy horizon without external textures.
        let h = screen_height();
        let w = screen_width();
        for i in 0..48 {
            let t = i as f32 / 47.;
            let c = Color::new(0.38 + 0.36 * t, 0.64 + 0.21 * t, 0.75 + 0.10 * t, 1.);
            draw_rectangle(0., h * i as f32 / 48., w, h / 48. + 1., c);
        }
        // The sun has a fixed world direction; turning, climbing, and banking
        // change its windshield position. Draw before scenery for occlusion.
        if let Some((center, radius)) = camera.project_sky_disc(
            vec3(0.45, 0.35, 1.).normalize(),
            4_f32.to_radians(),
            vec2(w, h),
        ) {
            draw_circle(
                center.x,
                center.y,
                radius,
                Color::new(0.97, 0.93, 0.73, 0.9),
            );
        }
        set_camera(camera);
        self.material.set_uniform("Eye", camera.position);
        gl_use_material(&self.material);
        for c in &self.chunks {
            if c.visible_from(camera.position) {
                draw_mesh(&c.mesh);
            }
        }
        gl_use_default_material();
        set_default_camera();
    }
}
fn roadside_sign(b: &mut Builder, s: RoadSample, ground_y: f32, side: f32) {
    let mut p = edge(s, side * (s.width * 0.5 + 9.), 0.);
    // Signs stand three quarters of the way across the 12 m shoulder.
    // Its surface descends to terrain, even when the road climbs or banks;
    // a fixed offset from the road plane would leave the sign floating.
    let road_edge_y = edge(s, side * s.width * 0.5, 0.).y;
    p.y = road_edge_y + (ground_y - road_edge_y) * 0.75 + 0.45;
    b.block(
        p + Vec3::Y * 0.65,
        s.right,
        Vec3::Y,
        s.forward,
        vec3(3., 1.3, 0.16),
        Color::new(0.12, 0.27, 0.28, 1.),
    );
    b.block(
        p - Vec3::Y * 0.4,
        s.right,
        Vec3::Y,
        s.forward,
        vec3(0.12, 1.5, 0.14),
        CREAM,
    );
    b.block(
        p + Vec3::Y * 0.65 - s.forward * 0.1,
        s.right,
        Vec3::Y,
        s.forward,
        vec3(2.45, 0.16, 0.03),
        TEAL,
    );
}
fn tree(b: &mut Builder, p: Vec3, h: f32, seed: u32) {
    let leaf = Color::new(
        0.15 + noise(seed) * 0.1,
        0.32 + noise(seed + 1) * 0.13,
        0.25 + noise(seed + 2) * 0.06,
        1.,
    );
    // Flat, stylized ambient shadow and layered evergreen crown.
    let shadow = Color::new(0.26, 0.37, 0.26, 1.);
    for i in 0..10 {
        let a = i as f32 / 10. * std::f32::consts::TAU;
        let z = (i + 1) as f32 / 10. * std::f32::consts::TAU;
        b.tri(
            p + Vec3::Y * 0.012,
            p + vec3(a.cos() * h * 0.34, 0.012, a.sin() * h * 0.22),
            p + vec3(z.cos() * h * 0.34, 0.012, z.sin() * h * 0.22),
            shadow,
        );
    }
    b.block(
        p + Vec3::Y * h * 0.25,
        Vec3::X,
        Vec3::Y,
        Vec3::Z,
        vec3(0.36, h * 0.5, 0.36),
        Color::new(0.33, 0.28, 0.21, 1.),
    );
    b.cone(p + Vec3::Y * h * 0.22, h * 0.29, h * 0.53, leaf, 9);
    b.cone(
        p + Vec3::Y * h * 0.42,
        h * 0.23,
        h * 0.45,
        tint(leaf, 1.08),
        9,
    );
    b.cone(
        p + Vec3::Y * h * 0.62,
        h * 0.16,
        h * 0.38,
        tint(leaf, 1.16),
        9,
    );
}
fn arch_rib(b: &mut Builder, s: RoadSample) {
    for j in 0..12 {
        let t = j as f32 / 12. * std::f32::consts::PI;
        let u = (j + 1) as f32 / 12. * std::f32::consts::PI;
        let point = |angle: f32, offset: f32| {
            edge(
                s,
                angle.cos() * (s.width * 0.5 + 0.58),
                1. + angle.sin() * 5.14,
            ) + s.forward * offset
        };
        b.quad(
            point(t, -0.16),
            point(u, -0.16),
            point(u, 0.16),
            point(t, 0.16),
            Color::new(0.64, 0.70, 0.66, 1.),
        );
    }
}
fn mountain(b: &mut Builder, p: Vec3, r: f32, h: f32, seed: u32) {
    let rings = 7;
    let sides = 24;
    let point = |j: usize, k: usize| {
        let a = k as f32 / sides as f32 * std::f32::consts::TAU;
        let t = j as f32 / rings as f32;
        let radial = r * t;
        let height =
            h * (1. - t * t).max(0.).powf(1.7) * (0.84 + 0.16 * (a * 3. + seed as f32).cos());
        p + vec3(a.cos() * radial, height, a.sin() * radial)
    };
    for j in 0..rings {
        for k in 0..sides {
            let color = if j < 2 {
                Color::new(0.66, 0.70, 0.64, 1.)
            } else {
                Color::new(0.35, 0.48, 0.40, 1.)
            };
            let c = tint(color, 0.87 + 0.18 * (k as f32 * 0.26).cos());
            b.quad(
                point(j, k),
                point(j + 1, k),
                point(j + 1, k + 1),
                point(j, k + 1),
                c,
            );
        }
    }
}

const VERTEX: &str = r#"#version 100
attribute vec3 position;
attribute vec4 color0;
uniform mat4 Model;
uniform mat4 Projection;
varying lowp vec4 color;
varying highp vec3 world;
void main() { vec4 p=Model*vec4(position,1.0);gl_Position=Projection*p;world=p.xyz;color=color0/255.0; }
"#;
const FRAGMENT: &str = r#"#version 100
precision highp float;
varying lowp vec4 color;
varying highp vec3 world;
uniform vec3 Eye;
void main() {
    float d=length(world-Eye);
    float fog=1.0-exp(-d*d*0.0000011);
    vec3 haze=vec3(0.72,0.84,0.86);
    gl_FragColor=vec4(mix(color.rgb,haze,clamp(fog,0.0,0.97)),color.a);
}
"#;

#[cfg(test)]
mod tests {
    use super::*;

    fn road_mesh(track: &Track) -> Mesh {
        let mut builder = Builder::new();
        for pair in track.samples.windows(2) {
            if pair[0].kind != RoadKind::Gap {
                road_segment(&mut builder, track, pair[0], pair[1], ASPHALT);
            }
        }
        builder.finish()
    }

    fn triangle_height(mesh: &Mesh, point: Vec3) -> Option<f32> {
        mesh.indices.as_chunks::<3>().0.iter().find_map(|indices| {
            let [a, b, c] =
                [indices[0], indices[1], indices[2]].map(|i| mesh.vertices[i as usize].position);
            let ab = (b - a).xz();
            let ac = (c - a).xz();
            let ap = (point - a).xz();
            let determinant = ab.perp_dot(ac);
            if determinant.abs() < 0.0000001 {
                return None;
            }
            let u = ap.perp_dot(ac) / determinant;
            let v = ab.perp_dot(ap) / determinant;
            (u >= -0.00001 && v >= -0.00001 && u + v <= 1.00001)
                .then_some(a.y + (b.y - a.y) * u + (c.y - a.y) * v)
        })
    }

    #[test]
    fn changing_banks_keep_asphalt_close_to_the_driving_plane() {
        let track =
            Track::parse("width 40\nstraight 20 bank 60\nstraight 1 bank -60\nstraight 20 bank 0")
                .unwrap();
        let mut coarse = Builder::new();
        for pair in track.samples.windows(2) {
            let [a, b] = [pair[0], pair[1]];
            coarse.quad(
                edge(a, -a.width * 0.5, 0.),
                edge(a, a.width * 0.5, 0.),
                edge(b, b.width * 0.5, 0.),
                edge(b, -b.width * 0.5, 0.),
                ASPHALT,
            );
        }
        let deviation = |mesh: &Mesh| {
            mesh.indices
                .as_chunks::<3>()
                .0
                .iter()
                .flat_map(|indices| {
                    let [a, b, c] = [indices[0], indices[1], indices[2]]
                        .map(|i| mesh.vertices[i as usize].position);
                    [
                        (a + b + c) / 3.0,
                        (a + b) * 0.5,
                        (b + c) * 0.5,
                        (c + a) * 0.5,
                    ]
                })
                .map(|point| {
                    // On this straight, level centerline, z is route distance.
                    // The interpolated normal is also the vehicle contact plane.
                    let sample = track.sample_at(point.z);
                    (point - sample.pos).dot(sample.up).abs()
                })
                .fold(0.0_f32, f32::max)
        };
        assert!(deviation(&coarse.finish()) > 0.5);
        let refined = deviation(&road_mesh(&track));
        assert!(
            refined < 0.025,
            "road departs from driving plane by {refined} m"
        );
        let flat = Track::parse("straight 40").unwrap();
        assert!(
            flat.samples
                .windows(2)
                .all(|pair| surface_steps(pair[0], pair[1]) == 1)
        );
    }

    #[test]
    fn short_banked_hills_keep_the_full_rendered_road_width() {
        for bank in [-60, 60] {
            for rise in [-0.6, 0.6] {
                let track = Track::parse(&format!(
                    "width 40\nstraight 20 bank {bank}\nstraight 1 rise {rise}\nstraight 20"
                ))
                .unwrap();
                for pair in track.samples.windows(2) {
                    let [a, b] = [pair[0], pair[1]];
                    let mut builder = Builder::new();
                    road_segment(&mut builder, &track, a, b, ASPHALT);
                    let mesh = builder.finish();
                    let quads = mesh.vertices.as_chunks::<4>().0;
                    for (index, quad) in quads.iter().enumerate() {
                        let distance = a.distance
                            + (b.distance - a.distance) * (index as f32 + 0.5) / quads.len() as f32;
                        let middle = track.sample_at(distance);
                        for (first, last, side) in [(0, 3, -1.0), (1, 2, 1.0)] {
                            let rendered = (quad[first].position + quad[last].position) * 0.5;
                            let expected = edge(middle, side * middle.width * 0.5, 0.0);
                            assert!(
                                rendered.distance(expected) < 0.025,
                                "road edge departs from driving width by {} m at {distance}, bank {bank}, rise {rise}",
                                rendered.distance(expected)
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn checkerboards_follow_rendered_hills_banks_and_gap_boundaries() {
        for source in [
            "straight 2 rise 1\nstraight 40",
            "width 40\nstraight 1 bank 60\nstraight 40",
        ] {
            let track = Track::parse(source).unwrap();
            let road = road_mesh(&track);
            for mesh in checkerboard(&track, 0.0) {
                for indices in mesh.indices.as_chunks::<3>().0.iter().step_by(7) {
                    let center = indices
                        .iter()
                        .map(|i| mesh.vertices[*i as usize].position)
                        .sum::<Vec3>()
                        / 3.0;
                    let road_y =
                        triangle_height(&road, center).expect("marking is over solid road");
                    assert!(
                        center.y > road_y,
                        "marking buried in road: {source}, {center:?}, {road_y}"
                    );
                    assert!(
                        center.y - road_y < 0.1,
                        "marking floats above road: {source}"
                    );
                }
            }
        }
        let track = Track::parse("straight 1\ngap 10\nstraight 40").unwrap();
        for mesh in checkerboard(&track, 0.0) {
            assert!(mesh.vertices.iter().all(|v| v.position.z <= 1.0));
        }
    }

    #[test]
    fn lane_markings_remain_above_widening_banked_hills() {
        let track = Track::parse(
            "width 12\nstraight 20\nwidth 40\nstraight 10 rise 1 bank 60\nstraight 20",
        )
        .unwrap();
        let chunks = World::build_chunks(&track);
        let asphalt_colors: [[u8; 4]; 2] = [ASPHALT.into(), tint(ASPHALT, 1.015).into()];
        let marking_color: [u8; 4] = CREAM.into();
        let mut road = Builder::new();
        let mut markings = Vec::new();
        for chunk in chunks.iter().filter(|chunk| !chunk.distant) {
            for indices in chunk.mesh.indices.as_chunks::<3>().0 {
                let vertices = indices.map(|i| chunk.mesh.vertices[i as usize]);
                if asphalt_colors.contains(&vertices[0].color) {
                    road.tri(
                        vertices[0].position,
                        vertices[1].position,
                        vertices[2].position,
                        ASPHALT,
                    );
                }
                // Lane stripes are the cream quads with 16 cm cross edges;
                // this excludes curbs, roadside furniture and distant gates.
                if vertices.iter().all(|v| v.color == marking_color)
                    && [(0, 1), (1, 2)].iter().any(|&(a, b)| {
                        (vertices[a].position.distance(vertices[b].position) - 0.16).abs() < 0.001
                    })
                {
                    markings.push(vertices.iter().map(|v| v.position).sum::<Vec3>() / 3.0);
                }
            }
        }
        let road = road.finish();
        let mut checked = 0;
        for point in markings {
            let height = triangle_height(&road, point).expect("stripe lies over the asphalt");
            assert!(
                point.y > height,
                "lane marking buried at {point:?}, road height {height}"
            );
            checked += 1;
        }
        assert!(checked > 100);
    }

    #[test]
    fn banked_shoulders_follow_the_contact_surface_between_samples() {
        let track = Track::parse("width 40\nstraight 20\nstraight 1 bank 60\nstraight 20").unwrap();
        let ground = track.ground_height();
        let mut builder = Builder::new();
        for pair in track.samples.windows(2) {
            for side in [-1.0, 1.0] {
                shoulder_segment(
                    &mut builder,
                    &track,
                    [pair[0], pair[1]],
                    ground,
                    side,
                    GRASS,
                );
            }
        }
        let mesh = builder.finish();
        for step in 1..40 {
            let sample = track.sample_at(20.0 + step as f32 / 40.0);
            for side in [-1.0, 1.0] {
                for fraction in [0.2, 0.5, 0.8] {
                    let point = edge(sample, side * (sample.width * 0.5 + 12.0 * fraction), 0.0);
                    let road_edge = edge(sample, side * sample.width * 0.5, 0.0);
                    let contact_y = road_edge.y + (ground - road_edge.y) * fraction;
                    let rendered_y = triangle_height(&mesh, point).unwrap();
                    assert!(
                        (rendered_y - contact_y).abs() < 0.05,
                        "shoulder differs from contact: {point:?}, rendered {rendered_y}, contact {contact_y}"
                    );
                }
            }
        }
    }

    #[test]
    fn elevated_curving_shoulders_follow_the_contact_surface() {
        let track =
            Track::parse("width 40\nstraight 1000 rise 500\nright 180 radius 40\nstraight 100")
                .unwrap();
        let ground = track.ground_height();
        let chunks = World::build_chunks(&track);
        let mut checked = 0;
        for pair in track.samples.windows(2) {
            // Inspect the level curve, whose complete shoulders fit outside
            // its center of curvature and do not overlap another road.
            if pair[0].pos.y < 499.9 || pair[0].forward.distance(pair[1].forward) < 0.01 {
                continue;
            }
            for side in [-1.0, 1.0] {
                for fraction in [0.2, 0.5, 0.8] {
                    for t in [0.25, 0.5, 0.75] {
                        let sample = surface_sample(&track, pair[0], pair[1], t);
                        let point =
                            edge(sample, side * (sample.width * 0.5 + 12.0 * fraction), 0.0);
                        let edge_y = edge(sample, side * sample.width * 0.5, 0.0).y;
                        let expected = edge_y + (ground - edge_y) * fraction;
                        let actual = chunks
                            .iter()
                            .filter(|chunk| !chunk.distant)
                            .find_map(|chunk| triangle_height(&chunk.mesh, point))
                            .expect("the rendered shoulder covers its contact footprint");
                        assert!(
                            (actual - expected).abs() < 0.05,
                            "shoulder error {} at {point:?}, side {side}, fraction {fraction}, t {t}",
                            (actual - expected).abs()
                        );
                        checked += 1;
                    }
                }
            }
        }
        assert!(checked > 1_000);
    }

    #[test]
    fn rapid_bank_markings_and_road_chunks_fit_renderer_limits() {
        let check_chunks = |track: &Track| {
            for chunk in World::build_chunks(track) {
                assert!(chunk.mesh.vertices.len() <= 60_000);
                assert!(chunk.mesh.indices.len() <= 120_000);
                assert!(
                    chunk
                        .mesh
                        .indices
                        .iter()
                        .all(|i| (*i as usize) < chunk.mesh.vertices.len())
                );
            }
        };
        for kind in ["road", "tunnel"] {
            let track = Track::parse(&format!(
                "width 40\nkind {kind}\nstraight 1 bank 60\nstraight 1 bank -60\nstraight 30\nstraight 1 bank 60\nstraight 1 bank -60\nstraight 2",
            ))
            .unwrap();
            check_chunks(&track);

            // Frame refinement adds slices to steep banked hills even when
            // the bank stays constant. Exercise those alongside the widest
            // legal width changes and repeated shoulder height transitions.
            let mut source = format!("width 40\nkind {kind}\nstraight 20 bank 60\n");
            source.push_str(
                &"width 4\nstraight 1 rise 0.6\nwidth 40\nstraight 1 rise -0.6\n".repeat(80),
            );
            check_chunks(&Track::parse(&source).unwrap());
        }
        let mut source = String::from("width 40\nstraight 20 bank 60\n");
        source.push_str(&"width 4\nstraight 1\nwidth 40\nstraight 1\n".repeat(40));
        let track = Track::parse(&source).unwrap();
        check_chunks(&track);

        // Curvature refinement is largest when a wide, banked road is at
        // the maximum height above terrain. It must still fit GPU batches.
        let mut source = String::from("start 0 -1000 0\nwidth 40\n");
        source.push_str(&"straight 1000 rise 500\n".repeat(6));
        source.push_str("right 180 radius 40 bank 60\n");
        source.push_str(&"width 4\nstraight 1\nwidth 40\nstraight 1\n".repeat(40));
        check_chunks(&Track::parse(&source).unwrap());
    }

    #[test]
    fn nearby_bridge_supports_remain_visible_below_a_distant_deck() {
        let track = Track::parse(
            "straight 100\nright 180 radius 300 rise 100\nstraight 1000 rise 500\nright 180 radius 300 rise 400\nbridge 1000\nfinish",
        )
        .unwrap();
        let eye = vec3(0., 1.4, 5.);
        let chunks = World::build_chunks(&track);
        let nearby_supports: Vec<_> = chunks
            .iter()
            .filter(|chunk| {
                !chunk.distant
                    && chunk.mesh.vertices.iter().any(|v| v.position.y > 900.)
                    && chunk
                        .mesh
                        .vertices
                        .iter()
                        .any(|v| v.position.distance(eye) < 50.)
            })
            .collect();
        assert!(!nearby_supports.is_empty());
        for chunk in nearby_supports {
            assert!(chunk.visible_from(eye), "nearby bridge support was culled");
            assert!(!chunk.visible_from(eye + Vec3::X * 10_000.));
        }
    }

    #[test]
    fn roadside_signs_stand_on_the_rendered_shoulders_of_banked_hills() {
        for bank in [-45, 0, 45] {
            let track = Track::parse(&format!(
                "straight 100\nstraight 1000 rise 500 bank {bank}\nstraight 100"
            ))
            .unwrap();
            let pair = &track.samples[300..302];
            let sample = surface_sample(&track, pair[0], pair[1], 0.5);
            for side in [-1., 1.] {
                let mut shoulder = Builder::new();
                shoulder_segment(
                    &mut shoulder,
                    &track,
                    [pair[0], pair[1]],
                    track.ground_height(),
                    side,
                    GRASS,
                );
                let mut sign = Builder::new();
                roadside_sign(&mut sign, sample, track.ground_height(), side);
                let sign = sign.finish();
                let board_top = sign.vertices[..4].iter().map(|v| v.position).sum::<Vec3>() * 0.25;
                let shoulder_y = triangle_height(&shoulder.finish(), board_top).unwrap();
                assert!(
                    (board_top.y - shoulder_y - 1.75).abs() < 0.05,
                    "sign floats above its shoulder at bank {bank}, side {side}: top {}, shoulder {shoulder_y}",
                    board_top.y
                );
            }
        }
    }

    #[test]
    fn trees_stay_outside_the_shoulders_of_neighboring_straights() {
        let track = Track::parse("straight 500\nright 180 radius 20\nstraight 500").unwrap();
        let trunk_top: [u8; 4] = tint(Color::new(0.33, 0.28, 0.21, 1.), 1.15).into();
        let mut trunks = 0;
        for chunk in World::build_chunks(&track) {
            for vertex in chunk.mesh.vertices {
                let p = vertex.position;
                if vertex.color == trunk_top && (20.0..480.0).contains(&p.z) {
                    trunks += 1;
                    // The parallel straights are at x = 0 and x = 40. Their
                    // drivable shoulders extend 18 m from each centerline.
                    let clearance = p.x.abs().min((p.x - 40.0).abs());
                    assert!(clearance > 18.0, "tree intrudes into a shoulder at {p:?}");
                }
            }
        }
        assert!(trunks > 0, "the course must still contain roadside trees");
    }
}
