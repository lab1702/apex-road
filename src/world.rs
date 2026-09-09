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

struct Chunk {
    center: Vec3,
    mesh: Mesh,
    distant: bool,
}
pub struct World {
    chunks: Vec<Chunk>,
    material: Material,
}
impl World {
    pub fn new(track: &Track) -> Self {
        let ground_y = track.ground_height();
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
                b.quad(
                    edge(a, -w, 0.),
                    edge(a, w, 0.),
                    edge(z, wz, 0.),
                    edge(z, -wz, 0.),
                    tint(ASPHALT, shade),
                );
                for side in [-1., 1.] {
                    // A generous shoulder matches the off-road contact surface.
                    if matches!(a.kind, RoadKind::Road | RoadKind::Ramp) {
                        let mut far_a = edge(a, side * (w + 12.), 0.);
                        far_a.y = ground_y;
                        let mut far_z = edge(z, side * (wz + 12.), 0.);
                        far_z.y = ground_y;
                        b.quad(
                            edge(a, side * w, -0.025),
                            far_a,
                            far_z,
                            edge(z, side * wz, -0.025),
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
                    if matches!(a.kind, RoadKind::Bridge | RoadKind::Tunnel) {
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
                        // Avoid planting trees on nearby stretches of a folded track.
                        if track.samples.iter().step_by(5).all(|s| {
                            vec2(s.pos.x - pos.x, s.pos.z - pos.z).length() > s.width * 0.5 + 8.
                        }) {
                            tree(&mut b, pos, 5. + noise(seed + 7) * 8., seed);
                        }
                    }
                }
                if i % 40 == 0 && a.kind == RoadKind::Road {
                    let side = if i % 80 == 0 { 1. } else { -1. };
                    let mut p = edge(a, side * (w + 9.), 0.);
                    p.y -= 1.8;
                    b.block(
                        p + Vec3::Y * 0.65,
                        a.right,
                        Vec3::Y,
                        a.forward,
                        vec3(3., 1.3, 0.16),
                        Color::new(0.12, 0.27, 0.28, 1.),
                    );
                    b.block(
                        p - Vec3::Y * 0.4,
                        a.right,
                        Vec3::Y,
                        a.forward,
                        vec3(0.12, 1.5, 0.14),
                        CREAM,
                    );
                    b.block(
                        p + Vec3::Y * 0.65 - a.forward * 0.1,
                        a.right,
                        Vec3::Y,
                        a.forward,
                        vec3(2.45, 0.16, 0.03),
                        TEAL,
                    );
                }
            }
            chunks.push(Chunk {
                center: points[points.len() / 2][0].pos,
                mesh: b.finish(),
                distant: false,
            });
        }
        let mut gates = Builder::new();
        // Start/finish and checkpoint gates make progress visible in the world.
        for (idx, d) in std::iter::once(0.)
            .chain(track.checkpoints.iter().copied())
            .chain(std::iter::once(track.length - 3.))
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
            if idx == 0 || d > track.length - 4. {
                for x in 0..16 {
                    for row in 0..2 {
                        let c = if (x + row) % 2 == 0 { CREAM } else { ASPHALT };
                        let left = -s.width * 0.5 + x as f32 * s.width / 16.;
                        let front = row as f32 * 0.7;
                        gates.quad(
                            edge(s, left, 0.04) + s.forward * front,
                            edge(s, left + s.width / 16., 0.04) + s.forward * front,
                            edge(s, left + s.width / 16., 0.04) + s.forward * (front + 0.7),
                            edge(s, left, 0.04) + s.forward * (front + 0.7),
                            c,
                        );
                    }
                }
            }
            if idx % 32 == 31 {
                chunks.push(Chunk {
                    center: s.pos,
                    mesh: gates.finish(),
                    distant: true,
                });
                gates = Builder::new();
            }
        }
        chunks.push(Chunk {
            center: Vec3::ZERO,
            mesh: gates.finish(),
            distant: true,
        });
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
        chunks.push(Chunk {
            center: mid,
            mesh: ground.finish(),
            distant: true,
        });
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
            chunks.push(Chunk {
                center: p,
                mesh: b.finish(),
                distant: true,
            });
        }
        Self { chunks, material }
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
            if c.distant || c.center.distance(camera.position) < 900. {
                draw_mesh(&c.mesh);
            }
        }
        gl_use_default_material();
        set_default_camera();
    }
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
