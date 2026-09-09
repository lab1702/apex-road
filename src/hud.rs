//! The race UI is deliberately small: the world is the cockpit.
use macroquad::prelude::*;
use std::cell::RefCell;

use crate::track::Track;

const PAPER: Color = Color::new(0.94, 0.95, 0.90, 1.0);
const MUTED: Color = Color::new(0.61, 0.69, 0.70, 1.0);
const CYAN: Color = Color::new(0.38, 0.89, 0.84, 1.0);
const ORANGE: Color = Color::new(1.0, 0.60, 0.34, 1.0);
const PANEL: Color = Color::new(0.035, 0.065, 0.075, 0.88);

thread_local! {
    static HUD_FONT: RefCell<Option<Font>> = const { RefCell::new(None) };
}

/// Load the embedded UI face once, after the graphics context has been created.
pub fn init() {
    HUD_FONT.with(|slot| {
        if slot.borrow().is_none() {
            let mut font = load_ttf_font_from_bytes(include_bytes!("../assets/DejaVuSans.ttf"))
                .expect("the embedded DejaVu Sans font is valid");
            font.set_filter(FilterMode::Linear);
            *slot.borrow_mut() = Some(font);
        }
    });
}

/// Release the font's GPU atlas while the graphics context still exists.
/// Call before returning from the game's async function, after the last draw.
pub fn shutdown() {
    HUD_FONT.with(|slot| drop(slot.borrow_mut().take()));
}

fn measure(label: &str, size: f32) -> TextDimensions {
    HUD_FONT.with(|slot| {
        measure_text(
            label,
            slot.borrow().as_ref(),
            size.max(1.0).round() as u16,
            1.0,
        )
    })
}

pub struct HudState<'a> {
    pub track_name: &'a str,
    /// Speed in kilometers per hour.
    pub speed: f32,
    pub rpm: f32,
    pub gear: u8,
    pub throttle: f32,
    pub brake: f32,
    pub steering: f32,
    pub slip: f32,
    pub elapsed: f32,
    pub best: Option<f32>,
    pub last: Option<f32>,
    pub checkpoint: usize,
    pub checkpoint_count: usize,
    pub progress: f32,
    pub paused: bool,
    pub started: bool,
    pub finished: bool,
    pub invalid: bool,
    pub airborne: bool,
    pub offroad: bool,
    pub mouse_enabled: bool,
    pub autodrive: bool,
    pub help: bool,
    pub notification: Option<&'a str>,
    pub fps: i32,
}

fn alpha(color: Color, opacity: f32) -> Color {
    Color {
        a: opacity,
        ..color
    }
}

fn text(label: &str, x: f32, y: f32, size: f32, color: Color) {
    HUD_FONT.with(|slot| {
        let font = slot.borrow();
        draw_text_ex(
            label,
            x,
            y,
            TextParams {
                font: font.as_ref(),
                font_size: size.max(1.0).round() as u16,
                color,
                ..Default::default()
            },
        );
    });
}

fn text_right(label: &str, right: f32, y: f32, size: f32, color: Color) {
    let width = measure(label, size).width;
    text(label, right - width, y, size, color);
}

fn text_center(label: &str, center: f32, y: f32, size: f32, color: Color) {
    let width = measure(label, size).width;
    text(label, center - width * 0.5, y, size, color);
}

/// Wrap user-provided text at word boundaries, splitting long paths or words
/// when needed. The final line uses an ellipsis if the available space is full.
fn fit_lines(
    label: &str,
    width: f32,
    max_lines: usize,
    measure_width: impl Fn(&str) -> f32,
) -> Vec<String> {
    let normalized = label.split_whitespace().collect::<Vec<_>>().join(" ");
    let mut remaining = normalized.as_str();
    let mut lines = Vec::new();
    if width <= 0.0 || measure_width("…") > width {
        return lines;
    }
    while !remaining.is_empty() && lines.len() < max_lines {
        if measure_width(remaining) <= width {
            lines.push(remaining.to_owned());
            break;
        }
        let last_line = lines.len() + 1 == max_lines;
        let mut end = 0;
        let mut word_break = None;
        for (offset, character) in remaining.char_indices() {
            let next = offset + character.len_utf8();
            let candidate = &remaining[..next];
            let candidate_width = if last_line {
                measure_width(&format!("{candidate}…"))
            } else {
                measure_width(candidate)
            };
            if candidate_width > width {
                break;
            }
            end = next;
            if character.is_whitespace() {
                word_break = Some(offset);
            }
        }
        if last_line || end == 0 {
            lines.push(format!("{}…", remaining[..end].trim_end()));
            break;
        }
        if !remaining[end..].starts_with(char::is_whitespace) {
            end = word_break.filter(|offset| *offset > 0).unwrap_or(end);
        }
        lines.push(remaining[..end].trim_end().to_owned());
        remaining = remaining[end..].trim_start();
    }
    lines
}

fn tracking(label: &str, x: f32, y: f32, size: f32, spacing: f32, color: Color) {
    let size = size * 1.1;
    let mut cursor = x;
    for character in label.chars() {
        let glyph = character.to_string();
        text(&glyph, cursor, y, size, color);
        cursor += measure(&glyph, size).width + spacing * 0.7;
    }
}

fn panel(x: f32, y: f32, width: f32, height: f32, radius: f32, color: Color) {
    if width <= 0.0 || height <= 0.0 {
        return;
    }
    let r = radius.min(width * 0.5).min(height * 0.5);
    let center = vec2(x + width * 0.5, y + height * 0.5);
    let corners = [
        (vec2(x + r, y + r), std::f32::consts::PI),
        (vec2(x + width - r, y + r), -std::f32::consts::FRAC_PI_2),
        (vec2(x + width - r, y + height - r), 0.0),
        (vec2(x + r, y + height - r), std::f32::consts::FRAC_PI_2),
    ];
    let mut outline = [Vec2::ZERO; 36];
    for (corner, (origin, start)) in corners.into_iter().enumerate() {
        for step in 0..=8 {
            let angle = start + step as f32 / 8.0 * std::f32::consts::FRAC_PI_2;
            outline[corner * 9 + step] = origin + vec2(angle.cos(), angle.sin()) * r;
        }
    }
    // Adjacent fan triangles share only their edges, so translucent corners
    // receive exactly the same single blend as the center of the panel.
    for i in 0..outline.len() {
        draw_triangle(center, outline[i], outline[(i + 1) % outline.len()], color);
    }
}

fn start_scrim(y: f32, u: f32) {
    let mut vertices = Vec::with_capacity(25 * 13);
    let mut indices = Vec::with_capacity(24 * 12 * 6);
    for row in 0..=12 {
        let v = row as f32 / 12.0;
        let vertical = (v * std::f32::consts::PI).sin().max(0.0).powf(0.6);
        for column in 0..=24 {
            let f = column as f32 / 24.0;
            vertices.push(Vertex::new2(
                vec3(f * screen_width() * 0.69, y + (-95.0 + v * 400.0) * u, 0.0),
                Vec2::ZERO,
                alpha(PANEL, 0.69 * (1.0 - f).powi(2) * vertical),
            ));
            if row < 12 && column < 24 {
                let n = (row * 25 + column) as u16;
                indices.extend_from_slice(&[n, n + 1, n + 26, n, n + 26, n + 25]);
            }
        }
    }
    draw_mesh(&Mesh {
        vertices,
        indices,
        texture: None,
    });
}

fn key(label: &str, x: f32, y: f32, width: f32, u: f32) {
    panel(x, y, width, 24.0 * u, 4.0 * u, alpha(PAPER, 0.09));
    text_center(label, x + width * 0.5, y + 17.0 * u, 13.0 * u, PAPER);
}

fn time_string(seconds: f32) -> String {
    let millis = (seconds.max(0.0) * 1000.0) as u64;
    format!(
        "{:02}:{:02}.{:03}",
        millis / 60_000,
        (millis / 1000) % 60,
        millis % 1000
    )
}

fn optional_time(seconds: Option<f32>) -> String {
    seconds
        .map(time_string)
        .unwrap_or_else(|| "--:--.---".to_owned())
}

fn track_map(track: &Track, position: Vec3, heading: f32, x: f32, y: f32, u: f32) {
    if track.samples.is_empty() {
        return;
    }
    let width = 224.0 * u;
    let height = 167.0 * u;
    panel(x, y, width, height, 12.0 * u, alpha(PANEL, 0.62));
    tracking(
        "THE COURSE",
        x + 17.0 * u,
        y + 25.0 * u,
        10.0 * u,
        1.6 * u,
        MUTED,
    );
    text_right(
        &format!("{:.1} KM", track.length / 1000.0),
        x + width - 17.0 * u,
        y + 25.0 * u,
        11.0 * u,
        PAPER,
    );

    let mut min = vec2(f32::INFINITY, f32::INFINITY);
    let mut max = vec2(f32::NEG_INFINITY, f32::NEG_INFINITY);
    for sample in &track.samples {
        let p = vec2(sample.pos.x, sample.pos.z);
        min = min.min(p);
        max = max.max(p);
    }
    let size = max - min;
    let map_extent = vec2(width - 47.0 * u, height - 63.0 * u);
    let map_scale = (map_extent.x / size.x.max(1.0)).min(map_extent.y / size.y.max(1.0));
    let center = (min + max) * 0.5;
    let map_center = vec2(x + width * 0.5, y + 42.0 * u + (height - 54.0 * u) * 0.5);
    let project = |p: Vec3| map_center + vec2(p.x - center.x, -(p.z - center.y)) * map_scale;
    for pair in track.samples.windows(2) {
        let a = project(pair[0].pos);
        let b = project(pair[1].pos);
        draw_line(a.x, a.y, b.x, b.y, 5.0 * u, alpha(BLACK, 0.40));
        draw_line(a.x, a.y, b.x, b.y, 2.3 * u, alpha(PAPER, 0.56));
    }
    let start = project(track.samples[0].pos);
    draw_circle(start.x, start.y, 3.0 * u, PAPER);
    // Keep the entire marker within the map when the car drives beyond the
    // course bounds. The projection's padding also contains its 9 px halo.
    let car = project(position).clamp(map_center - map_extent * 0.5, map_center + map_extent * 0.5);
    // In world space heading zero points along +Z.
    let forward = vec2(heading.sin(), -heading.cos());
    let right = vec2(-forward.y, forward.x);
    draw_circle(car.x, car.y, 9.0 * u, alpha(CYAN, 0.12));
    draw_triangle(
        car + forward * 6.5 * u,
        car - forward * 4.5 * u + right * 4.0 * u,
        car - forward * 4.5 * u - right * 4.0 * u,
        CYAN,
    );
}

fn gauges(state: &HudState<'_>, u: f32) {
    let width = 490.0 * u;
    let height = 135.0 * u;
    let x = (screen_width() - width) * 0.5;
    let y = screen_height() - height - 31.0 * u;
    panel(x, y + 5.0 * u, width, height, 18.0 * u, alpha(BLACK, 0.15));
    panel(x, y, width, height, 18.0 * u, PANEL);
    draw_line(
        x + 24.0 * u,
        y,
        x + width - 24.0 * u,
        y,
        1.0 * u,
        alpha(PAPER, 0.12),
    );

    // A clean segmented tachometer caps the entire floating instrument pod.
    let revs = ((state.rpm - 850.0) / 6650.0).clamp(0.0, 1.0);
    for i in 0..32 {
        let fraction = (i + 1) as f32 / 32.0;
        let color = if fraction <= revs {
            if i > 26 { ORANGE } else { CYAN }
        } else {
            alpha(PAPER, 0.12)
        };
        draw_rectangle(
            x + (24.0 + i as f32 * 13.85) * u,
            y + 17.0 * u,
            10.3 * u,
            4.0 * u,
            color,
        );
    }

    let speed = format!("{:03}", state.speed.abs().round() as u32);
    text(&speed, x + 27.0 * u, y + 91.0 * u, 67.0 * u, PAPER);
    tracking(
        "KM/H",
        x + 31.0 * u,
        y + 116.0 * u,
        11.0 * u,
        2.0 * u,
        MUTED,
    );
    draw_line(
        x + 170.0 * u,
        y + 39.0 * u,
        x + 170.0 * u,
        y + 113.0 * u,
        u,
        alpha(PAPER, 0.12),
    );

    let gear = match state.gear {
        0 => "N".to_owned(),
        7 => "R".to_owned(),
        value => value.to_string(),
    };
    text_center(&gear, x + 206.0 * u, y + 88.0 * u, 47.0 * u, CYAN);
    text_center("GEAR", x + 206.0 * u, y + 113.0 * u, 10.0 * u, MUTED);
    draw_line(
        x + 241.0 * u,
        y + 39.0 * u,
        x + 241.0 * u,
        y + 113.0 * u,
        u,
        alpha(PAPER, 0.12),
    );

    let tx = x + 264.0 * u;
    tracking("THROTTLE", tx, y + 47.0 * u, 9.0 * u, 1.1 * u, MUTED);
    panel(
        tx,
        y + 55.0 * u,
        88.0 * u,
        5.0 * u,
        2.0 * u,
        alpha(PAPER, 0.12),
    );
    if state.throttle > 0.0 {
        panel(
            tx,
            y + 55.0 * u,
            88.0 * u * state.throttle.clamp(0.0, 1.0),
            5.0 * u,
            2.0 * u,
            CYAN,
        );
    }
    tracking("BRAKE", tx, y + 81.0 * u, 9.0 * u, 1.1 * u, MUTED);
    panel(
        tx,
        y + 89.0 * u,
        88.0 * u,
        5.0 * u,
        2.0 * u,
        alpha(PAPER, 0.12),
    );
    if state.brake > 0.0 {
        panel(
            tx,
            y + 89.0 * u,
            88.0 * u * state.brake.clamp(0.0, 1.0),
            5.0 * u,
            2.0 * u,
            ORANGE,
        );
    }
    // Steering is a tiny center-zero indicator for either input method.
    draw_line(
        tx,
        y + 114.0 * u,
        tx + 88.0 * u,
        y + 114.0 * u,
        u,
        alpha(PAPER, 0.24),
    );
    draw_line(
        tx + 44.0 * u,
        y + 109.0 * u,
        tx + 44.0 * u,
        y + 118.0 * u,
        u,
        alpha(PAPER, 0.32),
    );
    draw_circle(
        tx + (44.0 + state.steering.clamp(-1.0, 1.0) * 41.0) * u,
        y + 114.0 * u,
        2.5 * u,
        PAPER,
    );

    let grip_x = x + 413.0 * u;
    let grip_y = y + 69.0 * u;
    let grip_color = if state.airborne || state.slip > 0.25 || state.offroad {
        ORANGE
    } else {
        CYAN
    };
    draw_circle_lines(grip_x, grip_y, 25.0 * u, 1.0 * u, alpha(PAPER, 0.18));
    draw_line(
        grip_x - 17.0 * u,
        grip_y,
        grip_x + 17.0 * u,
        grip_y,
        u,
        alpha(PAPER, 0.14),
    );
    draw_line(
        grip_x,
        grip_y - 17.0 * u,
        grip_x,
        grip_y + 17.0 * u,
        u,
        alpha(PAPER, 0.14),
    );
    let shift = state.slip.clamp(0.0, 1.0) * 16.0 * u;
    let slip_pos = vec2(
        grip_x + state.steering.clamp(-1.0, 1.0) * shift,
        grip_y + (state.brake - state.throttle) * shift,
    );
    draw_circle(slip_pos.x, slip_pos.y, 7.0 * u, alpha(grip_color, 0.14));
    draw_circle(slip_pos.x, slip_pos.y, 3.2 * u, grip_color);
    let status = if state.airborne {
        "AIRBORNE"
    } else if state.offroad {
        "OFF ROAD"
    } else if state.slip > 0.25 {
        "TRACTION"
    } else {
        "GRIP"
    };
    text_center(status, grip_x, y + 113.0 * u, 10.0 * u, grip_color);
}

fn controls_overlay(state: &HudState<'_>, u: f32) {
    let u = u.min(screen_width() / 650.0).min(screen_height() / 750.0);
    draw_rectangle(
        0.0,
        0.0,
        screen_width(),
        screen_height(),
        alpha(PANEL, 0.73),
    );
    let width = 610.0 * u;
    let height = 710.0 * u;
    let x = (screen_width() - width) * 0.5;
    let y = (screen_height() - height) * 0.5;
    panel(x, y, width, height, 20.0 * u, alpha(PANEL, 0.97));
    tracking(
        "APEX / ROAD",
        x + 32.0 * u,
        y + 37.0 * u,
        11.0 * u,
        2.0 * u,
        CYAN,
    );
    text(
        if state.help {
            "Make every corner count."
        } else {
            "Take a breath."
        },
        x + 32.0 * u,
        y + 84.0 * u,
        30.0 * u,
        PAPER,
    );
    text(
        if state.help {
            "DRIVING CONTROLS"
        } else {
            "TIME TRIAL PAUSED"
        },
        x + 32.0 * u,
        y + 111.0 * u,
        11.0 * u,
        MUTED,
    );
    let bindings = [
        ("W / UP", "Accelerate"),
        ("S / DOWN", "Brake / hold to reverse"),
        ("A D / ← →", "Steer left / right"),
        ("RIGHT CLICK", "Toggle mouse driving anywhere"),
        ("MOUSE L / R", "Steer left / right"),
        ("MOUSE UP", "More throttle / less brake"),
        ("MOUSE DOWN", "Less throttle / more brake"),
        ("SPACE", "Handbrake"),
        ("R", "Restart time trial"),
        ("TAB", "Next track"),
        ("F5", "Reload track file"),
        ("ESC", "Pause / resume"),
        ("F11", "Toggle fullscreen"),
        ("Q", "Quit while paused"),
    ];
    for (i, (button, description)) in bindings.iter().enumerate() {
        let row = y + (141.0 + i as f32 * 30.0) * u;
        key(button, x + 32.0 * u, row, 130.0 * u, u);
        text(description, x + 182.0 * u, row + 17.0 * u, 16.0 * u, PAPER);
    }
    text(
        "Hold the mouse still to keep inputs; move back to ease them off.",
        x + 32.0 * u,
        y + 584.0 * u,
        13.0 * u,
        MUTED,
    );
    text(
        "Pausing clears mouse inputs. Resume with neutral controls.",
        x + 32.0 * u,
        y + 607.0 * u,
        13.0 * u,
        MUTED,
    );
    draw_line(
        x + 32.0 * u,
        y + 633.0 * u,
        x + width - 32.0 * u,
        y + 633.0 * u,
        u,
        alpha(PAPER, 0.12),
    );
    text(
        if state.help {
            "F1   CLOSE GUIDE"
        } else {
            "ESC / ENTER   RESUME"
        },
        x + 32.0 * u,
        y + 674.0 * u,
        14.0 * u,
        CYAN,
    );
    text_right(
        &format!("{} FPS", state.fps),
        x + width - 32.0 * u,
        y + 674.0 * u,
        11.0 * u,
        MUTED,
    );
}

/// Draw the windshield overlay after the 3D scene, using Macroquad's default 2D camera.
pub fn draw(state: &HudState<'_>, track: &Track, position: Vec3, heading: f32) {
    let w = screen_width();
    let h = screen_height();
    // Keep the full layout inside small resizable windows as well as large ones.
    let u = (w / 1440.0).min(h / 900.0).min(1.75);
    let margin = 37.0 * u;
    let timer_x = w - margin - 224.0 * u;

    // Soft edge shade keeps white UI legible against snow and sky without a cockpit frame.
    for i in 0..64 {
        let t = i as f32 / 64.0;
        draw_rectangle(
            0.0,
            i as f32 * 2.5 * u,
            w,
            2.6 * u,
            alpha(BLACK, (1.0 - t).powi(2) * 0.43),
        );
        draw_rectangle(
            0.0,
            h - (i + 1) as f32 * 2.5 * u,
            w,
            2.6 * u,
            alpha(BLACK, (1.0 - t).powi(2) * 0.32),
        );
    }

    // Compact wordmark, with a track-like double apex motif.
    draw_line(
        margin,
        margin + 17.0 * u,
        margin + 11.0 * u,
        margin - 1.0 * u,
        3.0 * u,
        CYAN,
    );
    draw_line(
        margin + 11.0 * u,
        margin - 1.0 * u,
        margin + 20.0 * u,
        margin + 17.0 * u,
        3.0 * u,
        CYAN,
    );
    draw_line(
        margin + 16.0 * u,
        margin + 17.0 * u,
        margin + 25.0 * u,
        margin + 2.0 * u,
        3.0 * u,
        alpha(CYAN, 0.55),
    );
    tracking(
        "APEX / ROAD",
        margin + 39.0 * u,
        margin + 15.0 * u,
        19.0 * u,
        2.3 * u,
        PAPER,
    );
    let name_right = if w >= 800.0 && state.started {
        timer_x.min((w - 248.0 * u) * 0.5)
    } else {
        timer_x
    };
    for name in fit_lines(
        state.track_name,
        name_right - margin - 16.0 * u,
        1,
        |label| measure(label, 18.0 * u).width,
    ) {
        text(&name, margin, margin + 49.0 * u, 18.0 * u, PAPER);
    }
    panel(
        margin,
        margin + 63.0 * u,
        106.0 * u,
        23.0 * u,
        4.0 * u,
        alpha(CYAN, 0.12),
    );
    tracking(
        "TIME TRIAL",
        margin + 10.0 * u,
        margin + 79.0 * u,
        10.0 * u,
        1.2 * u,
        CYAN,
    );

    let timer_y = margin - 10.0 * u;
    panel(
        timer_x,
        timer_y,
        224.0 * u,
        160.0 * u,
        12.0 * u,
        alpha(PANEL, 0.76),
    );
    tracking(
        if state.finished {
            "FINISH TIME"
        } else {
            "CURRENT RUN"
        },
        timer_x + 17.0 * u,
        timer_y + 26.0 * u,
        10.0 * u,
        1.7 * u,
        MUTED,
    );
    text(
        &time_string(state.elapsed),
        timer_x + 15.0 * u,
        timer_y + 67.0 * u,
        37.0 * u,
        if state.invalid { ORANGE } else { PAPER },
    );
    draw_line(
        timer_x + 17.0 * u,
        timer_y + 83.0 * u,
        timer_x + 207.0 * u,
        timer_y + 83.0 * u,
        u,
        alpha(PAPER, 0.14),
    );
    text(
        "BEST",
        timer_x + 17.0 * u,
        timer_y + 109.0 * u,
        11.0 * u,
        CYAN,
    );
    text_right(
        &optional_time(state.best),
        timer_x + 207.0 * u,
        timer_y + 109.0 * u,
        17.0 * u,
        PAPER,
    );
    text(
        "LAST",
        timer_x + 17.0 * u,
        timer_y + 137.0 * u,
        11.0 * u,
        MUTED,
    );
    text_right(
        &optional_time(state.last),
        timer_x + 207.0 * u,
        timer_y + 137.0 * u,
        17.0 * u,
        MUTED,
    );
    track_map(track, position, heading, timer_x, timer_y + 174.0 * u, u);

    if w >= 800.0 && state.started {
        let ribbon_w = 248.0 * u;
        let rx = (w - ribbon_w) * 0.5;
        panel(
            rx,
            margin - 8.0 * u,
            ribbon_w,
            51.0 * u,
            8.0 * u,
            alpha(PANEL, 0.62),
        );
        text(
            "CHECKPOINT",
            rx + 16.0 * u,
            margin + 14.0 * u,
            10.0 * u,
            MUTED,
        );
        text_right(
            &format!(
                "{:02} / {:02}",
                state.checkpoint.min(state.checkpoint_count),
                state.checkpoint_count
            ),
            rx + ribbon_w - 16.0 * u,
            margin + 15.0 * u,
            14.0 * u,
            PAPER,
        );
        draw_rectangle(
            rx + 16.0 * u,
            margin + 26.0 * u,
            ribbon_w - 32.0 * u,
            3.0 * u,
            alpha(PAPER, 0.16),
        );
        draw_rectangle(
            rx + 16.0 * u,
            margin + 26.0 * u,
            (ribbon_w - 32.0 * u) * state.progress.clamp(0.0, 1.0),
            3.0 * u,
            CYAN,
        );
    }

    gauges(state, u);

    text_center(
        if state.autodrive {
            "AUTO DRIVER  ·  RIGHT CLICK TO TAKE CONTROL"
        } else if state.mouse_enabled {
            "MOUSE  ·  RIGHT CLICK FOR KEYBOARD"
        } else {
            "KEYBOARD  ·  RIGHT CLICK FOR MOUSE"
        },
        w * 0.5,
        h - 12.0 * u,
        10.0 * u,
        if state.mouse_enabled || state.autodrive {
            CYAN
        } else {
            MUTED
        },
    );

    if w > 1050.0 {
        let hint_y = h - 50.0 * u;
        key(
            if state.autodrive {
                "AUTO"
            } else if state.mouse_enabled {
                "MOUSE"
            } else {
                "W A S D"
            },
            margin,
            hint_y - 26.0 * u,
            76.0 * u,
            u,
        );
        text(
            "DRIVE",
            margin + 88.0 * u,
            hint_y - 9.0 * u,
            10.0 * u,
            PAPER,
        );
        key("F1", margin, hint_y + 7.0 * u, 27.0 * u, u);
        text(
            "CONTROLS",
            margin + 38.0 * u,
            hint_y + 24.0 * u,
            10.0 * u,
            MUTED,
        );
        text_right(
            "R  RESTART     ESC  PAUSE",
            w - margin,
            h - 38.0 * u,
            11.0 * u,
            alpha(PAPER, 0.72),
        );
    }

    if state.invalid && state.started && !state.finished {
        let label = "RUN INVALID  /  MISSED CHECKPOINT";
        let ww = 314.0 * u;
        let xx = (w - ww) * 0.5;
        panel(xx, h - 204.0 * u, ww, 26.0 * u, 5.0 * u, alpha(PANEL, 0.86));
        text_center(label, w * 0.5, h - 186.0 * u, 11.0 * u, ORANGE);
    }
    if let Some(notification) = state.notification {
        let size = 16.0 * u;
        let lines = fit_lines(notification, w - 2.0 * margin - 42.0 * u, 6, |label| {
            measure(label, size).width
        });
        if !lines.is_empty() {
            let notification_w = lines
                .iter()
                .map(|line| measure(line, size).width)
                .fold(0.0, f32::max)
                + 42.0 * u;
            panel(
                (w - notification_w) * 0.5,
                h * 0.24,
                notification_w,
                (41.0 + (lines.len() - 1) as f32 * 22.0) * u,
                8.0 * u,
                alpha(PANEL, 0.90),
            );
            for (index, line) in lines.iter().enumerate() {
                text_center(
                    line,
                    w * 0.5,
                    h * 0.24 + (26.0 + index as f32 * 22.0) * u,
                    size,
                    PAPER,
                );
            }
        }
    }

    if !state.started && !state.help {
        let x = w * 0.075;
        let y = h * 0.37;
        // Vertex-interpolated transparency avoids overlapping strips and fades
        // both horizontally and vertically into the windshield view.
        start_scrim(y, u);
        tracking(
            "YOU. THE ROAD. THE CLOCK.",
            x,
            y - 9.0 * u,
            11.0 * u,
            2.0 * u,
            CYAN,
        );
        text("Find your", x - 2.0 * u, y + 54.0 * u, 64.0 * u, PAPER);
        text("racing line.", x - 2.0 * u, y + 116.0 * u, 64.0 * u, PAPER);
        text(
            "Smooth inputs. Late apexes. One perfect run.",
            x,
            y + 150.0 * u,
            16.0 * u,
            alpha(PAPER, 0.80),
        );
        panel(x, y + 179.0 * u, 224.0 * u, 43.0 * u, 6.0 * u, CYAN);
        tracking(
            "ENTER  /  START ENGINE",
            x + 16.0 * u,
            y + 206.0 * u,
            11.0 * u,
            0.7 * u,
            PANEL,
        );
        text(
            "TAB  CHANGE TRACK    F1  CONTROLS",
            x,
            y + 249.0 * u,
            11.0 * u,
            MUTED,
        );
    }

    if state.finished && state.started && !state.paused && !state.help {
        let fw = 420.0 * u;
        let fx = (w - fw) * 0.5;
        let fy = h * 0.31;
        panel(fx, fy, fw, 190.0 * u, 16.0 * u, alpha(PANEL, 0.94));
        tracking(
            if state.invalid {
                "RUN COMPLETE"
            } else {
                "ACROSS THE LINE"
            },
            fx + 30.0 * u,
            fy + 34.0 * u,
            11.0 * u,
            2.0 * u,
            if state.invalid { ORANGE } else { CYAN },
        );
        text(
            &time_string(state.elapsed),
            fx + 27.0 * u,
            fy + 101.0 * u,
            58.0 * u,
            PAPER,
        );
        text(
            if state.invalid {
                "Missed a checkpoint. Give it another run."
            } else {
                "The next lap is yours."
            },
            fx + 30.0 * u,
            fy + 131.0 * u,
            15.0 * u,
            MUTED,
        );
        text(
            "R  /  RACE AGAIN",
            fx + 30.0 * u,
            fy + 166.0 * u,
            13.0 * u,
            CYAN,
        );
    }
    if state.paused || state.help {
        controls_overlay(state, u);
    }
}

#[cfg(test)]
mod tests {
    use super::fit_lines;

    fn monospace_width(label: &str) -> f32 {
        label.chars().count() as f32
    }

    #[test]
    fn notifications_wrap_at_words_without_losing_the_error() {
        assert_eq!(
            fit_lines(
                "Reload failed: invalid banking value",
                15.0,
                3,
                monospace_width
            ),
            ["Reload failed:", "invalid banking", "value"]
        );
        assert_eq!(
            fit_lines("NEW PERSONAL BEST", 24.0, 6, monospace_width),
            ["NEW PERSONAL BEST"]
        );
    }

    #[test]
    fn long_unicode_names_and_paths_fit_without_splitting_utf8() {
        assert_eq!(
            fit_lines("東京東京東京東京", 3.0, 2, monospace_width),
            ["東京東", "京東…"]
        );
        let lines = fit_lines(&"/very-long-path".repeat(100), 18.0, 6, monospace_width);
        assert_eq!(lines.len(), 6);
        assert!(lines.iter().all(|line| monospace_width(line) <= 18.0));
        assert!(lines.last().unwrap().ends_with('…'));
        assert_eq!(
            fit_lines("A custom course with a long name", 18.0, 1, monospace_width),
            ["A custom course w…"]
        );
    }

    #[test]
    fn fitting_handles_whitespace_and_insufficient_space() {
        assert_eq!(
            fit_lines("  Reload\n\tfailed  ", 15.0, 2, monospace_width),
            ["Reload failed"]
        );
        assert!(fit_lines("Course", 0.5, 1, monospace_width).is_empty());
        assert!(fit_lines("Course", 20.0, 0, monospace_width).is_empty());
    }
}
