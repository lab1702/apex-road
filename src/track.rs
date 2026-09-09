//! A small, validated road-building language. See `docs/TRACK_FORMAT.md`.
use macroquad::prelude::*;
use std::path::Path;

const SAMPLE_SPACING: f32 = 2.0;
const MAX_SAMPLES: usize = 25_001;
const MAX_LENGTH: f32 = 50_000.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoadKind {
    Road,
    Bridge,
    Tunnel,
    Ramp,
    Gap,
}

#[derive(Clone, Copy, Debug)]
pub struct RoadSample {
    pub pos: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub width: f32,
    /// Banking in radians. Positive angles raise the right side of the road.
    pub bank: f32,
    /// Cumulative three-dimensional distance along the centerline, in metres.
    pub distance: f32,
    /// Kind of the outgoing segment, from this sample to the next sample.
    pub kind: RoadKind,
}

#[derive(Clone, Debug)]
pub struct Track {
    pub name: String,
    pub description: String,
    pub samples: Vec<RoadSample>,
    pub length: f32,
    pub closed: bool,
    /// Ordered interior checkpoint distances; start and finish are implicit.
    pub checkpoints: Vec<f32>,
}

impl Track {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("Cannot read track '{}': {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn parse(text: &str) -> Result<Self, String> {
        if text.len() > 1_000_000 {
            return Err("Track file exceeds the 1 MB limit".into());
        }
        let mut builder = Builder::new();
        for (line_index, line) in text.lines().enumerate() {
            let line_number = line_index + 1;
            let tokens = tokenize(line).map_err(|e| format!("Line {line_number}: {e}"))?;
            if tokens.is_empty() {
                continue;
            }
            builder
                .command(&tokens)
                .map_err(|e| format!("Line {line_number}: {e}"))?;
        }
        builder.finish()
    }

    /// Base landscape height, 3 m below the lowest banked road edge.
    /// Deriving this from the route keeps negative-elevation tracks above
    /// their terrain instead of burying them under a fixed world plane.
    pub fn ground_height(&self) -> f32 {
        self.samples
            .iter()
            .map(|sample| sample.pos.y - sample.right.y.abs() * sample.width * 0.5)
            .reduce(f32::min)
            .unwrap_or(0.0)
            - 3.0
    }

    /// Closed courses wrap in either direction; point-to-point courses clamp.
    /// Non-finite input safely returns the starting sample.
    pub fn sample_at(&self, distance: f32) -> RoadSample {
        let distance = if !distance.is_finite() {
            0.0
        } else if self.closed {
            distance.rem_euclid(self.length)
        } else {
            distance.clamp(0.0, self.length)
        };
        let upper = self.samples.partition_point(|s| s.distance <= distance);
        if upper == 0 {
            return self.samples[0];
        }
        if upper == self.samples.len() {
            return self.samples[self.samples.len() - 1];
        }
        let a = self.samples[upper - 1];
        let b = self.samples[upper];
        let t = (distance - a.distance) / (b.distance - a.distance);
        let forward = a.forward.lerp(b.forward, t).normalize();
        let right_hint = a.right.lerp(b.right, t);
        let right = (right_hint - forward * right_hint.dot(forward)).normalize();
        RoadSample {
            pos: a.pos.lerp(b.pos, t),
            forward,
            right,
            up: forward.cross(right).normalize(),
            width: a.width + (b.width - a.width) * t,
            bank: a.bank + (b.bank - a.bank) * t,
            distance,
            kind: a.kind,
        }
    }
}

struct Builder {
    track: Track,
    heading: f32,
    grade: f32,
    target_width: f32,
    default_kind: RoadKind,
    ended: bool,
}

impl Builder {
    fn new() -> Self {
        Self {
            track: Track {
                name: "Untitled Track".into(),
                description: String::new(),
                samples: vec![make_sample(
                    Vec3::ZERO,
                    0.0,
                    0.0,
                    0.0,
                    12.0,
                    0.0,
                    RoadKind::Road,
                )],
                length: 0.0,
                closed: false,
                checkpoints: Vec::new(),
            },
            heading: 0.0,
            grade: 0.0,
            target_width: 12.0,
            default_kind: RoadKind::Road,
            ended: false,
        }
    }

    fn command(&mut self, tokens: &[String]) -> Result<(), String> {
        let cmd = tokens[0].as_str();
        let args = &tokens[1..];
        if self.ended {
            return Err("No commands are allowed after 'finish' or 'close'".into());
        }
        match cmd {
            "name" | "description" => {
                if args.is_empty() {
                    return Err(format!("'{cmd}' requires text"));
                }
                let value = args.join(" ");
                if value.len() > 500 {
                    return Err(format!("'{cmd}' text must be 500 bytes or fewer"));
                }
                if cmd == "name" {
                    self.track.name = value;
                } else {
                    self.track.description = value;
                }
            }
            "start" => {
                if self.track.samples.len() != 1 {
                    return Err("'start' must appear before the first road command".into());
                }
                if args.len() != 3 && args.len() != 4 {
                    return Err("Usage: start <x> <y> <z> [heading_degrees]".into());
                }
                let x = bounded(&args[0], "start x", -10_000.0, 10_000.0)?;
                let y = bounded(&args[1], "start y", -1_000.0, 1_000.0)?;
                let z = bounded(&args[2], "start z", -10_000.0, 10_000.0)?;
                self.heading = if args.len() == 4 {
                    bounded(&args[3], "heading", -360.0, 360.0)?.to_radians()
                } else {
                    0.0
                };
                self.track.samples[0] = make_sample(
                    vec3(x, y, z),
                    self.heading,
                    0.0,
                    0.0,
                    self.target_width,
                    0.0,
                    self.default_kind,
                );
            }
            "width" => {
                exact_args(cmd, args, 1)?;
                self.target_width = bounded(&args[0], "width", 4.0, 40.0)?;
                if self.track.samples.len() == 1 {
                    self.track.samples[0].width = self.target_width;
                }
            }
            "kind" => {
                exact_args(cmd, args, 1)?;
                self.default_kind = parse_kind(&args[0])?;
            }
            "checkpoint" => {
                exact_args(cmd, args, 0)?;
                let d = self.track.length;
                if d < 10.0 {
                    return Err("A checkpoint must be at least 10 m after the start".into());
                }
                if self
                    .track
                    .checkpoints
                    .last()
                    .is_some_and(|last| d - last < 10.0)
                {
                    return Err("Checkpoints must be at least 10 m apart".into());
                }
                if self.track.samples[self.track.samples.len() - 2].kind == RoadKind::Gap {
                    return Err("Place a checkpoint after a landing road, not inside or at the end of a gap".into());
                }
                self.track.checkpoints.push(d);
            }
            "finish" | "close" => {
                exact_args(cmd, args, 0)?;
                if cmd == "close" {
                    self.close()?;
                }
                self.ended = true;
            }
            "straight" | "bridge" | "tunnel" | "ramp" | "gap" | "left" | "right" => {
                self.road(cmd, args)?;
            }
            _ => {
                return Err(format!(
                    "Unknown command '{cmd}'. Expected straight, left, right, bridge, tunnel, ramp, gap, width, kind, checkpoint, start, name, description, finish or close"
                ));
            }
        }
        Ok(())
    }

    fn road(&mut self, cmd: &str, args: &[String]) -> Result<(), String> {
        if args.is_empty() {
            return Err(format!("'{cmd}' requires a length or turn angle"));
        }
        let (length, turn, options_start) = if cmd == "left" || cmd == "right" {
            if args.len() < 3 || args[1] != "radius" {
                return Err(format!(
                    "Usage: {cmd} <angle_degrees> radius <metres> [rise <metres>] [bank <degrees>] [kind <type>]"
                ));
            }
            let angle = bounded(&args[0], "turn angle", 0.1, 360.0)?.to_radians();
            let radius = bounded(&args[2], "turn radius", 8.0, 5_000.0)?;
            if radius
                <= self
                    .target_width
                    .max(self.track.samples.last().unwrap().width)
                    * 0.5
                    + 1.0
            {
                return Err("Turn radius must exceed half the road width plus 1 m".into());
            }
            (
                angle * radius,
                angle * if cmd == "right" { 1.0 } else { -1.0 },
                3,
            )
        } else {
            (bounded(&args[0], "length", 1.0, 5_000.0)?, 0.0, 1)
        };
        if !(1.0..=5_000.0).contains(&length) {
            return Err("A road command must cover between 1 and 5000 horizontal metres".into());
        }
        let mut rise = 0.0;
        let start = *self.track.samples.last().unwrap();
        let mut end_bank = start.bank;
        let mut kind = match cmd {
            "bridge" => RoadKind::Bridge,
            "tunnel" => RoadKind::Tunnel,
            "ramp" => RoadKind::Ramp,
            "gap" => RoadKind::Gap,
            _ => self.default_kind,
        };
        let mut seen = [false; 3];
        let options = &args[options_start..];
        if !options.len().is_multiple_of(2) {
            return Err(
                "Road options must be pairs: rise <metres>, bank <degrees>, kind <type>".into(),
            );
        }
        for pair in options.as_chunks::<2>().0 {
            let idx = match pair[0].as_str() {
                "rise" => 0,
                "bank" => 1,
                "kind" => 2,
                unknown => return Err(format!("Unknown road option '{unknown}'")),
            };
            if seen[idx] {
                return Err(format!("Duplicate '{}' option", pair[0]));
            }
            seen[idx] = true;
            match idx {
                0 => rise = bounded(&pair[1], "rise", -500.0, 500.0)?,
                1 => end_bank = bounded(&pair[1], "bank", -60.0, 60.0)?.to_radians(),
                _ => kind = parse_kind(&pair[1])?,
            }
        }
        if (rise / length).abs() > 0.6 {
            return Err("Rise may not exceed 60% of horizontal length".into());
        }
        if kind == RoadKind::Ramp && rise <= 0.0 {
            return Err("A ramp needs a positive 'rise' to create a launch slope".into());
        }
        if start.pos.y + rise < -1_000.0 || start.pos.y + rise > 2_000.0 {
            return Err("Road elevation must remain between -1000 and 2000 m".into());
        }
        // Ramp elevation is a cubic matching the incoming slope and leaving at
        // 1.5 times the average grade. Ordinary hills and gap guides end level.
        let end_grade = if kind == RoadKind::Ramp {
            1.5 * rise / length
        } else {
            0.0
        };
        let steps =
            (length * (1.0 + self.grade.abs() + end_grade.abs()) / SAMPLE_SPACING).ceil() as usize;
        let steps = steps.max(1);
        if self.track.samples.len() + steps > MAX_SAMPLES {
            return Err(format!(
                "Track exceeds the {} sample limit; shorten the route",
                MAX_SAMPLES
            ));
        }
        let first_heading = self.heading;
        let mut generated = Vec::with_capacity(steps);
        let mut previous = start;
        for step in 1..=steps {
            let t = step as f32 / steps as f32;
            let smooth = t * t * (3.0 - 2.0 * t);
            let heading = first_heading + turn * t;
            let mut pos = start.pos;
            if turn.abs() > f32::EPSILON {
                let signed_radius = length / turn;
                pos.x += signed_radius * (first_heading.cos() - heading.cos());
                pos.z += signed_radius * (heading.sin() - first_heading.sin());
            } else {
                pos.x += first_heading.sin() * length * t;
                pos.z += first_heading.cos() * length * t;
            }
            let m0 = self.grade * length;
            let m1 = end_grade * length;
            pos.y += smooth * rise + (t * t * t - 2.0 * t * t + t) * m0 + (t * t * t - t * t) * m1;
            let grade = ((6.0 * t - 6.0 * t * t) * rise
                + (3.0 * t * t - 4.0 * t + 1.0) * m0
                + (3.0 * t * t - 2.0 * t) * m1)
                / length;
            if grade.abs() > 1.0 || !(-1_000.0..=2_000.0).contains(&pos.y) {
                return Err("Elevation transition is too steep (maximum slope 100%) or exceeds height limits; use a longer segment".into());
            }
            let distance = previous.distance + previous.pos.distance(pos);
            if distance > MAX_LENGTH {
                return Err("Track exceeds the 50 km length limit".into());
            }
            let sample = make_sample(
                pos,
                heading,
                grade,
                start.bank + (end_bank - start.bank) * smooth,
                start.width + (self.target_width - start.width) * smooth,
                distance,
                kind,
            );
            generated.push(sample);
            previous = sample;
        }
        // Metadata belongs to the segment leaving a sample, including exactly
        // at a command boundary. This makes gaps unambiguous to mesh and physics.
        self.track.samples.last_mut().unwrap().kind = kind;
        self.track.samples.extend(generated);
        self.track.length = previous.distance;
        self.heading = (first_heading + turn).rem_euclid(std::f32::consts::TAU);
        self.grade = end_grade;
        Ok(())
    }

    fn close(&mut self) -> Result<(), String> {
        if self.track.length < 20.0 {
            return Err("A closed course must contain at least 20 m of road".into());
        }
        let first = self.track.samples[0];
        let last = *self.track.samples.last().unwrap();
        let separation = first.pos.distance(last.pos);
        if separation > 0.25 {
            return Err(format!(
                "'close' needs matching endpoints; the finish is {separation:.2} m from the start. Build the return road before closing"
            ));
        }
        if first.forward.dot(last.forward) < 0.999
            || first.up.dot(last.up) < 0.999
            || (first.width - last.width).abs() > 0.01
        {
            return Err(
                "'close' needs matching heading, slope, bank and width at the start and finish"
                    .into(),
            );
        }
        if first.kind == RoadKind::Gap || last.kind == RoadKind::Gap {
            return Err("A course cannot close across a gap".into());
        }
        // Remove floating point drift at the seam, keeping cumulative distance.
        let index = self.track.samples.len() - 1;
        let previous = self.track.samples[index - 1];
        self.track.length = previous.distance + previous.pos.distance(first.pos);
        self.track.samples[index] = RoadSample {
            distance: self.track.length,
            ..first
        };
        self.track.closed = true;
        Ok(())
    }

    fn finish(mut self) -> Result<Track, String> {
        if self.track.length < 20.0 {
            return Err("Track needs at least 20 m of road".into());
        }
        if self.track.samples[0].kind == RoadKind::Gap {
            return Err("Track must start on a drivable road, not a gap".into());
        }
        if self.track.samples[self.track.samples.len() - 2].kind == RoadKind::Gap {
            return Err("Track must finish on a drivable road, not a gap".into());
        }
        if self
            .track
            .checkpoints
            .last()
            .is_some_and(|d| self.track.length - d < 10.0)
        {
            return Err("The final checkpoint must be at least 10 m before the finish".into());
        }
        if self.track.checkpoints.is_empty() {
            // A few implicit gates prevent shortcuts in even the simplest file.
            for fraction in [0.25, 0.5, 0.75] {
                let candidate = self.track.length * fraction;
                if candidate >= 10.0
                    && self.track.length - candidate >= 10.0
                    && self.track.sample_at(candidate).kind != RoadKind::Gap
                {
                    self.track.checkpoints.push(candidate);
                }
            }
        }
        Ok(self.track)
    }
}

fn make_sample(
    pos: Vec3,
    heading: f32,
    grade: f32,
    bank: f32,
    width: f32,
    distance: f32,
    kind: RoadKind,
) -> RoadSample {
    let forward = vec3(heading.sin(), grade, heading.cos()).normalize();
    let flat_right = vec3(heading.cos(), 0.0, -heading.sin());
    let base_up = forward.cross(flat_right).normalize();
    let right = flat_right * bank.cos() + base_up * bank.sin();
    let up = base_up * bank.cos() - flat_right * bank.sin();
    RoadSample {
        pos,
        forward,
        right,
        up,
        width,
        bank,
        distance,
        kind,
    }
}

fn parse_kind(token: &str) -> Result<RoadKind, String> {
    match token {
        "road" => Ok(RoadKind::Road),
        "bridge" => Ok(RoadKind::Bridge),
        "tunnel" => Ok(RoadKind::Tunnel),
        "ramp" => Ok(RoadKind::Ramp),
        "gap" => Ok(RoadKind::Gap),
        _ => Err(format!(
            "Unknown road kind '{token}'; use road, bridge, tunnel, ramp or gap"
        )),
    }
}

fn exact_args(command: &str, args: &[String], count: usize) -> Result<(), String> {
    if args.len() != count {
        Err(format!(
            "'{command}' expects {count} argument(s), received {}",
            args.len()
        ))
    } else {
        Ok(())
    }
}

fn bounded(token: &str, label: &str, min: f32, max: f32) -> Result<f32, String> {
    let value: f32 = token
        .parse()
        .map_err(|_| format!("Invalid {label} '{token}': expected a number"))?;
    if !value.is_finite() {
        return Err(format!("{label} must be finite, received '{token}'"));
    }
    if !(min..=max).contains(&value) {
        return Err(format!(
            "{label} must be between {min} and {max}, received {value}"
        ));
    }
    Ok(value)
}

fn tokenize(line: &str) -> Result<Vec<String>, String> {
    let mut tokens = Vec::new();
    let mut token = String::new();
    let mut quoted = false;
    let mut has_token = false;
    let mut chars = line.chars();
    while let Some(ch) = chars.next() {
        match ch {
            '#' if !quoted => break,
            '"' => {
                quoted = !quoted;
                has_token = true;
            }
            '\\' if quoted => {
                let escaped = chars.next().ok_or("Unfinished escape in quoted text")?;
                match escaped {
                    '\\' | '"' => token.push(escaped),
                    _ => {
                        return Err(
                            "Only \\\\ and \\\" escapes are supported in quoted text".into()
                        );
                    }
                }
            }
            c if c.is_whitespace() && !quoted => {
                if has_token {
                    tokens.push(std::mem::take(&mut token));
                    has_token = false;
                }
            }
            c => {
                token.push(c);
                has_token = true;
            }
        }
    }
    if quoted {
        return Err("Unclosed quotation mark".into());
    }
    if has_token {
        tokens.push(token);
    }
    Ok(tokens)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terrain_is_below_banked_edges_at_negative_elevations() {
        let track = Track::parse(
            "start 0 -100 0\nwidth 20\nstraight 40 rise -10 bank 30\nstraight 40 rise 5 bank -15",
        )
        .unwrap();
        let ground = track.ground_height();
        assert!(ground < -113.0);
        let mut lowest = f32::INFINITY;
        for sample in &track.samples {
            for side in [-1.0, 1.0] {
                let edge = sample.pos + sample.right * sample.width * 0.5 * side;
                assert!(ground <= edge.y - 3.0 + 0.0001);
                lowest = lowest.min(edge.y);
            }
        }
        assert!((ground - (lowest - 3.0)).abs() < 0.0001);
        assert_eq!(Track::parse("straight 40").unwrap().ground_height(), -3.0);
    }

    #[test]
    fn hills_banks_and_sample_frames_are_continuous() {
        let track = Track::parse("straight 40 rise 8 bank 20\nstraight 40 rise -8 bank 0").unwrap();
        assert!((track.samples.last().unwrap().pos.y).abs() < 0.001);
        let middle = track
            .samples
            .iter()
            .max_by(|a, b| a.pos.y.total_cmp(&b.pos.y))
            .unwrap();
        assert!((middle.pos.y - 8.0).abs() < 0.001);
        assert!(middle.right.y > 0.3);
        for pair in track.samples.windows(2) {
            assert!(pair[1].distance > pair[0].distance);
            assert!((pair[1].bank - pair[0].bank).abs() < 0.04);
        }
        for sample in &track.samples {
            assert!(sample.forward.dot(sample.right).abs() < 0.0001);
            assert!(sample.forward.dot(sample.up).abs() < 0.0001);
            assert!((sample.up.length() - 1.0).abs() < 0.0001);
            assert!(sample.forward.cross(sample.right).distance(sample.up) < 0.0001);
        }
    }

    #[test]
    fn circular_geometry_and_closed_wrap_match() {
        let track = Track::parse(
            "straight 100\nright 180 radius 40\nstraight 100\nright 180 radius 40\nclose",
        )
        .unwrap();
        assert!(track.closed);
        assert!(
            track
                .samples
                .first()
                .unwrap()
                .pos
                .distance(track.samples.last().unwrap().pos)
                < 0.0001
        );
        assert!(
            track
                .sample_at(10.0)
                .pos
                .distance(track.sample_at(track.length + 10.0).pos)
                < 0.001
        );
        assert!(
            track
                .sample_at(-10.0)
                .pos
                .distance(track.sample_at(track.length - 10.0).pos)
                < 0.001
        );
        assert!(track.sample_at(track.length).pos.distance(Vec3::ZERO) < 0.0001);
    }

    #[test]
    fn gaps_mark_outgoing_segments_and_ramps_launch_upward() {
        let track =
            Track::parse("straight 30\nramp 20 rise 3\ngap 14 rise -3\nstraight 40").unwrap();
        let gap_start = track
            .samples
            .iter()
            .position(|s| s.kind == RoadKind::Gap)
            .unwrap();
        assert!((track.samples[gap_start].pos.z - 50.0).abs() < 0.001);
        assert!(track.samples[gap_start].forward.y > 0.1);
        let gap_end = track
            .samples
            .iter()
            .rposition(|s| s.kind == RoadKind::Gap)
            .unwrap()
            + 1;
        assert!((track.samples[gap_end].pos.z - 64.0).abs() < 0.001);
        assert_eq!(
            track
                .sample_at(track.samples[gap_start].distance + 0.1)
                .kind,
            RoadKind::Gap
        );
        assert_eq!(
            track.sample_at(track.samples[gap_end].distance).kind,
            RoadKind::Road
        );
    }

    #[test]
    fn useful_line_errors_and_finite_limits() {
        for (input, expected) in [
            ("name Test\nstraight NaN", "Line 2: length must be finite"),
            ("straight inf", "finite"),
            ("straight 0", "length must be between"),
            ("straight 40 rise NaN", "rise must be finite"),
            ("straight 40 bank 90", "bank must be between"),
            ("left 90 radius 4", "turn radius must be between"),
            ("straight 40 rise 2 rise 4", "Duplicate 'rise'"),
            ("straight 40 bananas 2", "Unknown road option"),
            ("ramp 40", "positive 'rise'"),
            ("name \"Oops", "Unclosed quotation"),
            ("straight 40\nclose", "matching endpoints"),
            ("straight 40\nfinish\nstraight 40", "No commands"),
        ] {
            let error = Track::parse(input).unwrap_err();
            assert!(error.contains(expected), "{input}: {error}");
        }
    }

    #[test]
    fn input_limits_and_illegal_checkpoint_positions() {
        assert!(Track::parse(&"straight 5000\n".repeat(12)).is_err());
        assert!(Track::parse("checkpoint\nstraight 40").is_err());
        assert!(Track::parse("straight 40\ncheckpoint").is_err());
        assert!(Track::parse("gap 30\nstraight 40").is_err());
        assert!(Track::parse("straight 40\ngap 20").is_err());
        assert!(Track::parse("straight 40\nstart 0 0 0").is_err());
    }

    #[test]
    fn point_to_point_clamps_and_comments_preserve_quoted_hash() {
        let track = Track::parse("name \"Route #1\" # comment\nstraight 40").unwrap();
        assert_eq!(track.name, "Route #1");
        assert_eq!(track.sample_at(-100.0).pos, Vec3::ZERO);
        assert_eq!(track.sample_at(f32::NAN).pos, Vec3::ZERO);
        assert_eq!(track.sample_at(100.0).pos, vec3(0.0, 0.0, 40.0));
        assert_eq!(track.checkpoints.len(), 3);
    }

    #[test]
    fn bundled_tracks_parse_and_have_required_features() {
        let alpine = Track::parse(include_str!("../tracks/alpine.track")).unwrap();
        assert!(!alpine.closed);
        assert!(alpine.length > 1_500.0);
        for kind in [
            RoadKind::Road,
            RoadKind::Bridge,
            RoadKind::Tunnel,
            RoadKind::Ramp,
            RoadKind::Gap,
        ] {
            assert!(
                alpine.samples.iter().any(|s| s.kind == kind),
                "missing {kind:?}"
            );
        }
        let club = Track::parse(include_str!("../tracks/club.track")).unwrap();
        assert!(club.closed);
        assert!(club.length > 400.0 && club.length < 1_000.0);
    }
}
