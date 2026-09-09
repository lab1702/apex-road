//! A small, validated road-building language. See `docs/TRACK_FORMAT.md`.
use macroquad::prelude::*;
use std::io::Read;
use std::path::Path;

const SAMPLE_SPACING: f32 = 2.0;
const MAX_BANK_STEP: f32 = 5.0_f32.to_radians();
const MAX_SAMPLES: usize = 25_001;
const MAX_LENGTH: f32 = 50_000.0;
const MAX_FILE_BYTES: usize = 1_000_000;

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
    source_hash: u64,
}

impl Track {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let mut bytes = Vec::new();
        std::fs::File::open(path)
            .and_then(|file| file.take(MAX_FILE_BYTES as u64 + 1).read_to_end(&mut bytes))
            .map_err(|e| format!("Cannot read track '{}': {e}", path.display()))?;
        if bytes.len() > MAX_FILE_BYTES {
            return Err(format!(
                "{}: Track file exceeds the 1 MB limit",
                path.display()
            ));
        }
        let text = String::from_utf8(bytes)
            .map_err(|e| format!("Cannot read track '{}': {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn parse(text: &str) -> Result<Self, String> {
        if text.len() > MAX_FILE_BYTES {
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
                .command(&tokens, line_number)
                .map_err(|e| format!("Line {line_number}: {e}"))?;
        }
        let mut track = builder.finish()?;
        track.source_hash = text.bytes().fold(0xcbf29ce484222325u64, |hash, byte| {
            (hash ^ byte as u64).wrapping_mul(0x100000001b3)
        });
        Ok(track)
    }

    /// Stable identity of the source that produced this loaded route. Records
    /// keep using this version even if its file changes before the next reload.
    pub fn source_hash(&self) -> u64 {
        self.source_hash
    }

    /// Timing and gate location: the circuit seam or 3 m before a sprint's end.
    pub fn finish_distance(&self) -> f32 {
        if self.closed {
            self.length
        } else {
            self.length - 3.0
        }
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
    checkpoint_lines: Vec<usize>,
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
                source_hash: 0,
            },
            heading: 0.0,
            grade: 0.0,
            target_width: 12.0,
            default_kind: RoadKind::Road,
            checkpoint_lines: Vec::new(),
            ended: false,
        }
    }

    fn command(&mut self, tokens: &[String], line_number: usize) -> Result<(), String> {
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
                self.checkpoint_lines.push(line_number);
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
        // Sampling only a short hill's level endpoints erases its slope from
        // the road frame. Capture the interior of every elevation transition,
        // including a level target that eases out of an incoming ramp slope.
        let elevation_transition = rise != 0.0 || self.grade != 0.0 || end_grade != 0.0;
        let steps = steps.max(if elevation_transition { 4 } else { 1 });
        // Mesh edges interpolate linearly, while driving frames keep a unit
        // right vector. Large bank changes between samples would pinch the
        // rendered road far inside its collision surface. Smoothstep's maximum
        // derivative is 1.5, so this bounds adjacent bank changes to five degrees.
        let bank_steps = ((end_bank - start.bank).abs() * 1.5 / MAX_BANK_STEP).ceil() as usize;
        let steps = steps.max(bank_steps);
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
        let closing_chord = first.pos - previous.pos;
        let length = previous.distance + closing_chord.length();
        // A short bank transition may have samples much closer than the seam
        // tolerance. Snapping a slight overshoot back to the start must not
        // leave an overlapping backwards segment or duplicate sample distance.
        if closing_chord.dot(previous.forward) <= 0.0
            || closing_chord.dot(first.forward) <= 0.0
            || length <= previous.distance
        {
            return Err(
                "'close' would collapse or reverse the final road segment; match the endpoint more precisely or use a longer final segment"
                    .into(),
            );
        }
        self.track.length = length;
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
        let finish = self.track.finish_distance();
        if self.track.sample_at(finish).kind == RoadKind::Gap {
            return Err(
                "Sprint finish line is 3 m before the endpoint and must be on solid road; extend the landing road".into(),
            );
        }
        // A later road command sets the outgoing kind at an existing gate, so
        // its launch-side validation must wait until the route is complete.
        for (&distance, &line_number) in self.track.checkpoints.iter().zip(&self.checkpoint_lines) {
            if self.track.sample_at(distance).kind == RoadKind::Gap {
                return Err(format!(
                    "Line {line_number}: Place a checkpoint on solid road before the launch or after a landing road, not at the start of a gap"
                ));
            }
        }
        if self
            .track
            .checkpoints
            .last()
            .is_some_and(|d| finish - d < 10.0)
        {
            return Err("The final checkpoint must be at least 10 m before the finish".into());
        }
        if self.track.checkpoints.is_empty() {
            // A few implicit gates prevent shortcuts in even the simplest file.
            for fraction in [0.25, 0.5, 0.75] {
                let candidate = self.track.length * fraction;
                // An exact landing boundary has a solid outgoing sample but
                // still touches the preceding gap. Apply the same restriction
                // as an authored checkpoint, which needs road on both sides.
                let incoming = self
                    .track
                    .samples
                    .partition_point(|sample| sample.distance < candidate)
                    .saturating_sub(1);
                if candidate >= 10.0
                    && finish - candidate >= 10.0
                    && self.track.sample_at(candidate).kind != RoadKind::Gap
                    && self.track.samples[incoming].kind != RoadKind::Gap
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

    struct TrackFile(std::path::PathBuf);

    impl TrackFile {
        fn new() -> Self {
            static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let id = NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            Self(std::env::temp_dir().join(format!(
                "apex-road-track-test-{}-{id}.track",
                std::process::id()
            )))
        }
    }

    impl Drop for TrackFile {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    #[test]
    fn source_identity_belongs_to_the_loaded_version() {
        let file = TrackFile::new();
        std::fs::write(&file.0, "straight 40\n").unwrap();
        let loaded = Track::load(&file.0).unwrap();
        let original_hash = loaded.source_hash();
        assert_eq!(original_hash, 0x64633c68e54952d9);
        assert_eq!(
            original_hash,
            Track::parse("straight 40\n").unwrap().source_hash()
        );

        std::fs::write(&file.0, "straight 80\n").unwrap();
        let reloaded = Track::load(&file.0).unwrap();
        assert_ne!(original_hash, reloaded.source_hash());
        assert_eq!(loaded.source_hash(), original_hash);
        assert_eq!(loaded.length, 40.0);
        assert_eq!(reloaded.length, 80.0);

        std::fs::remove_file(&file.0).unwrap();
        assert_eq!(loaded.source_hash(), original_hash);
        assert_eq!(loaded.clone().source_hash(), original_hash);
        // Source identity includes comments and whitespace, preserving the
        // record keys used by existing versions of the game.
        assert_ne!(
            original_hash,
            Track::parse("straight 40").unwrap().source_hash()
        );
    }

    #[test]
    fn loading_enforces_the_byte_limit_before_decoding() {
        let file = TrackFile::new();
        let mut source = String::from("straight 40\n#");
        source.extend(std::iter::repeat_n(' ', MAX_FILE_BYTES - source.len()));
        std::fs::write(&file.0, &source).unwrap();
        assert_eq!(Track::load(&file.0).unwrap().length, 40.0);

        let mut oversized = source.into_bytes();
        oversized.push(0xff);
        std::fs::write(&file.0, oversized).unwrap();
        let error = Track::load(&file.0).unwrap_err();
        assert!(error.contains("exceeds the 1 MB limit"), "{error}");
        assert!(error.contains(&file.0.display().to_string()), "{error}");

        std::fs::write(&file.0, b"straight 40\n#\xff").unwrap();
        let error = Track::load(&file.0).unwrap_err();
        assert!(error.contains("Cannot read track"), "{error}");
        assert!(error.contains(&file.0.display().to_string()), "{error}");
    }

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
    fn close_rejects_reversing_a_short_final_segment() {
        let source = "straight 20\nright 180 radius 20\nstraight 20.9\nright 180 radius 20 bank 60\nstraight 1 bank 0";
        let open = Track::parse(source).unwrap();
        let previous = open.samples[open.samples.len() - 2];
        assert!(previous.pos.z > open.samples[0].pos.z);
        let error = Track::parse(&format!("{source}\nclose")).unwrap_err();
        assert!(error.contains("collapse or reverse"), "{error}");
    }

    #[test]
    fn close_rejects_collapsing_a_short_final_segment() {
        let source = "start 0 0 1000\nstraight 20\nright 180 radius 20\nstraight 20.75\nright 180 radius 20 bank 12\nstraight 1 bank 0";
        let open = Track::parse(source).unwrap();
        assert_eq!(
            open.samples[open.samples.len() - 2].pos,
            open.samples[0].pos
        );
        let error = Track::parse(&format!("{source}\nclose")).unwrap_err();
        assert!(error.contains("collapse or reverse"), "{error}");
    }

    #[test]
    fn close_rejects_a_segment_below_cumulative_distance_precision() {
        let source = "straight 20\nright 180 radius 20\nstraight 20.947372\nright 180 radius 20 bank 60\nstraight 1 bank 0";
        let open = Track::parse(source).unwrap();
        let previous = open.samples[open.samples.len() - 2];
        assert!(previous.pos.z < open.samples[0].pos.z);
        assert!((previous.pos.z - open.samples[0].pos.z).abs() < 0.000001);
        let error = Track::parse(&format!("{source}\nclose")).unwrap_err();
        assert!(error.contains("collapse or reverse"), "{error}");
    }

    #[test]
    fn close_keeps_safe_drift_correction_on_short_final_segments() {
        for return_length in [20.95, 21.1] {
            let source = format!(
                "straight 20\nright 180 radius 20\nstraight {return_length}\nright 180 radius 20 bank 60\nstraight 1 bank 0\nclose",
            );
            let track = Track::parse(&source).unwrap();
            assert_eq!(track.samples.last().unwrap().pos, track.samples[0].pos);
            for pair in track.samples.windows(2) {
                assert!(pair[1].distance > pair[0].distance);
                assert!((pair[1].pos - pair[0].pos).dot(pair[0].forward) > 0.0);
            }
        }
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
    fn sprint_finish_cannot_fall_inside_the_last_gap() {
        let approach = "straight 100\ncheckpoint\nramp 24 rise 2.5\ngap 14 rise -2.5";
        for landing in [
            "straight 1",
            "straight 2",
            "bridge 2",
            "straight 1\nstraight 1",
        ] {
            for ending in ["", "\nfinish"] {
                let Err(error) = Track::parse(&format!("{approach}\n{landing}{ending}")) else {
                    panic!("the timing line would be inside the gap");
                };
                assert!(error.contains("Sprint finish line"), "{error}");
            }
        }
        // The solid landing can span multiple commands, and its exact first
        // sample belongs to the landing rather than the preceding gap.
        for landing in [
            "straight 3",
            "straight 4",
            "straight 1\nstraight 1\nstraight 2",
        ] {
            let track = Track::parse(&format!("{approach}\n{landing}")).unwrap();
            assert_ne!(track.sample_at(track.length - 3.0).kind, RoadKind::Gap);
        }
        // Circuits finish at the seam, so a short solid segment after a gap
        // is legal even when the point three metres before the seam is a gap.
        let circuit = Track::parse(
            "straight 20\nright 180 radius 20\nstraight 20\nright 170 radius 20\nright 7 radius 20 kind gap\nright 3 radius 20\nclose",
        )
        .unwrap();
        assert_eq!(circuit.sample_at(circuit.length - 3.0).kind, RoadKind::Gap);
        assert_ne!(circuit.sample_at(circuit.length).kind, RoadKind::Gap);
    }

    #[test]
    fn checkpoints_are_validated_after_the_following_road_kind_is_known() {
        for gap_command in ["gap 14 rise -2.5", "straight 14 rise -2.5 kind gap"] {
            let text =
                format!("straight 100\nramp 24 rise 2.5\ncheckpoint\n{gap_command}\nstraight 100");
            let error = Track::parse(&text).unwrap_err();
            assert!(error.starts_with("Line 3:"), "{error}");
            assert!(error.contains("at the start of a gap"), "{error}");
        }
        assert!(
            Track::parse(
                "straight 100\ncheckpoint\nramp 24 rise 2.5\ngap 14 rise -2.5\nstraight 20\ncheckpoint\nstraight 100"
            )
            .is_ok()
        );
    }

    #[test]
    fn checkpoints_leave_ten_metres_before_the_timing_finish() {
        for landing_length in [10, 11, 12] {
            let source = format!("straight 30\ncheckpoint\nstraight {landing_length}");
            let error = Track::parse(&source).unwrap_err();
            assert!(error.contains("at least 10 m before the finish"), "{error}");
        }
        let sprint = Track::parse("straight 30\ncheckpoint\nstraight 13").unwrap();
        assert_eq!(sprint.finish_distance() - sprint.checkpoints[0], 10.0);

        let circuit = Track::parse(
            "straight 20\nright 180 radius 20\nstraight 30\nright 180 radius 20\ncheckpoint\nstraight 10\nclose",
        )
        .unwrap();
        assert!((circuit.finish_distance() - circuit.checkpoints[0] - 10.0).abs() < 0.001);
    }

    #[test]
    fn automatic_checkpoints_respect_the_timing_finish_and_skip_gaps() {
        for (source, expected) in [
            ("straight 20", vec![]),
            ("straight 40", vec![10.0, 20.0]),
            ("straight 52", vec![13.0, 26.0, 39.0]),
            ("straight 20\ngap 50\nstraight 50", vec![90.0]),
            ("straight 20\ngap 10\nstraight 90", vec![60.0, 90.0]),
            ("straight 30\ngap 30\nstraight 60", vec![90.0]),
            ("straight 20\ngap 70\nstraight 30", vec![]),
        ] {
            let track = Track::parse(source).unwrap();
            assert_eq!(track.checkpoints, expected, "{source}");
            for &distance in &track.checkpoints {
                assert!(track.finish_distance() - distance >= 10.0, "{source}");
                assert_ne!(track.sample_at(distance).kind, RoadKind::Gap, "{source}");
            }
        }
    }

    #[test]
    fn short_hills_retain_their_surface_slope_and_downhill_gravity() {
        let track = Track::parse("straight 20\nstraight 2 rise 1\nstraight 20").unwrap();
        let interior: Vec<_> = track
            .samples
            .windows(3)
            .filter(|samples| samples[1].pos.z > 20.0 && samples[1].pos.z < 22.0)
            .collect();
        assert!(interior.len() >= 3);
        for samples in &interior {
            let geometric_forward = (samples[2].pos - samples[0].pos).normalize();
            assert!(samples[1].forward.y > 0.4);
            assert!(samples[1].forward.distance(geometric_forward) < 0.07);
        }
        let middle = interior[interior.len() / 2][1];
        let mut car = crate::vehicle::Car::new(&track);
        car.reset(&track, middle.distance);
        for _ in 0..30 {
            car.update(&track, crate::vehicle::Control::default(), 1.0 / 120.0);
        }
        assert!(car.position.z < middle.pos.z - 0.02);
        assert!(car.velocity.z < -0.2);
    }

    #[test]
    fn short_level_target_retains_its_incoming_ramp_transition() {
        let track = Track::parse("straight 20\nramp 2 rise 1\nstraight 1\nstraight 20").unwrap();
        let transition: Vec<_> = track
            .samples
            .iter()
            .filter(|sample| sample.pos.z > 22.0 && sample.pos.z < 23.0)
            .collect();
        assert!(transition.len() >= 3);
        assert!(transition.iter().any(|sample| sample.pos.y > 1.05));
        assert!(transition.iter().any(|sample| sample.forward.y < -0.1));
        let end = track
            .samples
            .iter()
            .find(|sample| sample.pos.z == 23.0)
            .unwrap();
        assert_eq!(end.pos.y, 1.0);
        assert_eq!(end.forward.y, 0.0);
    }

    #[test]
    fn short_bank_reversals_keep_the_rendered_width_close_to_the_road_frame() {
        let track =
            Track::parse("width 40\nstraight 20 bank 60\nstraight 1 bank -60\nstraight 20 bank 0")
                .unwrap();
        let mut transitions = 0;
        for pair in track.samples.windows(2) {
            let [a, b] = [pair[0], pair[1]];
            if a.pos.z < 20.0 || b.pos.z > 21.0 {
                continue;
            }
            transitions += 1;
            let middle = track.sample_at((a.distance + b.distance) * 0.5);
            // The mesh joins sampled edges with straight lines, whereas
            // sample_at and collision queries normalize the road frame.
            // Sparse bank samples must not leave metres of invisible road
            // outside those rendered edges.
            let rendered_edge =
                (a.pos + a.right * a.width * 0.5).lerp(b.pos + b.right * b.width * 0.5, 0.5);
            let frame_edge = middle.pos + middle.right * middle.width * 0.5;
            assert!(
                rendered_edge.distance(frame_edge) < 0.02,
                "rendered edge {rendered_edge:?} differs from road edge {frame_edge:?}"
            );
            assert!((b.bank - a.bank).abs() <= 5.0_f32.to_radians() + 0.0001);
        }
        assert!(transitions > 1);
    }

    #[test]
    fn point_to_point_clamps_and_comments_preserve_quoted_hash() {
        let track = Track::parse("name \"Route #1\" # comment\nstraight 40").unwrap();
        assert_eq!(track.name, "Route #1");
        assert_eq!(track.sample_at(-100.0).pos, Vec3::ZERO);
        assert_eq!(track.sample_at(f32::NAN).pos, Vec3::ZERO);
        assert_eq!(track.sample_at(100.0).pos, vec3(0.0, 0.0, 40.0));
        assert_eq!(track.checkpoints, vec![10.0, 20.0]);
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
