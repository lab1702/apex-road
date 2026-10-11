//! Direction-sensitive, ordered checkpoint timing.
use crate::track::Track;

/// Course progress (m) that can be gained off the paved road, net of the
/// progress driven on it afterwards, before a lap or run counts as a cut.
/// Brief wheel drops and short slides are repaid as soon as the car rejoins
/// the road; driving across an infield or the inside of a bend is not.
pub const CUT_ALLOWANCE: f32 = 30.0;

/// Total course progress (m) a lap or run may gain off the paved road,
/// repaid or not. Several cuts that each stay under [`CUT_ALLOWANCE`] add up
/// here; the auto driver, which only drops a wheel now and then, stays far
/// below it.
pub const OFFROAD_LIMIT: f32 = 2.0 * CUT_ALLOWANCE;

/// Progress change (m) in one step beyond which the course was skipped.
const JUMP_LIMIT: f32 = 12.0;

/// The first rule a lap or run broke. Later violations keep the first reason.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvalidReason {
    /// A checkpoint or the finish was passed off the road, or a gate is missing.
    MissedCheckpoint,
    /// Driven back across the start line during a lap.
    WrongWay,
    /// Course progress was gained off the road or skipped in one jump.
    CutCourse,
}

impl InvalidReason {
    /// A sentence for the finish panel.
    pub fn explanation(self) -> &'static str {
        match self {
            Self::MissedCheckpoint => "Missed a checkpoint. Give it another run.",
            Self::WrongWay => "Went the wrong way. Give it another run.",
            Self::CutCourse => "Cut the course. Give it another run.",
        }
    }
}

pub struct Race {
    pub started: bool,
    pub finished: bool,
    /// Finish crossings since the last reset, including invalid laps/runs.
    pub completed_runs: u64,
    /// Keep accumulated simulation time precise; display and stored records use f32.
    pub elapsed: f64,
    pub best: Option<f32>,
    pub last: Option<f32>,
    pub next_checkpoint: usize,
    /// The first rule this lap or run broke; `None` while it is valid.
    pub invalid_reason: Option<InvalidReason>,
    previous: f32,
    /// Off-road progress not yet repaid by on-road progress, in metres.
    cut_debt: f32,
    /// All off-road progress this lap or run, in metres.
    offroad_progress: f32,
    /// A circuit car went back across the start line. Its next forward
    /// crossing restarts the lap instead of completing one.
    behind_line: bool,
    /// Signed course progress since this lap or run began, in metres.
    lap_progress: f32,
    /// The furthest `lap_progress` reached this lap or run. Only ground
    /// beyond it is new: reversing and driving the same stretch again
    /// neither adds cut debt nor repays it.
    lap_furthest: f32,
}
impl Race {
    pub fn new(best: Option<f32>, start: f32) -> Self {
        Self {
            started: false,
            finished: false,
            completed_runs: 0,
            elapsed: 0.,
            best: best.filter(|time| time.is_finite() && *time > 0.0),
            last: None,
            next_checkpoint: 0,
            invalid_reason: None,
            previous: if start.is_finite() { start } else { 0.0 },
            cut_debt: 0.0,
            offroad_progress: 0.0,
            behind_line: false,
            lap_progress: 0.0,
            lap_furthest: 0.0,
        }
    }

    /// Whether this lap or run has broken a rule and cannot set a time.
    pub fn invalid(&self) -> bool {
        self.invalid_reason.is_some()
    }

    fn invalidate(&mut self, reason: InvalidReason) {
        self.invalid_reason.get_or_insert(reason);
    }

    /// Start a new lap at the line, `progress` metres of which the step that
    /// crossed it has already driven. Only circuits use this.
    fn reset_lap(&mut self, elapsed: f64, progress: f32) {
        self.elapsed = elapsed;
        self.next_checkpoint = 0;
        self.invalid_reason = None;
        self.cut_debt = 0.0;
        self.offroad_progress = 0.0;
        self.behind_line = false;
        self.lap_progress = progress;
        self.lap_furthest = progress.max(0.0);
    }

    /// Record signed progress and return the part of it that is new ground.
    fn advance_progress(&mut self, delta: f32) -> f32 {
        self.lap_progress += delta;
        let new_ground = (self.lap_progress - self.lap_furthest).max(0.0);
        self.lap_furthest = self.lap_furthest.max(self.lap_progress);
        new_ground
    }

    /// Returns a newly established record, if any.
    pub fn update(
        &mut self,
        track: &Track,
        dt: f32,
        distance: f32,
        on_course: bool,
    ) -> Option<f32> {
        if !distance.is_finite() {
            self.invalidate(InvalidReason::CutCourse);
            return None;
        }
        let distance = if track.closed {
            distance.rem_euclid(track.length)
        } else {
            distance.clamp(0.0, track.length)
        };
        if !self.started || self.finished {
            self.previous = distance;
            return None;
        }
        if !dt.is_finite() || dt <= 0.0 {
            return None;
        }
        self.elapsed += f64::from(dt);
        let finish = track.finish_distance();
        let mut delta = distance - self.previous;
        let mut crossed_back = false;
        if track.closed {
            if delta < -track.length * 0.5 {
                delta += track.length;
            }
            if delta > track.length * 0.5 {
                delta -= track.length;
                crossed_back = true;
            }
        }
        if delta.abs() > JUMP_LIMIT {
            self.invalidate(InvalidReason::CutCourse);
            let end = self.previous + delta;
            if track.closed && crossed_back {
                // Skipped backwards over the line: as with reversing over it,
                // the next forward crossing restarts the lap.
                self.behind_line = true;
            } else if track.closed && delta > 0.0 && end >= finish {
                // Skipped forwards over the line. The skipped lap is over and
                // sets no time; a fresh lap starts at the line.
                let remainder = f64::from(dt) * f64::from(((end - finish) / delta).clamp(0.0, 1.0));
                if !self.behind_line {
                    self.completed_runs += 1;
                    self.last = None;
                }
                let carried = end - track.length;
                self.reset_lap(remainder, carried);
                // Skipped ground collects no gates. A long skip into the new
                // lap cuts that lap as well.
                if carried > JUMP_LIMIT {
                    self.invalidate(InvalidReason::CutCourse);
                }
                self.previous = distance;
                return None;
            } else if !track.closed && self.previous < finish && distance >= finish {
                // A sprint that lands beyond its finish cannot cross it later.
                // End the run there instead of leaving its clock running.
                self.completed_runs += 1;
                self.last = None;
                self.finished = true;
            }
            self.advance_progress(delta);
            self.previous = distance;
            return None;
        }
        if crossed_back {
            // Returning across the start backwards cannot complete the
            // current lap by immediately crossing forwards again. Rolling
            // back within a metre of the line, as off the grid, has not
            // started a lap yet, so it is not shown as wrong-way driving.
            if !self.behind_line && (self.lap_furthest > 1.0 || self.next_checkpoint > 0) {
                self.invalidate(InvalidReason::WrongWay);
            }
            self.behind_line = true;
        }
        // Only new ground counts, so rocking back and forth neither builds
        // up cut debt off the road nor repays it on the road.
        let new_ground = self.advance_progress(delta);
        if on_course {
            self.cut_debt = (self.cut_debt - new_ground).max(0.0);
        } else {
            self.cut_debt += new_ground;
            self.offroad_progress += new_ground;
            if self.cut_debt > CUT_ALLOWANCE || self.offroad_progress > OFFROAD_LIMIT {
                self.invalidate(InvalidReason::CutCourse);
            }
        }
        if delta > 0. {
            let end = self.previous + delta;
            self.cross_checkpoints(track, self.previous, end.min(finish), on_course);
            if self.previous < finish && end >= finish {
                // Divide the simulation step at the finish plane. Otherwise
                // the completed lap gains a frame and the next one loses it.
                let remainder = f64::from(dt) * f64::from(((end - finish) / delta).clamp(0.0, 1.0));
                if self.behind_line {
                    // Back over the line it was reversed across: this starts
                    // a fresh lap, but no lap was completed.
                    self.reset_lap(remainder, end - track.length);
                    self.cross_checkpoints(track, 0.0, end - track.length, on_course);
                    self.previous = distance;
                    return None;
                }
                self.completed_runs += 1;
                if self.next_checkpoint != track.checkpoints.len() || !on_course {
                    self.invalidate(InvalidReason::MissedCheckpoint);
                }
                let completed_time = self.elapsed - remainder;
                let record_time = completed_time as f32;
                let result = if !self.invalid() && self.best.is_none_or(|best| record_time < best) {
                    self.best = Some(record_time);
                    Some(record_time)
                } else {
                    None
                };
                self.last = if self.invalid() {
                    None
                } else {
                    Some(record_time)
                };
                if track.closed {
                    self.reset_lap(remainder, end - track.length);
                    // A legal step can cross both the seam and the first gate
                    // of a short circuit; count the part after the seam too.
                    self.cross_checkpoints(track, 0.0, end - track.length, on_course);
                } else {
                    self.elapsed = completed_time;
                    self.finished = true;
                }
                self.previous = distance;
                return result;
            }
        }
        self.previous = distance;
        None
    }

    fn cross_checkpoints(&mut self, track: &Track, from: f32, to: f32, on_course: bool) {
        while let Some(&gate) = track.checkpoints.get(self.next_checkpoint) {
            if gate <= from || gate > to {
                break;
            }
            if !on_course {
                self.invalidate(InvalidReason::MissedCheckpoint);
                break;
            }
            self.next_checkpoint += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn straight() -> Track {
        Track::parse("name Test\nstraight 100\ncheckpoint\nstraight 100\n").unwrap()
    }
    #[test]
    fn ordered_gates_produce_record() {
        let t = straight();
        let mut r = Race::new(None, 5.);
        r.started = true;
        let mut record = None;
        for d in 6..=198 {
            record = r.update(&t, 0.1, d as f32, true).or(record);
        }
        assert!(r.finished);
        assert_eq!(r.completed_runs, 1);
        assert!(record.is_some());
        assert_eq!(r.next_checkpoint, 1);
    }
    #[test]
    fn bypassed_gate_does_not_count() {
        let t = straight();
        let mut r = Race::new(None, 5.);
        r.started = true;
        for d in 6..=198 {
            assert!(
                r.update(&t, 0.1, d as f32, !(95..=105).contains(&d))
                    .is_none()
            );
        }
        assert!(r.finished && r.invalid());
        assert_eq!(r.invalid_reason, Some(InvalidReason::MissedCheckpoint));
        assert_eq!(r.completed_runs, 1);
        assert!(r.best.is_none());
    }
    #[test]
    fn teleport_invalidates_time() {
        let t = straight();
        let mut r = Race::new(None, 5.);
        r.started = true;
        r.update(&t, 1., 150., true);
        assert!(r.invalid());
        assert_eq!(r.completed_runs, 0);
    }
    #[test]
    fn unstarted_clock_stays_still() {
        let t = straight();
        let mut r = Race::new(None, 5.);
        r.update(&t, 3., 5., true);
        assert_eq!(r.elapsed, 0.);
        assert_eq!(r.completed_runs, 0);
    }

    fn circuit() -> Track {
        Track::parse("straight 20\ncheckpoint\nright 180 radius 20\nstraight 20\ncheckpoint\nright 180 radius 20\nclose").unwrap()
    }

    fn advance(race: &mut Race, track: &Track, from: f32, to: f32, speed: f32) -> Vec<f32> {
        let mut cursor = from;
        let mut records = Vec::new();
        while cursor < to - 0.0001 {
            let next = (cursor + 1.0).min(to);
            if let Some(record) = race.update(track, (next - cursor) / speed, next, true) {
                records.push(record);
            }
            cursor = next;
        }
        records
    }

    #[test]
    fn reverse_checkpoint_and_reverse_finish_do_not_count() {
        let track = circuit();
        let gate = track.checkpoints[0];
        let mut race = Race::new(None, gate + 2.0);
        race.started = true;
        race.update(&track, 0.1, gate - 2.0, true);
        assert_eq!(race.next_checkpoint, 0);
        assert!(!race.invalid());
        race.update(&track, 0.1, gate + 2.0, true);
        assert_eq!(race.next_checkpoint, 1);
        assert!(!race.invalid());

        let mut race = Race::new(None, 2.0);
        race.started = true;
        race.next_checkpoint = track.checkpoints.len();
        race.update(&track, 0.1, track.length - 2.0, true);
        assert!(race.last.is_none());
        assert!(race.best.is_none());
        assert!(race.invalid());
        assert_eq!(race.next_checkpoint, track.checkpoints.len());
        assert_eq!(race.completed_runs, 0);
    }

    #[test]
    fn reversing_over_start_cannot_finish_a_shortcut_but_next_full_lap_can() {
        let track = Track::parse(
            "straight 20\ncheckpoint\nstraight 180\nright 180 radius 20\nstraight 200\nright 180 radius 20\nclose",
        )
        .unwrap();
        let mut race = Race::new(Some(100.0), 0.0);
        race.started = true;
        assert!(advance(&mut race, &track, 0.0, 22.0, 10.0).is_empty());
        assert_eq!(race.next_checkpoint, track.checkpoints.len());

        // Each step is small and on course, so only the backwards start-line
        // crossing distinguishes this shortcut from an ordinary valid lap.
        for distance in (-1..=21).rev() {
            assert!(race.update(&track, 0.1, distance as f32, true).is_none());
        }
        assert!(race.invalid());
        assert_eq!(race.invalid_reason, Some(InvalidReason::WrongWay));
        assert!(race.update(&track, 0.1, 0.0, true).is_none());
        assert_eq!(race.best, Some(100.0));
        assert!(race.last.is_none());
        assert!(!race.invalid() && race.invalid_reason.is_none());
        assert_eq!(race.next_checkpoint, 0);
        // Driving back over the line restarts the lap; it completes none.
        assert_eq!(race.completed_runs, 0);
        assert!(race.elapsed < 0.001);

        let records = advance(&mut race, &track, 0.0, track.length, 10.0);
        assert_eq!(records.len(), 1);
        assert!((records[0] - track.length / 10.0).abs() < 0.001);
        assert_eq!(race.last, race.best);
    }

    #[test]
    fn complete_circuit_repeats_laps_and_preserves_step_time() {
        let track = circuit();
        let mut race = Race::new(None, 0.0);
        race.started = true;
        let speed = 10.0;
        let records = advance(&mut race, &track, 0.0, track.length * 2.0 + 3.0, speed);
        assert!(!records.is_empty());
        assert!(!race.finished && !race.invalid());
        assert_eq!(race.completed_runs, 2);
        assert!((race.last.unwrap() - track.length / speed).abs() < 0.001);
        assert!((race.elapsed - 0.3).abs() < 0.001);
        assert_eq!(race.next_checkpoint, 0);
    }

    #[test]
    fn missed_gate_invalidates_lap_but_next_full_lap_can_set_record() {
        let track = circuit();
        let mut race = Race::new(None, 0.0);
        race.started = true;
        let mut cursor = 0.0;
        while cursor < track.length {
            let next = (cursor + 1.0).min(track.length);
            let on_course = !(19.0..=21.0).contains(&next);
            assert!(
                race.update(&track, (next - cursor) / 10.0, next, on_course)
                    .is_none()
            );
            cursor = next;
        }
        assert!(race.best.is_none() && race.last.is_none());
        assert!(!race.invalid());
        assert_eq!(race.completed_runs, 1);
        assert_eq!(
            advance(&mut race, &track, track.length, track.length * 2.0, 10.0).len(),
            1
        );
        assert!(race.last.is_some());
        assert_eq!(race.completed_runs, 2);
    }

    #[test]
    fn shortcut_jump_cannot_set_a_record_even_after_collecting_later_gates() {
        let track = circuit();
        let mut race = Race::new(Some(100.0), 0.0);
        race.started = true;
        race.update(&track, 0.1, 30.0, true);
        assert!(race.invalid());
        assert!(advance(&mut race, &track, 30.0, track.length, 10.0).is_empty());
        assert_eq!(race.best, Some(100.0));
        assert!(race.last.is_none());
    }

    #[test]
    fn frame_can_cross_finish_and_first_checkpoint() {
        let mut track = circuit();
        // A second gate distinguishes the reset lap's first gate from the
        // completed lap's final count, which would also leave this at 1.
        track.checkpoints = vec![10.0, 50.0];
        let mut race = Race::new(None, track.length - 0.5);
        race.started = true;
        race.next_checkpoint = 2;
        race.elapsed = 20.0;
        let record = race.update(&track, 1.0, 10.5, true).unwrap();
        assert!((record - (20.0 + 0.5 / 11.0)).abs() < 0.0001);
        assert!((race.elapsed - 10.5 / 11.0).abs() < 0.0001);
        assert_eq!(race.next_checkpoint, 1);
        assert_eq!(race.completed_runs, 1);
    }

    #[test]
    fn rolling_back_off_the_grid_does_not_complete_or_invalidate_a_lap() {
        let track = circuit();
        let mut race = Race::new(None, 0.0);
        race.started = true;
        race.last = Some(30.0);
        race.update(&track, 0.25, track.length - 0.05, true);
        assert!(!race.invalid() && race.invalid_reason.is_none());
        race.update(&track, 0.25, track.length - 0.10, true);
        // Forward again across the line: 0.05 m of this 0.15 m step remains.
        assert!(race.update(&track, 0.3, 0.05, true).is_none());
        assert_eq!(race.completed_runs, 0);
        assert_eq!(race.last, Some(30.0));
        assert!(!race.invalid());
        assert!((race.elapsed - 0.1).abs() < 0.0001);
        let records = advance(&mut race, &track, 0.05, track.length, 10.0);
        assert_eq!(records.len(), 1);
        assert!((records[0] - (0.1 + (track.length - 0.05) / 10.0)).abs() < 0.001);
        assert_eq!(race.completed_runs, 1);
        assert_eq!(race.last, Some(records[0]));
    }

    #[test]
    fn sprint_jump_beyond_the_finish_ends_the_run_as_invalid() {
        let track = straight();
        let mut race = Race::new(Some(50.0), 185.0);
        race.started = true;
        race.next_checkpoint = track.checkpoints.len();
        race.last = Some(60.0);
        race.elapsed = 30.0;
        assert!(race.update(&track, 0.1, 199.0, true).is_none());
        assert!(race.finished && race.invalid());
        assert_eq!(race.invalid_reason, Some(InvalidReason::CutCourse));
        assert_eq!(race.completed_runs, 1);
        assert!(race.last.is_none());
        assert_eq!(race.best, Some(50.0));
        // The clock stops with the run.
        let stopped = race.elapsed;
        race.update(&track, 0.1, 199.5, true);
        assert_eq!(race.elapsed, stopped);
    }

    /// Drive a straight sprint, off the road wherever `off` says so.
    fn sprint_with_excursions(off: impl Fn(f32) -> bool) -> Race {
        let track = Track::parse("straight 200\ncheckpoint\nstraight 300\n").unwrap();
        let mut race = Race::new(None, 5.0);
        race.started = true;
        let mut cursor = 5.0;
        while !race.finished {
            let next = cursor + 0.25;
            race.update(&track, 0.01, next, !off(next));
            cursor = next;
        }
        race
    }

    #[test]
    fn brief_excursions_are_repaid_by_driving_on_the_road() {
        // Several short slides add up to more than the allowance, but stay
        // within the lap's total.
        let race = sprint_with_excursions(|d| d < 190.0 && d % 40.0 < 8.0);
        assert!(!race.invalid() && race.last.is_some());
        let race = sprint_with_excursions(|d| (100.0..100.0 + CUT_ALLOWANCE - 1.0).contains(&d));
        assert!(!race.invalid() && race.last.is_some());
    }

    #[test]
    fn sustained_off_road_progress_cuts_the_course() {
        let race = sprint_with_excursions(|d| (100.0..100.0 + CUT_ALLOWANCE + 1.0).contains(&d));
        assert!(race.finished && race.last.is_none());
        assert_eq!(race.invalid_reason, Some(InvalidReason::CutCourse));
        // A brief touch of the road mid-cut does not reset the meter.
        let race =
            sprint_with_excursions(|d| (100.0..120.0).contains(&d) || (124.0..140.0).contains(&d));
        assert_eq!(race.invalid_reason, Some(InvalidReason::CutCourse));
    }

    #[test]
    fn repeated_cuts_under_the_allowance_add_up_to_a_cut() {
        // Each 25 m cut is repaid by the road before the next one starts.
        let cut = |d: f32| d < 190.0 && d % 60.0 < 25.0;
        let race = sprint_with_excursions(cut);
        assert!(race.finished && race.last.is_none());
        assert_eq!(race.invalid_reason, Some(InvalidReason::CutCourse));
        // Two of them stay within the lap's total.
        let race = sprint_with_excursions(|d| d < 120.0 && cut(d));
        assert!(!race.invalid() && race.last.is_some());
    }

    #[test]
    fn a_circuit_cut_invalidates_only_that_lap() {
        let track = circuit();
        let mut race = Race::new(None, 0.0);
        race.started = true;
        let gate = track.checkpoints[0];
        let mut cursor = 0.0;
        while cursor < track.length {
            let next = (cursor + 0.5).min(track.length);
            let on_course = !(gate + 2.0..gate + 4.0 + CUT_ALLOWANCE).contains(&next);
            assert!(race.update(&track, 0.05, next, on_course).is_none());
            cursor = next;
        }
        assert_eq!(race.completed_runs, 1);
        assert!(race.last.is_none() && !race.invalid());
        assert_eq!(advance(&mut race, &track, 0.0, track.length, 10.0).len(), 1);
        assert!(race.last.is_some());
    }

    #[test]
    fn sprint_finish_is_interpolated_within_the_step() {
        let track = straight();
        let mut race = Race::new(None, 195.0);
        race.started = true;
        race.next_checkpoint = track.checkpoints.len();
        race.elapsed = 10.0;
        assert_eq!(race.update(&track, 0.4, 199.0, true), Some(10.2));
        assert!((race.elapsed - 10.2).abs() < 0.000001);
        assert!(race.finished);
    }

    #[test]
    fn long_run_keeps_clock_and_record_within_a_millisecond() {
        let track = straight();
        let mut race = Race::new(None, 5.0);
        race.started = true;
        let dt = 1.0 / 120.0;
        let total_steps = 1800 * 120;
        let steps_to_finish = 192;
        for step in 1..=total_steps - steps_to_finish {
            assert!(race.update(&track, dt, 5.0, true).is_none());
            if step == 600 * 120 {
                assert!((race.elapsed - 600.0).abs() < 0.001);
            }
        }
        let mut record = None;
        for distance in 6..=197 {
            record = race.update(&track, dt, distance as f32, true).or(record);
        }
        assert!(race.finished && !race.invalid());
        assert!((race.elapsed - 1800.0).abs() < 0.001);
        assert!((record.unwrap() - 1800.0).abs() < 0.001);
        assert_eq!(race.last, record);
        assert_eq!(race.best, record);
    }

    /// Step from `from` to `to`, in either direction, 0.25 m at a time.
    fn drive(race: &mut Race, track: &Track, from: f32, to: f32, on_course: bool) {
        let steps = ((to - from).abs() / 0.25).round() as usize;
        for step in 1..=steps {
            let distance = from + (to - from) * step as f32 / steps as f32;
            race.update(track, 0.025, distance, on_course);
        }
    }

    #[test]
    fn forward_jump_over_the_line_ends_the_cut_lap_and_starts_a_clean_one() {
        let track = circuit();
        let mut race = Race::new(None, 0.0);
        race.started = true;
        assert!(advance(&mut race, &track, 0.0, track.length - 5.0, 10.0).is_empty());
        assert!(!race.invalid());
        // Cut across the infield from 5 m before the line to 10 m past it.
        assert!(race.update(&track, 0.1, 10.0, true).is_none());
        assert_eq!(race.completed_runs, 1);
        assert!(race.last.is_none() && race.best.is_none());
        assert!(!race.invalid());
        assert_eq!(race.next_checkpoint, 0);
        assert!((race.elapsed - 0.1 * 10.0 / 15.0).abs() < 0.0001);
        let records = advance(&mut race, &track, 10.0, track.length, 10.0);
        assert_eq!(records.len(), 1);
        let expected = 0.1 * 10.0 / 15.0 + (track.length - 10.0) / 10.0;
        assert!((records[0] - expected).abs() < 0.001);
        assert_eq!(race.completed_runs, 2);
    }

    #[test]
    fn backward_jump_over_the_line_does_not_complete_a_lap() {
        let track = circuit();
        let mut race = Race::new(None, 0.0);
        race.started = true;
        assert!(advance(&mut race, &track, 0.0, 5.0, 10.0).is_empty());
        assert!(
            race.update(&track, 0.1, track.length - 10.0, true)
                .is_none()
        );
        assert_eq!(race.invalid_reason, Some(InvalidReason::CutCourse));
        assert!(
            advance(
                &mut race,
                &track,
                track.length - 10.0,
                track.length + 1.0,
                10.0
            )
            .is_empty()
        );
        // Back over the line it skipped across: a fresh lap, none completed.
        assert_eq!(race.completed_runs, 0);
        assert!(!race.invalid());
        let records = advance(&mut race, &track, 1.0, track.length, 10.0);
        assert_eq!(records.len(), 1);
        assert_eq!(race.completed_runs, 1);
    }

    #[test]
    fn rocking_off_the_road_counts_only_net_progress() {
        let track = Track::parse(
            "straight 200
checkpoint
straight 300
",
        )
        .unwrap();
        let mut race = Race::new(None, 5.0);
        race.started = true;
        drive(&mut race, &track, 5.0, 100.0, true);
        drive(&mut race, &track, 100.0, 120.0, false);
        drive(&mut race, &track, 120.0, 105.0, false);
        drive(&mut race, &track, 105.0, 121.0, false);
        // 21 m net off-road gain stays within the allowance.
        assert!(!race.invalid());
        drive(&mut race, &track, 121.0, 497.0, true);
        assert!(race.finished && !race.invalid() && race.last.is_some());
    }

    #[test]
    fn rocking_on_the_road_does_not_repay_cut_debt() {
        let track = Track::parse(
            "straight 200
checkpoint
straight 300
",
        )
        .unwrap();
        let mut race = Race::new(None, 5.0);
        race.started = true;
        drive(&mut race, &track, 5.0, 100.0, true);
        drive(&mut race, &track, 100.0, 125.0, false);
        for _ in 0..3 {
            drive(&mut race, &track, 125.0, 110.0, true);
            drive(&mut race, &track, 110.0, 125.0, true);
        }
        assert!(!race.invalid());
        // 25 m of debt is still owed, so 10 m more off the road is a cut.
        drive(&mut race, &track, 125.0, 135.0, false);
        assert_eq!(race.invalid_reason, Some(InvalidReason::CutCourse));
    }

    #[test]
    fn reversing_over_the_line_after_starting_a_lap_is_wrong_way() {
        let track = circuit();
        let mut race = Race::new(None, 0.0);
        race.started = true;
        assert!(advance(&mut race, &track, 0.0, 8.0, 10.0).is_empty());
        assert_eq!(race.next_checkpoint, 0);
        for distance in (-1..8).rev() {
            assert!(race.update(&track, 0.1, distance as f32, true).is_none());
        }
        assert_eq!(race.invalid_reason, Some(InvalidReason::WrongWay));
    }

    #[test]
    fn invalid_numeric_input_cannot_poison_the_clock_or_record() {
        let track = straight();
        let mut race = Race::new(Some(f32::NAN), 5.0);
        race.started = true;
        assert!(race.best.is_none());
        race.update(&track, f32::NAN, 6.0, true);
        race.update(&track, -1.0, 6.0, true);
        assert_eq!(race.elapsed, 0.0);
        race.update(&track, 0.1, f32::NAN, true);
        assert!(race.invalid());
        assert_eq!(race.previous, 5.0);
        assert!(race.elapsed.is_finite());
    }
}

/// Drive real car physics across a hairpin's infield: the case only an
/// accumulated off-road rule can catch, since progress never jumps.
#[cfg(test)]
mod corner_cutting {
    use super::*;
    use crate::vehicle::{Car, Control};
    use macroquad::prelude::*;

    const STEP: f32 = 1.0 / 120.0;

    /// Hold the car at `speed` along a scripted polyline. Returns the race
    /// and the largest progress change of a single physics step.
    fn follow(track: &Track, start: f32, path: &[Vec3], speed: f32) -> (Race, f32) {
        let mut car = Car::new(track);
        car.reset(track, start);
        let mut race = Race::new(None, car.distance);
        race.started = true;
        let mut largest_step: f32 = 0.0;
        for target in path {
            for _ in 0..2400 {
                let to = (*target - car.position) * vec3(1.0, 0.0, 1.0);
                if to.length() < speed * STEP * 1.5 {
                    break;
                }
                let direction = to.normalize();
                car.heading = direction.x.atan2(direction.z);
                car.yaw_rate = 0.0;
                car.velocity = direction * speed + Vec3::Y * car.velocity.y;
                let before = car.distance;
                car.update(track, Control::default(), STEP);
                race.update(track, STEP, car.distance, car.on_course());
                largest_step = largest_step.max((car.distance - before).abs());
            }
        }
        (race, largest_step)
    }

    #[test]
    fn crossing_a_hairpin_infield_cuts_the_course_but_hugging_its_edge_does_not() {
        // 12 m road: the paved inside edge is at a 14 m radius.
        let track = Track::parse(
            "straight 40\ncheckpoint\nstraight 20\nright 180 radius 20\nstraight 60\n",
        )
        .unwrap();
        let entry = track.sample_at(60.0);
        let exit = track.sample_at(60.0 + 20.0 * std::f32::consts::PI);
        // No gate inside the hairpin: a cut never crosses one off the road.
        assert!(
            track
                .checkpoints
                .iter()
                .all(|gate| *gate < 55.0 || *gate > exit.distance + 10.0)
        );
        let center = (entry.pos + exit.pos) * 0.5;
        let outward = (entry.pos - center).normalize();
        // The car's centre stays on the paved inside edge at 14.5 m.
        for (radius, cut) in [(14.5, false), (10.0, true), (6.0, true), (3.0, true)] {
            let mut path = vec![track.sample_at(58.0).pos - outward * (20.0 - radius)];
            path.extend((0..=36).map(|step| {
                let angle = std::f32::consts::PI * step as f32 / 36.0;
                center + (outward * angle.cos() + entry.forward * angle.sin()) * radius
            }));
            path.push(track.sample_at(exit.distance + 10.0).pos);
            let (race, largest_step) = follow(&track, 55.0, &path, 8.0);
            // Progress stays continuous, so the 12 m jump rule never fires.
            assert!(largest_step < 1.0, "radius {radius}: {largest_step}");
            assert_eq!(
                race.invalid_reason,
                cut.then_some(InvalidReason::CutCourse),
                "radius {radius}"
            );
        }
    }

    #[test]
    fn jumping_a_gap_longer_than_the_cut_allowance_is_not_a_cut() {
        let track =
            Track::parse("straight 60\nramp 30 rise 6\ngap 40 rise -6\nstraight 160\n").unwrap();
        const { assert!(40.0 > CUT_ALLOWANCE) };
        let mut car = Car::new(&track);
        car.reset(&track, 40.0);
        car.velocity = track.sample_at(40.0).forward * 35.0;
        let mut race = Race::new(None, car.distance);
        race.started = true;
        let mut airborne_over_gap = 0.0;
        for _ in 0..(12.0 / STEP) as usize {
            let before = car.distance;
            car.update(
                &track,
                Control {
                    throttle: 1.0,
                    ..Control::default()
                },
                STEP,
            );
            if !car.grounded && car.offroad {
                assert!(car.on_course(), "off the route guide at {}", car.distance);
                airborne_over_gap += car.distance - before;
            }
            race.update(&track, STEP, car.distance, car.on_course());
            if race.finished {
                break;
            }
        }
        assert!(airborne_over_gap > CUT_ALLOWANCE, "{airborne_over_gap}");
        assert!(race.finished, "never landed and finished: {car:?}");
        assert_eq!(race.invalid_reason, None);
    }

    #[test]
    fn landing_in_a_gaps_pit_is_off_the_course() {
        let track = Track::parse("straight 60\ngap 40\nstraight 60\n").unwrap();
        let mut car = Car::new(&track);
        car.reset(&track, 50.0);
        car.velocity = track.sample_at(50.0).forward * 8.0;
        for _ in 0..(6.0 / STEP) as usize {
            car.update(&track, Control::default(), STEP);
        }
        assert!(car.grounded && car.offroad && !car.on_course(), "{car:?}");
    }
}
