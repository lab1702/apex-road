//! Direction-sensitive, ordered checkpoint timing.
use crate::track::Track;

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
    pub invalid: bool,
    previous: f32,
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
            invalid: false,
            previous: if start.is_finite() { start } else { 0.0 },
        }
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
            self.invalid = true;
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
        let mut delta = distance - self.previous;
        if track.closed {
            if delta < -track.length * 0.5 {
                delta += track.length;
            }
            if delta > track.length * 0.5 {
                delta -= track.length;
                // Returning across the start backwards cannot complete the
                // current lap by immediately crossing forwards again.
                self.invalid = true;
            }
        }
        if delta.abs() > 12. {
            self.invalid = true;
            self.previous = distance;
            return None;
        }
        if delta > 0. {
            let end = self.previous + delta;
            let finish = track.finish_distance();
            self.cross_checkpoints(track, self.previous, end.min(finish), on_course);
            if self.previous < finish && end >= finish {
                self.completed_runs += 1;
                if self.next_checkpoint != track.checkpoints.len() || !on_course {
                    self.invalid = true;
                }
                // Divide the simulation step at the finish plane. Otherwise
                // the completed lap gains a frame and the next one loses it.
                let remainder = f64::from(dt) * f64::from(((end - finish) / delta).clamp(0.0, 1.0));
                let completed_time = self.elapsed - remainder;
                let record_time = completed_time as f32;
                let result = if !self.invalid && self.best.is_none_or(|best| record_time < best) {
                    self.best = Some(record_time);
                    Some(record_time)
                } else {
                    None
                };
                self.last = if self.invalid {
                    None
                } else {
                    Some(record_time)
                };
                if track.closed {
                    self.elapsed = remainder;
                    self.next_checkpoint = 0;
                    self.invalid = false;
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
                self.invalid = true;
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
        assert!(r.finished && r.invalid);
        assert_eq!(r.completed_runs, 1);
        assert!(r.best.is_none());
    }
    #[test]
    fn teleport_invalidates_time() {
        let t = straight();
        let mut r = Race::new(None, 5.);
        r.started = true;
        r.update(&t, 1., 150., true);
        assert!(r.invalid);
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
        assert!(!race.invalid);
        race.update(&track, 0.1, gate + 2.0, true);
        assert_eq!(race.next_checkpoint, 1);
        assert!(!race.invalid);

        let mut race = Race::new(None, 2.0);
        race.started = true;
        race.next_checkpoint = track.checkpoints.len();
        race.update(&track, 0.1, track.length - 2.0, true);
        assert!(race.last.is_none());
        assert!(race.best.is_none());
        assert!(race.invalid);
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
        assert!(race.invalid);
        assert!(race.update(&track, 0.1, 0.0, true).is_none());
        assert_eq!(race.best, Some(100.0));
        assert!(race.last.is_none());
        assert!(!race.invalid);
        assert_eq!(race.next_checkpoint, 0);

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
        assert!(!race.finished && !race.invalid);
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
        assert!(!race.invalid);
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
        assert!(race.invalid);
        assert!(advance(&mut race, &track, 30.0, track.length, 10.0).is_empty());
        assert_eq!(race.best, Some(100.0));
        assert!(race.last.is_none());
    }

    #[test]
    fn frame_can_cross_finish_and_first_checkpoint() {
        let mut track = circuit();
        track.checkpoints = vec![10.0];
        let mut race = Race::new(None, track.length - 0.5);
        race.started = true;
        race.next_checkpoint = 1;
        race.elapsed = 20.0;
        let record = race.update(&track, 1.0, 10.5, true).unwrap();
        assert!((record - (20.0 + 0.5 / 11.0)).abs() < 0.0001);
        assert!((race.elapsed - 10.5 / 11.0).abs() < 0.0001);
        assert_eq!(race.next_checkpoint, 1);
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
        assert!(race.finished && !race.invalid);
        assert!((race.elapsed - 1800.0).abs() < 0.001);
        assert!((record.unwrap() - 1800.0).abs() < 0.001);
        assert_eq!(race.last, record);
        assert_eq!(race.best, record);
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
        assert!(race.invalid);
        assert_eq!(race.previous, 5.0);
        assert!(race.elapsed.is_finite());
    }
}
