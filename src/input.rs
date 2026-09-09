//! Relative mouse driving, independent of the render rate and window edges.
use crate::vehicle::Control;
use macroquad::prelude::*;

// Logical pixels from neutral to full steering or full throttle/brake.
const STEERING_TRAVEL: f32 = 1200.;
const PEDAL_TRAVEL: f32 = 1000.;

#[derive(Default)]
pub struct MouseDriving {
    enabled: bool,
    steering: f32,
    pedal: f32,
    previous_position: Option<Vec2>,
}

impl MouseDriving {
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
        self.reset();
    }

    pub fn reset(&mut self) {
        self.steering = 0.;
        self.pedal = 0.;
        self.previous_position = None;
    }

    /// Call once per rendered frame. Captured positions accumulate raw mouse
    /// motion in Macroquad, so they can travel beyond the window boundaries.
    pub fn update(&mut self, position: Vec2, active: bool) {
        if !self.enabled || !active {
            self.reset();
            return;
        }
        if let Some(previous) = self.previous_position {
            let movement = position - previous;
            self.steering = (self.steering + movement.x / STEERING_TRAVEL).clamp(-1., 1.);
            self.pedal = (self.pedal - movement.y / PEDAL_TRAVEL).clamp(-1., 1.);
        }
        self.previous_position = Some(position);
    }

    pub fn control(&self, handbrake: bool) -> Control {
        Control {
            steer: self.steering,
            throttle: self.pedal.max(0.),
            brake: (-self.pedal).max(0.),
            handbrake,
        }
    }
}

/// Miniquad reports Linux focus changes as minimized/restored events.
/// Keep a loss latched even if focus returns within the same rendered frame.
pub struct WindowFocus {
    pub focused: bool,
    lost: bool,
}

impl WindowFocus {
    pub fn new() -> Self {
        Self {
            focused: true,
            lost: false,
        }
    }

    pub fn take_loss(&mut self) -> bool {
        std::mem::take(&mut self.lost)
    }
}

impl macroquad::miniquad::EventHandler for WindowFocus {
    fn update(&mut self) {}
    fn draw(&mut self) {}

    fn window_minimized_event(&mut self) {
        self.focused = false;
        self.lost = true;
    }

    fn window_restored_event(&mut self) {
        self.focused = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn active_mouse() -> MouseDriving {
        let mut mouse = MouseDriving::default();
        mouse.toggle();
        mouse.update(Vec2::ZERO, true);
        mouse
    }

    #[test]
    fn horizontal_motion_steers_and_holds_until_moved_back() {
        let mut mouse = active_mouse();
        mouse.update(vec2(150., 0.), true);
        assert_eq!(mouse.control(false).steer, 0.125);
        mouse.update(vec2(150., 0.), true);
        assert_eq!(mouse.control(false).steer, 0.125);
        mouse.update(vec2(-150., 0.), true);
        assert_eq!(mouse.control(false).steer, -0.125);
        assert!(mouse.control(true).handbrake);
    }

    #[test]
    fn vertical_motion_crosses_coasting_between_throttle_and_brake() {
        let mut mouse = active_mouse();
        for (y, throttle, brake) in [
            (-250., 0.25, 0.),
            (-125., 0.125, 0.),
            (0., 0., 0.),
            (125., 0., 0.125),
            (250., 0., 0.25),
            (125., 0., 0.125),
            (0., 0., 0.),
            (-125., 0.125, 0.),
        ] {
            mouse.update(vec2(0., y), true);
            let control = mouse.control(false);
            assert_eq!((control.throttle, control.brake), (throttle, brake));
        }
    }

    #[test]
    fn limits_do_not_accumulate_hidden_travel() {
        let mut mouse = active_mouse();
        mouse.update(vec2(3600., -3000.), true);
        assert_eq!(mouse.control(false).steer, 1.);
        assert_eq!(mouse.control(false).throttle, 1.);
        mouse.update(vec2(3000., -2500.), true);
        assert_eq!(mouse.control(false).steer, 0.5);
        assert_eq!(mouse.control(false).throttle, 0.5);
    }

    #[test]
    fn motion_is_independent_of_frame_splitting() {
        let mut single = active_mouse();
        single.update(vec2(150., -125.), true);
        let mut split = active_mouse();
        for frame in 1..=10 {
            split.update(vec2(15., -12.5) * frame as f32, true);
        }
        assert!((single.control(false).steer - split.control(false).steer).abs() < 1e-6);
        assert!((single.control(false).throttle - split.control(false).throttle).abs() < 1e-6);
    }

    #[test]
    fn toggling_suspending_and_resetting_discard_stale_motion() {
        let mut mouse = active_mouse();
        mouse.update(vec2(100., -100.), true);
        mouse.toggle();
        mouse.update(vec2(900., 900.), true);
        assert!(!mouse.enabled());
        assert_eq!(mouse.control(false).steer, 0.);
        assert_eq!(mouse.control(false).throttle, 0.);
        mouse.toggle();
        mouse.update(vec2(-900., -900.), true);
        assert_eq!(mouse.control(false).steer, 0.);
        assert_eq!(mouse.control(false).throttle, 0.);
        mouse.update(vec2(-800., -1000.), true);
        mouse.update(vec2(900., 900.), false);
        mouse.update(vec2(500., 500.), true);
        assert_eq!(mouse.control(false).steer, 0.);
        assert_eq!(mouse.control(false).throttle, 0.);
        mouse.update(vec2(650., 375.), true);
        mouse.reset();
        mouse.update(vec2(-500., 500.), true);
        assert!(mouse.enabled());
        assert_eq!(mouse.control(false).steer, 0.);
        assert_eq!(mouse.control(false).throttle, 0.);
    }

    #[test]
    fn focus_loss_is_latched_until_handled() {
        use macroquad::miniquad::EventHandler;
        let mut focus = WindowFocus::new();
        focus.window_minimized_event();
        assert!(!focus.focused);
        focus.window_restored_event();
        assert!(focus.focused && focus.take_loss());
        assert!(!focus.take_loss());
    }
}
