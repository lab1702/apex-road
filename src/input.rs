//! Relative mouse driving, independent of the render rate and window edges.
use crate::vehicle::Control;
use macroquad::miniquad::KeyMods;
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

    /// Replay every motion event before reading the final position. Clamping
    /// only the frame's net motion loses reversals after reaching a limit.
    pub fn update_frame(
        &mut self,
        motion: impl IntoIterator<Item = Vec2>,
        position: Vec2,
        active: bool,
    ) {
        if self.enabled && active && self.previous_position.is_some() {
            for position in motion {
                self.update(position, true);
            }
        }
        // A reset establishes a new baseline at the current position and skips
        // queued motion from before activation, resume, or the cursor grab.
        self.update(position, active);
    }

    /// Captured positions accumulate raw mouse motion in Macroquad, so they can
    /// travel beyond the window boundaries.
    fn update(&mut self, position: Vec2, active: bool) {
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

/// Preserve mouse event order and keep a focus loss latched even if focus
/// returns within the same rendered frame. Miniquad reports Linux focus
/// changes as minimized/restored events.
pub struct WindowInput {
    pub focused: bool,
    lost: bool,
    mouse_motion: Vec<Vec2>,
}

impl WindowInput {
    pub fn new() -> Self {
        Self {
            focused: true,
            lost: false,
            mouse_motion: Vec::new(),
        }
    }

    pub fn take_loss(&mut self) -> bool {
        std::mem::take(&mut self.lost)
    }

    /// Event coordinates are physical pixels; callers convert them to the same
    /// logical pixels returned by Macroquad's `mouse_position`.
    pub fn drain_mouse_motion(&mut self) -> impl Iterator<Item = Vec2> + '_ {
        self.mouse_motion.drain(..)
    }
}

impl macroquad::miniquad::EventHandler for WindowInput {
    fn update(&mut self) {}
    fn draw(&mut self) {}

    fn mouse_motion_event(&mut self, x: f32, y: f32) {
        self.mouse_motion.push(vec2(x, y));
    }

    fn key_down_event(&mut self, _keycode: KeyCode, _keymods: KeyMods, repeat: bool) {
        // Miniquad also emits "minimized" when a macOS window moves, without
        // a matching restore event. A fresh press proves input is reaching the
        // window again. Keep the loss latched so mouse driving still pauses.
        if !repeat {
            self.focused = true;
        }
    }

    fn mouse_button_down_event(&mut self, _button: MouseButton, _x: f32, _y: f32) {
        self.focused = true;
    }

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
    fn motion_at_limits_is_independent_of_frame_splitting() {
        let positions = [vec2(3600., -3000.), vec2(3000., -2500.)];
        let mut single = active_mouse();
        single.update_frame(positions, positions[1], true);
        let mut split = active_mouse();
        for position in positions {
            split.update_frame([position], position, true);
        }
        assert_eq!(single.control(false).steer, 0.5);
        assert_eq!(single.control(false).throttle, 0.5);
        assert_eq!(single.control(false).steer, split.control(false).steer);
        assert_eq!(
            single.control(false).throttle,
            split.control(false).throttle
        );
    }

    #[test]
    fn frame_after_reset_discards_queued_motion_before_the_new_baseline() {
        let mut mouse = active_mouse();
        mouse.update(vec2(600., -500.), true);
        mouse.reset();
        mouse.update_frame(
            [vec2(3600., -3000.), vec2(3000., -2500.)],
            vec2(100., 200.),
            true,
        );
        assert_eq!(mouse.control(false).steer, 0.);
        assert_eq!(mouse.control(false).throttle, 0.);
        mouse.update_frame([vec2(250., 75.)], vec2(250., 75.), true);
        assert_eq!(mouse.control(false).steer, 0.125);
        assert_eq!(mouse.control(false).throttle, 0.125);
        mouse.update_frame([vec2(3600., -3000.)], vec2(3600., -3000.), false);
        assert_eq!(mouse.control(false).steer, 0.);
        assert_eq!(mouse.control(false).throttle, 0.);
    }

    #[test]
    fn queued_mouse_motion_retains_event_order_and_drains_each_frame() {
        use macroquad::miniquad::EventHandler;
        let mut input = WindowInput::new();
        input.mouse_motion_event(3600., -3000.);
        input.mouse_motion_event(3000., -2500.);
        assert_eq!(
            input.drain_mouse_motion().collect::<Vec<_>>(),
            [vec2(3600., -3000.), vec2(3000., -2500.)]
        );
        assert_eq!(input.drain_mouse_motion().count(), 0);
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
        let mut focus = WindowInput::new();
        focus.window_minimized_event();
        assert!(!focus.focused);
        focus.window_restored_event();
        assert!(focus.focused && focus.take_loss());
        assert!(!focus.take_loss());
    }

    #[test]
    fn fresh_input_recovers_a_missing_restore_event_without_discarding_loss() {
        use macroquad::miniquad::EventHandler;
        for mouse_press in [false, true] {
            let mut focus = WindowInput::new();
            focus.window_minimized_event();
            if mouse_press {
                focus.mouse_button_down_event(MouseButton::Right, 10.0, 20.0);
            } else {
                focus.key_down_event(KeyCode::Enter, KeyMods::default(), false);
            }
            assert!(focus.focused);
            assert!(
                focus.take_loss(),
                "active input must preserve the pause latch"
            );
            assert!(!focus.take_loss());

            // The final event controls focus: an earlier press cannot mask a
            // later focus loss in the same rendered frame.
            focus.window_minimized_event();
            assert!(!focus.focused && focus.take_loss());
        }
    }

    #[test]
    fn passive_input_does_not_restore_an_unfocused_window() {
        use macroquad::miniquad::EventHandler;
        let mut focus = WindowInput::new();
        focus.window_minimized_event();
        focus.mouse_motion_event(10.0, 20.0);
        focus.key_down_event(KeyCode::W, KeyMods::default(), true);
        focus.key_up_event(KeyCode::W, KeyMods::default());
        focus.mouse_button_up_event(MouseButton::Right, 10.0, 20.0);
        assert!(!focus.focused);
        assert!(focus.take_loss());
        assert_eq!(
            focus.drain_mouse_motion().collect::<Vec<_>>(),
            [vec2(10.0, 20.0)]
        );
    }
}
