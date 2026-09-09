//! Windshield projection for the road's +Z-forward, +X-right convention.
use macroquad::{camera::Camera, prelude::*};

pub struct DriverCamera {
    pub position: Vec3,
    view_projection: Mat4,
}

impl DriverCamera {
    pub fn new(
        position: Vec3,
        heading: f32,
        pitch: f32,
        roll: f32,
        speed: f32,
        aspect: f32,
    ) -> Self {
        let forward = vec3(
            heading.sin() * pitch.cos(),
            pitch.sin(),
            heading.cos() * pitch.cos(),
        );
        let right = vec3(heading.cos(), 0., -heading.sin());
        let up = forward.cross(right).normalize() * roll.cos() - right * roll.sin();
        let eye = position + up * 0.84 + forward * 0.4;
        let fov = (66. + (speed / 180.).min(1.) * 5.).to_radians();
        let projection = Mat4::perspective_rh_gl(fov, aspect, 0.08, 3000.);
        let view = Mat4::look_at_rh(eye, eye + forward, up);
        // An ordinary RH camera looking along +Z puts +X on screen-left.
        // Reflect clip-space X so the driver's right agrees with the road,
        // vehicle, keyboard controls, and course map. Depth and up are unchanged.
        let screen_handedness = Mat4::from_scale(vec3(-1., 1., 1.));
        Self {
            position: eye,
            view_projection: screen_handedness * projection * view,
        }
    }
}

impl Camera for DriverCamera {
    fn matrix(&self) -> Mat4 {
        self.view_projection
    }
    fn depth_enabled(&self) -> bool {
        true
    }
    fn render_pass(&self) -> Option<RenderPass> {
        None
    }
    fn viewport(&self) -> Option<(i32, i32, i32, i32)> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drivers_right_is_screen_right_at_every_heading() {
        for heading in [
            0.,
            0.7,
            std::f32::consts::FRAC_PI_2,
            std::f32::consts::PI,
            -std::f32::consts::FRAC_PI_2,
        ] {
            let camera = DriverCamera::new(Vec3::ZERO, heading, 0., 0., 60., 16. / 9.);
            let forward = vec3(heading.sin(), 0., heading.cos());
            let right = vec3(heading.cos(), 0., -heading.sin());
            let ahead = camera.position + forward * 20.;
            let center = camera.matrix().project_point3(ahead);
            assert!(camera.matrix().project_point3(ahead + right * 3.).x > center.x);
            assert!(camera.matrix().project_point3(ahead - right * 3.).x < center.x);
            assert!(camera.matrix().project_point3(ahead + Vec3::Y * 3.).y > center.y);
            assert!((-1.0..1.0).contains(&center.z));
        }
    }

    #[test]
    fn hills_and_banking_preserve_screen_handedness() {
        for (pitch, roll) in [(0.2_f32, 0.3_f32), (-0.2, -0.3)] {
            let forward = vec3(0., pitch.sin(), pitch.cos());
            let level_up = forward.cross(Vec3::X).normalize();
            let right = Vec3::X * roll.cos() + level_up * roll.sin();
            let up = level_up * roll.cos() - Vec3::X * roll.sin();
            let camera = DriverCamera::new(Vec3::ZERO, 0., pitch, roll, 60., 16. / 9.);
            let ahead = camera.position + forward * 20.;
            let center = camera.matrix().project_point3(ahead);
            assert!(camera.matrix().project_point3(ahead + right * 3.).x > center.x);
            assert!(camera.matrix().project_point3(ahead + up * 3.).y > center.y);
        }
    }
}
