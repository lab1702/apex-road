//! Windshield projection for the road's +Z-forward, +X-right convention.
use macroquad::{camera::Camera, prelude::*};

pub struct DriverCamera {
    pub position: Vec3,
    view_projection: Mat4,
    vertical_focal_length: f32,
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
            vertical_focal_length: projection.y_axis.y,
        }
    }

    /// Project a distant sky disc into pixels, with no parallax as the car moves.
    pub fn project_sky_disc(
        &self,
        direction: Vec3,
        angular_radius: f32,
        viewport: Vec2,
    ) -> Option<(Vec2, f32)> {
        // A direction has homogeneous w = 0, so camera translation is ignored.
        let clip = self.view_projection * direction.extend(0.);
        if clip.w <= f32::EPSILON {
            return None;
        }
        let center = vec2(
            (clip.x / clip.w + 1.) * viewport.x * 0.5,
            (1. - clip.y / clip.w) * viewport.y * 0.5,
        );
        // Retain a circular, stylized disc while respecting the camera's FOV.
        let radius = viewport.y * 0.5 * self.vertical_focal_length * angular_radius.tan();
        if center.x + radius < 0.
            || center.x - radius > viewport.x
            || center.y + radius < 0.
            || center.y - radius > viewport.y
        {
            return None;
        }
        // Ignore clip-space depth: a sky direction is beyond the far plane.
        Some((center, radius))
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

    #[test]
    fn sky_disc_follows_heading_pitch_and_bank() {
        let viewport = vec2(1440., 900.);
        let azimuth = 0.4_f32;
        let elevation = 0.3_f32;
        let direction = vec3(
            azimuth.sin() * elevation.cos(),
            elevation.sin(),
            azimuth.cos() * elevation.cos(),
        );
        let project = |heading, pitch, roll| {
            DriverCamera::new(Vec3::ZERO, heading, pitch, roll, 0., 1.6)
                .project_sky_disc(direction, 4_f32.to_radians(), viewport)
                .map(|(center, _)| center)
        };
        let initial = project(0., 0., 0.).unwrap();
        let turning = project(0.2, 0., 0.).unwrap();
        assert!(initial.x > turning.x && turning.x > viewport.x * 0.5);
        let aligned = project(azimuth, elevation, 0.).unwrap();
        assert!(aligned.distance(viewport * 0.5) < 0.001);
        assert!(project(azimuth + std::f32::consts::PI, 0., 0.).is_none());
        let level = project(azimuth, 0., 0.).unwrap();
        let banked = project(azimuth, 0., 0.3).unwrap();
        assert!(level.y < viewport.y * 0.5);
        assert!(banked.x > level.x && banked.y > level.y);
    }

    #[test]
    fn sky_disc_has_no_translation_parallax() {
        let project = |position| {
            DriverCamera::new(position, 0.2, 0.1, -0.15, 60., 1.6)
                .project_sky_disc(vec3(0.45, 0.35, 1.).normalize(), 0.07, vec2(1440., 900.))
                .unwrap()
        };
        let (origin, radius) = project(Vec3::ZERO);
        let (translated, translated_radius) = project(vec3(500., 80., -700.));
        assert!(origin.distance(translated) < 0.05);
        assert_eq!(radius, translated_radius);
    }

    #[test]
    fn sky_disc_size_respects_resolution_aspect_and_fov() {
        let direction = vec3(0.3, 0.2, 1.).normalize();
        let project = |viewport: Vec2, speed| {
            DriverCamera::new(Vec3::ZERO, 0., 0., 0., speed, viewport.x / viewport.y)
                .project_sky_disc(direction, 0.07, viewport)
                .unwrap()
        };
        let (center, radius) = project(vec2(960., 540.), 0.);
        let (large_center, large_radius) = project(vec2(1920., 1080.), 0.);
        assert!(large_center.distance(center * 2.) < 0.001);
        assert_eq!(large_radius, radius * 2.);
        let (wide_center, wide_radius) = project(vec2(1280., 540.), 0.);
        assert_eq!(wide_radius, radius);
        assert!((wide_center.x - center.x - 160.).abs() < 0.001);
        assert!((wide_center.y - center.y).abs() < 0.001);
        assert!(project(vec2(960., 540.), 180.).1 < radius);
    }

    #[test]
    fn sky_disc_can_be_partially_visible_at_viewport_edge() {
        let viewport = vec2(1440., 900.);
        let camera = DriverCamera::new(Vec3::ZERO, 0., 0., 0., 0., 1.6);
        let angular_radius = 0.07;
        let (_, radius) = camera
            .project_sky_disc(Vec3::Z, angular_radius, viewport)
            .unwrap();
        let direction_at = |ndc_x| vec3(ndc_x * 1.6 / camera.vertical_focal_length, 0., 1.);
        let partial = camera
            .project_sky_disc(
                direction_at(1. + radius / viewport.x),
                angular_radius,
                viewport,
            )
            .unwrap();
        assert!(partial.0.x > viewport.x && partial.0.x - radius < viewport.x);
        assert!(
            camera
                .project_sky_disc(
                    direction_at(1. + 3. * radius / viewport.x),
                    angular_radius,
                    viewport,
                )
                .is_none()
        );
        assert!(
            camera
                .project_sky_disc(Vec3::X, angular_radius, viewport)
                .is_none()
        );
    }
}
