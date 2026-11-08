//! AprilTag camera model: which field tags a robot's camera can see.
//!
//! A tag is detected when it sits inside the camera frustum (horizontal +
//! vertical half-angles), within `max_range`, on the tag's front face
//! hemisphere, and inside the vertical band. No occlusion raycasts yet —
//! field tags are mounted high on perimeter/structure faces where line of
//! sight is mostly clear (noted as future work alongside piece colliders).
//!
//! Detection is deterministic ground truth (a real camera either resolves
//! a tag or it doesn't); the *pose estimate* noise/latency/dropout lives in
//! `noise.rs` and is gated on these detections.
use crate::archetype::CameraSpec;
use crate::world::field::AprilTagSpec;
use crate::drivetrain::yaw_from_quat;

/// Tags visible to one camera this tick, nearest first.
#[derive(Debug, Default, Clone)]
pub struct TagView {
    /// tag ids (AprilTagSpec.id) sorted by distance
    pub ids: Vec<u16>,
    /// distance to the nearest visible tag, meters (f32::INFINITY if none)
    pub nearest_m: f32,
}

/// Rotate a chassis-frame offset into world space and run detection.
/// `robot_pos`/`robot_quat` are the chassis pose; `cam` the `[camera]` spec.
pub fn detect(
    cam: &CameraSpec,
    tags: &[AprilTagSpec],
    robot_pos: [f32; 3],
    robot_quat: [f32; 4],
) -> TagView {
    let yaw = yaw_from_quat(robot_quat);
    let (sy, cy) = yaw.sin_cos();
    // camera world position: chassis-frame offset rotated by yaw
    let c = cam.pos;
    let campos = [
        robot_pos[0] + c[0] * cy + c[2] * sy,
        robot_pos[1] + c[1],
        robot_pos[2] - c[0] * sy + c[2] * cy,
    ];
    let cam_yaw = yaw + cam.yaw;
    let hfov = cam.hfov_deg.to_radians() * 0.5;
    let vfov = cam.vfov_deg.unwrap_or(cam.hfov_deg).to_radians() * 0.5;

    let mut hits: Vec<(f32, u16)> = Vec::new();
    for t in tags {
        let dx = t.pos[0] - campos[0];
        let dz = t.pos[2] - campos[2];
        let hd = (dx * dx + dz * dz).sqrt();
        if !(0.2..=cam.max_range).contains(&hd) {
            continue;
        }
        // horizontal bearing inside hfov
        let bearing = dz.atan2(dx);
        let mut diff = bearing - cam_yaw;
        while diff > std::f32::consts::PI {
            diff -= 2.0 * std::f32::consts::PI;
        }
        while diff < -std::f32::consts::PI {
            diff += 2.0 * std::f32::consts::PI;
        }
        if diff.abs() > hfov {
            continue;
        }
        // vertical angle inside vfov
        let dy = t.pos[1] - campos[1];
        if (dy.abs() / hd).atan() > vfov {
            continue;
        }
        // tag front face must look at the camera: normal (tag yaw) vs
        // camera->tag reversed (tag->camera) dot > ~0 (frontal hemisphere
        // with a little tolerance for oblique reads)
        let (ty, cy2) = t.yaw.sin_cos();
        // tag normal points along its yaw; vector tag->camera in xz
        let tx = campos[0] - t.pos[0];
        let tz = campos[2] - t.pos[2];
        let tl = (tx * tx + tz * tz).sqrt().max(1e-6);
        let facing = (tx / tl) * cy2 + (tz / tl) * ty;
        if facing < 0.2 {
            continue;
        }
        hits.push((hd, t.id));
    }
    hits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    TagView {
        nearest_m: hits.first().map(|h| h.0).unwrap_or(f32::INFINITY),
        ids: hits.into_iter().map(|h| h.1).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam() -> CameraSpec {
        CameraSpec {
            pos: [0.0, 0.45, 0.0],
            yaw: 0.0,
            hfov_deg: 90.0,
            vfov_deg: Some(60.0),
            max_range: 10.0,
        }
    }

    fn tag(id: u16, pos: [f32; 3], yaw: f32) -> AprilTagSpec {
        AprilTagSpec {
            id,
            family: "36h11".into(),
            pos,
            yaw,
            size: 0.165,
        }
    }

    // identity quat = facing +x
    const Q: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

    #[test]
    fn sees_tag_dead_ahead() {
        let tags = vec![tag(7, [3.0, 1.5, 0.0], std::f32::consts::PI)]; // faces -x, toward camera
        let v = detect(&cam(), &tags, [0.0, 0.0, 0.0], Q);
        assert_eq!(v.ids, vec![7]);
        assert!((v.nearest_m - 3.0).abs() < 0.01);
    }

    #[test]
    fn misses_tag_behind_and_out_of_frustum() {
        let tags = vec![
            tag(1, [-3.0, 1.5, 0.0], 0.0), // behind (faces +x, camera looks +x — sees its back anyway)
            tag(2, [3.0, 1.5, 5.0], std::f32::consts::PI), // faces -x but off-axis
            tag(3, [30.0, 1.5, 0.0], std::f32::consts::PI), // out of range
        ];
        let v = detect(&cam(), &tags, [0.0, 0.0, 0.0], Q);
        assert!(v.ids.is_empty());
        assert!(v.nearest_m.is_infinite());
    }

    #[test]
    fn misses_tag_facing_away() {
        // tag ahead but its face looks away from the camera (+x, away)
        let tags = vec![tag(9, [3.0, 0.5, 0.0], 0.0)];
        let v = detect(&cam(), &tags, [0.0, 0.0, 0.0], Q);
        assert!(v.ids.is_empty());
    }
}
