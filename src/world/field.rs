//! Field arena: ground plane + perimeter walls (PLAN A4 / future C1 will
//! move these to config).
use rapier3d::prelude::*;

pub const FIELD_HALF_X: f32 = 8.25;
pub const FIELD_HALF_Z: f32 = 4.0;

/// Insert the fixed ground body, floor collider, and four walls.
/// Returns the fixed ground body handle.
pub fn build_arena(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> RigidBodyHandle {
    let ground = bodies.insert(RigidBodyBuilder::fixed());
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.1, 20.0)
            .translation(vector![0.0, -0.1, 0.0])
            .friction(1.0)
            .build(),
        ground,
        bodies,
    );
    // field walls
    for (x, z, hx, hz) in [
        (0.0, -FIELD_HALF_Z, FIELD_HALF_X, 0.05),
        (0.0, FIELD_HALF_Z, FIELD_HALF_X, 0.05),
        (-FIELD_HALF_X, 0.0, 0.05, FIELD_HALF_Z),
        (FIELD_HALF_X, 0.0, 0.05, FIELD_HALF_Z),
    ] {
        colliders.insert_with_parent(
            ColliderBuilder::cuboid(hx, 0.6, hz)
                .translation(vector![x, 0.3, z])
                .friction(0.6)
                .build(),
            ground,
            bodies,
        );
    }
    ground
}
