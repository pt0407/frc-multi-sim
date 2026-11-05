//! Field arena: ground plane + perimeter walls (PLAN A4 / future C1 will
//! move these to config).
use rapier3d::prelude::*;

pub const FIELD_HALF_X: f32 = 8.25;
pub const FIELD_HALF_Z: f32 = 4.0;

/// FuelSim FIELD_COR (sqrt(22/51.5)), duplicated here to avoid a module
/// cycle. Fuel colliders combine with `Max`, so fuel-field bounces use
/// this; robot colliders keep their own combine rule where `Min` keeps
/// their restitution (0.1) dominant vs the livelier field surface.
const FIELD_COR: f32 = 0.6537;

/// Insert the fixed ground body, floor collider, and four walls.
/// Returns the fixed ground body handle.
pub fn build_arena(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> RigidBodyHandle {
    let ground = bodies.insert(RigidBodyBuilder::fixed());
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.1, 20.0)
            .translation(vector![0.0, -0.1, 0.0])
            .friction(1.0)
            .restitution(FIELD_COR)
            .restitution_combine_rule(CoefficientCombineRule::Min)
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
                .restitution(FIELD_COR)
                .restitution_combine_rule(CoefficientCombineRule::Min)
                .build(),
            ground,
            bodies,
        );
    }
    ground
}
