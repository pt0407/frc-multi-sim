//! Articulated mechanism bodies (issue #57, SPIKE) — ONE revolute-jointed
//! arm rigid body per robot, proving the pattern for real mechanisms
//! (deploying intakes) instead of abstract AABB capture volumes.
//!
//! Design notes / pitfalls found during the spike:
//!
//! - Impulse joints (not multibody joints): Rapier solves impulse joints as
//!   soft constraints alongside contacts, so a 4-kg arm on a 60-kg chassis
//!   stays stable at 400 Hz without changing solver iterations. Multibody
//!   joints would be stiffer but use a different (articulated) solver path
//!   with more setup; not needed for this scale.
//! - The revolute joint's free dof is `JointAxis::AngX` in Rapier's generic
//!   joint representation (the joint's local X is aligned to the supplied
//!   axis) — motor/limit calls must use `AngX`, not the world z axis.
//! - `contacts_enabled(false)` on the joint is essential: without it the
//!   arm's collider fights the chassis collider at the pivot and the robot
//!   jitters on the spot.
//! - The position servo is `MotorModel::ForceBased` (default). Stiffness
//!   here is impulse-domain, not a clean spring constant — values around
//!   300 N·m/rad with ~10% critical damping deploy a light arm in ~0.4 s
//!   without oscillation. `motor_max_force` caps what the arm can transmit
//!   back into the chassis; leave it finite or a deployed arm acts like a
//!   rigid lever that can shove the 60-kg chassis around unrealistically.
//! - Joint limits keep the stowed arm off the ground; without them a sagging
//!   arm drags on the floor and adds a friction force the drivetrain model
//!   never asked for.
use crate::archetype::MechSpec;
use rapier3d::prelude::*;

/// Runtime handles for one robot's mechanism.
pub struct Mech {
    /// the arm rigid body (dynamic)
    pub arm: RigidBodyHandle,
    /// arm collider
    pub arm_collider: ColliderHandle,
    /// chassis<->arm revolute impulse joint
    pub joint: ImpulseJointHandle,
    /// spawn-time spec (motor gains, limits) for command application
    pub spec: MechSpec,
}

/// Spawn the arm body + collider + revolute joint for `spec`, attached to
/// `chassis` which currently sits at `chassis_t` with yaw 0 (all spawns are
/// unrotated — see `chassis::default_spec`). The arm extends
/// `half_extents[0]` along its local +x from the pivot; its center is
/// placed at `pivot + R_z(stow_angle) * [half_x, 0, 0]`.
pub fn spawn_arm(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut ImpulseJointSet,
    chassis: RigidBodyHandle,
    spec: &MechSpec,
) -> Mech {
    let base = *bodies[chassis].translation();
    let hx = spec.half_extents[0];
    let (s, c) = spec.stow_angle.sin_cos();
    let center = vector![
        base.x + spec.pivot[0] + c * hx,
        base.y + spec.pivot[1] + s * hx,
        base.z + spec.pivot[2]
    ];
    let arm = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(center)
            .rotation(Vector::z() * spec.stow_angle)
            .additional_mass(spec.mass)
            // light arm: a touch of damping kills servo-limit buzz
            .linear_damping(0.1)
            .angular_damping(0.4)
            .ccd_enabled(true)
            .build(),
    );
    let arm_collider = colliders.insert_with_parent(
        ColliderBuilder::cuboid(
            spec.half_extents[0],
            spec.half_extents[1],
            spec.half_extents[2],
        )
        .friction(0.5)
        .restitution(0.05)
        .density(1.0)
        .build(),
        arm,
        bodies,
    );
    let joint = RevoluteJointBuilder::new(Vector::z_axis())
        .local_anchor1(point![
            spec.pivot[0],
            spec.pivot[1],
            spec.pivot[2]
        ])
        // pivot sits at the arm's inboard (-x) end
        .local_anchor2(point![-hx, 0.0, 0.0])
        .contacts_enabled(false)
        .limits(spec.limits)
        .motor_model(MotorModel::ForceBased)
        .motor_position(spec.stow_angle, spec.motor_stiffness, spec.motor_damping)
        .motor_max_force(spec.motor_max_force)
        .build();
    let jh = joints.insert(chassis, arm, joint, true);
    Mech {
        arm,
        arm_collider,
        joint: jh,
        spec: *spec,
    }
}

/// Signed arm angle about the pivot's z axis, relative to world +x (chassis
/// yaw is not subtracted — chassis yaw ≈ 0 in every current spawn). Used by
/// tests and (later) mechanism telemetry.
pub fn arm_angle(bodies: &RigidBodySet, arm: RigidBodyHandle) -> f32 {
    let rot = bodies[arm].rotation();
    // arm direction = rotated local +x; atan2 of its y/x world components
    let d = rot * vector![1.0, 0.0, 0.0];
    d.y.atan2(d.x)
}
