const std = @import("std");
const math = @import("math");
const MotionComp = @import("body.zig").MotionComp;
const contact = @import("collision/contact.zig");
const PenetrationConstraint = contact.PenetrationConstraint;


/// Solve Jacobian constraint(-n, -(r_0 x n, n, r_2 x n)
inline fn solveContactConstraint(constraint: *contact.PenetrationConstraint, motionA: *MotionComp, motionB: *MotionComp) void {
    const r1_cross_n = constraint.r1.cross(&constraint.n);
    const r2_cross_n = constraint.r2.cross(&constraint.n);
    
    // Compute effective mass for normal direction using precomputed inertia transforms
    const k1_n = constraint.inv_mass_a + constraint.inv_mass_b;
    const k2_n = r1_cross_n.dot(&constraint.inv_inertia_r1_cross_n);
    const k3_n = r2_cross_n.dot(&constraint.inv_inertia_r2_cross_n);
    const inverse_effective_mass_n = if (k1_n + k2_n + k3_n > 0) 1.0 / (k1_n + k2_n + k3_n) else 0;
    
    var jv = constraint.n.dot(&motionB.velocity.sub(&motionA.velocity));
    jv -= r1_cross_n.dot(&motionA.angularVelocity);
    jv += r2_cross_n.dot(&motionB.angularVelocity);

    var impulse = -inverse_effective_mass_n * (jv - constraint.velocity_bias);
    {
        const new_accumulated = @max(0.0, constraint.accumulated_impulse + impulse);
        impulse = new_accumulated - constraint.accumulated_impulse;
        constraint.accumulated_impulse = new_accumulated;
    }

    if (impulse != 0) {
        const impulse_n = constraint.n.mulScalar(impulse);
        
        motionA.velocity = motionA.velocity.sub(&impulse_n.mulScalar(constraint.inv_mass_a));
        motionA.angularVelocity = motionA.angularVelocity.sub(&constraint.inv_inertia_r1_cross_n.mulScalar(impulse));
        motionB.velocity = motionB.velocity.add(&impulse_n.mulScalar(constraint.inv_mass_b));
        motionB.angularVelocity = motionB.angularVelocity.add(&constraint.inv_inertia_r2_cross_n.mulScalar(impulse));
     }

    // Friction constraints using Coulomb friction model
    // todo: Can we optimize the friction calculation by doing it in a single step?
    // todo: Remove code repetition
    const max_friction_impulse = constraint.friction * constraint.accumulated_impulse;
    const out_tangent1 = constraint.n.getNormalizePerpendicular();
    
    const r1_cross_t1 = constraint.r1.cross(&out_tangent1);
    const r2_cross_t1 = constraint.r2.cross(&out_tangent1);
    
    // Compute effective mass for tangent1 direction using precomputed inertia transforms
    const k1_t1 = constraint.inv_mass_a + constraint.inv_mass_b;
    const k2_t1 = r1_cross_t1.dot(&constraint.inv_inertia_r1_cross_t1);
    const k3_t1 = r2_cross_t1.dot(&constraint.inv_inertia_r2_cross_t1);
    const inverse_effective_mass_t1 = if (k1_t1 + k2_t1 + k3_t1 > 0) 1.0 / (k1_t1 + k2_t1 + k3_t1) else 0;

    jv = out_tangent1.dot(&motionB.velocity.sub(&motionA.velocity));
    jv -= r1_cross_t1.dot(&motionA.angularVelocity);
    jv += r2_cross_t1.dot(&motionB.angularVelocity);

    impulse = -inverse_effective_mass_t1 * jv;
    {
        const new_accumulated = @min(@max(-max_friction_impulse, constraint.accumulated_impulse_tangent1 + impulse), max_friction_impulse);
        impulse = new_accumulated - constraint.accumulated_impulse_tangent1;
        constraint.accumulated_impulse_tangent1 = new_accumulated;
    }

    if (impulse != 0) {
        const impulse_t1 = out_tangent1.mulScalar(impulse);
        
        motionA.velocity = motionA.velocity.sub(&impulse_t1.mulScalar(constraint.inv_mass_a));
        motionA.angularVelocity = motionA.angularVelocity.sub(&constraint.inv_inertia_r1_cross_t1.mulScalar(impulse));
        motionB.velocity = motionB.velocity.add(&impulse_t1.mulScalar(constraint.inv_mass_b));
        motionB.angularVelocity = motionB.angularVelocity.add(&constraint.inv_inertia_r2_cross_t1.mulScalar(impulse));
    }

    const out_tangent2 = constraint.n.cross(&out_tangent1);
    const r1_cross_t2 = constraint.r1.cross(&out_tangent2);
    const r2_cross_t2 = constraint.r2.cross(&out_tangent2);
    
    // Compute effective mass for tangent2 direction using precomputed inertia transforms
    const k1_t2 = constraint.inv_mass_a + constraint.inv_mass_b;
    const k2_t2 = r1_cross_t2.dot(&constraint.inv_inertia_r1_cross_t2);
    const k3_t2 = r2_cross_t2.dot(&constraint.inv_inertia_r2_cross_t2);
    const inverse_effective_mass_t2 = if (k1_t2 + k2_t2 + k3_t2 > 0) 1.0 / (k1_t2 + k2_t2 + k3_t2) else 0;
    
    jv = out_tangent2.dot(&motionB.velocity.sub(&motionA.velocity));
    jv -= r1_cross_t2.dot(&motionA.angularVelocity);
    jv += r2_cross_t2.dot(&motionB.angularVelocity);

    impulse = -inverse_effective_mass_t2 * jv;
    {
        const new_accumulated = @min(@max(-max_friction_impulse, constraint.accumulated_impulse_tangent2 + impulse), max_friction_impulse);
        impulse = new_accumulated - constraint.accumulated_impulse_tangent2;
        constraint.accumulated_impulse_tangent2 = new_accumulated;
    }

    if (impulse != 0) {
        const impulse_t2 = out_tangent2.mulScalar(impulse);
        
        motionA.velocity = motionA.velocity.sub(&impulse_t2.mulScalar(constraint.inv_mass_a));
        motionA.angularVelocity = motionA.angularVelocity.sub(&constraint.inv_inertia_r1_cross_t2.mulScalar(impulse));
        motionB.velocity = motionB.velocity.add(&impulse_t2.mulScalar(constraint.inv_mass_b));
        motionB.angularVelocity = motionB.angularVelocity.add(&constraint.inv_inertia_r2_cross_t2.mulScalar(impulse));
    }
}

/// Iteratively solve all penetration constraints
/// This is the main constraint solver that should be called from the physics step
pub fn solveConstraints(
    motion: []  MotionComp,
    constraints: []  PenetrationConstraint,
    iterations: u32
) void {
    // Warm Start: Apply accumulated impulses from previous frame
    for (constraints) |*constraint| {
        const motionA = &motion[constraint.body_a];
        const motionB = &motion[constraint.body_b];

        // Normal impulse
        if (constraint.accumulated_impulse != 0) {
            const impulse_n = constraint.n.mulScalar(constraint.accumulated_impulse);
            motionA.velocity = motionA.velocity.sub(&impulse_n.mulScalar(constraint.inv_mass_a));
            motionA.angularVelocity = motionA.angularVelocity.sub(&constraint.inv_inertia_r1_cross_n.mulScalar(constraint.accumulated_impulse));
            motionB.velocity = motionB.velocity.add(&impulse_n.mulScalar(constraint.inv_mass_b));
            motionB.angularVelocity = motionB.angularVelocity.add(&constraint.inv_inertia_r2_cross_n.mulScalar(constraint.accumulated_impulse));
        }

        // Friction impulses
        const out_tangent1 = constraint.n.getNormalizePerpendicular();
        const out_tangent2 = constraint.n.cross(&out_tangent1);

        if (constraint.accumulated_impulse_tangent1 != 0) {
            const impulse_t1 = out_tangent1.mulScalar(constraint.accumulated_impulse_tangent1);
            motionA.velocity = motionA.velocity.sub(&impulse_t1.mulScalar(constraint.inv_mass_a));
            motionA.angularVelocity = motionA.angularVelocity.sub(&constraint.inv_inertia_r1_cross_t1.mulScalar(constraint.accumulated_impulse_tangent1));
            motionB.velocity = motionB.velocity.add(&impulse_t1.mulScalar(constraint.inv_mass_b));
            motionB.angularVelocity = motionB.angularVelocity.add(&constraint.inv_inertia_r2_cross_t1.mulScalar(constraint.accumulated_impulse_tangent1));
        }

        if (constraint.accumulated_impulse_tangent2 != 0) {
            const impulse_t2 = out_tangent2.mulScalar(constraint.accumulated_impulse_tangent2);
            motionA.velocity = motionA.velocity.sub(&impulse_t2.mulScalar(constraint.inv_mass_a));
            motionA.angularVelocity = motionA.angularVelocity.sub(&constraint.inv_inertia_r1_cross_t2.mulScalar(constraint.accumulated_impulse_tangent2));
            motionB.velocity = motionB.velocity.add(&impulse_t2.mulScalar(constraint.inv_mass_b));
            motionB.angularVelocity = motionB.angularVelocity.add(&constraint.inv_inertia_r2_cross_t2.mulScalar(constraint.accumulated_impulse_tangent2));
        }
    }

    var iteration: u32 = 0;
    while (iteration < iterations) : (iteration += 1) {
        for (constraints) |*constraint| {
            solveContactConstraint(
                constraint,
                &motion[constraint.body_a],
                &motion[constraint.body_b]
            );
        }
    }
}
