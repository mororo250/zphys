const std = @import("std");
const math = @import("math");
const Body = @import("../body.zig").Body;

const contact = @import("contact.zig");
const sphere_sphere = @import("sphere_sphere.zig");
const sphere_box = @import("sphere_box.zig");
const box_box = @import("box_box.zig");
const gjk = @import("gjk.zig");
const sat = @import("sat.zig");

pub const Contact = contact.Contact;

pub const collideSphereSphere = sphere_sphere.collideSphereSphere;
pub const collideSphereBox = sphere_box.collideSphereBox;
pub const collideBoxBox = box_box.collideBoxBox;

pub const gjkBoxesIntersect = gjk.gjkBoxesIntersect;
pub const satBoxBoxContact = sat.satBoxBoxContact;

// Todo: Add BroadPhase collision check in here
pub fn generateContacts(bodies: []const Body, out: *std.ArrayList(Contact)) void {
    var index_a: usize = 0;
    while (index_a < bodies.len) : (index_a += 1) {
        var index_b: usize = index_a + 1;
        while (index_b < bodies.len) : (index_b += 1) {
            const body_a = &bodies[index_a];
            const body_b = &bodies[index_b];

            if (body_a.mass == 0 and body_b.mass == 0) continue;

            switch (body_a.shape) {
                .Sphere => |_| {
                    switch (body_b.shape) {
                        .Sphere => |_| collideSphereSphere(@intCast(index_a), body_a, @intCast(index_b), body_b, out),
                        .Box => |_| collideSphereBox(@intCast(index_a), body_a, @intCast(index_b), body_b, out),
                        else => {},
                    }
                },
                .Box => |_| {
                    switch (body_b.shape) {
                        .Sphere => |_| {
                            // sphere-box expects sphere as A, box as B; swap roles
                            const previous_len = out.items.len;
                            collideSphereBox(@intCast(index_b), body_b, @intCast(index_a), body_a, out);
                            // If a contact was added, remap to keep ordering A=index_a (box), B=index_b (sphere)
                            if (out.items.len > previous_len) {
                                var contact_ref = &out.items[out.items.len - 1];
                                // ensure normal points from A(index_a) to B(index_b)
                                contact_ref.normal = contact_ref.normal.negate();
                                // set indices to match (index_a -> index_b)
                                contact_ref.body_a = @intCast(index_a);
                                contact_ref.body_b = @intCast(index_b);
                            }
                        },
                        .Box => |_| collideBoxBox(@intCast(index_a), body_a, @intCast(index_b), body_b, out),
                        else => {},
                    }
                },
                else => {},
            }
        }
    }
}

pub fn solveVelocity(bodies: []Body, contacts: []const Contact, iterations: u32, delta_time: f32) void {
    const baumgarte: f32 = 0.3;
    const penetration_slop: f32 = 0.003;

    var iteration_index: u32 = 0;
    while (iteration_index < iterations) : (iteration_index += 1) {
        for (contacts) |contact_entry| {
            const body_a = &bodies[contact_entry.body_a];
            const body_b = &bodies[contact_entry.body_b];

            const inv_mass_a = body_a.inverseMass;
            const inv_mass_b = body_b.inverseMass;
            const inv_mass_sum = inv_mass_a + inv_mass_b;
            if (inv_mass_sum == 0) continue;

            const contact_normal = contact_entry.normal.normalize(0);

            // Relative velocity at contact (linear only for now)
            const relative_velocity = body_b.velocity.sub(&body_a.velocity);
            const velocity_along_normal = relative_velocity.dot(&contact_normal);

            // Penetration slop and early exit for separating contacts
            const corrected_penetration = @max(contact_entry.penetration - penetration_slop, 0.0);
            if (velocity_along_normal > 0 and corrected_penetration <= 0) continue;

            // Restitution only on closing velocity
            const restitution = if (velocity_along_normal < -0.5) contact_entry.restitution else 0.0;

            // Baumgarte positional bias as velocity term
            const bias_velocity = if (delta_time > 0) (baumgarte * corrected_penetration / delta_time) else 0.0;

            // Normal impulse (include restitution + bias)
            var normal_impulse_magnitude = (-(1.0 + restitution) * velocity_along_normal - bias_velocity) / inv_mass_sum;
            if (normal_impulse_magnitude < 0) normal_impulse_magnitude = 0;

            if (normal_impulse_magnitude > 0) {
                const normal_impulse = contact_normal.mulScalar(normal_impulse_magnitude);
                body_a.velocity = body_a.velocity.sub(&normal_impulse.mulScalar(inv_mass_a));
                body_b.velocity = body_b.velocity.add(&normal_impulse.mulScalar(inv_mass_b));
            }

            // Friction (Coulomb, clamped by mu * |jn|)
            const relative_velocity_after_normal = body_b.velocity.sub(&body_a.velocity);
            var tangent = relative_velocity_after_normal.sub(&contact_normal.mulScalar(relative_velocity_after_normal.dot(&contact_normal)));
            const tangent_len2 = tangent.len2();
            if (tangent_len2 > 1e-12) {
                tangent = tangent.mulScalar(1.0 / std.math.sqrt(tangent_len2));
                const tangential_impulse_magnitude = -(relative_velocity_after_normal.dot(&tangent)) / inv_mass_sum;
                const friction_coefficient = contact_entry.friction;

                const max_friction = friction_coefficient * normal_impulse_magnitude;
                var clamped_tangential_impulse = tangential_impulse_magnitude;
                if (clamped_tangential_impulse > max_friction) clamped_tangential_impulse = max_friction;
                if (clamped_tangential_impulse < -max_friction) clamped_tangential_impulse = -max_friction;

                const tangential_impulse = tangent.mulScalar(clamped_tangential_impulse);
                body_a.velocity = body_a.velocity.sub(&tangential_impulse.mulScalar(inv_mass_a));
                body_b.velocity = body_b.velocity.add(&tangential_impulse.mulScalar(inv_mass_b));
            }
        }
    }
}

pub fn solvePosition(bodies: []Body, contacts: []const Contact) void {
    const correction_percent: f32 = 0.2;
    const penetration_slop: f32 = 0.005;

    for (contacts) |contact_entry| {
        const body_a = &bodies[contact_entry.body_a];
        const body_b = &bodies[contact_entry.body_b];

        const inv_mass_a = body_a.inverseMass;
        const inv_mass_b = body_b.inverseMass;
        const inv_mass_sum = inv_mass_a + inv_mass_b;
        if (inv_mass_sum == 0) continue;

        const correction_magnitude = correction_percent * @max(contact_entry.penetration - penetration_slop, 0.0) / inv_mass_sum;
        const correction = contact_entry.normal.normalize(0).mulScalar(correction_magnitude);

        body_a.position = body_a.position.sub(&correction.mulScalar(inv_mass_a));
        body_b.position = body_b.position.add(&correction.mulScalar(inv_mass_b));
    }
}
