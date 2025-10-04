const std = @import("std");
const math = @import("math");
const Body = @import("../body.zig").Body;
const contact = @import("contact.zig");

// Expects: A is Sphere, B is Box
pub fn collideSphereBox(a_id: u32, sphere_body: *const Body, b_id: u32, box_body: *const Body, out: *std.ArrayList(contact.Contact)) void {
    const sphere = sphere_body.shape.Sphere;
    const box = box_body.shape.Box;

    const closest = closestPointOnOBB(sphere_body.position, box_body.position, box_body.orientation, box.half_extents);
    const vector_box_to_sphere = closest.sub(&sphere_body.position);
    const distance_squared = vector_box_to_sphere.len2();

    if (distance_squared > sphere.radius * sphere.radius) return;

    var normal: math.Vec3 = undefined;
    const distance = std.math.sqrt(distance_squared);
    if (distance > 1e-6) {
        normal = vector_box_to_sphere.mulScalar(1.0 / distance); // from box->sphere
    } else {
        // Choose a reasonable normal (up)
        normal = math.vec3(0, 1, 0);
    }

    const penetration = sphere.radius - distance;
    const point = closest;

    const friction = std.math.sqrt(@max(sphere_body.friction, 0) * @max(box_body.friction, 0));
    const restitution = @max(sphere_body.restitution, box_body.restitution);

    out.appendAssumeCapacity( .{
        .body_a = a_id,
        .body_b = b_id,
        .normal = normal, // from A(sphere) to B(box)? Here delta was sphere - closest => normal points from box to sphere.
        .point = point,
        .penetration = penetration,
        .friction = friction,
        .restitution = restitution,
    });
}

fn closestPointOnOBB(point: math.Vec3, center: math.Vec3, orientation: math.Quat, half_extents: math.Vec3) math.Vec3 {
    const p_local = point.sub(&center).mulQuat(&orientation.inverse());
    const clamped = math.vec3(
        std.math.clamp(p_local.x(), -half_extents.x(), half_extents.x()),
        std.math.clamp(p_local.y(), -half_extents.y(), half_extents.y()),
        std.math.clamp(p_local.z(), -half_extents.z(), half_extents.z()),
    );
    // Back to world
    return center.add(&clamped.mulQuat(&orientation));
}
