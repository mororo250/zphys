const std = @import("std");
const math = @import("math");
const TransformComp = @import("../body.zig").TransformComp;
const ShapeModule = @import("shape.zig");
const Sphere = ShapeModule.Sphere;
const Box = ShapeModule.Box;
const contact = @import("contact.zig");

// Expects: A is Sphere, B is Box
pub fn collideSphereBox(
    a_id: u32, 
    transform_a: TransformComp, 
    sphere: Sphere,
    b_id: u32, 
    transform_b: TransformComp, 
    box: Box,
    read_cache: *const std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.Contact),
    write_cache: *std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.Contact),
) void {
    const closest = closestPointOnOBB(transform_a.position, transform_b.position, transform_b.orientation, box.half_extents);
    const vector_box_to_sphere = closest.sub(&transform_a.position);
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

    // Contact points stored in world space
    const point_a = transform_a.position.add(&normal.mulScalar(sphere.radius));
    const point_b = closest;

    const key = contact.CacheMainfoldKey{ .body_a = a_id, .body_b = b_id };
    const result = write_cache.getOrPutAssumeCapacity(key);
    var new_contact = result.value_ptr;
    
    new_contact.normal = normal;
    new_contact.point_a = point_a;
    new_contact.point_b = point_b;
    new_contact.penetration = penetration;
    new_contact.accumulated_impulse = 0;
    new_contact.accumulated_impulse_tangent1 = 0;
    new_contact.accumulated_impulse_tangent2 = 0;

    if (read_cache.get(key)) |cached| {
        const threshold_sq = 0.05 * 0.05;
        const dist_sq_a = point_a.dist2(&cached.point_a);
        const dist_sq_b = point_b.dist2(&cached.point_b);
        
        if (dist_sq_a < threshold_sq and dist_sq_b < threshold_sq) {
            new_contact.accumulated_impulse = cached.accumulated_impulse;
            new_contact.accumulated_impulse_tangent1 = cached.accumulated_impulse_tangent1;
            new_contact.accumulated_impulse_tangent2 = cached.accumulated_impulse_tangent2;
        }
    }
}

fn closestPointOnOBB(point: math.Vec3, center: math.Vec3, orientation: math.Quat, half_extents: math.Vec3) math.Vec3 {
    const p_local = point.sub(&center).mulQuat(&orientation.conjugate());
    const clamped = math.vec3(
        std.math.clamp(p_local.x(), -half_extents.x(), half_extents.x()),
        std.math.clamp(p_local.y(), -half_extents.y(), half_extents.y()),
        std.math.clamp(p_local.z(), -half_extents.z(), half_extents.z()),
    );
    // Back to world
    return center.add(&clamped.mulQuat(&orientation));
}
