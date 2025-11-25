const std = @import("std");
const math = @import("math");
const TransformComp = @import("../body.zig").TransformComp;
const Shape = @import("shape.zig").Shape;
const contact = @import("contact.zig");

pub fn collideSphereSphere(
    a_id: u32, 
    transform_a: TransformComp, 
    shape_a: Shape,
    b_id: u32, 
    transform_b: TransformComp, 
    shape_b: Shape,
    read_cache: *const std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.Contact),
    write_cache: *std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.Contact),
) void {
    const sphere_a = shape_a.Sphere;
    const sphere_b = shape_b.Sphere;

    const vector_a_to_b = transform_b.position.sub(&transform_a.position);
    const distance_squared = vector_a_to_b.len2();
    const combined_radius = sphere_a.radius + sphere_b.radius;

    if (distance_squared > combined_radius * combined_radius) return; // no contact

    const distance = std.math.sqrt(distance_squared);
    var normal: math.Vec3 = undefined;
    if (distance > 1e-6) {
        normal = vector_a_to_b.mulScalar(1.0 / distance);
    } else {
        normal = math.vec3(0, 1, 0);
    }

    const penetration = combined_radius - distance;

    // Contact points stored in world space
    const point_a = transform_a.position.add(&normal.mulScalar(sphere_a.radius));
    const point_b = transform_b.position.sub(&normal.mulScalar(sphere_b.radius));

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
