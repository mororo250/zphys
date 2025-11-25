const std = @import("std");
const math = @import("math");
const TransformComp = @import("../body.zig").TransformComp;
const Shape = @import("shape.zig").Shape;
const contact = @import("contact.zig");
const gjk = @import("gjk.zig");
const epa = @import("epa.zig");
const manifold_between_two_faces = @import("manifold_between_two_faces.zig");

pub fn collideBoxBox(
    a_id: u32, 
    transform_a: TransformComp, 
    shape_a: Shape,
    b_id: u32, 
    transform_b: TransformComp, 
    shape_b: Shape,
    read_cache: *const std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.ContactManifold),
    write_cache: *std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.ContactManifold),
) void {
    const box_a = shape_a.Box;
    const box_b = shape_b.Box;

    // GJK for detection
    const gjk_shape_a = gjk.GjkBox{ .center = transform_a.position, .orientation = transform_a.orientation, .half_extents = box_a.half_extents };
    const gjk_shape_b = gjk.GjkBox{ .center = transform_b.position, .orientation = transform_b.orientation, .half_extents = box_b.half_extents };

    // CSO (A − B) and support-point buffers:
    // - For box–box, the CSO can have up to 16 vertices in general position.
    // - EPA grows the Minkowski simplex by appending support points; we must also
    //   store the matching A/B support points for each Minkowski vertex.
    // - Therefore, all three arrays below must have the same capacity (16).
    var minkowski_points: [16]math.Vec3 = undefined;
    var shape_a_points: [16]math.Vec3 = undefined;
    var shape_b_points: [16]math.Vec3 = undefined;
    // Note: With V = 16, EPA's worst-case face count is F ≤ 2V − 4 = 28 (see epa.zig).
    const simplex_arrays: [3][]math.Vec3 = .{ minkowski_points[0..], shape_a_points[0..], shape_b_points[0..] };

    const intersects = gjk.gjkIntersect(simplex_arrays, gjk_shape_a, gjk_shape_b);
    if (!intersects) return;

    const epa_result = epa.epa(simplex_arrays, gjk_shape_a, gjk_shape_b);

    var penetration_axis = epa_result.penetration_axis;
    const delta_centers = transform_b.position.sub(&transform_a.position);
    if (delta_centers.dot(&penetration_axis) < 0) {
        penetration_axis = penetration_axis.negate();
    }

    const face_a = gjk_shape_a.getSupportFace(penetration_axis);
    const face_b = gjk_shape_b.getSupportFace(penetration_axis.negate());

    const max_length= face_a.len + face_b.len;
    var face_a_contact_points: [max_length]math.Vec3 = undefined;
    var face_b_contact_points: [max_length]math.Vec3 = undefined;

    const key = contact.CacheMainfoldKey{ .body_a = a_id, .body_b = b_id };
    const result = write_cache.getOrPutAssumeCapacity(key);
    var manifold = result.value_ptr;
    
    // Initialize manifold
    manifold.normal = penetration_axis.normalize(math.eps_f32);
    manifold.penetration_depth = epa_result.penetration_depth;
    manifold.accumulated_impulse = [_]f32{0} ** 4;
    manifold.accumulated_impulse_tangent1 = [_]f32{0} ** 4;
    manifold.accumulated_impulse_tangent2 = [_]f32{0} ** 4;

    const manifold_size = manifold_between_two_faces.manifoldBetweenTwoFaces(
        face_a.len + face_b.len,
        &face_a,
        &face_b,
        penetration_axis,
        &face_a_contact_points,
        &face_b_contact_points,
    ) catch {
        // Fallback: use EPA result
        manifold.length = 1;
        manifold.contact_points_a = .{ epa_result.collision_point_a, undefined, undefined, undefined };
        manifold.contact_points_b = .{ epa_result.collision_point_b, undefined, undefined, undefined };
        
        warmStartManifold(manifold, read_cache, key);
        return;
    };

    var contact_points_a: []math.Vec3 = &manifold.contact_points_a;
    var contact_points_b: []math.Vec3 = &manifold.contact_points_b;

    if (manifold_size > 4) {
        manifold_between_two_faces.pruneContactPoints(
            max_length,
            penetration_axis,
            face_a_contact_points[0..manifold_size],
            face_b_contact_points[0..manifold_size],
            &contact_points_a,
            &contact_points_b,
        );
        manifold.length = @intCast(contact_points_a.len);
    } else {
        for (0..manifold_size) |i| {
            manifold.contact_points_a[i] = face_a_contact_points[i];
            manifold.contact_points_b[i] = face_b_contact_points[i];
        }
        manifold.length = @intCast(manifold_size);
    }
    
    warmStartManifold(manifold, read_cache, key);
}

fn warmStartManifold(
    manifold: *contact.ContactManifold, 
    read_cache: *const std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.ContactManifold),
    key: contact.CacheMainfoldKey
) void {
    if (read_cache.get(key)) |cached| {
        const threshold_sq = 0.05 * 0.05;
        
        for (0..manifold.length) |i| {
            for (0..cached.length) |j| {
                const dist_sq_a = manifold.contact_points_a[i].dist2(&cached.contact_points_a[j]);
                const dist_sq_b = manifold.contact_points_b[i].dist2(&cached.contact_points_b[j]);
                
                if (dist_sq_a < threshold_sq and dist_sq_b < threshold_sq) {
                    manifold.accumulated_impulse[i] = cached.accumulated_impulse[j];
                    manifold.accumulated_impulse_tangent1[i] = cached.accumulated_impulse_tangent1[j];
                    manifold.accumulated_impulse_tangent2[i] = cached.accumulated_impulse_tangent2[j];
                    break;
                }
            }
        }
    }
}

test "epa_crash_repro" {
    const a = gjk.GjkBox{
        .center = math.vec3(0, 0, 0),
        .orientation = math.Quat.identity(),
        .half_extents = math.vec3(0.5, 0.5, 0.5),
    };
    // Very slight overlap (e.g. 0.00001)
    // This position was causing issues/crashes before the fix for max simplex size
    const b = gjk.GjkBox{
        .center = math.vec3(0.99999, 0, 0),
        .orientation = math.Quat.identity(),
        .half_extents = math.vec3(0.5, 0.5, 0.5),
    };
    var simplex_points: [16]math.Vec3 = undefined;
    var shape_a_points: [16]math.Vec3 = undefined;
    var shape_b_points: [16]math.Vec3 = undefined;
    const simplex_arrays: [3][]math.Vec3 = .{ simplex_points[0..], shape_a_points[0..], shape_b_points[0..] };
    
    if (gjk.gjkIntersect(simplex_arrays, a, b)) {
        const res = epa.epa(simplex_arrays, a, b);
        // We expect a very small penetration depth
        try std.testing.expect(res.penetration_depth < 0.001);
    }
}
