const std = @import("std");
const math = @import("math");

inline fn signf(value: f32) f32 {
    return if (value >= 0) 1.0 else -1.0;
}

fn supportBox(center: math.Vec3, orientation: math.Quat, half_extents: math.Vec3, direction: math.Vec3) math.Vec3 {
    const axis_x = math.vec3(1, 0, 0).mulQuat(&orientation);
    const axis_y = math.vec3(0, 1, 0).mulQuat(&orientation);
    const axis_z = math.vec3(0, 0, 1).mulQuat(&orientation);

    const sign_x = signf(axis_x.dot(&direction));
    const sign_y = signf(axis_y.dot(&direction));
    const sign_z = signf(axis_z.dot(&direction));

    const corner_offset = axis_x.mulScalar(half_extents.x() * sign_x)
        .add(&axis_y.mulScalar(half_extents.y() * sign_y))
        .add(&axis_z.mulScalar(half_extents.z() * sign_z));
    return center.add(&corner_offset);
}

pub const GjkBox = struct {
    center: math.Vec3,
    orientation: math.Quat,
    half_extents: math.Vec3,

    pub fn support(self: *const @This(), direction: math.Vec3) math.Vec3 {
        return supportBox(self.center, self.orientation, self.half_extents, direction);
    }
};



// Refactored: now receives two anytype parameters (expected to provide
//   - field: center: Vec3
//   - method: support(direction: Vec3) Vec3
// The provided GjkShapeA and GjkShapeB structs above satisfy this interface.
pub fn gjkBoxesIntersect(
    shape_a: anytype,
    shape_b: anytype,
) bool {
    // Initial search direction towards B from A
    var search_direction = shape_b.center.sub(&shape_a.center);
    if (search_direction.len2() < 1e-8) search_direction = math.vec3(1, 0, 0);

    var simplex: [4]math.Vec3 = undefined;
    var simplex_size: usize = 0;

    // Add first point from Minkowski difference: S(d) = supportA(d) - supportB(-d)
    simplex[0] = shape_a.support(search_direction).sub(&shape_b.support(search_direction.negate()));
    simplex_size = 1;
    if (simplex[0].dot(&search_direction) <= 0) return false;
    search_direction = simplex[0].negate();

    var iteration: usize = 0;
    while (iteration < 30) : (iteration += 1) {
        const new_point = shape_a.support(search_direction).sub(&shape_b.support(search_direction.negate()));
        if (new_point.dot(&search_direction) <= 0) return false;

        simplex[simplex_size] = new_point;
        simplex_size += 1;

        // Handle simplex
        const contains_origin = handleSimplex(&simplex, &simplex_size, &search_direction);
        if (contains_origin) return true;
    }
    // Fallback
    return false;
}

fn handleSimplex(simplex: *[4]math.Vec3, simplex_size: *usize, search_direction: *math.Vec3) bool {
    // Based on GJK in 3D - cases line, triangle, tetrahedron
    switch (simplex_size.*) {
        2 => {
            // Line AB (A = last)
            const last_point = simplex.*[1];
            const previous_point = simplex.*[0];
            const to_origin = last_point.negate();
            const ab_edge = previous_point.sub(&last_point);

            // New direction perpendicular to AB towards origin
            const ab_cross_ao = ab_edge.cross(&to_origin);
            search_direction.* = ab_cross_ao.cross(&ab_edge);
            if (search_direction.len2() < 1e-12) {
                // pick any perpendicular
                search_direction.* = math.vec3(-ab_edge.y(), ab_edge.x(), 0);
            }
            return false;
        },
        3 => {
            // Triangle ABC (A = last)
            const last_point = simplex.*[2];
            const point_b = simplex.*[1];
            const point_c = simplex.*[0];

            const to_origin = last_point.negate();
            const ab_edge = point_b.sub(&last_point);
            const ac_edge = point_c.sub(&last_point);
            const triangle_normal = ab_edge.cross(&ac_edge);

            // Determine which side of triangle the origin lies
            const ab_perp_direction = triangle_normal.cross(&ac_edge);
            if (ab_perp_direction.dot(&to_origin) > 0) {
                // Origin is outside AC edge
                simplex.*[0] = point_c;
                simplex.*[1] = last_point;
                simplex_size.* = 2;
                search_direction.* = ac_edge.cross(&to_origin).cross(&ac_edge);
                if (search_direction.len2() < 1e-12) search_direction.* = math.vec3(-ac_edge.y(), ac_edge.x(), 0);
                return false;
            }

            const ac_perp_direction = ab_edge.cross(&triangle_normal);
            if (ac_perp_direction.dot(&to_origin) > 0) {
                // Outside AB edge
                simplex.*[0] = point_b;
                simplex.*[1] = last_point;
                simplex_size.* = 2;
                search_direction.* = ab_edge.cross(&to_origin).cross(&ab_edge);
                if (search_direction.len2() < 1e-12) search_direction.* = math.vec3(-ab_edge.y(), ab_edge.x(), 0);
                return false;
            }

            // Otherwise, origin is above/below triangle
            if (triangle_normal.dot(&to_origin) > 0) {
                search_direction.* = triangle_normal;
            } else {
                // Wind triangle the other way
                simplex.*[0] = point_b;
                simplex.*[1] = point_c;
                simplex.*[2] = last_point;
                search_direction.* = triangle_normal.negate();
            }
            return false;
        },
        4 => {
            // Tetrahedron ABCD (A = last)
            const last_point = simplex.*[3];
            const point_b = simplex.*[2];
            const point_c = simplex.*[1];
            const point_d = simplex.*[0];

            const to_origin = last_point.negate();
            const ab_edge = point_b.sub(&last_point);
            const ac_edge = point_c.sub(&last_point);
            const ad_edge = point_d.sub(&last_point);

            const face_abc = ab_edge.cross(&ac_edge);
            const face_acd = ac_edge.cross(&ad_edge);
            const face_adb = ad_edge.cross(&ab_edge);

            if (face_abc.dot(&to_origin) > 0) {
                simplex.*[0] = point_c;
                simplex.*[1] = point_b;
                simplex.*[2] = last_point;
                simplex_size.* = 3;
                search_direction.* = face_abc;
                return false;
            }
            if (face_acd.dot(&to_origin) > 0) {
                simplex.*[0] = point_d;
                simplex.*[1] = point_c;
                simplex.*[2] = last_point;
                simplex_size.* = 3;
                search_direction.* = face_acd;
                return false;
            }
            if (face_adb.dot(&to_origin) > 0) {
                simplex.*[0] = point_b;
                simplex.*[1] = point_d;
                simplex.*[2] = last_point;
                simplex_size.* = 3;
                search_direction.* = face_adb;
                return false;
            }
            // Origin is inside tetrahedron
            return true;
        },
        else => return false,
    }
}
