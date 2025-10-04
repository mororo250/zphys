const std = @import("std");
const math = @import("math");

fn getBoxAxes(center: math.Vec3, orientation: math.Quat, half_extents: math.Vec3) struct {
    center: math.Vec3,
    axes: [3]math.Vec3,
    half_extents: math.Vec3,
} {
    const axis_x = math.vec3(1, 0, 0).mulQuat(&orientation);
    const axis_y = math.vec3(0, 1, 0).mulQuat(&orientation);
    const axis_z = math.vec3(0, 0, 1).mulQuat(&orientation);
    return .{ .center = center, .axes = .{ axis_x, axis_y, axis_z }, .half_extents = half_extents };
}

pub fn satBoxBoxContact(
    a_center: math.Vec3,
    a_orientation: math.Quat,
    a_half_extents: math.Vec3,
    b_center: math.Vec3,
    b_orientation: math.Quat,
    b_half_extents: math.Vec3,
) ?struct {
    normal: math.Vec3,
    penetration: f32,
} {
    const box_a = getBoxAxes(a_center, a_orientation, a_half_extents);
    const box_b = getBoxAxes(b_center, b_orientation, b_half_extents);

    // Compute rotation matrix expressing B in A's frame
    var rotation: [3][3]f32 = undefined;
    var abs_rotation: [3][3]f32 = undefined;
    inline for (0..3) |i| {
        inline for (0..3) |j| {
            rotation[i][j] = box_a.axes[i].dot(&box_b.axes[j]);
            // Add epsilon to account for parallelism
            abs_rotation[i][j] = @abs(rotation[i][j]) + 1e-6;
        }
    }

    // Translation vector from A to B in A's frame
    const translation_world = box_b.center.sub(&box_a.center);
    const translation_in_a = math.vec3(
        translation_world.dot(&box_a.axes[0]),
        translation_world.dot(&box_a.axes[1]),
        translation_world.dot(&box_a.axes[2]),
    );
    const translation_in_a_components = [_]f32{ translation_in_a.x(), translation_in_a.y(), translation_in_a.z() };

    var min_overlap: f32 = std.math.inf(f32);
    var best_axis_world: math.Vec3 = math.vec3(0, 1, 0);

    // Test axes L = A0, A1, A2
    {
        // A0
        const projection_radius_a = box_a.half_extents.x();
        const projection_radius_b = box_b.half_extents.x() * abs_rotation[0][0]
            + box_b.half_extents.y() * abs_rotation[0][1]
            + box_b.half_extents.z() * abs_rotation[0][2];
        const center_distance = @abs(translation_in_a.x());
        if (center_distance > projection_radius_a + projection_radius_b) return null;
        const overlap = (projection_radius_a + projection_radius_b) - center_distance;
        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_axis_world = if (translation_in_a.x() >= 0) box_a.axes[0] else box_a.axes[0].negate();
        }

        // A1
        const projection_radius_a_1 = box_a.half_extents.y();
        const projection_radius_b_1 = box_b.half_extents.x() * abs_rotation[1][0]
            + box_b.half_extents.y() * abs_rotation[1][1]
            + box_b.half_extents.z() * abs_rotation[1][2];
        const center_distance_1 = @abs(translation_in_a.y());
        if (center_distance_1 > projection_radius_a_1 + projection_radius_b_1) return null;
        const overlap_1 = (projection_radius_a_1 + projection_radius_b_1) - center_distance_1;
        if (overlap_1 < min_overlap) {
            min_overlap = overlap_1;
            best_axis_world = if (translation_in_a.y() >= 0) box_a.axes[1] else box_a.axes[1].negate();
        }

        // A2
        const projection_radius_a_2 = box_a.half_extents.z();
        const projection_radius_b_2 = box_b.half_extents.x() * abs_rotation[2][0]
            + box_b.half_extents.y() * abs_rotation[2][1]
            + box_b.half_extents.z() * abs_rotation[2][2];
        const center_distance_2 = @abs(translation_in_a.z());
        if (center_distance_2 > projection_radius_a_2 + projection_radius_b_2) return null;
        const overlap_2 = (projection_radius_a_2 + projection_radius_b_2) - center_distance_2;
        if (overlap_2 < min_overlap) {
            min_overlap = overlap_2;
            best_axis_world = if (translation_in_a.z() >= 0) box_a.axes[2] else box_a.axes[2].negate();
        }
    }

    // Test axes L = B0, B1, B2
    {
        // B0
        const projection_radius_a = box_a.half_extents.x() * abs_rotation[0][0]
            + box_a.half_extents.y() * abs_rotation[1][0]
            + box_a.half_extents.z() * abs_rotation[2][0];
        const projection_radius_b = box_b.half_extents.x();
        const center_distance = @abs(
            translation_in_a.x() * rotation[0][0] + translation_in_a.y() * rotation[1][0] + translation_in_a.z() * rotation[2][0],
        );
        if (center_distance > projection_radius_a + projection_radius_b) return null;
        const overlap = (projection_radius_a + projection_radius_b) - center_distance;
        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_axis_world = if ((translation_world.dot(&box_b.axes[0])) >= 0) box_b.axes[0] else box_b.axes[0].negate();
        }

        // B1
        const projection_radius_a_1 = box_a.half_extents.x() * abs_rotation[0][1]
            + box_a.half_extents.y() * abs_rotation[1][1]
            + box_a.half_extents.z() * abs_rotation[2][1];
        const projection_radius_b_1 = box_b.half_extents.y();
        const center_distance_1 = @abs(
            translation_in_a.x() * rotation[0][1] + translation_in_a.y() * rotation[1][1] + translation_in_a.z() * rotation[2][1],
        );
        if (center_distance_1 > projection_radius_a_1 + projection_radius_b_1) return null;
        const overlap_1 = (projection_radius_a_1 + projection_radius_b_1) - center_distance_1;
        if (overlap_1 < min_overlap) {
            min_overlap = overlap_1;
            best_axis_world = if ((translation_world.dot(&box_b.axes[1])) >= 0) box_b.axes[1] else box_b.axes[1].negate();
        }

        // B2
        const projection_radius_a_2 = box_a.half_extents.x() * abs_rotation[0][2]
            + box_a.half_extents.y() * abs_rotation[1][2]
            + box_a.half_extents.z() * abs_rotation[2][2];
        const projection_radius_b_2 = box_b.half_extents.z();
        const center_distance_2 = @abs(
            translation_in_a.x() * rotation[0][2] + translation_in_a.y() * rotation[1][2] + translation_in_a.z() * rotation[2][2],
        );
        if (center_distance_2 > projection_radius_a_2 + projection_radius_b_2) return null;
        const overlap_2 = (projection_radius_a_2 + projection_radius_b_2) - center_distance_2;
        if (overlap_2 < min_overlap) {
            min_overlap = overlap_2;
            best_axis_world = if ((translation_world.dot(&box_b.axes[2])) >= 0) box_b.axes[2] else box_b.axes[2].negate();
        }
    }

    // Test axes L = Ai x Bj
    inline for (0..3) |i| {
        inline for (0..3) |j| {
            const projection_radius_a = switch (i) {
                0 => box_a.half_extents.y() * abs_rotation[2][j] + box_a.half_extents.z() * abs_rotation[1][j],
                1 => box_a.half_extents.x() * abs_rotation[2][j] + box_a.half_extents.z() * abs_rotation[0][j],
                2 => box_a.half_extents.x() * abs_rotation[1][j] + box_a.half_extents.y() * abs_rotation[0][j],
                else => 0,
            };
            const projection_radius_b = switch (j) {
                0 => box_b.half_extents.y() * abs_rotation[i][2] + box_b.half_extents.z() * abs_rotation[i][1],
                1 => box_b.half_extents.x() * abs_rotation[i][2] + box_b.half_extents.z() * abs_rotation[i][0],
                2 => box_b.half_extents.x() * abs_rotation[i][1] + box_b.half_extents.y() * abs_rotation[i][0],
                else => 0,
            };

            const i_plus_1 = (i + 1) % 3;
            const i_plus_2 = (i + 2) % 3;

            const projected_center_distance = @abs(
                translation_in_a_components[i_plus_2] * rotation[i_plus_1][j] - translation_in_a_components[i_plus_1] * rotation[i_plus_2][j],
            );
            if (projected_center_distance > projection_radius_a + projection_radius_b) return null;

            const overlap = (projection_radius_a + projection_radius_b) - projected_center_distance;
            if (overlap < min_overlap and overlap >= 0) {
                min_overlap = overlap;
                var candidate_axis = box_a.axes[i].cross(&box_b.axes[j]);
                // If axes are near parallel, skip
                if (candidate_axis.len2() > 1e-8) {
                    candidate_axis = candidate_axis.normalize(0);
                    // Ensure axis points from A to B
                    if (candidate_axis.dot(&translation_world) < 0) candidate_axis = candidate_axis.negate();
                    best_axis_world = candidate_axis;
                }
            }
        }
    }

    return .{ .normal = best_axis_world.normalize(0), .penetration = min_overlap };
}
