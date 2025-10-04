const math = @import("math");

const Box = struct {
    half_extents: math.Vec3
};

const Sphere = struct {
    radius: f32
};

const Line = struct {
    point_a: math.Vec3,
    point_b: math.Vec3,
};

pub const Shape = union(enum) {
    Box: Box,
    Sphere: Sphere,
    Line: Line,
};

pub fn newBox(half_extents: math.Vec3) Shape {
    return .{ .Box = .{ .half_extents = half_extents } };
}

pub fn newSphere(radius: f32) Shape {
    return .{ .Sphere = .{ .radius = radius } };
}

pub fn newLine(point_a: math.Vec3, point_b: math.Vec3) Shape {
    return .{ .Line = .{ .point_a = point_a, .point_b = point_b } };
}
