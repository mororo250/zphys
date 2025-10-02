const math = @import("math");

const Box = struct {
    halfExtends: math.Vec3
};

const Sphere = struct {
    radius: f32
};

const Line = struct {
    p1: math.Vec3,
    p2: math.Vec3,
};

pub const Shape = union(enum) {
    Box: Box,
    Sphere: Sphere,
    Line: Line,
};


pub fn newBox(halfExtends: math.Vec3) Shape {
    return .{ .Box = .{ .halfExtends = halfExtends } };
}

pub fn newSphere(radius: f32) Shape {
    return .{ .Sphere = .{ .radius = radius } };
}

pub fn newLine(p1: math.Vec3, p2: math.Vec3) Shape {
    return .{ .Line = .{ .p1 = p1, .p2 = p2 } };
}