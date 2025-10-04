const math = @import("math");

pub const Contact = struct {
    body_a: u32,
    body_b: u32,
    normal: math.Vec3,
    point: math.Vec3,
    penetration: f32,
    friction: f32,
    restitution: f32,
};
