// For now we are just dumping everything into a single structure but after basic layout structure of the code is done we
// need to refactor this to better use space and cache

const math = @import("math");
const Shape = @import("shape.zig").Shape;

angularVelocity : math.Vec3,
orientation : math.Quat,

velocity : math.Vec3,
position : math.Vec3,

inertia : math.Mat3x3,
mass : f32,
inverseMass : f32,
centerOfMass : math.Vec3,
friction: f32,
shape: Shape,
