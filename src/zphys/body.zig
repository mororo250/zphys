const math = @import("math");
const shape = @import("shape.zig");
const Shape = shape.Shape;

// Todo: Should I move this into it's own file?
pub const BodyDef = struct {
    angularVelocity: math.Vec3,
    orientation: math.Quat,

    velocity: math.Vec3,
    position: math.Vec3,

    inertia: math.Mat3x3,
    mass: f32,
    centerOfMass: math.Vec3,
    friction: f32,
    restitution: f32,

    shape: Shape,

    pub fn default() BodyDef {
        return BodyDef{
            .angularVelocity = math.vec3(0, 0, 0),
            .orientation = math.Quat.identity(),
            .velocity = math.vec3(0, 0, 0),
            .position = math.vec3(0, 0, 0),
            .inertia = math.Mat3x3.init(
                &math.vec3(1, 0, 0),
                &math.vec3(0, 1, 0),
                &math.vec3(0, 0, 1),
            ),
            .mass = 0.0,
            .centerOfMass = math.vec3(0, 0, 0),
            .friction = 0.5,
            .restitution = 0.5,
            // default placeholder shape (unit sphere)
            .shape = shape.newSphere(1.0),
        };
    }
};

pub const Body = struct {
    angularVelocity: math.Vec3,
    orientation: math.Quat,

    velocity: math.Vec3,
    position: math.Vec3,

    inertia: math.Mat3x3,
    mass: f32,
    inverseMass: f32,
    centerOfMass: math.Vec3,
    friction: f32,
    restitution: f32,

    shape: Shape,

    pub fn fromDef(def: BodyDef) Body {
        const inv = if (def.mass == 0) 0 else 1.0 / def.mass;
        return Body{
            .angularVelocity = def.angularVelocity,
            .orientation = def.orientation,
            .velocity = def.velocity,
            .position = def.position,
            .inertia = def.inertia,
            .mass = def.mass,
            .inverseMass = inv,
            .centerOfMass = def.centerOfMass,
            .friction = def.friction,
            .restitution = def.restitution,
            .shape = def.shape,
        };
    }
};
