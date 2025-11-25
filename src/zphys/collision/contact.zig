const math = @import("math");

pub const CacheMainfoldKey = struct {
    body_a: u32,
    body_b: u32,
};

pub const ContactManifold = struct {
    normal: math.Vec3,
    contact_points_a: [4]math.Vec3,
    contact_points_b: [4]math.Vec3,
    accumulated_impulse: [4]f32,
    accumulated_impulse_tangent1: [4]f32,
    accumulated_impulse_tangent2: [4]f32,
    length: u32,
    penetration_depth: f32,
};

pub const Contact = struct {
    normal: math.Vec3,
    point_a: math.Vec3,
    point_b: math.Vec3,
    accumulated_impulse: f32,
    accumulated_impulse_tangent1: f32,
    accumulated_impulse_tangent2: f32,
    penetration: f32,
};


/// Todo: Check paper constraints derivations for rigid body simulation in 3D - Daniel Chappuis -> U term
/// Jacobian constraint(-n, -(r_1 x n, n, r_2 x n)
/// n -> axis of the constraint
///
pub const PenetrationConstraint = struct {
    r1: math.Vec3, // lever arm 1
    r2: math.Vec3, // lever arm 2
    n: math.Vec3, // collision normal

    // Precomputed world-space inertia transformations
    // Todo: performance test if this is faster than passing the inverse inertia and the orientation  -> It would be less memory, but it would increase the calculation quite a bit in every constraint
    inv_inertia_r1_cross_n: math.Vec3,
    inv_inertia_r2_cross_n: math.Vec3,
    inv_inertia_r1_cross_t1: math.Vec3,
    inv_inertia_r2_cross_t1: math.Vec3,
    inv_inertia_r1_cross_t2: math.Vec3,
    inv_inertia_r2_cross_t2: math.Vec3,

    accumulated_impulse: f32,
    accumulated_impulse_tangent1: f32,
    accumulated_impulse_tangent2: f32,
    inv_mass_a: f32,
    inv_mass_b: f32,
    friction: f32, // Combined friction coefficient

    velocity_bias: f32, // Check slide 44 of Erin catto presentation: https://box2d.org/files/ErinCatto_ModelingAndSolvingConstraints_GDC2009.pdf -> treat bounce as velocity bias

    body_a: u32,
    body_b: u32,
};
