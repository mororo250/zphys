const std = @import("std");
const Body = @import("body.zig");
const math = @import("math");

pub const World = struct {
    bodies: std.ArrayList(Body),
    gravity: math.Vec3,

    pub fn worldStep(timestep: f32, substep: u32) !void{
        // Update broadphase
        // Narrow-Phase Collision
        // Integrate velocities
        // Solve Constraints
        // Integrate positions
    }
};