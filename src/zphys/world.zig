const std = @import("std");
const math = @import("math");
const Body = @import("body.zig").Body;
const BodyDef = @import("body.zig").BodyDef;

pub const World = struct {
    allocator: std.mem.Allocator,
    bodies: std.ArrayList(Body),
    gravity: math.Vec3,

    pub fn init(allocator: std.mem.Allocator) World {
        return World.initWithGravity(allocator, math.vec3(0, -9.81, 0));
    }

    pub fn initWithGravity(allocator: std.mem.Allocator, gravity: math.Vec3) World {
        return .{
            .allocator = allocator,
            .bodies = .{},
            .gravity = gravity,
        };
    }

    pub fn deinit(self: *World) void {
        self.bodies.deinit(self.allocator);
    }

    pub fn createBody(self: *World, def: BodyDef) !u32 {
        const id: u32 = @intCast(self.bodies.items.len);
        try self.bodies.append(self.allocator, Body.fromDef(def));
        return id;
    }

    pub fn step(self: *World, timestep: f32, substep: u32) !void {
        _ = self;
        _ = timestep;
        _ = substep;

        // TODO: integrate velocities, apply gravity, collisions, etc.
    }
};
