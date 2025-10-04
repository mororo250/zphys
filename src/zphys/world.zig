const std = @import("std");
const math = @import("math");
const Body = @import("body.zig").Body;
const BodyDef = @import("body.zig").BodyDef;
const collision = @import("collision/collision.zig");


pub const World = struct {
    allocator: std.mem.Allocator,
    bodies: std.ArrayList(Body),
    gravity: math.Vec3,
    temp: WorldTemp,

    pub fn init(allocator: std.mem.Allocator) World {
        return World.initWithGravity(allocator, math.vec3(0, -9.81, 0));
    }

    pub fn initWithGravity(allocator: std.mem.Allocator, gravity: math.Vec3) World {
        return .{
            .allocator = allocator,
            .bodies = .{},
            .gravity = gravity,
            .temp = WorldTemp.init(allocator),
        };
    }

    pub fn deinit(self: *World) void {
        self.temp.deinit();
        self.bodies.deinit(self.allocator);
    }

    pub fn createBody(self: *World, def: BodyDef) !u32 {
        const id: u32 = @intCast(self.bodies.items.len);
        try self.bodies.append(self.allocator, Body.fromDef(def));
        return id;
    }

    pub fn step(self: *World, timestep: f32, substep: u16) !void {
        std.debug.assert(substep > 0);
        const dt: f32 = timestep / @as(f32, @floatFromInt(substep));

        var substep_index: u16 = 0;
        try self.temp.ensureCapacity(self.bodies.items.len);
        while (substep_index < substep) : (substep_index += 1) {
            IntegrateVelocities(self, dt);

            self.temp.clear();
            collision.generateContacts(self.bodies.items, &self.temp.contacts);
            collision.solveVelocity(self.bodies.items, self.temp.slice(), 12, dt);

            IntegratePositions(self, dt);
       }
    }

    fn IntegrateVelocities(self: *World, dt: f32) void {
        var body_index: u16 = 0;
        while (body_index < self.bodies.items.len) : (body_index += 1) {
            var body = &self.bodies.items[body_index];
            if (body.mass == 0) continue; // static
            const gravity_delta_velocity = self.gravity.mulScalar(dt);
            body.velocity = body.velocity.add(&gravity_delta_velocity);
        }
    }

    fn IntegratePositions(self: *World, dt: f32) void {
        for (0..self.bodies.items.len) |body_index| {
            var body = &self.bodies.items[body_index];
            if (body.mass == 0) continue;
            const position_delta = body.velocity.mulScalar(dt);
            body.position = body.position.add(&position_delta);
        }

        var iteration: u8 = 0;
        while (iteration < 10) : (iteration += 1) {
            self.temp.clear();
            collision.generateContacts(self.bodies.items, &self.temp.contacts);
            collision.solvePosition(self.bodies.items, self.temp.slice());
        }
    }
};

pub const WorldTemp = struct {
    allocator: std.mem.Allocator,
    contacts: std.ArrayList(collision.Contact),

    pub fn init(allocator: std.mem.Allocator) WorldTemp {
        return .{
            .allocator = allocator,
            .contacts = .{},
        };
    }

    pub fn deinit(self: *WorldTemp) void {
        self.contacts.deinit(self.allocator);
    }

    pub fn clear(self: *WorldTemp) void {
        self.contacts.clearRetainingCapacity();
    }

    pub fn ensureCapacity(self: *WorldTemp, bodies_count: usize) !void {
        if (bodies_count <= 1) return;
        const max_pairs = bodies_count * (bodies_count - 1) / 2;
        try self.contacts.ensureTotalCapacity(self.allocator, max_pairs);
    }

    pub fn slice(self: *WorldTemp) []const collision.Contact {
        return self.contacts.items;
    }
};
