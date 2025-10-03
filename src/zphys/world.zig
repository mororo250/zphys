const std = @import("std");
const math = @import("math");
const Body = @import("body.zig").Body;
const BodyDef = @import("body.zig").BodyDef;
const collision = @import("collision.zig");

const DEFAULT_CONTACT_CAP: usize = 8192;

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
            .temp = WorldTemp.init(),
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

    pub fn step(self: *World, timestep: f32, substep: u16) !void {
        std.debug.assert(substep > 0);
        const dt: f32 = timestep / @as(f32, @floatFromInt(substep));

        var i: u16 = 0;
        while (i < substep) : (i += 1) {
            IntegrateVelocities(self, dt);

            self.temp.clear();
            try collision.generateContacts(self.bodies.items, self.temp.outItems(), self.temp.outLenPtr());
            collision.solveVelocity(self.bodies.items, self.temp.slice(), 12, dt);

            try IntegratePositions(self, dt);
       }
    }

    fn IntegrateVelocities(self: *World, dt: f32) void {
        var j: u16 = 0;
        while (j < self.bodies.items.len) : (j += 1) {
            var b = &self.bodies.items[j];
            if (b.mass == 0) continue; // static
                    const gdt = self.gravity.mulScalar(dt);
            b.velocity = b.velocity.add(&gdt);
        }
    }

    fn IntegratePositions(self: *World, dt: f32) !void {
        for (0..self.bodies.items.len) |j| {
            var b = &self.bodies.items[j];
            if (b.mass == 0) continue;
            const vdt = b.velocity.mulScalar(dt);
            b.position = b.position.add(&vdt);
        }

        var j: u8 = 0;
        while (j < 10) : (j += 1) {
            self.temp.clear();
            try collision.generateContacts(self.bodies.items, self.temp.outItems(), self.temp.outLenPtr());
            collision.solvePosition(self.bodies.items, self.temp.slice());
        }
    }
};

pub const WorldTemp = struct {
    contacts_buf: [DEFAULT_CONTACT_CAP]collision.Contact,
    contacts_len: usize,

    pub fn init() WorldTemp {
        return .{
            .contacts_buf = undefined,
            .contacts_len = 0,
        };
    }

    pub fn clear(self: *WorldTemp) void {
        self.contacts_len = 0;
    }

    pub fn outItems(self: *WorldTemp) []collision.Contact {
        return self.contacts_buf[0..self.contacts_buf.len];
    }

    pub fn outLenPtr(self: *WorldTemp) *usize {
        return &self.contacts_len;
    }

    pub fn slice(self: *WorldTemp) []const collision.Contact {
        return self.contacts_buf[0..self.contacts_len];
    }
};
