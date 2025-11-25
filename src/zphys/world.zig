const std = @import("std");
const math = @import("math");
const body_module = @import("body.zig");
const BodyDef = body_module.BodyDef;
const BodyComponents = body_module.BodyComponents;
const MotionComp = body_module.MotionComp;
const TransformComp = body_module.TransformComp;
const PhysicsPropsComp = body_module.PhysicsPropsComp;
const Shape = @import("collision/shape.zig").Shape;
const contact = @import("collision/contact.zig");
const collision = @import("collision/collision.zig");
const constraint = @import("constraint.zig");

// Todo: Separate static bodies into different list for optimization reasons
pub const World = struct {
    allocator: std.mem.Allocator,
    bodies: std.MultiArrayList(BodyComponents),
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

    pub fn bodyCount(self: *const World) usize {
        return self.bodies.len;
    }

    pub fn getMotion(self: *const World, index: usize) MotionComp {
        return self.bodies.items(.motion)[index];
    }

    pub fn getTransform(self: *const World, index: usize) TransformComp {
        return self.bodies.items(.transform)[index];
    }

    pub fn getPhysicsProps(self: *const World, index: usize) PhysicsPropsComp {
        return self.bodies.items(.physics_props)[index];
    }

    pub fn getShape(self: *const World, index: usize) Shape {
        return self.bodies.items(.shape)[index];
    }

    pub fn createBody(self: *World, def: BodyDef) !u32 {
        const id: u32 = @intCast(self.bodies.len);

        const components = body_module.componentsFromDef(def);

        try self.bodies.append(self.allocator, components);

        try self.temp.ensureCapacity(self.bodyCount());
        return id;
    }

    pub fn step(self: *World, timestep: f32, substep_count: u16) !void {
        std.debug.assert(substep_count > 0);
        const dt: f32 = timestep / @as(f32, @floatFromInt(substep_count));

        var substep_index: u16 = 0;
        while (substep_index < substep_count) : (substep_index += 1) {
            substep(self, dt);
        }
    }

    pub fn substep(self: *World, dt: f32) void {
        applyGravity(self, dt);
        self.temp.clear();
        self.temp.swapCacheBuffers();
        
        collision.generateContacts(
            self.bodies.slice(),
            self.temp.getReadManifoldCache(),
            self.temp.getWriteManifoldCache(),
            self.temp.getReadContactCache(),
            self.temp.getWriteContactCache(),
        );

        collision.buildPenetrationConstraints(
            self.bodies.slice(),
            self.temp.getWriteManifoldCache(),
            self.temp.getWriteContactCache(),
            &self.temp.penetrationConstraints
        );
        constraint.solveConstraints(self.bodies.items(.motion), self.temp.penetrationConstraints.items, 10);
        
        collision.updateCacheFromConstraints(
            self.temp.getWriteManifoldCache(),
            self.temp.getWriteContactCache(),
            self.temp.penetrationConstraints.items
        );

        integratePositions(self, dt);
        collision.solvePosition(
            self.bodies.slice(),
            self.temp.getWriteManifoldCache(),
            self.temp.getWriteContactCache(),
            10
        );
    }

    fn applyGravity(self: *World, dt: f32) void {
        const motion = self.bodies.items(.motion);
        const physics_props = self.bodies.items(.physics_props);
        
        for (0..self.bodyCount()) |i| {
            // Todo: Separate static and dynamic object into different lists. This way we wont need physcs probs in here
            if (physics_props[i].inverseMass == 0) continue;
            const gravity_delta_velocity = self.gravity.mulScalar(dt);
            motion[i].velocity = motion[i].velocity.add(&gravity_delta_velocity);
        }
    }

    fn integratePositions(self: *World, dt: f32) void {
        const motion = self.bodies.items(.motion);
        const transform = self.bodies.items(.transform);
        const physics_props = self.bodies.items(.physics_props);
        
        for (0..self.bodyCount()) |i| {
            if (physics_props[i].inverseMass == 0) continue;
            
            const position_delta = motion[i].velocity.mulScalar(dt);
            transform[i].position = transform[i].position.add(&position_delta);

            const omega = motion[i].angularVelocity;
            const omega_len2 = omega.len2();
            if (omega_len2 > 1e-12) {
                const omega_len = std.math.sqrt(omega_len2);
                const axis = omega.mulScalar(1.0 / omega_len);
                const angle = omega_len * dt;
                const dq = math.Quat.fromAxisAngle(axis, angle);
                transform[i].orientation = math.Quat.mul(&dq, &transform[i].orientation).normalize();
            }
        }
    }
};

pub const WorldTemp = struct {
    allocator: std.mem.Allocator,
    penetrationConstraints: std.ArrayList(contact.PenetrationConstraint),
    manifold_cache: [2]std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.ContactManifold),
    contact_cache: [2]std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.Contact),
    current_buffer: BufferIndex,

    pub const BufferIndex = enum(usize) {
        index0 = 0,
        index1 = 1,

        pub fn next(self: BufferIndex) BufferIndex {
            return @enumFromInt(@intFromEnum(self) ^ 1);
        }
    };

    pub fn init(allocator: std.mem.Allocator) WorldTemp {
        return .{
            .allocator = allocator,
            .manifold_cache = .{ .{}, .{} },
            .contact_cache = .{ .{}, .{} },
            .current_buffer = .index0,
            .penetrationConstraints= .{},
        };
    }

    pub fn deinit(self: *WorldTemp) void {
        self.manifold_cache[0].deinit(self.allocator);
        self.manifold_cache[1].deinit(self.allocator);
        self.contact_cache[0].deinit(self.allocator);
        self.contact_cache[1].deinit(self.allocator);
        self.penetrationConstraints.deinit(self.allocator);
    }

    pub fn clear(self: *WorldTemp) void {
        self.penetrationConstraints.clearRetainingCapacity();
    }

    pub fn ensureCapacity(self: *WorldTemp, bodies_count: usize) !void {
        if (bodies_count <= 1) return;
        const max_pairs = bodies_count * (bodies_count - 1) / 2;
        try self.manifold_cache[0].ensureTotalCapacity(self.allocator, max_pairs);
        try self.manifold_cache[1].ensureTotalCapacity(self.allocator, max_pairs);
        try self.contact_cache[0].ensureTotalCapacity(self.allocator, max_pairs);
        try self.contact_cache[1].ensureTotalCapacity(self.allocator, max_pairs);
        try self.penetrationConstraints.ensureTotalCapacity(self.allocator, max_pairs * 4);
    }

    pub fn penConstraintsSlice(self: *WorldTemp) []const contact.PenetrationConstraint {
        return self.penetrationConstraints.items;
    }

    pub fn swapCacheBuffers(self: *WorldTemp) void {
        self.current_buffer = self.current_buffer.next();
        self.manifold_cache[@intFromEnum(self.current_buffer)].clearRetainingCapacity();
        self.contact_cache[@intFromEnum(self.current_buffer)].clearRetainingCapacity();
    }

    pub fn getReadManifoldCache(self: *WorldTemp) *const std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.ContactManifold) {
        return &self.manifold_cache[@intFromEnum(self.current_buffer.next())];
    }

    pub fn getWriteManifoldCache(self: *WorldTemp) *std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.ContactManifold) {
        return &self.manifold_cache[@intFromEnum(self.current_buffer)];
    }

    pub fn getReadContactCache(self: *WorldTemp) *const std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.Contact) {
        return &self.contact_cache[@intFromEnum(self.current_buffer.next())];
    }

    pub fn getWriteContactCache(self: *WorldTemp) *std.AutoArrayHashMapUnmanaged(contact.CacheMainfoldKey, contact.Contact) {
        return &self.contact_cache[@intFromEnum(self.current_buffer)];
    }
};
