const std = @import("std");
const math = @import("math");
const Body = @import("body.zig").Body;

pub const Contact = struct {
    a: u32,
    b: u32,
    normal: math.Vec3,
    point: math.Vec3, // contact point (approx)
    penetration: f32,
    friction: f32,
    restitution: f32,
};

pub fn generateContacts(bodies: []const Body, out_items: []Contact, out_len: *usize) !void {

    // Todo: Add BroadPhase collision check in here
    var i: usize = 0;
    while (i < bodies.len) : (i += 1) {
        var j: usize = i + 1;
        while (j < bodies.len) : (j += 1) {
            const a = &bodies[i];
            const b = &bodies[j];

            // Skip static-static
            if (a.mass == 0 and b.mass == 0) continue;

            switch (a.shape) {
                .Sphere => |_| {
                    switch (b.shape) {
                        .Sphere => |_| try collideSphereSphere(@intCast(i), a, @intCast(j), b, out_items, out_len),
                        .Box => |_| try collideSphereBox(@intCast(i), a, @intCast(j), b, out_items, out_len),
                        else => {},
                    }
                },
                .Box => |_| {
                    switch (b.shape) {
                        .Sphere => |_| {
                            // sphere-box expects sphere as A, box as B; swap roles
                            const prev_len = out_len.*;
                            try collideSphereBox(@intCast(j), b, @intCast(i), a, out_items, out_len);
                            // If a contact was added, remap to keep ordering A=i (box), B=j (sphere)
                            if (out_len.* > prev_len) {
                                var c_ref = &out_items[out_len.* - 1];
                                // ensure normal points from A(i) to B(j)
                                c_ref.normal = c_ref.normal.negate();
                                // set indices to match (i -> j)
                                c_ref.a = @intCast(i);
                                c_ref.b = @intCast(j);
                            }
                        },
                        .Box => |_| try collideBoxBox(@intCast(i), a, @intCast(j), b, out_items, out_len),
                        else => {},
                    }
                },
                else => {},
            }
        }
    }
}

pub fn solveVelocity(bodies: []Body, contacts: []const Contact, iterations: u32, dt: f32) void {
    const baumgarte: f32 = 0.3;
    const slop: f32 = 0.003;

    var it: u32 = 0;
    while (it < iterations) : (it += 1) {
        for (contacts) |c| {
            const a = &bodies[c.a];
            const b = &bodies[c.b];

            const invMassA = a.inverseMass;
            const invMassB = b.inverseMass;
            const invMassSum = invMassA + invMassB;
            if (invMassSum == 0) continue;

            const n = c.normal.normalize(0);

            // Relative velocity at contact (linear only for now)
            const rv = b.velocity.sub(&a.velocity);
            const velAlongNormal = rv.dot(&n);

            // Penetration slop and early exit for separating contacts
            const pen = @max(c.penetration - slop, 0.0);
            if (velAlongNormal > 0 and pen <= 0) continue;

            // Restitution only on closing velocity
            const restitution = if (velAlongNormal < -0.5) c.restitution else 0.0;

            // Baumgarte positional bias as velocity term
            const biasVel = if (dt > 0) (baumgarte * pen / dt) else 0.0;

            // Normal impulse (include restitution + bias)
            var jn = (-(1.0 + restitution) * velAlongNormal - biasVel) / invMassSum;
            if (jn < 0) jn = 0;

            if (jn > 0) {
                const impulseN = n.mulScalar(jn);
                a.velocity = a.velocity.sub(&impulseN.mulScalar(invMassA));
                b.velocity = b.velocity.add(&impulseN.mulScalar(invMassB));
            }

            // Friction (Coulomb, clamped by mu * |jn|)
            const rv2 = b.velocity.sub(&a.velocity);
            var tangent = rv2.sub(&n.mulScalar(rv2.dot(&n)));
            const t_len2 = tangent.len2();
            if (t_len2 > 1e-12) {
                tangent = tangent.mulScalar(1.0 / std.math.sqrt(t_len2));
                const jt = -(rv2.dot(&tangent)) / invMassSum;
                const mu = c.friction;

                const maxFriction = mu * jn;
                var jt_clamped = jt;
                if (jt_clamped > maxFriction) jt_clamped = maxFriction;
                if (jt_clamped < -maxFriction) jt_clamped = -maxFriction;

                const impulseT = tangent.mulScalar(jt_clamped);
                a.velocity = a.velocity.sub(&impulseT.mulScalar(invMassA));
                b.velocity = b.velocity.add(&impulseT.mulScalar(invMassB));
            }
        }
    }
}

pub fn solvePosition(bodies: []Body, contacts: []const Contact) void {
    const percent: f32 = 0.2; // gentler correction to reduce jitter
    const slop: f32 = 0.005;

    for (contacts) |c| {
        const a = &bodies[c.a];
        const b = &bodies[c.b];

        const invMassA = a.inverseMass;
        const invMassB = b.inverseMass;
        const invMassSum = invMassA + invMassB;
        if (invMassSum == 0) continue;

        const correction_mag = percent * @max(c.penetration - slop, 0.0) / invMassSum;
        const correction = c.normal.normalize(0).mulScalar(correction_mag);

        a.position = a.position.sub(&correction.mulScalar(invMassA));
        b.position = b.position.add(&correction.mulScalar(invMassB));
    }
}


inline fn tryAppend(out_items: []Contact, out_len: *usize, c: Contact) !void {
    if (out_len.* >= out_items.len) return error.OutOfContacts;
    out_items[out_len.*] = c;
    out_len.* += 1;
}

inline fn signf(x: f32) f32 {
    return if (x >= 0) 1.0 else -1.0;
}

fn getBoxAxes(center: math.Vec3, q: math.Quat, halfExtents: math.Vec3) struct {
    c: math.Vec3,
    axes: [3]math.Vec3,
    he: math.Vec3,
} {
    const ax = math.vec3(1, 0, 0).mulQuat(&q);
    const ay = math.vec3(0, 1, 0).mulQuat(&q);
    const az = math.vec3(0, 0, 1).mulQuat(&q);
    return .{ .c = center, .axes = .{ ax, ay, az }, .he = halfExtents };
}

fn supportBox(center: math.Vec3, q: math.Quat, he: math.Vec3, dir: math.Vec3) math.Vec3 {
    // World axes of the box
    const ax = math.vec3(1, 0, 0).mulQuat(&q);
    const ay = math.vec3(0, 1, 0).mulQuat(&q);
    const az = math.vec3(0, 0, 1).mulQuat(&q);

    const sx = signf(ax.dot(&dir));
    const sy = signf(ay.dot(&dir));
    const sz = signf(az.dot(&dir));

    const corner = ax.mulScalar(he.x() * sx)
        .add(&ay.mulScalar(he.y() * sy))
        .add(&az.mulScalar(he.z() * sz));
    return center.add(&corner);
}

fn supportSphere(center: math.Vec3, radius: f32, dir: math.Vec3) math.Vec3 {
    var d = dir;
    const len2 = d.len2();
    if (len2 <= 1e-12) {
        d = math.vec3(1, 0, 0);
    } else {
        d = d.normalize(0);
    }
    return center.add(&d.mulScalar(radius));
}

fn closestPointOnOBB(point: math.Vec3, center: math.Vec3, q: math.Quat, he: math.Vec3) math.Vec3 {
    // Transform point to box local space
    const p_local = point.sub(&center).mulQuat(&q.inverse());
    const clamped = math.vec3(
        std.math.clamp(p_local.x(), -he.x(), he.x()),
        std.math.clamp(p_local.y(), -he.y(), he.y()),
        std.math.clamp(p_local.z(), -he.z(), he.z()),
    );
    // Back to world
    return center.add(&clamped.mulQuat(&q));
}

pub fn collideSphereSphere(a_id: u32, a: *const Body, b_id: u32, b: *const Body, out_items: []Contact, out_len: *usize) !void {
    const sa = a.shape.Sphere;
    const sb = b.shape.Sphere;

    const ab = b.position.sub(&a.position);
    const dist2 = ab.len2();
    const r = sa.radius + sb.radius;

    if (dist2 > r * r) return; // no contact

    const dist = std.math.sqrt(dist2);
    var normal: math.Vec3 = undefined;
    if (dist > 1e-6) {
        normal = ab.mulScalar(1.0 / dist);
    } else {
        normal = math.vec3(0, 1, 0);
    }

    const penetration = r - dist;
    // Approx contact point as point on surface of A along normal
    const point = a.position.add(&normal.mulScalar(sa.radius - penetration * 0.5));

    const friction = std.math.sqrt(@max(a.friction, 0) * @max(b.friction, 0));
    const restitution = @max(a.restitution, b.restitution);

    try tryAppend(out_items, out_len, .{
        .a = a_id,
        .b = b_id,
        .normal = normal,
        .point = point,
        .penetration = penetration,
        .friction = friction,
        .restitution = restitution,
    });
}

pub fn collideSphereBox(a_id: u32, a: *const Body, b_id: u32, b: *const Body, out_items: []Contact, out_len: *usize) !void {
    const sp = a.shape.Sphere;
    const bx = b.shape.Box;

    const closest = closestPointOnOBB(a.position, b.position, b.orientation, bx.halfExtends);
    const delta = closest.sub(&a.position);
    const d2 = delta.len2();

    if (d2 > sp.radius * sp.radius) return;

    var normal: math.Vec3 = undefined;
    const d = std.math.sqrt(d2);
    if (d > 1e-6) {
        normal = delta.mulScalar(1.0 / d); // from box->sphere
    } else {
        // Choose a reasonable normal (up)
        normal = math.vec3(0, 1, 0);
    }

    const penetration = sp.radius - d;
    const point = closest;

    const friction = std.math.sqrt(@max(a.friction, 0) * @max(b.friction, 0));
    const restitution = @max(a.restitution, b.restitution);

    try tryAppend(out_items, out_len, .{
        .a = a_id,
        .b = b_id,
        .normal = normal, // from A(sphere) to B(box)? Here delta was sphere - closest => normal points from box to sphere.
        .point = point,
        .penetration = penetration,
        .friction = friction,
        .restitution = restitution,
    });
}

// ------------- OBB vs OBB using GJK for detection, SAT for contact details -------------

fn gjkBoxesIntersect(a_c: math.Vec3, a_q: math.Quat, a_he: math.Vec3, b_c: math.Vec3, b_q: math.Quat, b_he: math.Vec3) bool {
    // Minkowski support function: S(dir) = supportA(dir) - supportB(-dir)
    const support = struct {
        fn s(dir: math.Vec3, ac: math.Vec3, aq: math.Quat, ahe: math.Vec3, bc: math.Vec3, bq: math.Quat, bhe: math.Vec3) math.Vec3 {
            const pA = supportBox(ac, aq, ahe, dir);
            const pB = supportBox(bc, bq, bhe, dir.negate());
            return pA.sub(&pB);
        }
    };

    var dir = b_c.sub(&a_c);
    if (dir.len2() < 1e-8) dir = math.vec3(1, 0, 0);

    var simplex: [4]math.Vec3 = undefined;
    var size: usize = 0;

    // Add first point
    simplex[0] = support.s(dir, a_c, a_q, a_he, b_c, b_q, b_he);
    size = 1;
    if (simplex[0].dot(&dir) <= 0) return false;
    dir = simplex[0].negate();

    var iter: usize = 0;
    while (iter < 30) : (iter += 1) {
        const A = support.s(dir, a_c, a_q, a_he, b_c, b_q, b_he);
        if (A.dot(&dir) <= 0) return false;

        simplex[size] = A;
        size += 1;

        // Handle simplex
        const res = handleSimplex(&simplex, &size, &dir);
        if (res) return true;
    }
    // Fallback
    return false;
}

fn handleSimplex(simplex: *[4]math.Vec3, size: *usize, dir: *math.Vec3) bool {
    // Based on GJK in 3D - cases line, triangle, tetrahedron
    switch (size.*) {
        2 => {
            // Line AB (A = last)
            const A = simplex.*[1];
            const B = simplex.*[0];
            const AO = A.negate();
            const AB = B.sub(&A);

            // New direction perpendicular to AB towards origin
            // But we already have cross on Vec3:
            const ABxAO = AB.cross(&AO);
            dir.* = ABxAO.cross(&AB);
            if (dir.len2() < 1e-12) {
                // pick any perpendicular
                dir.* = math.vec3(-AB.y(), AB.x(), 0);
            }
            return false;
        },
        3 => {
            // Triangle ABC (A = last)
            const A = simplex.*[2];
            const B = simplex.*[1];
            const C = simplex.*[0];

            const AO = A.negate();
            const AB = B.sub(&A);
            const AC = C.sub(&A);
            const ABC = AB.cross(&AC);

            // Determine which side of triangle the origin lies
            const ab_perp = ABC.cross(&AC);
            if (ab_perp.dot(&AO) > 0) {
                // Origin is outside AC edge
                simplex.*[0] = C;
                simplex.*[1] = A;
                size.* = 2;
                dir.* = AC.cross(&AO).cross(&AC);
                if (dir.len2() < 1e-12) dir.* = math.vec3(-AC.y(), AC.x(), 0);
                return false;
            }

            const ac_perp = AB.cross(&ABC);
            if (ac_perp.dot(&AO) > 0) {
                // Outside AB edge
                simplex.*[0] = B;
                simplex.*[1] = A;
                size.* = 2;
                dir.* = AB.cross(&AO).cross(&AB);
                if (dir.len2() < 1e-12) dir.* = math.vec3(-AB.y(), AB.x(), 0);
                return false;
            }

            // Otherwise, origin is above/below triangle
            if (ABC.dot(&AO) > 0) {
                dir.* = ABC;
            } else {
                // Wind triangle the other way
                simplex.*[0] = B;
                simplex.*[1] = C;
                simplex.*[2] = A;
                dir.* = ABC.negate();
            }
            return false;
        },
        4 => {
            // Tetrahedron ABCD (A = last)
            const A = simplex.*[3];
            const B = simplex.*[2];
            const C = simplex.*[1];
            const D = simplex.*[0];

            const AO = A.negate();
            const AB = B.sub(&A);
            const AC = C.sub(&A);
            const AD = D.sub(&A);

            const ABC = AB.cross(&AC);
            const ACD = AC.cross(&AD);
            const ADB = AD.cross(&AB);

            if (ABC.dot(&AO) > 0) {
                simplex.*[0] = C;
                simplex.*[1] = B;
                simplex.*[2] = A;
                size.* = 3;
                dir.* = ABC;
                return false;
            }
            if (ACD.dot(&AO) > 0) {
                simplex.*[0] = D;
                simplex.*[1] = C;
                simplex.*[2] = A;
                size.* = 3;
                dir.* = ACD;
                return false;
            }
            if (ADB.dot(&AO) > 0) {
                simplex.*[0] = B;
                simplex.*[1] = D;
                simplex.*[2] = A;
                size.* = 3;
                dir.* = ADB;
                return false;
            }
            // Origin is inside tetrahedron
            return true;
        },
        else => return false,
    }
}

fn satBoxBoxContact(a_c: math.Vec3, a_q: math.Quat, a_he: math.Vec3, b_c: math.Vec3, b_q: math.Quat, b_he: math.Vec3) ?struct {
    normal: math.Vec3,
    penetration: f32,
} {
    const A = getBoxAxes(a_c, a_q, a_he);
    const B = getBoxAxes(b_c, b_q, b_he);

    // Compute rotation matrix expressing B in A&#x27;s frame
    var R: [3][3]f32 = undefined;
    var AbsR: [3][3]f32 = undefined;
    inline for (0..3) |i| {
        inline for (0..3) |j| {
            R[i][j] = A.axes[i].dot(&B.axes[j]);
            AbsR[i][j] = @abs(R[i][j]) + 1e-6; // add epsilon to account for parallelism
        }
    }

    // Translation vector t from A to B in A frame
    const t_world = B.c.sub(&A.c);
    const t = math.vec3(t_world.dot(&A.axes[0]), t_world.dot(&A.axes[1]), t_world.dot(&A.axes[2]));
    const t_arr = [_]f32{ t.x(), t.y(), t.z() };

    var min_overlap: f32 = std.math.inf(f32);
    var best_axis_world: math.Vec3 = math.vec3(0, 1, 0);

    // Test axes L = A0, A1, A2
    {
        const ra0 = A.he.x();
        const rb0 = B.he.x() * AbsR[0][0] + B.he.y() * AbsR[0][1] + B.he.z() * AbsR[0][2];
        const d0 = @abs(t.x());
        if (d0 > ra0 + rb0) return null;
        const overlap0 = (ra0 + rb0) - d0;
        if (overlap0 < min_overlap) {
            min_overlap = overlap0;
            best_axis_world = if (t.x() >= 0) A.axes[0] else A.axes[0].negate();
        }

        const ra1 = A.he.y();
        const rb1 = B.he.x() * AbsR[1][0] + B.he.y() * AbsR[1][1] + B.he.z() * AbsR[1][2];
        const d1 = @abs(t.y());
        if (d1 > ra1 + rb1) return null;
        const overlap1 = (ra1 + rb1) - d1;
        if (overlap1 < min_overlap) {
            min_overlap = overlap1;
            best_axis_world = if (t.y() >= 0) A.axes[1] else A.axes[1].negate();
        }

        const ra2 = A.he.z();
        const rb2 = B.he.x() * AbsR[2][0] + B.he.y() * AbsR[2][1] + B.he.z() * AbsR[2][2];
        const d2 = @abs(t.z());
        if (d2 > ra2 + rb2) return null;
        const overlap2 = (ra2 + rb2) - d2;
        if (overlap2 < min_overlap) {
            min_overlap = overlap2;
            best_axis_world = if (t.z() >= 0) A.axes[2] else A.axes[2].negate();
        }
    }

    // Test axes L = B0, B1, B2
    {
        const ra0 = A.he.x() * AbsR[0][0] + A.he.y() * AbsR[1][0] + A.he.z() * AbsR[2][0];
        const rb0 = B.he.x();
        const d0 = @abs(t.x() * R[0][0] + t.y() * R[1][0] + t.z() * R[2][0]);
        if (d0 > ra0 + rb0) return null;
        const overlap0 = (ra0 + rb0) - d0;
        if (overlap0 < min_overlap) {
            min_overlap = overlap0;
            best_axis_world = if ((t_world.dot(&B.axes[0])) >= 0) B.axes[0] else B.axes[0].negate();
        }

        const ra1 = A.he.x() * AbsR[0][1] + A.he.y() * AbsR[1][1] + A.he.z() * AbsR[2][1];
        const rb1 = B.he.y();
        const d1 = @abs(t.x() * R[0][1] + t.y() * R[1][1] + t.z() * R[2][1]);
        if (d1 > ra1 + rb1) return null;
        const overlap1 = (ra1 + rb1) - d1;
        if (overlap1 < min_overlap) {
            min_overlap = overlap1;
            best_axis_world = if ((t_world.dot(&B.axes[1])) >= 0) B.axes[1] else B.axes[1].negate();
        }

        const ra2 = A.he.x() * AbsR[0][2] + A.he.y() * AbsR[1][2] + A.he.z() * AbsR[2][2];
        const rb2 = B.he.z();
        const d2 = @abs(t.x() * R[0][2] + t.y() * R[1][2] + t.z() * R[2][2]);
        if (d2 > ra2 + rb2) return null;
        const overlap2 = (ra2 + rb2) - d2;
        if (overlap2 < min_overlap) {
            min_overlap = overlap2;
            best_axis_world = if ((t_world.dot(&B.axes[2])) >= 0) B.axes[2] else B.axes[2].negate();
        }
    }

    // Test axes L = Ai x Bj
    inline for (0..3) |i| {
        inline for (0..3) |j| {
            const ra = switch (i) {
                0 => A.he.y() * AbsR[2][j] + A.he.z() * AbsR[1][j],
                1 => A.he.x() * AbsR[2][j] + A.he.z() * AbsR[0][j],
                2 => A.he.x() * AbsR[1][j] + A.he.y() * AbsR[0][j],
                else => 0,
            };
            const rb = switch (j) {
                0 => B.he.y() * AbsR[i][2] + B.he.z() * AbsR[i][1],
                1 => B.he.x() * AbsR[i][2] + B.he.z() * AbsR[i][0],
                2 => B.he.x() * AbsR[i][1] + B.he.y() * AbsR[i][0],
                else => 0,
            };

            const ip1 = (i + 1) % 3;
            const ip2 = (i + 2) % 3;

            const t_term = @abs(t_arr[ip2] * R[ip1][j] - t_arr[ip1] * R[ip2][j]);
            if (t_term > ra + rb) return null;

            const overlap = (ra + rb) - t_term;
            if (overlap < min_overlap and overlap >= 0) {
                min_overlap = overlap;
                var axis = A.axes[i].cross(&B.axes[j]);
                // If axes are near parallel, skip
                if (axis.len2() > 1e-8) {
                    axis = axis.normalize(0);
                    // Ensure axis points from A to B
                    if (axis.dot(&t_world) < 0) axis = axis.negate();
                    best_axis_world = axis;
                }
            }
        }
    }

    return .{ .normal = best_axis_world.normalize(0), .penetration = min_overlap };
}

pub fn collideBoxBox(a_id: u32, a: *const Body, b_id: u32, b: *const Body, out_items: []Contact, out_len: *usize) !void {
    const bxA = a.shape.Box;
    const bxB = b.shape.Box;

    // GJK for detection
    const intersects = gjkBoxesIntersect(a.position, a.orientation, bxA.halfExtends, b.position, b.orientation, bxB.halfExtends);
    if (!intersects) return;

    // SAT to get contact normal + penetration depth
    const sat_res = satBoxBoxContact(a.position, a.orientation, bxA.halfExtends, b.position, b.orientation, bxB.halfExtends) orelse return;

    const friction = std.math.sqrt(@max(a.friction, 0) * @max(b.friction, 0));
    const restitution = @max(a.restitution, b.restitution);

    const point = a.position.add(&b.position).mulScalar(0.5);
    try tryAppend(out_items, out_len, .{
        .a = a_id,
        .b = b_id,
        .normal = sat_res.normal,
        .point = point,
        .penetration = sat_res.penetration,
        .friction = friction,
        .restitution = restitution,
    });
}

