const std = @import("std");
const math = @import("math");
const Body = @import("../body.zig").Body;
const contact = @import("contact.zig");
const gjk = @import("gjk.zig");
const sat = @import("sat.zig");

pub fn collideBoxBox(a_id: u32, body_a: *const Body, b_id: u32, body_b: *const Body, out: *std.ArrayList(contact.Contact)) void {
    const box_a = body_a.shape.Box;
    const box_b = body_b.shape.Box;

    // GJK for detection
    const shape_a = gjk.GjkBox{ .center = body_a.position, .orientation = body_a.orientation, .half_extents = box_a.half_extents };
    const shape_b = gjk.GjkBox{ .center = body_b.position, .orientation = body_b.orientation, .half_extents = box_b.half_extents };
    const intersects = gjk.gjkBoxesIntersect(shape_a, shape_b);
    if (!intersects) return;

    // SAT to get contact normal + penetration depth
    // Todo: Use EBA to get better contact point + penetration depth
    const sat_result = sat.satBoxBoxContact(body_a.position, body_a.orientation, box_a.half_extents, body_b.position, body_b.orientation, box_b.half_extents) orelse return;

    const friction = std.math.sqrt(@max(body_a.friction, 0) * @max(body_b.friction, 0));
    const restitution = @max(body_a.restitution, body_b.restitution);

    const contact_point = body_a.position.add(&body_b.position).mulScalar(0.5);
    out.appendAssumeCapacity( .{
        .body_a = a_id,
        .body_b = b_id,
        .normal = sat_result.normal,
        .point = contact_point,
        .penetration = sat_result.penetration,
        .friction = friction,
        .restitution = restitution,
    });
}
