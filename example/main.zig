const std = @import("std");
const math = @import("math");
const zphys = @import("zphys");
const rl = @import("raylib");

pub fn main() !void {
    const a: math.Vec3 = math.vec3(1, 2, 3);
    const b: math.Vec3 = math.vec3(4, 5, 6);
    const c = a.add(&b);
    std.debug.print("math: a = {any}, b = {any}\n", .{ a, b });
    std.debug.print("math: a + b = {any}\n", .{ c });
    std.debug.print("math: dot(a, b) = {d}\n", .{ a.dot(&b) });

    const v: math.Vec2 = math.vec2(10, 20);
    std.debug.print("zphys: v = {any}, len = {d}\n", .{ v, v.len() });

    std.debug.print("Example complete.\n", .{});

    const screenWidth = 800;
    const screenHeight = 450;

    rl.initWindow(screenWidth, screenHeight, "raylib [core] example - 3d camera free");
    defer rl.closeWindow();

    var camera = rl.Camera{
        .position = .init(10, 10, 10),
        .target = .init(0, 0, 0),
        .up = .init(0, 1, 0),
        .fovy = 45,
        .projection = .perspective,
    };

    var world = zphys.World.init(std.heap.page_allocator);
    defer world.deinit();

    var ground = zphys.BodyDef.default();
    ground.shape = zphys.shape.newBox(math.vec3(5, 0.5, 5));
    ground.position = math.vec3(0, -0.5, 0);
    ground.mass = 0.0;
    _ = try world.createBody(ground);

    var i: i32 = 0;
    while (i < 3) : (i += 1) {
        var d = zphys.BodyDef.default();
        d.shape = zphys.shape.newSphere(0.5);
        d.position = math.vec3(0, 1 + @as(f32, @floatFromInt(i)) * 1.1, 0);
        d.mass = 1.0;
        _ = try world.createBody(d);
    }

    rl.disableCursor();
    rl.setTargetFPS(60);

    while (!rl.windowShouldClose()) {
        camera.update(.free);

        if (rl.isKeyPressed(.z)) {
            camera.target = .init(0, 0, 0);
        }
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(.ray_white);

        {
            camera.begin();
            defer camera.end();

            for (world.bodies.items) |body| {
                switch (body.shape) {
                    .Box => |bx| {
                        const s = bx.halfExtends;
                        const pos = rl.Vector3.init(body.position.x(), body.position.y(), body.position.z());
                        rl.drawCube(pos, s.x() * 2, s.y() * 2, s.z() * 2, .red);
                        rl.drawCubeWires(pos, s.x() * 2, s.y() * 2, s.z() * 2, .maroon);
                    },
                    .Sphere => |sp| {
                        const pos = rl.Vector3.init(body.position.x(), body.position.y(), body.position.z());
                        rl.drawSphere(pos, sp.radius, .blue);
                    },
                    .Line => |ln| {
                        const p1 = rl.Vector3.init(
                            body.position.x() + ln.p1.x(),
                            body.position.y() + ln.p1.y(),
                            body.position.z() + ln.p1.z(),
                        );
                        const p2 = rl.Vector3.init(
                            body.position.x() + ln.p2.x(),
                            body.position.y() + ln.p2.y(),
                            body.position.z() + ln.p2.z(),
                        );
                        rl.drawLine3D(p1, p2, .black);
                    },
                }
            }

            rl.drawGrid(10, 1);
        }

        rl.drawRectangle(10, 10, 320, 93, .fade(.sky_blue, 0.5));
        rl.drawRectangleLines(10, 10, 320, 93, .blue);

        rl.drawText("Free camera default controls:", 20, 20, 10, .black);
        rl.drawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, .dark_gray);
        rl.drawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, .dark_gray);
        rl.drawText("- Z to zoom to (0, 0, 0)", 40, 80, 10, .dark_gray);
    }
}
