const std = @import("std");
const math = @import("math");
const zphys = @import("zphys");
const rl = @import("raylib");
const App = @import("shared/app.zig").App;

pub fn main() !void {
    var app = try App.init("zphys - Pyramid Stack");
    defer app.deinit();

    // Custom camera setup
    app.camera.position = .init(10, 10, 20);
    app.camera.target = .init(0, 4, 0);
    app.sub_steps = 4;

    try createPyramid(&app.world);

    while (!app.shouldClose()) {
        try app.update();

        app.beginDraw();
        app.drawScene();

        if (app.show_ui) {
            rl.drawText("Pyramid Stack Test", 20, 20, 20, .black);
        }

        app.endDraw();
    }
}

fn createPyramid(world: *zphys.World) !void {
    // Ground
    var ground = zphys.BodyDef.default();
    ground.shape = zphys.shape.newBox(math.vec3(50, 1.0, 50));
    ground.position = math.vec3(0, -1.0, 0);
    ground.inverseMass = 0.0;
    ground.friction = 1.0;
    _ = try world.createBody(ground);

    // Pyramid
    const stack_height = 30;
    const box_size = 1.0;
    const gap = 0.05;

    var i: usize = 0;
    while (i < stack_height) : (i += 1) {
        const row_count = stack_height - i;
        const level_y = 0.5 + @as(f32, @floatFromInt(i)) * (box_size + gap);

        // Center the row along X
        const row_width = @as(f32, @floatFromInt(row_count)) * (box_size + gap) - gap;
        const start_x = -row_width * 0.5 + box_size * 0.5;

        var j: usize = 0;
        while (j < row_count) : (j += 1) {
            const x = start_x + @as(f32, @floatFromInt(j)) * (box_size + gap);
            
            var box = zphys.BodyDef.default();
            box.shape = zphys.shape.newBox(math.vec3(box_size * 0.5, box_size * 0.5, box_size * 0.5));
            box.position = math.vec3(x, level_y, 0);
            box.inverseMass = 1.0; // Dynamic
            box.friction = 1.0;
            box.restitution = 0.0;
            _ = try world.createBody(box);
        }
    }
}
