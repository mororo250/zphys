const std = @import("std");
const zphys = @import("zphys");
const rl = @import("raylib");
const DebugRenderer = @import("debug_renderer.zig").DebugRenderer;
const SceneRenderer = @import("scene_renderer.zig").SceneRenderer;

pub const App = struct {
    world: zphys.World,
    camera: rl.Camera,
    scene_renderer: SceneRenderer,
    
    paused: bool = false,
    step_one: bool = false,
    sub_steps: u16 = 1,
    
    show_ui: bool = true,

    pub fn init(title: [:0]const u8) !App {
        const screenWidth = 1280;
        const screenHeight = 720;

        rl.initWindow(screenWidth, screenHeight, title);
        rl.disableCursor();
        rl.setTargetFPS(60);

        const camera = rl.Camera{
            .position = .init(10, 10, 10),
            .target = .init(0, 0, 0),
            .up = .init(0, 1, 0),
            .fovy = 45,
            .projection = .perspective,
        };

        const world = zphys.World.init(std.heap.page_allocator);
        const scene_renderer = try SceneRenderer.init();

        return App{
            .world = world,
            .camera = camera,
            .scene_renderer = scene_renderer,
        };
    }

    pub fn deinit(self: *App) void {
        self.scene_renderer.deinit();
        self.world.deinit();
        rl.closeWindow();
    }

    pub fn shouldClose(self: *App) bool {
        _ = self;
        return rl.windowShouldClose();
    }

    pub fn update(self: *App) !void {
        self.camera.update(.free);

        if (rl.isKeyPressed(.f11)) {
            rl.toggleFullscreen();
        }

        if (rl.isKeyPressed(.h)) {
            self.show_ui = !self.show_ui;
        }

        if (rl.isKeyPressed(.space)) {
            self.paused = !self.paused;
        }
        
        if (rl.isKeyPressed(.right) and self.paused) {
            self.step_one = true;
        }

        if (!self.paused or self.step_one) {
            try self.world.step(1.0/60.0, self.sub_steps);
            self.step_one = false;
        }

        if (rl.isKeyPressed(.z)) {
            self.camera.target = .init(0, 0, 0);
        }
    }

    pub fn beginDraw(self: *App) void {
        rl.beginDrawing();
        SceneRenderer.drawSky();
        self.camera.begin();
    }

    pub fn endDraw(self: *App) void {
        self.camera.end();
        if (self.show_ui) {
            DebugRenderer.drawDebugInfo(self.paused);
        }
        rl.endDrawing();
    }

    pub fn drawScene(self: *App) void {
        self.scene_renderer.drawWorld(&self.world);
        //if (self.show_ui) {
        //    DebugRenderer.drawContacts(self.world.temp.getReadContactCache().values());
        //    DebugRenderer.drawManifolds(self.world.temp.getReadManifoldCache().values());
        //}
    }
};
