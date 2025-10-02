const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const math_mod = b.addModule("math", .{
        .root_source_file = b.path("src/math/math.zig"),
        .target = target,
        .optimize = optimize,
    });

    const physics_mod = b.addModule("zphys", .{
        .root_source_file = b.path("src/zphys/zphys.zig"),
        .target = target,
        .optimize = optimize,
    });
    // allow the zphys module to import the math module via @import("math")
    physics_mod.addImport("math", math_mod);

    const exe_mod = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "math", .module = math_mod },
                .{ .name = "zphys", .module = physics_mod },
            },
        });


    const exe = b.addExecutable(.{
        .name = "zphys",
        .root_module = exe_mod
    });

    b.installArtifact(exe);

    const run_step = b.step("run", "Run the app");

    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    const test_step = b.step("test", "Run tests");

    // tests for the math library
    const math_mod_tests = b.addTest(.{
        .name = "zig_tests",
        .root_module = math_mod,
        .test_runner = .{.path = b.path("src/test_runner.zig"), .mode = .simple },
    });
    const run_math_tests = b.addRunArtifact(math_mod_tests);
    test_step.dependOn(&run_math_tests.step);

    // tests for the physics library
    const physics_mod_tests = b.addTest(.{
        .name = "zig_tests",
        .root_module = physics_mod,
        .test_runner = .{.path = b.path("src/test_runner.zig"), .mode = .simple },
    });
    const run_physics_tests = b.addRunArtifact(physics_mod_tests);
    test_step.dependOn(&run_physics_tests.step);

    if (b.args) |args| {
        run_math_tests.addArgs(args);
        run_physics_tests.addArgs(args);
        run_cmd.addArgs(args);
    }
    const check = b.addExecutable(.{
        .name = "Check",
        .root_module = exe_mod,
    });

    // This is a test of making zls work in tests.
    const check_test = b.addTest(.{
        .root_module = math_mod,
    });

    const check_step = b.step("check", "Check for zls analysis");
    check_step.dependOn(&check.step);
    check_step.dependOn(&check_test.step);
}
