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

    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });
    const raylib = raylib_dep.module("raylib");
    const raylib_artifact = raylib_dep.artifact("raylib");

    // create a module that will be used to build the example executable
    const example_mod = b.createModule(.{
        .root_source_file = b.path("example/main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "math", .module = math_mod },
            .{ .name = "zphys", .module = physics_mod },
            .{ .name = "raylib", .module = raylib },
        },
    });

    const example_exe = b.addExecutable(.{
        .name = "zphys_example",
        .root_module = example_mod,
    });
    example_exe.linkLibrary(raylib_artifact);
    b.installArtifact(example_exe);

    const example_step = b.step("example", "Run the example");

    const example_cmd = b.addRunArtifact(example_exe);
    example_step.dependOn(&example_cmd.step);

    example_cmd.step.dependOn(b.getInstallStep());

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
        example_cmd.addArgs(args);
    }

    const run_examples_step = b.step("run", "Run examples");
    run_examples_step.dependOn(&example_cmd.step);
    run_examples_step.dependOn(b.getInstallStep());

    const check = b.addExecutable(.{
        .name = "Check",
        .root_module = example_mod,
    });

    // This is a test of making zls work in tests.
    const check_test = b.addTest(.{
        .root_module = math_mod,
    });

    const check_step = b.step("check", "Check for zls analysis");
    check_step.dependOn(&check.step);
    check_step.dependOn(&check_test.step);
}
