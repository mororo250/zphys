const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const mod = b.addModule("lib_mod", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize
    });
    const exe_mod = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "tgbot_zig", .module = mod },
            },
        });


    const exe = b.addExecutable(.{
        .name = "exe_template",
        .root_module = exe_mod
    });

    b.installArtifact(exe);

    const run_step = b.step("run", "Run the app");

    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    const mod_tests = b.addTest(.{
        .name = "zig_tests",
        .root_module = mod,
        .test_runner = .{.path = b.path("src/test_runner.zig"), .mode = .simple },
    });

    b.installArtifact(mod_tests);
    const run_mod_tests = b.addRunArtifact(mod_tests);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);

    if (b.args) |args| {
        run_mod_tests.addArgs(args);
        run_cmd.addArgs(args);
    }

    const check = b.addExecutable(.{
        .name = "Check",
        .root_module = exe_mod,
    });

    // This is a test of making zls work in tests.
    const check_test = b.addTest(.{
        .root_module = mod,
    });

    const check_step = b.step("check", "Check for zls analysis");
    check_step.dependOn(&check.step);
    check_step.dependOn(&check_test.step);
}
