const std = @import("std");
const builtin = @import("builtin");

pub fn main() !void {
    var mem: [4096]u8 = undefined;
    var fpa = std.heap.FixedBufferAllocator.init(&mem);
    const allocator = fpa.allocator();
    var it = try std.process.argsWithAllocator(allocator);
    defer it.deinit();

    var test_filter_opt: ?[]const u8 = null;
    while (it.next()) |arg| {
        if (std.mem.eql(u8, arg, "--test-filter")) {
            if (it.next()) |value| {
                test_filter_opt = try allocator.dupe(u8, value);
            } else {
                return error.MissingValueForTestFilter;
            }
        } else if (std.mem.startsWith(u8, arg, "--test-filter=")) {
            test_filter_opt = arg["--test-filter=".len..];
        } else {
            // handle other args
        }
    }

    for (builtin.test_functions) |t| {
        if (test_filter_opt) |test_filter| {
            if (std.mem.indexOf(u8, t.name, test_filter) == null)
                continue;
        }

        std.testing.allocator_instance = .{};
        const name = extractName(t);
        const start = std.time.milliTimestamp();
        const result = t.func();
        const elapsed = std.time.milliTimestamp() - start;
        if (std.testing.allocator_instance.deinit() == .leak) {
            std.debug.print( "{s} leaked memory\n", .{name});
        }
        if (result) |_| {
            std.debug.print("{s} passed - ({d}ms)\n", .{name, elapsed});
        } else |err| {
            std.debug.print("{s} failed - {}\n", .{t.name, err});
        }
    }
}

fn extractName(t: std.builtin.TestFn) []const u8 {
    const marker = std.mem.lastIndexOf(u8, t.name, ".test.") orelse return t.name;
    return t.name[marker+6..];
}
