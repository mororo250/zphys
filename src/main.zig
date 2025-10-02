const std = @import("std");
const tgbot_zig = @import("tgbot_zig");

pub fn main() !void {
    // Prints to stderr, ignoring potential errors.
    std.debug.print("All your {s} are belong to us.\n", .{"codebase"});
    try tgbot_zig.bufferedPrint();
}