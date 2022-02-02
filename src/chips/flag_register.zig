const z = @import("std");

// A struct to conveniently access the F register, which is special because it
// stores flags for varios math operations:
//
//     ZNHC 0000
//     |||| |--|
//     ||||   |_ Unused, should always be zero
//     ||||
//     ||||_____ Carry: set if a carry occurred in the last math operation of
//     |||       if the register A is the smallest value of a CP instruction
//     |||
//     |||______ Half Carry: set if a carry occurred from the lower nibble in
//     ||        the last math operation
//     ||
//     ||_______ Subtract: set if a subtraction was performed in the last math
//     |         instruction
//     |
//     |________ Zero: set if the result of a math operation is zero or if two
//               values match in a CP instruction
pub const FlagRegister = packed struct {
    nibble: u4,

    carry: bool,
    half_carry: bool,
    subtract: bool,
    zero: bool,

    pub fn format(
        self: FlagRegister,
        comptime _: []const u8,
        _: z.fmt.FormatOptions,
        out_stream: anytype,
    ) !void {
        try out_stream.writeAll(if (self.zero) "Z" else "z");
        try out_stream.writeAll(if (self.subtract) "N" else "n");
        try out_stream.writeAll(if (self.half_carry) "H" else "h");
        try out_stream.writeAll(if (self.carry) "C" else "c");
    }
};

comptime {
    const expectEq = z.testing.expectEqual;

    expectEq(@sizeOf(FlagRegister), @sizeOf(u8)) catch |err| {
        @compileError(err);
    };
}
