const z = @import("std");

const FlagRegister = @import("flag_register.zig").FlagRegister;
const Register8 = @import("register8.zig").Register8;
const Register16 = @import("register16.zig").Register16;

// Number of bytes needed to store all registers in a continuous array.
//
//     (number of 8-bit registers) + (number of 16-bit registers) * 2
pub const TOTAL_REGISTER_SIZE: usize = @typeInfo(Register8).Enum.fields.len *
    @sizeOf(u8) + @typeInfo(Register16).Enum.fields.len * @sizeOf(u16);

// A struct to conveniently access individual registers. You should be able to
// cast the register array to this.
pub const Registers = packed struct {
    const Self = @This();

    a: u8,
    f: FlagRegister,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,

    sp: u16,
    pc: u16,

    pub fn format(
        value: Self,
        comptime fmt: []const u8,
        _: z.fmt.FormatOptions,
        out_stream: anytype,
    ) !void {
        inline for (@typeInfo(Self).Struct.fields) |f| {
            try out_stream.writeAll(fmt);
            try out_stream.writeAll(".");
            try out_stream.writeAll(f.name);
            try out_stream.writeAll(" = ");

            if (f.field_type == u8) {
                try z.fmt.format(out_stream, "{b:0>8}", .{
                    @field(value, f.name),
                });
            } else if (f.field_type == u16) {
                try z.fmt.format(out_stream, "{b:0>16}", .{
                    @field(value, f.name),
                });
            } else {
                try z.fmt.format(out_stream, "{}", .{
                    @field(value, f.name),
                });
            }

            try out_stream.writeAll(",\n");
        }
    }
};

comptime {
    // Just making sure I didn't mess up in either of these places.
    if (@bitSizeOf(Registers) / 8 != TOTAL_REGISTER_SIZE) {
        @compileLog("@sizeOf(Registers) = ", @sizeOf(Registers));
        @compileLog("TOTAL_REGISTER_SIZE = ", TOTAL_REGISTER_SIZE);
        @compileLog("@sizeOf(FlagRegister) = ", @sizeOf(FlagRegister));

        @compileError("Size of Registers does not match the total " ++
            "number of bytes used to store all registers");
    }

    // Make sure the order of the registers is correct
    const regs_members = @typeInfo(Registers).Struct.fields;

    // Check the 8-bit registers
    inline for (@typeInfo(Register8).Enum.fields) |f, i| {
        const reg8_f = f.name;
        const regs_m = regs_members[i].name;

        if (!z.mem.eql(u8, reg8_f, regs_m)) {
            @compileError("Out of order registers: Reg8." ++
                reg8_f ++
                ", Regs." ++
                regs_m);
        }
    }

    // Now the 16-bit ones
    inline for (@typeInfo(Register16).Enum.fields) |f, idx| {
        // Pretend we're still using the same counter as the 8-bit registers
        const i = idx + @typeInfo(Register8).Enum.fields.len;

        const reg16_f = f.name;
        const regs_m = regs_members[i].name;

        if (!z.mem.eql(u8, reg16_f, regs_m)) {
            @compileError("Out of order registers: Reg8." ++
                reg16_f ++
                ", Regs." ++
                regs_m);
        }
    }
}
