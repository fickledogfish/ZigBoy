// https://github.com/djhworld/gomeboycolor/blob/master/cpu/instructions.go
// https://www.pastraiser.com/cpu/gameboy/gameboy_opcodes.html
// http://imrannazar.com/Gameboy-Z80-Opcode-Map

const z = @import("std");

const testing = z.testing;

const String = @import("buffer-string.zig").BufferString;
const Class = @import("class.zig").Class;

const Register8 = @import("../chips.zig").Register8;
const Register16 = @import("../chips.zig").Register16;

const Target = @import("target.zig").Target;
const ImmediateVal = @import("immediate.zig").ImmediateVal;
const JumpCondition = @import("jump_condition.zig").JumpCondition;

const INSTRUCTIONS = @import("table.zig").INSTRUCTIONS;
const INSTRUCTIONS_EXTENDED = @import("table_extended.zig").INSTRUCTIONS_EXTENDED;

pub const Instruction = struct {
    const Self = @This();

    opcode: usize,
    class: Class,
    cycles: usize,

    dst: ?Target = null,
    src: ?Target = null,

    // Is the value stored in (dst,src)(1,2) a pointer?
    dst_ptr: bool = false,
    src_ptr: bool = false,

    // If one of the targets is an immediate value, the
    immediate_val: ?ImmediateVal = null,

    cond: ?JumpCondition = null,

    fn fill_immediate(
        self: *Self,
        comptime field_name: []const u8,
        memory: []const u8,
        offset: usize,
    ) void {
        const field: ?Target = @field(self, field_name);
        if (field == null) return;

        switch (field.?) {
            .im => |im| switch (im) {
                .n => self.immediate_val = ImmediateVal{
                    .n = memory[offset + 1],
                },

                .nn => {
                    self.immediate_val = ImmediateVal{
                        .nn = @intCast(u16, memory[offset + 2]) << 8 |
                            memory[offset + 1],
                    };
                },
            },

            else => {},
        }
    }

    pub fn parse(memory: []const u8, offset: usize, extended: bool) Self {
        var parsed_instruction: Self = if (extended)
            INSTRUCTIONS_EXTENDED[memory[offset]]
        else
            INSTRUCTIONS[memory[offset]];
        parsed_instruction.fill_immediate("dst", memory, offset);
        parsed_instruction.fill_immediate("src", memory, offset);

        return parsed_instruction;
    }

    pub fn stringfy(
        self: Self,
        buffer: []u8,
    ) String {
        var str = String.init(buffer);

        for (@tagName(self.class)) |ch| {
            str.append(z.ascii.toUpper(ch));
        }

        if (self.cond == null and self.dst == null) {
            return str;
        }

        str.append(' ');

        if (self.cond != null) {
            for (@tagName(self.cond.?)) |ch| {
                str.append(z.ascii.toUpper(ch));
            }
        } else if (self.dst) |dst| {
            // TODO: better size
            var buf: [100]u8 = undefined;

            var dst_str = dst.stringfy(
                &buf,
                self.dst_ptr,
                self.immediate_val,
            );

            str.appendSlice(dst_str.str());
        } else {
            return str;
        }

        if (self.src) |src| {
            str.append(',');

            // TODO: better size
            var buf: [100]u8 = undefined;

            var src_str = src.stringfy(
                &buf,
                self.src_ptr,
                self.immediate_val,
            );

            str.appendSlice(src_str.str());
        } else {
            return str;
        }

        return str;
    }

    pub fn format(
        self: Self,
        comptime _: []const u8,
        _: z.fmt.FormatOptions,
        out_stream: anytype,
    ) !void {
        // TODO: what number to make it?
        var buffer: [100]u8 = undefined;
        var str = self.stringfy(&buffer);

        try out_stream.writeAll("0x");
        try z.fmt.formatIntValue(
            self.opcode,
            "X",
            z.fmt.FormatOptions{},
            out_stream,
        );
        try out_stream.writeAll(" ");
        try out_stream.writeAll(str.str());
    }
};

fn testInstruction(instr: Instruction, expected: []const u8) void {
    var instr_str = instr.stringfy(testing.allocator) catch unreachable;
    defer instr_str.deinit();

    // z.debug.warn("\n{} == {}\n", .{ instr_str, expected });
    testing.expectEqualSlices(u8, expected, instr_str.items);
}

test "Instruction stringfication" {
    testInstruction(INSTRUCTIONS[0x00], "NOP");
    testInstruction(INSTRUCTIONS[0xD3], "REMOVED");
    testInstruction(INSTRUCTIONS[0x76], "HALT");
    testInstruction(INSTRUCTIONS[0xF3], "DI");
    testInstruction(INSTRUCTIONS[0xFB], "EI");

    testInstruction(INSTRUCTIONS[0x39], "ADD HL,SP");
    testInstruction(INSTRUCTIONS[0x80], "ADD A,B");
    testInstruction(INSTRUCTIONS[0x86], "ADD A,(HL)");

    testInstruction(INSTRUCTIONS[0x18], "JR n");
    testInstruction(INSTRUCTIONS[0x28], "JR Z,n");
    testInstruction(INSTRUCTIONS[0xE9], "JP (HL)");
    testInstruction(INSTRUCTIONS[0xDA], "JP C,nn");
    testInstruction(INSTRUCTIONS[0xC2], "JP NZ,nn");
    testInstruction(INSTRUCTIONS[0xC3], "JP nn");

    testInstruction(INSTRUCTIONS[0xC7], "RST 0");
    testInstruction(INSTRUCTIONS[0xCF], "RST 8");
    testInstruction(INSTRUCTIONS[0xFF], "RST 38");

    testInstruction(INSTRUCTIONS_EXTENDED[0x34], "SWAP H");
    testInstruction(INSTRUCTIONS_EXTENDED[0x60], "BIT 4,B");
    testInstruction(INSTRUCTIONS_EXTENDED[0x9A], "RES 3,D");
    testInstruction(INSTRUCTIONS_EXTENDED[0xCF], "SET 1,A");
}
