// https://github.com/djhworld/gomeboycolor/blob/master/cpu/instructions.go
// https://www.pastraiser.com/cpu/gameboy/gameboy_opcodes.html
// http://imrannazar.com/Gameboy-Z80-Opcode-Map

const z = @import("std");

const String = @import("buffer-string.zig").BufferString;

const testing = z.testing;
const expectEq = testing.expectEqual;
const expectEqS = testing.expectEqualSlices;
const testAlloc = testing.allocator;

const Register8 = @import("sharp_lr35902.zig").Register8;
const Register16 = @import("sharp_lr35902.zig").Register16;

pub const InstructionClass = enum {
    @"and",
    @"or",
    adc,
    add,
    call,
    ccf,
    cp,
    cpl,
    daa,
    dec,
    di,
    ei,
    halt,
    inc,
    jp,
    jr,
    ld,
    ldd,
    ldh,
    ldhl,
    ldi,
    nop,
    pop,
    push,
    ret,
    reti,
    rst,
    sbc,
    scf,
    stop,
    sub,
    xor,

    bit,
    res,
    rl,
    rlc,
    rr,
    rrc,
    set,
    sla,
    sra,
    srl,
    swap,

    extended,
    removed,
};

pub const Target = union(enum) {
    const Self = @This();

    r8: Register8,
    r88: [2]Register8,
    r16: Register16,
    im: Immediate,
    addr: usize,
    bit: u8, // 0-7

    pub fn stringfy(
        self: Self,
        buffer: []u8,
        is_ptr: bool,
        immediate_val: ?ImmediateVal,
    ) String {
        const toUpper = z.ascii.toUpper;

        var str = String.init(buffer);

        if (is_ptr) {
            str.append('(');
        }

        switch (self) {
            .r8 => |val| {
                str.append(toUpper(@tagName(val)[0]));
            },

            .r88 => |val| {
                str.append(toUpper(@tagName(val[0])[0]));
                str.append(toUpper(@tagName(val[1])[0]));
            },

            .r16 => |val| {
                for (@tagName(val)) |ch| {
                    str.append(toUpper(ch));
                }
            },

            .im => {
                z.debug.assert(immediate_val != null);

                var buf: [@typeInfo(usize).Int.bits]u8 = undefined;
                var end: usize = undefined;

                switch (immediate_val.?) {
                    .n => |n| end = z.fmt.formatIntBuf(
                        &buf,
                        n,
                        16,
                        .upper,
                        z.fmt.FormatOptions{},
                    ),

                    .nn => |n| end = z.fmt.formatIntBuf(
                        &buf,
                        n,
                        16,
                        .upper,
                        z.fmt.FormatOptions{},
                    ),
                }

                str.appendSlice(buf[0..end]);
            },

            .addr => |val| {
                var numbuf: [@typeInfo(usize).Int.bits]u8 = undefined;

                const end = z.fmt.formatIntBuf(
                    &numbuf,
                    val,
                    16,
                    .upper,
                    z.fmt.FormatOptions{},
                );

                str.appendSlice(numbuf[0..end]);
            },

            .bit => |val| {
                str.append(val + 48);
            },
        }

        if (is_ptr) {
            str.append(')');
        }

        return str;
    }
};

// Some instructions depend on immediate n-bit data, this encodes how much
// should be loaded
const Immediate = enum {
    const Self = @This();

    n, // 8-bit data
    nn, // 16-bit data

    pub fn toSize(self: Self) usize {
        return switch (self) {
            .n => 8,
            .nn => 16,
        };
    }
};

const ImmediateVal = union(enum) {
    n: u8,
    nn: u16,
};

// Jump conditions
const Condition = enum {
    c, // carry
    z, // zero
    nc, // no carry
    nz, // not zero
};

pub const Instruction = struct {
    const Self = @This();

    opcode: usize,
    class: InstructionClass,
    cycles: usize,

    dst: ?Target = null,
    src: ?Target = null,

    // Is the value stored in (dst,src)(1,2) a pointer?
    dst_ptr: bool = false,
    src_ptr: bool = false,

    // If one of the targets is an immediate value, the
    immediate_val: ?ImmediateVal = null,

    cond: ?Condition = null,

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
    var instr_str = instr.stringfy(testAlloc) catch unreachable;
    defer instr_str.deinit();

    // z.debug.warn("\n{} == {}\n", .{ instr_str, expected });

    expectEqS(u8, expected, instr_str.items);
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

fn checkOpcodes(table: []const Instruction) void {
    var i: usize = 0x00;
    while (i <= 0xFF) : (i += 1) {
        // z.debug.warn("0x{x:0>2}: 0x{x:0>2} {}\n", .{
        // i,
        // INSTRUCTIONS[i].opcode,
        // INSTRUCTIONS[i],
        // });
        expectEq(i, table[i].opcode);
    }
}

test "Instruction table" {
    checkOpcodes(INSTRUCTIONS[0..]);
    checkOpcodes(INSTRUCTIONS_EXTENDED[0..]);
}

pub const INSTRUCTIONS = [_]Instruction{
    .{ // NOP
        .opcode = 0x00,
        .cycles = 1,
        .class = .nop,
    },
    .{ // LD BC,nn
        .opcode = 0x01,
        .cycles = 3,
        .class = .ld,
        .dst = .{ .r88 = .{ .b, .c } },
        .src = .{ .im = .nn },
    },
    .{ // LD (BC),A
        .opcode = 0x02,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .b, .c } },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // INC BC
        .opcode = 0x03,
        .cycles = 2,
        .class = .inc,
        .dst = .{ .r88 = .{ .b, .c } },
    },
    .{ // INC B
        .opcode = 0x04,
        .cycles = 1,
        .class = .inc,
        .dst = .{ .r8 = .b },
    },
    .{ // DEC B
        .opcode = 0x05,
        .cycles = 1,
        .class = .dec,
        .dst = .{ .r8 = .b },
    },
    .{ // LD B,n
        .opcode = 0x06,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .im = .n },
    },
    .{ // RLC A
        .opcode = 0x07,
        .cycles = 1,
        .class = .rlc,
        .dst = .{ .r8 = .a },
    },
    .{ // LD (nn),SP
        .opcode = 0x08,
        .cycles = 5,
        .class = .ld,
        .dst = .{ .im = .nn },
        .dst_ptr = true,
        .src = .{ .r16 = .sp },
    },
    .{ // ADD HL,BC
        .opcode = 0x09,
        .cycles = 2,
        .class = .add,
        .dst = .{ .r88 = .{ .h, .l } },
        .src = .{ .r88 = .{ .b, .c } },
    },
    .{ // LD A,(BC)
        .opcode = 0x0A,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .b, .c } },
        .src_ptr = true,
    },
    .{ // DEC BC
        .opcode = 0x0B,
        .cycles = 2,
        .class = .dec,
        .dst = .{ .r88 = .{ .b, .c } },
    },
    .{ // INC C
        .opcode = 0x0C,
        .cycles = 1,
        .class = .inc,
        .dst = .{ .r8 = .c },
    },
    .{ // DEC C
        .opcode = 0x0D,
        .cycles = 1,
        .class = .dec,
        .dst = .{ .r8 = .c },
    },
    .{ // LD C,n
        .opcode = 0x0E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .im = .n },
    },
    .{ // RRC A
        .opcode = 0x0F,
        .cycles = 1,
        .class = .rrc,
        .dst = .{ .r8 = .a },
    },
    .{ // STOP
        // Technically, this intruction is 0x1000, but apparently there's no
        // reason for this, so I'm encoding it as having an 8-bit immediate and
        // asserting it to be 0x00.
        .opcode = 0x10,
        .cycles = 0,
        .class = .stop,
        .dst = .{ .im = .n },
    },
    .{ // LD DE,nn
        .opcode = 0x11,
        .cycles = 3,
        .class = .ld,
        .src = .{ .r88 = .{ .d, .e } },
        .dst = .{ .im = .nn },
    },
    .{ // LD (DE),A
        .opcode = 0x12,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .d, .e } },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // INC DE
        .opcode = 0x13,
        .cycles = 2,
        .class = .inc,
        .dst = .{ .r88 = .{ .d, .e } },
    },
    .{ // INC D
        .opcode = 0x14,
        .cycles = 1,
        .class = .inc,
        .dst = .{ .r8 = .d },
    },
    .{ // DEC D
        .opcode = 0x15,
        .cycles = 1,
        .class = .dec,
        .dst = .{ .r8 = .d },
    },
    .{ // LD D,n
        .opcode = 0x16,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .im = .n },
    },
    .{ // RL A
        .opcode = 0x17,
        .cycles = 1,
        .class = .rl,
        .dst = .{ .r8 = .a },
    },
    .{ // JR n
        .opcode = 0x18,
        .cycles = 3,
        .class = .jr,
        .dst = .{ .im = .n },
    },
    .{ // ADD HL,DE
        .opcode = 0x19,
        .cycles = 2,
        .class = .add,
        .dst = .{ .r88 = .{ .h, .l } },
        .src = .{ .r88 = .{ .d, .e } },
    },
    .{ // LD A,(DE)
        .opcode = 0x1A,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .d, .e } },
        .src_ptr = true,
    },
    .{ // DEC DE
        .opcode = 0x1B,
        .cycles = 2,
        .class = .dec,
        .dst = .{ .r88 = .{ .d, .e } },
    },
    .{ // INC E
        .opcode = 0x1C,
        .cycles = 1,
        .class = .inc,
        .dst = .{ .r8 = .e },
    },
    .{ // DEC E
        .opcode = 0x1D,
        .cycles = 1,
        .class = .dec,
        .dst = .{ .r8 = .e },
    },
    .{ // LD E,n
        .opcode = 0x1E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .im = .n },
    },
    .{ // RR A
        .opcode = 0x1F,
        .cycles = 1,
        .class = .rr,
        .dst = .{ .r8 = .a },
    },
    .{ // JR NZ,n
        .opcode = 0x20,
        .cycles = 0,
        .class = .jr,
        .cond = .nz,
        .src = .{ .im = .n },
    },
    .{ // LD HL,nn
        .opcode = 0x21,
        .cycles = 3,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .src = .{ .im = .nn },
    },
    .{ // LD (HL),A
        .opcode = 0x22,
        .cycles = 2,
        .class = .ldi,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // INC HL
        .opcode = 0x23,
        .cycles = 2,
        .class = .inc,
        .dst = .{ .r88 = .{ .h, .l } },
    },
    .{ // INC H
        .opcode = 0x24,
        .cycles = 1,
        .class = .inc,
        .dst = .{ .r8 = .h },
    },
    .{ // DEC H
        .opcode = 0x25,
        .cycles = 1,
        .class = .dec,
        .dst = .{ .r8 = .h },
    },
    .{ // LD H,n
        .opcode = 0x26,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .im = .n },
    },
    .{ // DAA
        .opcode = 0x27,
        .cycles = 1,
        .class = .daa,
    },
    .{ // JR Z,n
        .opcode = 0x28,
        .cycles = 0,
        .class = .jr,
        .cond = .z,
        .dst = .{ .im = .n },
    },
    .{ // ADD HL,HL
        .opcode = 0x29,
        .cycles = 2,
        .class = .add,
        .dst = .{ .r88 = .{ .h, .l } },
        .src = .{ .r88 = .{ .h, .l } },
    },
    .{ // LDI A,(HL)
        .opcode = 0x2A,
        .cycles = 2,
        .class = .ldi,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // DEC HL
        .opcode = 0x2B,
        .cycles = 2,
        .class = .dec,
        .dst = .{ .r88 = .{ .h, .l } },
    },
    .{ // INC L
        .opcode = 0x2C,
        .cycles = 1,
        .class = .inc,
        .dst = .{ .r8 = .l },
    },
    .{ // DEC L
        .opcode = 0x2D,
        .cycles = 1,
        .class = .dec,
        .dst = .{ .r8 = .l },
    },
    .{ // LD L,n
        .opcode = 0x2E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .im = .n },
    },
    .{ // CPL
        .opcode = 0x2F,
        .cycles = 1,
        .class = .cpl,
    },
    .{ // JR NC,n
        .opcode = 0x30,
        .cycles = 0,
        .class = .jr,
        .cond = .nc,
        .src = .{ .im = .n },
    },
    .{ // LD SP,nn
        .opcode = 0x31,
        .cycles = 3,
        .class = .ld,
        .dst = .{ .r16 = .sp },
        .src = .{ .im = .nn },
    },
    .{ // LDD (HL),A
        .opcode = 0x32,
        .cycles = 2,
        .class = .ldd,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // INC SP
        .opcode = 0x33,
        .cycles = 2,
        .class = .inc,
        .dst = .{ .r16 = .sp },
    },
    .{ // INC (HL)
        .opcode = 0x34,
        .cycles = 3,
        .class = .inc,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // DEC (HL)
        .opcode = 0x35,
        .cycles = 3,
        .class = .dec,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // LD (HL),n
        .opcode = 0x36,
        .cycles = 3,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .im = .n },
    },
    .{ // SCF
        .opcode = 0x37,
        .cycles = 1,
        .class = .scf,
    },
    .{ // JR C,n
        .opcode = 0x38,
        .cycles = 0,
        .class = .jr,
        .cond = .c,
        .src = .{ .im = .n },
    },
    .{ // ADD HL,SP
        .opcode = 0x39,
        .cycles = 2,
        .class = .add,
        .dst = .{ .r88 = .{ .h, .l } },
        .src = .{ .r16 = .sp },
    },
    .{ // LDD A,(HL)
        .opcode = 0x3A,
        .cycles = 2,
        .class = .ldd,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // DEC SP
        .opcode = 0x3B,
        .cycles = 2,
        .class = .dec,
        .dst = .{ .r16 = .sp },
    },
    .{ // INC A
        .opcode = 0x3C,
        .cycles = 1,
        .class = .inc,
        .dst = .{ .r8 = .a },
    },
    .{ // DEC A
        .opcode = 0x3D,
        .cycles = 1,
        .class = .dec,
        .dst = .{ .r8 = .a },
    },
    .{ // LD A,n
        .opcode = 0x3E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // CCF
        .opcode = 0x3F,
        .cycles = 1,
        .class = .ccf,
    },
    .{ // LD B,B
        .opcode = 0x40,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r8 = .b },
    },
    .{ // LD B,C
        .opcode = 0x41,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r8 = .c },
    },
    .{ // LD B,D
        .opcode = 0x42,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r8 = .d },
    },
    .{ // LD B,E
        .opcode = 0x43,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r8 = .e },
    },
    .{ // LD B,H
        .opcode = 0x44,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r8 = .h },
    },
    .{ // LD B,L
        .opcode = 0x45,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r8 = .l },
    },
    .{ // LD B,(HL)
        .opcode = 0x46,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // LD B,A
        .opcode = 0x47,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .b },
        .src = .{ .r8 = .a },
    },
    .{ // LD C,B
        .opcode = 0x48,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r8 = .b },
    },
    .{ // LD C,C
        .opcode = 0x49,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r8 = .c },
    },
    .{ // LD C,D
        .opcode = 0x4A,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r8 = .d },
    },
    .{ // LD C,E
        .opcode = 0x4B,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r8 = .e },
    },
    .{ // LD C,H
        .opcode = 0x4C,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r8 = .h },
    },
    .{ // LD C,L
        .opcode = 0x4D,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r8 = .l },
    },
    .{ // LD C,(HL)
        .opcode = 0x4E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // LD C,A
        .opcode = 0x4F,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .c },
        .src = .{ .r8 = .a },
    },
    .{ // LD D,B
        .opcode = 0x50,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r8 = .b },
    },
    .{ // LD D,C
        .opcode = 0x51,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r8 = .c },
    },
    .{ // LD D,D
        .opcode = 0x52,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r8 = .d },
    },
    .{ // LD D,E
        .opcode = 0x53,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r8 = .e },
    },
    .{ // LD D,H
        .opcode = 0x54,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r8 = .h },
    },
    .{ // LD D,L
        .opcode = 0x55,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r8 = .l },
    },
    .{ // LD D,(HL)
        .opcode = 0x56,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // LD D,A
        .opcode = 0x57,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .d },
        .src = .{ .r8 = .a },
    },
    .{ // LD E,B
        .opcode = 0x58,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r8 = .b },
    },
    .{ // LD E,C
        .opcode = 0x59,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r8 = .c },
    },
    .{ // LD E,D
        .opcode = 0x5A,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r8 = .d },
    },
    .{ // LD E,D
        .opcode = 0x5B,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r8 = .e },
    },
    .{ // LD E,H
        .opcode = 0x5C,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r8 = .h },
    },
    .{ // LD E,L
        .opcode = 0x5D,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r8 = .l },
    },
    .{ // LD E,(HL)
        .opcode = 0x5E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // LD E,A
        .opcode = 0x5F,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .e },
        .src = .{ .r8 = .a },
    },
    .{ // LD H,B
        .opcode = 0x60,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r8 = .b },
    },
    .{ // LD H,C
        .opcode = 0x61,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r8 = .c },
    },
    .{ // LD H,D
        .opcode = 0x62,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r8 = .d },
    },
    .{ // LD H,E
        .opcode = 0x63,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r8 = .e },
    },
    .{ // LD H,H
        .opcode = 0x64,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r8 = .h },
    },
    .{ // LD H,L
        .opcode = 0x65,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r8 = .l },
    },
    .{ // LD H,(HL)
        .opcode = 0x66,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // LD H,A
        .opcode = 0x67,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .h },
        .src = .{ .r8 = .a },
    },
    .{ // LD L,B
        .opcode = 0x68,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r8 = .b },
    },
    .{ // LD L,C
        .opcode = 0x69,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r8 = .c },
    },
    .{ // LD L,D
        .opcode = 0x6A,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r8 = .d },
    },
    .{ // LD L,E
        .opcode = 0x6B,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r8 = .e },
    },
    .{ // LD L,H
        .opcode = 0x6C,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r8 = .h },
    },
    .{ // LD L,L
        .opcode = 0x6D,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r8 = .l },
    },
    .{ // LD L,(HL)
        .opcode = 0x6E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // LD L,A
        .opcode = 0x6F,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .l },
        .src = .{ .r8 = .a },
    },
    .{ // LD (HL),B
        .opcode = 0x70,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .b },
    },
    .{ // LD (HL),C
        .opcode = 0x71,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .c },
    },
    .{ // LD (HL),D
        .opcode = 0x72,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .d },
    },
    .{ // LD (HL),E
        .opcode = 0x73,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .e },
    },
    .{ // LD (HL),H
        .opcode = 0x74,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .h },
    },
    .{ // LD (HL),L
        .opcode = 0x75,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .l },
    },
    .{ // HALT
        .opcode = 0x76,
        .cycles = 0,
        .class = .halt,
    },
    .{ // LD (HL),A
        .opcode = 0x77,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // LD A,B
        .opcode = 0x78,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // LD A,C
        .opcode = 0x79,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // LD A,D
        .opcode = 0x7A,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // LD A,E
        .opcode = 0x7B,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .f },
    },
    .{ // LD A,H
        .opcode = 0x7C,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // LD A,L
        .opcode = 0x7D,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // LD A,(HL)
        .opcode = 0x7E,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // LD A,A
        .opcode = 0x7F,
        .cycles = 1,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // ADD A,B
        .opcode = 0x80,
        .cycles = 1,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // ADD A,C
        .opcode = 0x81,
        .cycles = 1,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // ADD A,D
        .opcode = 0x82,
        .cycles = 1,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // ADD A,E
        .opcode = 0x83,
        .cycles = 1,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // ADD A,H
        .opcode = 0x84,
        .cycles = 1,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // ADD A,L
        .opcode = 0x85,
        .cycles = 1,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // ADD A,(HL)
        .opcode = 0x86,
        .cycles = 2,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // ADD A,A
        .opcode = 0x87,
        .cycles = 1,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // ADC A,B
        .opcode = 0x88,
        .cycles = 1,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // ADC A,C
        .opcode = 0x89,
        .cycles = 1,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // ADC A,D
        .opcode = 0x8A,
        .cycles = 1,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // ADC A,E
        .opcode = 0x8B,
        .cycles = 1,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // ADC A,H
        .opcode = 0x8C,
        .cycles = 1,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // ADC A,L
        .opcode = 0x8D,
        .cycles = 1,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // ADC A,(HL)
        .opcode = 0x8E,
        .cycles = 2,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // ADC A,A
        .opcode = 0x8F,
        .cycles = 1,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // SUB A,B
        .opcode = 0x90,
        .cycles = 1,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // SUB A,C
        .opcode = 0x91,
        .cycles = 1,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // SUB A,D
        .opcode = 0x92,
        .cycles = 1,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // SUB A,E
        .opcode = 0x93,
        .cycles = 1,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // SUB A,H
        .opcode = 0x94,
        .cycles = 1,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // SUB A,L
        .opcode = 0x95,
        .cycles = 1,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // SUB A,(HL)
        .opcode = 0x96,
        .cycles = 2,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SUB A,A
        .opcode = 0x97,
        .cycles = 1,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // SBC A,B
        .opcode = 0x98,
        .cycles = 1,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // SBC A,C
        .opcode = 0x99,
        .cycles = 1,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // SBC A,D
        .opcode = 0x9A,
        .cycles = 1,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // SBC A,E
        .opcode = 0x9B,
        .cycles = 1,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // SBC A,H
        .opcode = 0x9C,
        .cycles = 1,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // SBC A,L
        .opcode = 0x9D,
        .cycles = 1,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // SBC A,(HL)
        .opcode = 0x9E,
        .cycles = 2,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SBC A,A
        .opcode = 0x9F,
        .cycles = 1,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // AND A,B
        .opcode = 0xA0,
        .cycles = 1,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // AND A,C
        .opcode = 0xA1,
        .cycles = 1,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // AND A,D
        .opcode = 0xA2,
        .cycles = 1,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // AND A,E
        .opcode = 0xA3,
        .cycles = 1,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // AND A,H
        .opcode = 0xA4,
        .cycles = 1,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // AND A,L
        .opcode = 0xA5,
        .cycles = 1,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // AND A,(HL)
        .opcode = 0xA6,
        .cycles = 2,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // AND A,A
        .opcode = 0xA7,
        .cycles = 1,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // XOR A,B
        .opcode = 0xA8,
        .cycles = 1,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // XOR A,C
        .opcode = 0xA9,
        .cycles = 1,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // XOR A,D
        .opcode = 0xAA,
        .cycles = 1,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // XOR A,E
        .opcode = 0xAB,
        .cycles = 1,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // XOR A,H
        .opcode = 0xAC,
        .cycles = 1,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // XOR A,L
        .opcode = 0xAD,
        .cycles = 1,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // XOR A,(HL)
        .opcode = 0xAE,
        .cycles = 2,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // XOR A,A
        .opcode = 0xAF,
        .cycles = 1,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // OR A,B
        .opcode = 0xB0,
        .cycles = 1,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // OR A,C
        .opcode = 0xB1,
        .cycles = 1,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // OR A,D
        .opcode = 0xB2,
        .cycles = 1,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // OR A,E
        .opcode = 0xB3,
        .cycles = 1,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // OR A,H
        .opcode = 0xB4,
        .cycles = 1,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // OR A,L
        .opcode = 0xB5,
        .cycles = 1,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // OR A,(HL)
        .opcode = 0xB6,
        .cycles = 2,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // OR A,A
        .opcode = 0xB7,
        .cycles = 1,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // CP A,B
        .opcode = 0xB8,
        .cycles = 1,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .b },
    },
    .{ // CP A,C
        .opcode = 0xB9,
        .cycles = 1,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .c },
    },
    .{ // CP A,D
        .opcode = 0xBA,
        .cycles = 1,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .d },
    },
    .{ // CP A,E
        .opcode = 0xBB,
        .cycles = 1,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .e },
    },
    .{ // CP A,H
        .opcode = 0xBC,
        .cycles = 1,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .h },
    },
    .{ // CP A,L
        .opcode = 0xBD,
        .cycles = 1,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .l },
    },
    .{ // CP A,(HL)
        .opcode = 0xBE,
        .cycles = 2,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // CP A,A
        .opcode = 0xBF,
        .cycles = 1,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .r8 = .a },
    },
    .{ // RET NZ
        .opcode = 0xC0,
        .cycles = 0,
        .class = .ret,
        .cond = .nz,
    },
    .{ // POP BC
        .opcode = 0xC1,
        .cycles = 3,
        .class = .pop,
        .dst = .{ .r88 = .{ .b, .c } },
    },
    .{ // JP NZ,nn
        .opcode = 0xC2,
        .cycles = 0,
        .class = .jp,
        .cond = .nz,
        .dst = .{ .im = .nn },
    },
    .{ // JP nn
        .opcode = 0xC3,
        .cycles = 4,
        .class = .jp,
        .dst = .{ .im = .nn },
    },
    .{ // CALL NZ,nn
        .opcode = 0xC4,
        .cycles = 0,
        .class = .call,
        .cond = .nz,
        .dst = .{ .im = .nn },
    },
    .{ // PUSH BC
        .opcode = 0xC5,
        .cycles = 4,
        .class = .push,
        .dst = .{ .r88 = .{ .b, .c } },
    },
    .{ // ADD A,n
        .opcode = 0xC6,
        .cycles = 2,
        .class = .add,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 0
        .opcode = 0xC7,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x0 },
    },
    .{ // RET Z
        .opcode = 0xC8,
        .cycles = 0,
        .class = .ret,
        .cond = .z,
    },
    .{ // RET
        .opcode = 0xC9,
        .cycles = 4,
        .class = .ret,
    },
    .{ // JP Z,nn
        .opcode = 0xCA,
        .cycles = 0,
        .class = .jp,
        .cond = .z,
        .dst = .{ .im = .nn },
    },
    .{
        // Extended operations, see the INSTRUCTIONS_EXTENDED table.
        .opcode = 0xCB,
        .cycles = 0,
        .class = .extended,
    },
    .{ // CALL Z,nn
        .opcode = 0xCC,
        .cycles = 0,
        .class = .call,
        .cond = .z,
        .dst = .{ .im = .nn },
    },
    .{ // CALL nn
        .opcode = 0xCD,
        .cycles = 6,
        .class = .call,
        .dst = .{ .im = .nn },
    },
    .{ // ADC A,n
        .opcode = 0xCE,
        .cycles = 2,
        .class = .adc,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 8
        .opcode = 0xCF,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x8 },
    },
    .{ // RET NC
        .opcode = 0xD0,
        .cycles = 0,
        .class = .ret,
        .cond = .nc,
    },
    .{ // POP DE
        .opcode = 0xD1,
        .cycles = 3,
        .class = .pop,
        .dst = .{ .r88 = .{ .d, .e } },
    },
    .{ // JP NC,nn
        .opcode = 0xD2,
        .cycles = 0,
        .class = .jp,
        .cond = .nc,
        .dst = .{ .im = .nn },
    },
    .{ // XX
        .opcode = 0xD3,
        .cycles = 0,
        .class = .removed,
    },
    .{ // CALL NC,nn
        .opcode = 0xD4,
        .cycles = 0,
        .class = .call,
        .cond = .nc,
        .dst = .{ .im = .nn },
    },
    .{ // PUSH DE
        .opcode = 0xD5,
        .cycles = 4,
        .class = .push,
        .dst = .{ .r88 = .{ .d, .e } },
    },
    .{ // SUB A,n
        .opcode = 0xD6,
        .cycles = 2,
        .class = .sub,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 10
        .opcode = 0xD7,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x10 },
    },
    .{ // RET C
        .opcode = 0xD8,
        .cycles = 0,
        .class = .ret,
        .cond = .c,
    },
    .{ // RETI
        .opcode = 0xD9,
        .cycles = 4,
        .class = .reti,
    },
    .{ // JP C,nn
        .opcode = 0xDA,
        .cycles = 0,
        .class = .jp,
        .cond = .c,
        .dst = .{ .im = .nn },
    },
    .{ // XX
        .opcode = 0xDB,
        .cycles = 0,
        .class = .removed,
    },
    .{ // CALL C,nn
        .opcode = 0xDC,
        .cycles = 0,
        .class = .call,
        .cond = .c,
        .dst = .{ .im = .nn },
    },
    .{ // XX
        .opcode = 0xDD,
        .cycles = 0,
        .class = .removed,
    },
    .{ // SBC A,n
        .opcode = 0xDE,
        .cycles = 2,
        .class = .sbc,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 18
        .opcode = 0xDF,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x18 },
    },
    .{ // LDH (n),A
        .opcode = 0xE0,
        .cycles = 3,
        .class = .ldh,
        .dst = .{ .im = .n },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // POP HL
        .opcode = 0xE1,
        .cycles = 3,
        .class = .pop,
        .dst = .{ .r88 = .{ .h, .l } },
    },
    .{ // LDH (C),A
        .opcode = 0xE2,
        .cycles = 2,
        .class = .ldh,
        .dst = .{ .r8 = .c },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // XX
        .opcode = 0xE3,
        .cycles = 0,
        .class = .removed,
    },
    .{ // XX
        .opcode = 0xE4,
        .cycles = 0,
        .class = .removed,
    },
    .{ // PUSH HL
        .opcode = 0xE5,
        .cycles = 4,
        .class = .push,
        .dst = .{ .r88 = .{ .h, .l } },
    },
    .{ // AND A,n
        .opcode = 0xE6,
        .cycles = 2,
        .class = .@"and",
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 20
        .opcode = 0xE7,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x20 },
    },
    .{ // ADD SP,d
        .opcode = 0xE8,
        .cycles = 4,
        .class = .add,
        .dst = .{ .r16 = .sp },
        .src = .{ .im = .n },
    },
    .{ // JP (HL)
        .opcode = 0xE9,
        .cycles = 1,
        .class = .jp,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // LD (nn),A
        .opcode = 0xEA,
        .cycles = 4,
        .class = .ld,
        .dst = .{ .im = .nn },
        .dst_ptr = true,
        .src = .{ .r8 = .a },
    },
    .{ // XX
        .opcode = 0xEB,
        .cycles = 0,
        .class = .removed,
    },
    .{ // XX
        .opcode = 0xEC,
        .cycles = 0,
        .class = .removed,
    },
    .{ // XX
        .opcode = 0xED,
        .cycles = 0,
        .class = .removed,
    },
    .{ // XOR A,n
        .opcode = 0xEE,
        .cycles = 2,
        .class = .xor,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 28
        .opcode = 0xEF,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x28 },
    },
    .{ // LDH A,(n)
        .opcode = 0xF0,
        .cycles = 3,
        .class = .ldh,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
        .src_ptr = true,
    },
    .{ // POP AF
        .opcode = 0xF1,
        .cycles = 3,
        .class = .pop,
        .dst = .{ .r88 = .{ .a, .f } },
    },
    .{ // XX
        .opcode = 0xF2,
        .cycles = 2,
        .class = .removed,
    },
    .{ // DI
        .opcode = 0xF3,
        .cycles = 1,
        .class = .di,
    },
    .{ // XX
        .opcode = 0xF4,
        .cycles = 0,
        .class = .removed,
    },
    .{ // PUSH AF
        .opcode = 0xF5,
        .cycles = 4,
        .class = .push,
        .dst = .{ .r88 = .{ .a, .f } },
    },
    .{ // OR A,n
        .opcode = 0xF6,
        .cycles = 2,
        .class = .@"or",
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 30
        .opcode = 0xF7,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x30 },
    },
    .{ // LDHL SP,d
        .opcode = 0xF8,
        .cycles = 3,
        .class = .ldhl,
        .dst = .{ .r16 = .sp },
        .src = .{ .im = .n },
    },
    .{ // LD SP,HL
        .opcode = 0xF9,
        .cycles = 2,
        .class = .ld,
        .dst = .{ .r16 = .sp },
        .src = .{ .r88 = .{ .h, .l } },
    },
    .{ // LD A,(nn)
        .opcode = 0xFA,
        .cycles = 4,
        .class = .ld,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .nn },
        .src_ptr = true,
    },
    .{ // EI
        .opcode = 0xFB,
        .cycles = 1,
        .class = .ei,
    },
    .{ // XX
        .opcode = 0xFC,
        .cycles = 0,
        .class = .removed,
    },
    .{ // XX
        .opcode = 0xFD,
        .cycles = 0,
        .class = .removed,
    },
    .{ // CP A,n
        .opcode = 0xFE,
        .cycles = 2,
        .class = .cp,
        .dst = .{ .r8 = .a },
        .src = .{ .im = .n },
    },
    .{ // RST 38
        .opcode = 0xFF,
        .cycles = 4,
        .class = .rst,
        .dst = .{ .addr = 0x38 },
    },
};

pub const INSTRUCTIONS_EXTENDED = [_]Instruction{
    .{ // RLC B
        .opcode = 0x00,
        .cycles = 2,
        .class = .rlc,
        .dst = .{ .r8 = .b },
    },
    .{ // RLC C
        .opcode = 0x01,
        .cycles = 2,
        .class = .rlc,
        .dst = .{ .r8 = .c },
    },
    .{ // RLC D
        .opcode = 0x02,
        .cycles = 2,
        .class = .rlc,
        .dst = .{ .r8 = .d },
    },
    .{ // RLC E
        .opcode = 0x03,
        .cycles = 2,
        .class = .rlc,
        .dst = .{ .r8 = .e },
    },
    .{ // RLC H
        .opcode = 0x04,
        .cycles = 2,
        .class = .rlc,
        .dst = .{ .r8 = .h },
    },
    .{ // RLC L
        .opcode = 0x05,
        .cycles = 2,
        .class = .rlc,
        .dst = .{ .r8 = .l },
    },
    .{ // RLC (HL)
        .opcode = 0x06,
        .cycles = 4,
        .class = .rlc,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // RLC A
        .opcode = 0x07,
        .cycles = 2,
        .class = .rlc,
        .dst = .{ .r8 = .a },
    },
    .{ // RRC B
        .opcode = 0x08,
        .cycles = 2,
        .class = .rrc,
        .dst = .{ .r8 = .b },
    },
    .{ // RRC C
        .opcode = 0x09,
        .cycles = 2,
        .class = .rrc,
        .dst = .{ .r8 = .c },
    },
    .{ // RRC D
        .opcode = 0x0A,
        .cycles = 2,
        .class = .rrc,
        .dst = .{ .r8 = .d },
    },
    .{ // RRC E
        .opcode = 0x0B,
        .cycles = 2,
        .class = .rrc,
        .dst = .{ .r8 = .e },
    },
    .{ // RRC H
        .opcode = 0x0C,
        .cycles = 2,
        .class = .rrc,
        .dst = .{ .r8 = .h },
    },
    .{ // RRC L
        .opcode = 0x0D,
        .cycles = 2,
        .class = .rrc,
        .dst = .{ .r8 = .l },
    },
    .{ // RRC (HL)
        .opcode = 0x0E,
        .cycles = 4,
        .class = .rrc,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // RRC A
        .opcode = 0x0F,
        .cycles = 2,
        .class = .rrc,
        .dst = .{ .r8 = .a },
    },
    .{ // RL B
        .opcode = 0x10,
        .cycles = 2,
        .class = .rl,
        .dst = .{ .r8 = .b },
    },
    .{ // RL C
        .opcode = 0x11,
        .cycles = 2,
        .class = .rl,
        .dst = .{ .r8 = .c },
    },
    .{ // RL D
        .opcode = 0x12,
        .cycles = 2,
        .class = .rl,
        .dst = .{ .r8 = .d },
    },
    .{ // RL E
        .opcode = 0x13,
        .cycles = 2,
        .class = .rl,
        .dst = .{ .r8 = .e },
    },
    .{ // RL H
        .opcode = 0x14,
        .cycles = 2,
        .class = .rl,
        .dst = .{ .r8 = .h },
    },
    .{ // RL L
        .opcode = 0x15,
        .cycles = 2,
        .class = .rl,
        .dst = .{ .r8 = .l },
    },
    .{ // RL (HL)
        .opcode = 0x16,
        .cycles = 4,
        .class = .rl,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // RL A
        .opcode = 0x17,
        .cycles = 2,
        .class = .rl,
        .dst = .{ .r8 = .a },
    },
    .{ // RR B
        .opcode = 0x18,
        .cycles = 2,
        .class = .rr,
        .dst = .{ .r8 = .b },
    },
    .{ // RR C
        .opcode = 0x19,
        .cycles = 2,
        .class = .rr,
        .dst = .{ .r8 = .c },
    },
    .{ // RR D
        .opcode = 0x1A,
        .cycles = 2,
        .class = .rr,
        .dst = .{ .r8 = .d },
    },
    .{ // RR E
        .opcode = 0x1B,
        .cycles = 2,
        .class = .rr,
        .dst = .{ .r8 = .e },
    },
    .{ // RR H
        .opcode = 0x1C,
        .cycles = 2,
        .class = .rr,
        .dst = .{ .r8 = .h },
    },
    .{ // RR L
        .opcode = 0x1D,
        .cycles = 2,
        .class = .rr,
        .dst = .{ .r8 = .l },
    },
    .{ // RR (HL)
        .opcode = 0x1E,
        .cycles = 4,
        .class = .rr,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // RR A
        .opcode = 0x1F,
        .cycles = 2,
        .class = .rr,
        .dst = .{ .r8 = .a },
    },
    .{ // SLA B
        .opcode = 0x20,
        .cycles = 2,
        .class = .sla,
        .dst = .{ .r8 = .b },
    },
    .{ // SLA C
        .opcode = 0x21,
        .cycles = 2,
        .class = .sla,
        .dst = .{ .r8 = .c },
    },
    .{ // SLA D
        .opcode = 0x22,
        .cycles = 2,
        .class = .sla,
        .dst = .{ .r8 = .d },
    },
    .{ // SLA E
        .opcode = 0x23,
        .cycles = 2,
        .class = .sla,
        .dst = .{ .r8 = .e },
    },
    .{ // SLA H
        .opcode = 0x24,
        .cycles = 2,
        .class = .sla,
        .dst = .{ .r8 = .h },
    },
    .{ // SLA L
        .opcode = 0x25,
        .cycles = 2,
        .class = .sla,
        .dst = .{ .r8 = .l },
    },
    .{ // SLA (HL)
        .opcode = 0x26,
        .cycles = 4,
        .class = .sla,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // SLA A
        .opcode = 0x27,
        .cycles = 2,
        .class = .sla,
        .dst = .{ .r8 = .a },
    },
    .{ // SRA B
        .opcode = 0x28,
        .cycles = 2,
        .class = .sra,
        .dst = .{ .r8 = .b },
    },
    .{ // SRA C
        .opcode = 0x29,
        .cycles = 2,
        .class = .sra,
        .dst = .{ .r8 = .c },
    },
    .{ // SRA D
        .opcode = 0x2A,
        .cycles = 2,
        .class = .sra,
        .dst = .{ .r8 = .d },
    },
    .{ // SRA E
        .opcode = 0x2B,
        .cycles = 2,
        .class = .sra,
        .dst = .{ .r8 = .e },
    },
    .{ // SRA H
        .opcode = 0x2C,
        .cycles = 2,
        .class = .sra,
        .dst = .{ .r8 = .h },
    },
    .{ // SRA L
        .opcode = 0x2D,
        .cycles = 2,
        .class = .sra,
        .dst = .{ .r8 = .l },
    },
    .{ // SRA (HL)
        .opcode = 0x2E,
        .cycles = 4,
        .class = .sra,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // SRA A
        .opcode = 0x2F,
        .cycles = 2,
        .class = .sra,
        .dst = .{ .r8 = .a },
    },
    .{ // SWAP B
        .opcode = 0x30,
        .cycles = 2,
        .class = .swap,
        .dst = .{ .r8 = .b },
    },
    .{ // SWAP C
        .opcode = 0x31,
        .cycles = 2,
        .class = .swap,
        .dst = .{ .r8 = .c },
    },
    .{ // SWAP D
        .opcode = 0x32,
        .cycles = 2,
        .class = .swap,
        .dst = .{ .r8 = .d },
    },
    .{ // SWAP E
        .opcode = 0x33,
        .cycles = 2,
        .class = .swap,
        .dst = .{ .r8 = .e },
    },
    .{ // SWAP H
        .opcode = 0x34,
        .cycles = 2,
        .class = .swap,
        .dst = .{ .r8 = .h },
    },
    .{ // SWAP L
        .opcode = 0x35,
        .cycles = 2,
        .class = .swap,
        .dst = .{ .r8 = .l },
    },
    .{ // SWAP (HL)
        .opcode = 0x36,
        .cycles = 4,
        .class = .swap,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // SWAP A
        .opcode = 0x37,
        .cycles = 2,
        .class = .swap,
        .dst = .{ .r8 = .a },
    },
    .{ // SRL B
        .opcode = 0x38,
        .cycles = 2,
        .class = .srl,
        .dst = .{ .r8 = .b },
    },
    .{ // SRL C
        .opcode = 0x39,
        .cycles = 2,
        .class = .srl,
        .dst = .{ .r8 = .c },
    },
    .{ // SRL D
        .opcode = 0x3A,
        .cycles = 2,
        .class = .srl,
        .dst = .{ .r8 = .d },
    },
    .{ // SRL E
        .opcode = 0x3B,
        .cycles = 2,
        .class = .srl,
        .dst = .{ .r8 = .e },
    },
    .{ // SRL H
        .opcode = 0x3C,
        .cycles = 2,
        .class = .srl,
        .dst = .{ .r8 = .h },
    },
    .{ // SRL L
        .opcode = 0x3D,
        .cycles = 2,
        .class = .srl,
        .dst = .{ .r8 = .l },
    },
    .{ // SRL (HL)
        .opcode = 0x3E,
        .cycles = 4,
        .class = .srl,
        .dst = .{ .r88 = .{ .h, .l } },
        .dst_ptr = true,
    },
    .{ // SRL A
        .opcode = 0x3F,
        .cycles = 2,
        .class = .srl,
        .dst = .{ .r8 = .a },
    },
    .{ // RES 0,B
        .opcode = 0x40,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 0,C
        .opcode = 0x41,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 0,D
        .opcode = 0x42,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 0,E
        .opcode = 0x43,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 0,H
        .opcode = 0x44,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 0,L
        .opcode = 0x45,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 0,(HL)
        .opcode = 0x46,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 0,A
        .opcode = 0x47,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .a },
    },
    .{ // BIT 1,B
        .opcode = 0x48,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 1,C
        .opcode = 0x49,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 1,D
        .opcode = 0x4A,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 1,E
        .opcode = 0x4B,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 1,H
        .opcode = 0x4C,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 1,L
        .opcode = 0x4D,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 1,(HL)
        .opcode = 0x4E,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 1,A
        .opcode = 0x4F,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .a },
    },
    .{ // BIT 2,B
        .opcode = 0x50,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 2,C
        .opcode = 0x51,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 2,D
        .opcode = 0x52,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 2,E
        .opcode = 0x53,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 2,H
        .opcode = 0x54,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 2,L
        .opcode = 0x55,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 2,(HL)
        .opcode = 0x56,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 2,A
        .opcode = 0x57,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .a },
    },
    .{ // BIT 3,B
        .opcode = 0x58,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 3,C
        .opcode = 0x59,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 3,D
        .opcode = 0x5A,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 3,E
        .opcode = 0x5B,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 3,H
        .opcode = 0x5C,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 3,L
        .opcode = 0x5D,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 3,(HL)
        .opcode = 0x5E,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 3,A
        .opcode = 0x5F,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .a },
    },
    .{ // BIT 4,B
        .opcode = 0x60,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 4,C
        .opcode = 0x61,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 4,D
        .opcode = 0x62,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 4,E
        .opcode = 0x63,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 4,H
        .opcode = 0x64,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 4,L
        .opcode = 0x65,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 4,(HL)
        .opcode = 0x66,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 4,A
        .opcode = 0x67,
        .cycles = 4,
        .class = .bit,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .a },
    },
    .{ // BIT 5,B
        .opcode = 0x68,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 5,C
        .opcode = 0x69,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 5,D
        .opcode = 0x6A,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 5,E
        .opcode = 0x6B,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 5,H
        .opcode = 0x6C,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 5,L
        .opcode = 0x6D,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 5,(HL)
        .opcode = 0x6E,
        .cycles = 5,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 5,A
        .opcode = 0x6F,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .a },
    },
    .{ // BIT 6,B
        .opcode = 0x70,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 6,C
        .opcode = 0x71,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 6,D
        .opcode = 0x72,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 6,E
        .opcode = 0x73,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 6,H
        .opcode = 0x74,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 6,L
        .opcode = 0x75,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 6,(HL)
        .opcode = 0x76,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 6,A
        .opcode = 0x77,
        .cycles = 6,
        .class = .bit,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .a },
    },
    .{ // BIT 7,B
        .opcode = 0x78,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .b },
    },
    .{ // BIT 7,C
        .opcode = 0x79,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .c },
    },
    .{ // BIT 7,D
        .opcode = 0x7A,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .d },
    },
    .{ // BIT 7,E
        .opcode = 0x7B,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .e },
    },
    .{ // BIT 7,H
        .opcode = 0x7C,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .h },
    },
    .{ // BIT 7,L
        .opcode = 0x7D,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .l },
    },
    .{ // BIT 7,(HL)
        .opcode = 0x7E,
        .cycles = 7,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // BIT 7,A
        .opcode = 0x7F,
        .cycles = 2,
        .class = .bit,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 0,B
        .opcode = 0x80,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 0,C
        .opcode = 0x81,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 0,D
        .opcode = 0x82,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 0,E
        .opcode = 0x83,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 0,H
        .opcode = 0x84,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 0,L
        .opcode = 0x85,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 0,(HL)
        .opcode = 0x86,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 0,A
        .opcode = 0x87,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 1,B
        .opcode = 0x88,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 1,C
        .opcode = 0x89,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 1,D
        .opcode = 0x8A,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 1,E
        .opcode = 0x8B,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 1,H
        .opcode = 0x8C,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 1,L
        .opcode = 0x8D,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 1,(HL)
        .opcode = 0x8E,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 1,A
        .opcode = 0x8F,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 2,B
        .opcode = 0x90,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 2,C
        .opcode = 0x91,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 2,D
        .opcode = 0x92,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 2,E
        .opcode = 0x93,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 2,H
        .opcode = 0x94,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 2,L
        .opcode = 0x95,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 2,(HL)
        .opcode = 0x96,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 2,A
        .opcode = 0x97,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 3,B
        .opcode = 0x98,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 3,C
        .opcode = 0x99,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 3,D
        .opcode = 0x9A,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 3,E
        .opcode = 0x9B,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 3,H
        .opcode = 0x9C,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 3,L
        .opcode = 0x9D,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 3,(HL)
        .opcode = 0x9E,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 3,A
        .opcode = 0x9F,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 4,B
        .opcode = 0xA0,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 4,C
        .opcode = 0xA1,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 4,D
        .opcode = 0xA2,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 4,E
        .opcode = 0xA3,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 4,H
        .opcode = 0xA4,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 4,L
        .opcode = 0xA5,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 4,(HL)
        .opcode = 0xA6,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 4,A
        .opcode = 0xA7,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 5,B
        .opcode = 0xA8,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 5,C
        .opcode = 0xA9,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 5,D
        .opcode = 0xAA,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 5,E
        .opcode = 0xAB,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 5,H
        .opcode = 0xAC,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 5,L
        .opcode = 0xAD,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 5,(HL)
        .opcode = 0xAE,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 5,A
        .opcode = 0xAF,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 6,B
        .opcode = 0xB0,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 6,C
        .opcode = 0xB1,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 6,D
        .opcode = 0xB2,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 6,E
        .opcode = 0xB3,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 6,H
        .opcode = 0xB4,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 6,L
        .opcode = 0xB5,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 6,(HL)
        .opcode = 0xB6,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 6,A
        .opcode = 0xB7,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .a },
    },
    .{ // RES 7,B
        .opcode = 0xB8,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .b },
    },
    .{ // RES 7,C
        .opcode = 0xB9,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .c },
    },
    .{ // RES 7,D
        .opcode = 0xBA,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .d },
    },
    .{ // RES 7,E
        .opcode = 0xBB,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .e },
    },
    .{ // RES 7,H
        .opcode = 0xBC,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .h },
    },
    .{ // RES 7,L
        .opcode = 0xBD,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .l },
    },
    .{ // RES 7,(HL)
        .opcode = 0xBE,
        .cycles = 4,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // RES 7,A
        .opcode = 0xBF,
        .cycles = 2,
        .class = .res,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 0,B
        .opcode = 0xC0,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 0,C
        .opcode = 0xC1,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 0,D
        .opcode = 0xC2,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 0,E
        .opcode = 0xC3,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 0,H
        .opcode = 0xC4,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 0,L
        .opcode = 0xC5,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 0,(HL)
        .opcode = 0xC6,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 0,A
        .opcode = 0xC7,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 0 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 1,B
        .opcode = 0xC8,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 1,C
        .opcode = 0xC9,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 1,D
        .opcode = 0xCA,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 1,E
        .opcode = 0xCB,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 1,H
        .opcode = 0xCC,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 1,L
        .opcode = 0xCD,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 1,(HL)
        .opcode = 0xCE,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 1,A
        .opcode = 0xCF,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 1 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 2,B
        .opcode = 0xD0,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 2,C
        .opcode = 0xD1,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 2,D
        .opcode = 0xD2,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 2,E
        .opcode = 0xD3,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 2,H
        .opcode = 0xD4,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 2,L
        .opcode = 0xD5,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 2,(HL)
        .opcode = 0xD6,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 2,A
        .opcode = 0xD7,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 2 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 3,B
        .opcode = 0xD8,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 3,C
        .opcode = 0xD9,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 3,D
        .opcode = 0xDA,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 3,E
        .opcode = 0xDB,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 3,H
        .opcode = 0xDC,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 3,L
        .opcode = 0xDD,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 3,(HL)
        .opcode = 0xDE,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 3,A
        .opcode = 0xDF,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 3 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 4,B
        .opcode = 0xE0,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 4,C
        .opcode = 0xE1,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 4,D
        .opcode = 0xE2,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 4,E
        .opcode = 0xE3,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 4,H
        .opcode = 0xE4,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 4,L
        .opcode = 0xE5,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 4,(HL)
        .opcode = 0xE6,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 4,A
        .opcode = 0xE7,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 4 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 5,B
        .opcode = 0xE8,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 5,C
        .opcode = 0xE9,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 5,D
        .opcode = 0xEA,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 5,E
        .opcode = 0xEB,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 5,H
        .opcode = 0xEC,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 5,L
        .opcode = 0xED,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 5,(HL)
        .opcode = 0xEE,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 5,A
        .opcode = 0xEF,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 5 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 6,B
        .opcode = 0xF0,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 6,C
        .opcode = 0xF1,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 6,D
        .opcode = 0xF2,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 6,E
        .opcode = 0xF3,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 6,H
        .opcode = 0xF4,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 6,L
        .opcode = 0xF5,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 6,(HL)
        .opcode = 0xF6,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 6,A
        .opcode = 0xF7,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 6 },
        .src = .{ .r8 = .a },
    },
    .{ // SET 7,B
        .opcode = 0xF8,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .b },
    },
    .{ // SET 7,C
        .opcode = 0xF9,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .c },
    },
    .{ // SET 7,D
        .opcode = 0xFA,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .d },
    },
    .{ // SET 7,E
        .opcode = 0xFB,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .e },
    },
    .{ // SET 7,H
        .opcode = 0xFC,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .h },
    },
    .{ // SET 7,L
        .opcode = 0xFD,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .l },
    },
    .{ // SET 7,(HL)
        .opcode = 0xFE,
        .cycles = 4,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r88 = .{ .h, .l } },
        .src_ptr = true,
    },
    .{ // SET 7,A
        .opcode = 0xFF,
        .cycles = 2,
        .class = .set,
        .dst = .{ .bit = 7 },
        .src = .{ .r8 = .a },
    },
};
