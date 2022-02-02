const z = @import("std");

const testing = z.testing;
const expect = testing.expect;
const expectEq = testing.expectEqual;

const Sharp_LH5264N4 = @import("sharp_lh5264n4.zig").Sharp_LH5264N4;

const Instruction = @import("instruction.zig").Instruction;

const CPU_CLOCK: usize = 4 * 1024 * 1024; // 4 MHz

// The 8-bit registers of the Gameboy's CPU.
//
// You may know that some of these are usually combined into a single 16-bit
// register. In this emulator, this is done manually in the (read,write)88
// functions from the Sharp_LR35902 struct, where you can input two Register8s.
pub const Register8 = enum(usize) {
    a, // usually the target for some instructions
    f, // see FlagRegister
    b,
    c,
    d,
    e,
    h,
    l,
};

// The 16-bit registers of the Gameboy's CPU. For the combinations of Register8
// into a single 16-bit register, check the (read,write)88 functions in the
// Sharp_LR35902 struct.
pub const Register16 = enum(usize) {
    sp, // Stack Pointer (points to the current stack position)
    pc, // Program Counter (points to the next instruction to be executed)
};

// Number of bytes needed to store all registers in a continuous array.
//
//     (number of 8-bit registers) + (number of 16-bit registers) * 2
const TOTAL_REGISTER_SIZE: usize = @typeInfo(Register8).Enum.fields.len *
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
    if (!(@sizeOf(Registers) == TOTAL_REGISTER_SIZE)) {
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
    expectEq(@sizeOf(FlagRegister), @sizeOf(u8)) catch |err| {
        @compileError(err);
    };
}

// The heart of it all. This chip includes:
//
//    - the regular CPU
//    - interrupt controller
//    - timer
//    - memory
//    - boot ROM
//    - joypad input
//    - serial data transfer
//    - sound controller
//    - pixel processing unit
pub const Sharp_LR35902 = struct {
    const Self = @This();

    pub const BOOT_ROM: []const u8 = @embedFile("../DMG_ROM.bin");

    const l = z.log.scoped(.sharp_lr35902);

    comptime {
        expectEq(BOOT_ROM.len, 256) catch |err| {
            @compileError(err);
        };
    }

    // An array to store the data of all registers.
    rgs: [TOTAL_REGISTER_SIZE]u8,

    ram: *Sharp_LH5264N4,
    vram: *Sharp_LH5264N4,

    // Pull the next instruction from the extended table
    pull_from_extended: bool = false,

    // Create a new chip, immediately calling reset.
    pub fn init(ram: *Sharp_LH5264N4, vram: *Sharp_LH5264N4) Self {
        var new_chip = Self{
            .rgs = undefined,

            .ram = ram,
            .vram = vram,
        };

        new_chip.reset();
        new_chip.loadBootRom();

        return new_chip;
    }

    // The real initialization happens here.
    pub fn reset(self: *Self) void {
        self.rgs = z.mem.zeroes(@TypeOf(self.rgs));
    }

    // Load the boot rom into memory
    fn loadBootRom(self: *Self) void {
        for (Self.BOOT_ROM) |byte, i| {
            self.ram.data[i] = byte;
        }
    }

    inline fn registers(self: *Self) *Registers {
        return @ptrCast(*Registers, &self.rgs);
    }

    inline fn flagRegister(self: *Self) *FlagRegister {
        return &self.registers().f;
    }

    fn read8(self: Self, r: Register8) u8 {
        return self.rgs[@enumToInt(r)];
    }

    fn write8(self: *Self, r: Register8, val: u8) void {
        self.rgs[@enumToInt(r)] = val;
    }

    fn read88(self: Self, r1: Register8, r2: Register8) u16 {
        const reg1 = self.read8(r1);
        const reg2 = self.read8(r2);

        return @as(u16, reg1) << 8 | @as(u16, reg2);
    }

    fn write88(self: *Self, r1: Register8, r2: Register8, val: u16) void {
        const v1 = @truncate(u8, (val & 0xFF00) >> 8);
        const v2 = @truncate(u8, val & 0xFF);

        self.write8(r1, v1);
        self.write8(r2, v2);
    }

    fn read16(self: Self, r: Register16) u16 {
        const idx: usize = @typeInfo(Register8).Enum.fields.len + 2 * @enumToInt(r);
        const v1: u16 = self.rgs[idx + 1];
        const v2: u16 = self.rgs[idx];

        //z.debug.warn("=== ({}) {} {} => {}\n", .{idx, v1, v2, v1 << 8 | v2});

        return v1 << 8 | v2;
    }

    fn write16(self: *Self, r: Register16, val: u16) void {
        const reg = switch (r) {
            .pc => &self.registers().pc,
            .sp => &self.registers().sp,
        };

        reg.* = val;
    }

    const MmappedAddress = enum {
        // 16KB ROM bank 0 {
        RestartAndInterruptVectors,
        CartridgeHeader,
        CartridgeROMBank0,
        // }
        CartridgeROMSwitchableBanks, // Switchable via MBC (if any)
        CharacterRAM,
        BGMapData1,
        BGMapData2,
        ExternalRAM, // Cartridge RAM (if available, switchable bank)
        WorkRAMBank0, // Internal RAM, Bank 0
        WorkRAMBank1, // Internal RAM, Bank 1 (1-7 in CGB mode)
        EchoRAM,
        SpriteAttributeTable, // Object Attribute Memory (OAM)
        Unusable,
        IORegisters,
        HighRAM, // Zero Page
        InterruptsEnableRegister,
    };

    fn mmap(_: Self, addr: usize) MmappedAddress {
        return switch (addr) {
            0x0000...0x00FF => .RestartAndInterruptVectors,
            0x0100...0x014F => .CartridgeHeader,
            0x0150...0x3FFF => .CartridgeROMBank0,
            0x4000...0x7FFF => .CartridgeROMSwitchableBanks,

            0x8000...0x97FF => .CharacterRAM,
            0x9800...0x9BFF => .BGMapData1,
            0x9C00...0x9FFF => .BGMapData2,

            0xA000...0xBFFF => .ExternalRAM,

            0xC000...0xCFFF => .WorkRAMBank0,
            0xD000...0xDFFF => .WorkRAMBank1,

            0xE000...0xFDFF => .EchoRAM,

            0xFE00...0xFE9F => .SpriteAttributeTable,

            0xFEA0...0xFEFF => .Unusable,

            0xFF00...0xFF7F => .IORegisters,
            0xFF80...0xFFFE => .HighRAM,
            0xFFFF => .InterruptsEnableRegister,

            else => unreachable,
        };
    }

    fn mmapRead(_: Self, _: usize) u8 {
        unreachable;
    }

    fn mmapWrite(self: *Self, addr: usize, val: u8) void {
        switch (self.mmap(addr)) {
            .BGMapData2 => self.vram.data[addr - 0x8000] = val,

            else => unreachable,
        }
    }

    pub fn exec(self: *Self, instr: []const u8) void {
        //var parsedInstrs = self.ram.parseInstructions();

        //z.debug.warn("{}\n", .{self});
        while (true) {
            const instruction = Instruction.parse(
                instr,
                self.read16(.pc),
                self.pull_from_extended,
            );

            if (self.pull_from_extended) self.pull_from_extended = false;

            self.execInstr(instruction, instr);
            l.warn("{}\n", .{self});
        }
    }

    fn incrementValByAmount(val: anytype, amnt: anytype) @TypeOf(val) {
        return switch (@typeInfo(@TypeOf(amnt))) {
            .ComptimeInt => if (amnt > 0)
                val +% amnt
            else
                val -% (-amnt),

            else => val +% amnt,
        };
    }

    fn inc(self: *Self, reg: anytype, amnt: anytype) void {
        switch (@typeInfo(@TypeOf(reg))) {
            .Array => |arr| {
                if (arr.child != Register8) {
                    @compileError("Expected an array of Register8, got '" ++
                        @typeName(arr.child) ++ "'");
                }

                if (arr.len != 2) {
                    @compileError("Expected an array of length two");
                }

                self.write88(
                    reg[0],
                    reg[1],
                    incrementValByAmount(self.read88(reg[0], reg[1]), -1),
                );
            },

            .EnumLiteral => if (@hasField(Register8, @tagName(reg))) {
                self.write8(
                    reg,
                    incrementValByAmount(self.read8(reg), amnt),
                );
            } else if (@hasField(Register16, @tagName(reg))) {
                self.write16(
                    reg,
                    incrementValByAmount(self.read16(reg), amnt),
                );
            } else {
                @compileError("Expected a Register8 or a Register16" ++
                    " enum literal, but none of these has a field '");
            },

            else => @compileError("Expected a register, got '" ++
                @typeName(reg) ++ "'"),
        }
    }

    fn execInstr(self: *Self, instr: Instruction, instrs: []const u8) void {
        l.warn("{}\n", .{instr});
        switch (instr.class) {
            .add => {
                switch (instr.dst.?) {
                    .r8 => self.instrAdd8(instr),
                    .r88 => self.instrAdd88(instr),

                    else => unreachable,
                }
            },

            .ld => self.instrLd(instr, instrs),

            .ldd => {
                self.instrLd(instr, instrs);

                self.inc([_]Register8{ .h, .l }, -1);
            },

            .ldi => {
                self.instrLd(instr, instrs);

                self.inc([_]Register8{ .h, .l }, 1);
            },

            .xor => self.instrXor(instr),

            .bit => {
                // which bit to check
                const bit = instr.dst.?.bit;

                // The BIT instruction set always take an 8-bit register or the
                // value pointed by HL as src.
                const num = num: {
                    break :num switch (instr.src.?) {
                        .r8 => |reg| self.read8(reg),

                        //.r16 => |hl_ptr| unreachable, // TODO: unimplemented
                        .r16 => |_| unreachable, // TODO: unimplemented

                        else => unreachable,
                    };
                };

                const flags = self.flagRegister();
                flags.zero = 0 == num & (@as(u16, 1) << @intCast(u4, bit));
                flags.subtract = false;
                flags.half_carry = true;
                // flags.carry is unmodified

                self.inc(.pc, 1);
            },

            .extended => {
                self.pull_from_extended = true;
                self.inc(.pc, 1);
            },

            else => unreachable,
        }
    }

    fn instrXor(self: *Self, instr: Instruction) void {
        switch (instr.dst.?) {
            .r8 => {
                const dst_reg = instr.dst.?.r8;
                const src_reg = instr.src.?.r8;

                const dst = self.read8(dst_reg);
                const src = blk: {
                    if (instr.src_ptr) unreachable;
                    break :blk self.read8(src_reg);
                };

                self.write8(dst_reg, dst ^ src);
                self.inc(.pc, 1);

                // Set the zero flag.
                self.flagRegister().zero = self.read8(dst_reg) == 0;
            },

            else => unreachable,
        }

        const flags = self.flagRegister();
        // flags.zero is set above
        flags.subtract = false;
        flags.half_carry = false;
        flags.carry = false;
    }

    fn instrLd(self: *Self, instr: Instruction, instrs: []const u8) void {
        switch (instr.dst.?) {
            .r88 => self.instrLd88(instr, instrs),
            .r16 => self.instrLd16(instr, instrs),
            else => unreachable,
        }

        // Flags aren't affected by these instructions.
    }

    fn instrLd88(self: *Self, instr: Instruction, _: []const u8) void {
        //const pc = self.read16(.pc);
        var pc_inc: u16 = 1;

        //const dst_rgs = instr.dst.?.r88;

        const dst = instr.dst.?.r88;
        const src: u16 = switch (instr.src.?) {
            .im => |im| im_val: {
                switch (im) {
                    .nn => {
                        pc_inc = 3;
                        break :im_val instr.immediate_val.?.nn;
                    },

                    else => unreachable,
                }
            },

            .r8 => |r| self.read8(r),

            else => unreachable,
        };

        if (instr.dst_ptr) {
            // http://gameboy.mongenel.com/dmg/asmmemmap.html
            const ptr = self.read88(.h, .l);
            self.mmapWrite(ptr, @intCast(u8, src));
        } else {
            self.write88(dst[0], dst[1], src);
        }

        self.inc(.pc, pc_inc);
    }

    fn instrLd16(self: *Self, instr: Instruction, _: []const u8) void {
        const pc = self.read16(.pc);

        const dst = instr.dst.?.r16;
        const src = blk: {
            break :blk switch (instr.src.?) {
                .im => |im| switch (im) {
                    .nn => instr.immediate_val.?.nn,

                    else => unreachable,
                },

                else => unreachable,
            };
        };

        self.write16(dst, src);
        self.write16(.pc, pc + 3);
    }

    fn instrAdd8(self: *Self, instr: Instruction) void {
        const dst_reg = instr.dst.?.r8;
        const src_reg = instr.src.?.r8;

        const dst_val = self.read8(dst_reg);
        const src_val = self.read8(src_reg);

        var new_val: u8 = undefined;
        const overflow = @addWithOverflow(u8, dst_val, src_val, &new_val);

        var flgs = self.flagRegister();
        flgs.zero = new_val == 0;
        flgs.subtract = false;
        flgs.half_carry = (dst_val & 0x0F) + (src_val & 0x0F) > 0x0F;
        flgs.carry = overflow;

        self.write8(dst_reg, new_val);
    }

    fn instrAdd88(self: *Self, instr: Instruction) void {
        const dst1 = instr.dst.?.r88[0];
        const dst2 = instr.dst.?.r88[1];
        const src1 = instr.src.?.r88[0];
        const src2 = instr.src.?.r88[1];

        const src_val = self.read88(src1, src2);
        const dst_val = self.read88(dst1, dst2);

        var new_val: u16 = undefined;
        const overflow = @addWithOverflow(u16, dst_val, src_val, &new_val);

        var flgs = self.flagRegister();
        flgs.zero = new_val == 0;
        flgs.subtract = false;
        flgs.half_carry = (dst_val & 0x00FF) + (src_val & 0x00FF) > 0x00FF;
        flgs.carry = overflow;

        self.write88(dst1, dst2, new_val);
    }

    pub fn format(
        value: Self,
        comptime _: []const u8,
        options: z.fmt.FormatOptions,
        out_stream: anytype,
    ) !void {
        try out_stream.writeAll(@typeName(Self));
        try out_stream.writeAll(" {\n");
        try @ptrCast(*const Registers, &value.rgs).format(
            "\t",
            options,
            out_stream,
        );
        try out_stream.writeAll("}");
    }
};

test "Read from and write to registers" {
    var cpu = Sharp_LR35902.init();

    // Make sure all registers are initialized to zero
    inline for (@typeInfo(Register8).Enum.fields) |r| {
        expectEq(cpu.read8(@intToEnum(Register8, r.value)), 0);
    }

    var regs = @ptrCast(*Registers, &cpu.rgs);

    // Making sure the casting works
    regs.a = 1;
    expectEq(cpu.read8(.a), 1);

    regs.l = 255;
    expectEq(cpu.read8(.l), 255);

    //z.debug.warn("\npc: {}, sp: {}, total: {}\n", .{
    //    @memberCount(Register8) + 2*@enumToInt(Register16.pc),
    //    @memberCount(Register8) + 2*@enumToInt(Register16.sp),
    //    TOTAL_REGISTER_SIZE,
    //});

    //{
    //    z.debug.warn("\n", .{});
    //    var i: usize = 0;
    //    while(i < cpu.rgs.len) : (i += 1) {
    //        z.debug.warn("{}, ", .{cpu.rgs[i]});
    //    }
    //    z.debug.warn("\n", .{});
    //}

    regs.pc = 427;
    //z.debug.warn("\n{} {}\n", .{ cpu.read16(.pc), regs.pc });
    expectEq(cpu.read16(.pc), 427);

    regs.sp = 512;
    //z.debug.warn("\n{} {}\n", .{ cpu.read16(.sp), regs.sp });
    expectEq(cpu.read16(.sp), 512);
}

test "All them instructions" {
    var cpu = Sharp_LR35902.init();

    // ADD register B to register A
    {
        const instr = Instruction.parse(&[_]u8{0x80}, 0);

        var regs = cpu.registers();
        regs.a = 0b0001;
        regs.b = 0b0010;

        cpu.instrAdd8(instr);

        expectEq(regs.a, 0b0011);
        expect(!regs.f.carry);
        expect(!regs.f.half_carry);
        expect(!regs.f.subtract);
        expect(!regs.f.zero);
    }

    cpu.reset();

    // ADD B, A with half carry
    {
        const instr = Instruction.parse(&[_]u8{0x80}, 0);

        var regs = cpu.registers();
        regs.a = 0b1001;
        regs.b = 0b1010;

        cpu.instrAdd8(instr);

        expectEq(regs.a, 0b0001_0011);
        expect(!regs.f.carry);
        expect(regs.f.half_carry);
        expect(!regs.f.subtract);
        expect(!regs.f.zero);
    }

    // ADD B, A with carry
    {
        const instr = Instruction.parse(&[_]u8{0x80}, 0);

        var regs = cpu.registers();
        regs.a = 0b1010_1001;
        regs.b = 0b1000_1010;

        cpu.instrAdd8(instr);

        expectEq(regs.a, 0b0011_0011);
        expect(regs.f.carry);
        expect(regs.f.half_carry);
        expect(!regs.f.subtract);
        expect(!regs.f.zero);
    }

    // ADD B, A with carry, resulting in a zero
    {
        const instr = Instruction.parse(&[_]u8{0x80}, 0);

        var regs = cpu.registers();
        regs.a = 0b1000_0000;
        regs.b = 0b1000_0000;

        cpu.instrAdd8(instr);

        expectEq(regs.a, 0b0000_0000);
        expect(regs.f.carry);
        expect(!regs.f.half_carry);
        expect(!regs.f.subtract);
        expect(regs.f.zero);
    }

    {
        const instr = Instruction.parse(&[_]u8{0x80}, 0);

        var regs = cpu.registers();
        regs.a = 0b1000_0000;
        regs.b = 0b1000_0000;

        cpu.instrAdd8(instr);

        expectEq(regs.a, 0b0000_0000);
        expect(regs.f.carry);
        expect(!regs.f.half_carry);
        expect(!regs.f.subtract);
        expect(regs.f.zero);

        regs.a = 0x00FF;
        regs.b = 0x00FF;
        regs.c = 0x00FF;
        regs.d = 0x00FF;
        cpu.execInstr(.{
            // irrelevant fields for this test
            .opcode = 0x00,
            .cycles = 0,

            // what we care about
            .class = .add,
            .dst = .{ .r88 = .{ .a, .b } },
            .src = .{ .r88 = .{ .c, .d } },
        }, &[_]u8{0});

        expect(regs.f.half_carry);
    }
}
