const z = @import("std");

const Instruction = @import("instruction.zig").Instruction;

const RAM_SIZE: usize = 8 * 1024; // 8KB (0xC000 -> 0xDFFF)

const RAM_CLOCK: usize = 1 * 1024 * 1024; // 1 MHz
const VRAM_CLOCK: usize = 2 * 1024 * 1024; // 2 MHz

// This chip is used for RAM and VRAM
pub const Sharp_LH5264N4 = struct {
    const Self = @This();

    data: [RAM_SIZE]u8,
    parsed_instructions: [RAM_SIZE]?Instruction,

    pub fn init() Self {
        var new_chip = Self{
            .data = undefined,
            .parsed_instructions = undefined,
        };

        new_chip.reset();

        return new_chip;
    }

    pub fn reset(self: *Self) void {
        for (self.data) |*byte| {
            byte.* = 0;
        }

        for (self.parsed_instructions) |*instr| {
            instr.* = null;
        }
    }

    pub fn parseInstructions(self: *Self) *[RAM_SIZE]?Instruction {
        var parsd_idx: usize = 0;
        var instr_idx: usize = 0;

        while (instr_idx < RAM_SIZE) {
            const curr_instr = &self.parsed_instructions[parsd_idx];
            curr_instr.* = Instruction.parse(
                &self.data,
                instr_idx,
                parsd_idx > 0 and
                    self.parsed_instructions[parsd_idx - 1].?.class ==
                    .extended,
            );

            const instruction_jump = 1 + blk: {
                if (curr_instr.*.?.dst) |dst| {
                    switch (dst) {
                        .im => |im| break :blk im.toSize() / 8,
                        else => {},
                    }
                }

                if (curr_instr.*.?.src) |src| {
                    switch (src) {
                        .im => |im| break :blk im.toSize() / 8,
                        else => {},
                    }
                }

                break :blk 0;
            };

            parsd_idx += 1;
            instr_idx += instruction_jump;
        }

        return &self.parsed_instructions;
    }
};
