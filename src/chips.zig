pub const Register8 = @import("chips/register8.zig").Register8;
pub const Register16 = @import("chips/register16.zig").Register16;
pub const Sharp_LH5264N4 = @import("chips/sharp_lh5264n4.zig").Sharp_LH5264N4;
pub const Sharp_LR35902 = @import("chips/sharp_lr35902.zig").Sharp_LR35902;

const PPU_CLOCK: usize = 4 * 1024 * 1024; // 4 MHz

const SCREEN_RESOLUTION = .{
    .x = 160,
    .y = 144,
};
