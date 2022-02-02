const components = @import("components.zig");

const RamChip = components.Sharp_LH5264N4;
const VramChip = components.Sharp_LH5264N4;
const CpuChip = components.Sharp_LR35902;

pub const Gameboy = struct {
    const Self = @This();

    ram: RamChip,
    vram: RamChip,
    cpu: CpuChip,

    pub fn init() Self {
        var ram = RamChip.init();
        var vram = VramChip.init();

        return .{
            .ram = ram,
            .vram = vram,
            .cpu = CpuChip.init(&ram, &vram),
        };
    }

    pub fn reset(self: *Self) void {
        self.ram.reset();
        self.vram.reset();
        self.cpu.reset();
    }

    pub fn loadBootRom(self: *Self) void {
        for (CpuChip.BOOT_ROM) |byte, idx| self.ram.data[idx] = byte;
    }

    pub fn run(self: *Self) void {
        // TODO: parse before run
        self.cpu.exec(&self.ram.data);
    }
};
