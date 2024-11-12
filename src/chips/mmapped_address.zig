pub const MmappedAddress = enum {
    const Self = @This();

    // 16KB ROM bank 0 {
    restart_and_interrupt_vectors,
    cartridge_header,
    cartridge_rom_bank_0,
    // }
    cartridge_rom_switchable_banks, // Switchable via MBC (if any)
    character_ram,
    bg_map_data_1,
    bg_map_data_2,
    external_ram, // Cartridge RAM (if available, switchable bank)
    work_ram_bank_0, // Internal RAM, Bank 0
    work_ram_bank_1, // Internal RAM, Bank 1 (1-7 in CGB mode)
    echo_ram,
    sprite_attribute_table, // Object Attribute Memory (OAM)
    unusable,
    io_registers,
    high_ram, // Zero Page
    interrupts_enable_register,

    pub fn fromAddress(addr: usize) !Self {
        return switch (addr) {
            0x0000...0x00FF => .restart_and_interrupt_vectors,
            0x0100...0x014F => .cartridge_header,
            0x0150...0x3FFF => .cartridge_rom_bank_0,
            0x4000...0x7FFF => .cartridge_rom_switchable_banks,

            0x8000...0x97FF => .character_ram,
            0x9800...0x9BFF => .bg_map_data_1,
            0x9C00...0x9FFF => .bg_map_data_2,

            0xA000...0xBFFF => .external_ram,

            0xC000...0xCFFF => .work_ram_bank_0,
            0xD000...0xDFFF => .work_ram_bank_1,

            0xE000...0xFDFF => .echo_ram,

            0xFE00...0xFE9F => .sprite_attribute_table,

            0xFEA0...0xFEFF => .unusable,

            0xFF00...0xFF7F => .io_registers,
            0xFF80...0xFFFE => .high_ram,
            0xFFFF => .interrupts_enable_register,

            // All addresses should be mapped above, if it gets something else,
            // we have a serious problem.
            else => error.unknown_address,
        };
    }
};
