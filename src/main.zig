// https://github.com/gbdev/awesome-gbdev#emulator-development
//
// reference emulator: https://github.com/binji/binjgb
//
// THE gb talk: https://invidio.us/watch?v=HyzD8pNlpwI
//
// https://gbdev.gg8.se/wiki/articles/Gameboy_Bootstrap_ROM

// https://github.com/Immediate-Mode-UI/Nuklear

const z = @import("std");

const Gameboy = @import("gameboy.zig").Gameboy;
const Instruction = @import("instruction.zig").Instruction;

const SCREEN = .{
    .width = 160,
    .height = 144,
    .vblank = 10,
};

const SCALE: usize = 1;

const SCALED_SCREEN = .{
    .width = SCREEN.width * SCALE,
    .height = SCREEN.height * SCALE,
    .vblank = SCREEN.vblank,
};

pub fn main() anyerror!void {
    const Gui = @import("ui.zig").Gui;
    var gui = try Gui.init("testing", 300, 300, .{});
    defer gui.deinit();
    // try gui.loop();

    var gb = Gameboy.init();
    gb.loadBootRom();
    gb.run();

    z.log.warn("{}\n", .{gb.cpu});
}
