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
