// The 16-bit registers of the Gameboy's CPU. For the combinations of Register8
// into a single 16-bit register, check the (read,write)88 functions in the
// Sharp_LR35902 struct.
pub const Register16 = enum(usize) {
    sp, // Stack Pointer (points to the current stack position)
    pc, // Program Counter (points to the next instruction to be executed)
};
