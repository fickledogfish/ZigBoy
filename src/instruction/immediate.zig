// Some instructions depend on immediate n-bit data, this encodes how much
// should be loaded
pub const Immediate = enum {
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

pub const ImmediateVal = union(enum) {
    n: u8,
    nn: u16,
};
