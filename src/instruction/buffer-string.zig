const z = @import("std");

pub const BufferString = struct {
    const Self = @This();

    data: []u8,
    ptr: usize,

    pub fn init(buffer: []u8) Self {
        return .{
            .data = buffer,
            .ptr = 0,
        };
    }

    pub fn append(self: *Self, char: u8) void {
        z.debug.assert(self.ptr < self.data.len);

        self.data[self.ptr] = char;
        self.ptr += 1;
    }

    pub fn appendSlice(self: *Self, slice: []const u8) void {
        for (slice) |ch| self.append(ch);
    }

    pub fn str(self: Self) []u8 {
        return self.data[0..self.ptr];
    }
};
