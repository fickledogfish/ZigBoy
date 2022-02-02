const z = @import("std");
const chips = @import("../chips.zig");

const String = @import("buffer-string.zig").BufferString;
const Register8 = chips.Register8;
const Register16 = chips.Register16;
const Immediate = @import("immediate.zig").Immediate;
const ImmediateVal = @import("immediate.zig").ImmediateVal;

pub const Target = union(enum) {
    const Self = @This();

    r8: Register8,
    r88: [2]Register8,
    r16: Register16,
    im: Immediate,
    addr: usize,
    bit: u8, // 0-7

    pub fn stringfy(
        self: Self,
        buffer: []u8,
        is_ptr: bool,
        immediate_val: ?ImmediateVal,
    ) String {
        const toUpper = z.ascii.toUpper;

        var str = String.init(buffer);

        if (is_ptr) {
            str.append('(');
        }

        switch (self) {
            .r8 => |val| {
                str.append(toUpper(@tagName(val)[0]));
            },

            .r88 => |val| {
                str.append(toUpper(@tagName(val[0])[0]));
                str.append(toUpper(@tagName(val[1])[0]));
            },

            .r16 => |val| {
                for (@tagName(val)) |ch| {
                    str.append(toUpper(ch));
                }
            },

            .im => {
                z.debug.assert(immediate_val != null);

                var buf: [@typeInfo(usize).Int.bits]u8 = undefined;
                var end: usize = undefined;

                switch (immediate_val.?) {
                    .n => |n| end = z.fmt.formatIntBuf(
                        &buf,
                        n,
                        16,
                        .upper,
                        z.fmt.FormatOptions{},
                    ),

                    .nn => |n| end = z.fmt.formatIntBuf(
                        &buf,
                        n,
                        16,
                        .upper,
                        z.fmt.FormatOptions{},
                    ),
                }

                str.appendSlice(buf[0..end]);
            },

            .addr => |val| {
                var numbuf: [@typeInfo(usize).Int.bits]u8 = undefined;

                const end = z.fmt.formatIntBuf(
                    &numbuf,
                    val,
                    16,
                    .upper,
                    z.fmt.FormatOptions{},
                );

                str.appendSlice(numbuf[0..end]);
            },

            .bit => |val| {
                str.append(val + 48);
            },
        }

        if (is_ptr) {
            str.append(')');
        }

        return str;
    }
};
