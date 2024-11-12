const z = @import("std");
const Timer = z.time.Timer;

const s = @import("c.zig").sdl;

pub const Gui = struct {
    const Self = @This();

    done: bool = false,

    win: *s.SDL_Window,
    ren: *s.SDL_Renderer,

    screen_vp: s.SDL_Rect,

    const l = z.log.scoped(.gui);

    pub const InitOptions = struct {
        window_flags: u32 = s.SDL_WINDOW_SHOWN,
        renderer_idx: c_int = -1,
        renderer_flags: u32 = 0,
    };

    pub fn init(
        title: [*c]const u8,
        width: c_int,
        height: c_int,
        options: InitOptions,
    ) !Self {
        if (s.SDL_Init(s.SDL_INIT_EVERYTHING) != 0) return error.SdlInit;
        errdefer s.SDL_Quit();

        const window = s.SDL_CreateWindow(
            title,
            s.SDL_WINDOWPOS_UNDEFINED,
            s.SDL_WINDOWPOS_UNDEFINED,
            width,
            height,
            options.window_flags,
        );
        if (window == null) return error.SdlCreateWindow;
        errdefer s.SDL_DestroyWindow(window);

        const renderer = s.SDL_CreateRenderer(
            window,
            options.renderer_idx,
            options.renderer_flags,
        );
        if (renderer == null) return error.CreateRenderer;
        errdefer s.SDL_DestroyRenderer(renderer);

        // Setup audio
        {
            const want = s.SDL_AudioSpec{
                .freq = 60,
                .format = s.AUDIO_S16LSB,
                .channels = 2,
                .samples = 4096,

                .silence = 0,
                .padding = 0,
                .size = 0,
                .userdata = null,

                .callback = null,
            };

            var got = z.mem.zeroes(s.SDL_AudioSpec);

            const device = s.SDL_OpenAudioDevice(null, 0, &want, &got, 0);
            if (device == 0) {
                return error.SdlOpenAudioDevice;
            }

            l.warn("Audio: {}\n", .{got});
        }

        return Self{
            .win = window.?,
            .ren = renderer.?,

            .screen_vp = .{
                .x = 0,
                .y = 0,
                .w = width,
                .h = height,
            },
        };
    }

    pub fn deinit(self: *Self) void {
        s.SDL_DestroyRenderer(self.ren);
        s.SDL_DestroyWindow(self.win);
        s.SDL_Quit();
    }

    pub fn loop(self: *Self) !void {
        const TARGET_FRAME_TIME = @truncate(u64, 1_000_000_000 / 60);

        var timer = try Timer.start();

        while (!self.done) {
            timer.reset();

            self.handleEvents();
            try self.render();

            const ns = timer.lap();
            z.debug.warn("{}\n", .{ns});

            if (ns < TARGET_FRAME_TIME) z.time.sleep(TARGET_FRAME_TIME - ns);

            s.SDL_RenderPresent(self.ren);
        }
    }

    fn handleEvents(self: *Self) void {
        var evnt: s.SDL_Event = undefined;

        while (s.SDL_PollEvent(&evnt) != 0) {
            switch (evnt.@"type") {
                s.SDL_QUIT => self.done = true,

                s.SDL_KEYDOWN => self.keydown(evnt.key.keysym),

                else => {},
            }
        }
    }

    fn keydown(self: *Self, key: s.SDL_Keysym) void {
        switch (key.sym) {
            s.SDLK_ESCAPE, s.SDLK_q => self.done = true,

            else => {},
        }
    }

    fn render(self: Self) !void {
        if (s.SDL_SetRenderDrawColor(self.ren, 0, 255, 0, 0) != 0) {
            return error.SdlSetDrawColor;
        }

        if (s.SDL_RenderClear(self.ren) != 0) {
            return error.SdlRenderClear;
        }
    }
};
