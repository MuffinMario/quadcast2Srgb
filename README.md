# Quadcast 2S RGB
Program to control the 108 individual LEDS on the HyperX Quadcast 2S RGB. Made initially, because there is no tool for this on Linux... and the LED defaults to a rolling animated rainbow.

Currently still in early development, however it has matured enough to be able to be used casually with minimal CPU consumption in mind.

# Installation

## Windows

TBD, probably going to be releasing a binary in Releases with a suggested path to build it yourself here...

## Linux

### Arch / AUR
There currently is a PKGBUILD to build the project based on the latest release via `makepkg -si` in `packages/PKGBUILD`. Will add it to AUR soon...

### Debian
TBD

### Manual build
You can build the program yourself, simply by going into the root directory and running:

```
# install required packages
./packages.sh

# cmake build the project
cmake -B build -S . \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_INSTALL_PREFIX=/usr \
-DUSE_SYSTEMD=OFF \
-DENABLE_CLANG_TIDY=OFF 

cmake --build build
```
This will build the project without any systemd notification/watchdog features, you can turn them on again of course again by instead writing `-DUSE_SYSTEMD=ON`

# Usage

## Service based (systemctl)
The package installation comes with a user based service available to launch.
You may need to first reload the daemon for it to be detected:
`systemctl --user daemon-reload`

To check the status of the service
`systemctl --user status quadcast2srgb`
**It is recommended to test the functionality by directly calling the program first.**
 For this, run `qc2srgb` (no arguments) which will try to find all QC2S devices and display a test color (deep indigo) on them. If you see no errors, and instead a message such as `[Handshake] Successfully connected to device, adding to communicator pipeline. Serial: <serial>`, the service should work just as well.

 After this seems to work, you can go ahead and start/stop/enable the service to your liking.
 `systemctl --user start quadcast2srgb`
 `systemctl --user enable quadcast2srgb`
 `systemctl --user stop quadcast2srgb`

 The service will invoke a command similar to `qc2srgb --config /etc/quadcast2srgb/config.toml`

## Direct use via tool
For the full argument reference run:
```sh
qc2srgb --help
```

### Via config file
Config files are the recommended way to use this tool. The linux package installation provides a user level service that can be enabled via systemctl (See further down in this readme) to automatically set up a more complex set up. Configs are the **only** way you can chain multiple displays with your own customized order and timings. More about configs can be found in the next section as well. To use a config:
```sh
# Load a TOML config — supports multiple sequential displays, all display types, etc.
qc2srgb --config ~/.config/quadcast2srgb/config.toml

# Same, but log extra diagnostic output
qc2srgb --config ~/.config/quadcast2srgb/config.toml --verbose
```

### Solid color
```sh
# Deep indigo (default color)
qc2srgb --display solid --color 290066

# Colors can also be specified with a # and alpha byte (will be ignored; mainly just so you can use the vscode color picker )
qc2srgb --display solid --color '#ff0000ff'
```

### Pulsing brightness
Pulse displays can display colors in pulsating brightness. Optionally with customizable speed, and a customizable bezier curve. (See e.g. https://cubic-bezier.com ):
```sh
# Slow indigo pulse 
qc2srgb --display pulse --color 290066

# With decreased pulse speed (default = 0.025)
qc2srgb --display pulse --color 290066 --pulse-speed 0.015

# With a custom bezier curve to change brightness progression throughout the pulse
qc2srgb --display pulse --color ff1020 --pulse-speed 0.04 --pulse-cubic-bezier 0.4 0.0 0.6 1.0
```

### Video-driven LEDs
Since the QC2S technically has a 12x9 display, it can also be used to display videos. Now, most videos in fact are not made for this tiny resolution, but if you manage to find something that you may want to display, you are able to using the video display:
```sh
# Play a raw RGB video at 24 fps
qc2srgb --display video --video-path ~/leds.raw --video-framerate 24

# Greyscale source file
qc2srgb --display video --video-path ~/leds_grey.raw --video-colors greyscale --video-framerate 30
```

To obtain a raw 12x9 video, one may be able to use ffmpeg, e.g. ```
ffmpeg -i bad_apple.mp4 \
        -vf "fps=30,scale=12:9:flags=neighbor:eval=frame" \
        -pix_fmt rgb24 \
        -f rawvideo \
        badapple_12x9.raw```

### Restricting to specific devices
In case you have multiple devices which you want to run this tool separately on (per-default it syncs to all devices) you can specify the serial which you can find either via verbose logging (--verbose) or by looking under the physical stand of your microphone:
```sh
# Only control the device with serial ABCDE123
qc2srgb --display solid --color 290066 --serial ABCDE123

# Control two specific devices at once
qc2srgb --display pulse --serial ABCDE123 --serial XYZ99887
```
# Config Syntax

Config files are [TOML](https://toml.io) documents. They are the only way to chain multiple displays in sequence with custom timings.

---

## Top-level keys

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `startup-display` | string | **yes** | `<empty>` | `name` of the `[[display]]` entry to start from |
| `verbose` | bool | no | `false` | Enable verbose logging (same as `--verbose`) |
| `allowed-serials` | array of strings | no | `[]` (all) | Restrict to these device serial numbers |

```toml
startup-display = "MyDisplay"
verbose = false
allowed-serials = ["12345678", "87654321"]
```

---

## `[[display]]` entries

Each display is a TOML array-of-tables entry. You can define as many as you like and link them into any graph (including cycles for infinite loops).

### Keys shared by all display types

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `type` | string | **yes** | — | Display type: `"solid"`, `"pulse"`, `"pulse-color"`, `"video"` |
| `name` | string | **yes** | — | Unique identifier used by `startup-display` and `next-display` |
| `next-display` | string | no | `""` (stop) | `name` of the display to transition to when this one ends |
| `end-condition` | inline table | no | none (loop forever) | When to stop this display and move to `next-display` |

#### `end-condition` variants

```toml
# Stop after a fixed duration
end-condition = { type = "time", duration-ms = 5000 }

# Stop when the video file ends (only valid for type = "video")
end-condition = { type = "video-end" }
# Additionally: specify the amount of loops before transitioning to the next display
end-condition = { type = "video-end", loop-count = 5 }
```

If `end-condition` is omitted, the display loops indefinitely. If `next-display` is also omitted or empty, the program ends after this display finishes.

---

### `type = "solid"` — solid color

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `color` | string | no | `"#290066"` | RGB hex color. Leading `#` and a trailing alpha byte are both accepted and the alpha is ignored |

```toml
[[display]]
type  = "solid"
name  = "MyColor"
color = "#ff0000"
end-condition = { type = "time", duration-ms = 5000 }
next-display  = "NextDisplay"
```

---

### `type = "pulse"` — pulsing brightness on a single hue

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `color` | string | no | `"#290066"` | Hue to pulse |
| `pulse-speed` | float | no | `0.025` | Phase advance per frame (20 frames/s). Larger → faster |
| `bezier` | array of 4 floats | no | `[0.11, 0.0, 0.35, 1.0]` | CSS cubic-bézier control points `[p1x, p1y, p2x, p2y]` — see https://cubic-bezier.com |

```toml
[[display]]
type        = "pulse"
name        = "SlowPulse"
color       = "#290066"
pulse-speed = 0.015
bezier      = [0.11, 0.0, 0.35, 1.0]
```

---

### `type = "video"` — raw video file

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `video-path` | string | **yes** | — | Absolute or relative path to a raw video file |
| `video-framerate` | integer | no | `30` | Playback frame rate in FPS |
| `video-colors` | string | no | `"rgb"` | Pixel format: `"rgb"` (3 bytes/pixel) or `"greyscale"` (1 byte/pixel) |

The expected resolution is **12 × 9** pixels (108 LEDs). See the ffmpeg snippet in the [Video-driven LEDs](#video-driven-leds) section for how to produce a compatible file. Supports `end-condition = { type = "video-end" }` to automatically move to the next display when the file ends.

```toml
[[display]]
type            = "video"
name            = "BadApple"
video-path      = "/home/user/badapple_12x9.raw"
video-framerate = 30
video-colors    = "rgb"
end-condition   = { type = "video-end" }
next-display    = "NextDisplay"
```

---

## Example — two solid colors, 5 seconds each

Alternates between indigo and crimson forever.

```toml
startup-display = "Indigo"

[[display]]
type          = "solid"
name          = "Indigo"
color         = "#290066"
end-condition = { type = "time", duration-ms = 5000 }
next-display  = "Crimson"

[[display]]
type          = "solid"
name          = "Crimson"
color         = "#861e27"
end-condition = { type = "time", duration-ms = 5000 }
next-display  = "Indigo"
```

# What else to implement?
- GLSL shader?
- Capture input/output devices -> response / interface for further displays
- Lua(JIT) interface for customization (including all features of the program)
- color transitions with pulse (e.g. #ff0000 to -> #00ff00)