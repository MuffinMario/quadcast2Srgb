# Quadcast 2S RGB
Program to control the 108 individual LEDS on the HyperX Quadcast 2S RGB. Made initially, because there is no tool for this on Linux... and the LED defaults to a rolling animated rainbow.

Currently still in early development, however it has matured enough to be able to be used casually with minimal CPU consumption in mind.

> **Disclaimer:** This software is provided as-is, without any warranty of any kind. Use it at your own risk. The author is not responsible for any damage to your hardware, data loss, or any other issues that may arise from using this software.

## Table of Contents
- [Installation](#installation)
- [Getting Started](#setting-up--preparation)
- [Usage](#usage)
  - [Command](#command)
  - [Service based usage (systemctl)](#service-based-usage-systemctl)
- [Config Syntax](#config-syntax)
- [GLSL Shaders](#glsl-shaders)
- [Contributing](#contributing)

# Installation

## Windows

TBD, probably going to be releasing a binary in Releases with a suggested way to build it yourself here...

## Linux

### Arch / AUR

The package can be installed using `yay`:
```sh
yay -Sy quadcast2srgb
```

### Debian
[Releases](https://github.com/MuffinMario/quadcast2Srgb/releases) include a .deb package that you are able to use to install the program & service. 

## Manual build
You can build the program yourself, simply by going into the root directory and running:

### Build binary
```sh
git clone https://github.com/MuffinMario/quadcast2Srgb.git
cd quadcast2Srgb
# install required packages (Windows users can use e.g. vcpkg to install hidapi)
chmod +x packages.sh
./packages.sh

# cmake build the project
cmake -B build -S . \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_INSTALL_PREFIX=/usr \
-DUSE_SYSTEMD=OFF \
-DENABLE_CLANG_TIDY=OFF 

cmake --build build
```

This will build the project without any systemd notification/watchdog features, you can turn them on again of course again by instead writing `-DUSE_SYSTEMD=ON`.

### Build the debian package

Using Docker, you can also build the debian package with the script below, which will output a deb package in `./packages/deb/`
```sh
git clone https://github.com/MuffinMario/quadcast2Srgb.git
cd quadcast2Srgb
chmod +x ./resources/deb/docker-build.sh
./resources/deb/docker-build.sh
```
and installed accordingly with `sudo dpkg -i ./packages/deb/quadcast2srgb_*.deb`


### Manual installation (Linux)

```sh
sudo cmake --install build --prefix /usr/local
sudo systemctl --user daemon-reload

# and start via
systemctl --user enable --now quadcast2srgb
```

### Uninstalling from manual installation (Linux)
This is for MANUAL uninstalling only! It practically only stops the service and removes any related files mentioned in the install manifest.
```
sudo cmake -P build/cmake_uninstall.cmake
```
# Getting Started
After installing, to test if it works, simply run `qc2srgb`. If you see connection success messages and your microphone(s) lighting up to a deep indigo color, you have all the signs that it works.

### Run via config
The preferred method to running this program is by using a config, with the parameter `--config`. The default package comes pre packaged with three configurations in the same folder. To confirm you have the default configs installed at the right path, run `qc2srgb --config /etc/quadcast2srgb/config.toml`. After running this command you should see your microphone light up slowly. For the full documentation on the vast customization options please check below section [Config Syntax](#config-syntax).
### Alternative: Run via args
If you want to quickly play around with the display options at hand, you can also run this tool completely via cli arguments.
E.g. `qc2srgb --display pulse --color 290066` to create a pulsing effect. For the complete options you can check run `qc2srgb -h` or look below at [CLI Arguments](#CLI-Arguments) for the complete documentation.

### Automatically start the service at log in
Once you are satisfied with your configation, you can easily start the service by using the systemd service that comes pre-installed with the package. Simply enable it by running `systemctl --user enable --now quadcast2srgb`.

### Troubleshooting
If you somehow see nothing change on your microphone while the device is plugged in, check the logs by running the program again with the verbose flag `qc2srgb --verbose`. If you see no device being detected, ensure that the udev rule has been successfully installed in `/usr/lib/udev/rules.d/99-quadcast2srgb.rules`. If it still does not work, even using `sudo`, please create a new issue [here](https://github.com/MuffinMario/quadcast2Srgb/issues/new) with the verbose log. Please remember that this program only works with Quadcast 2**S** as it restricts device search to the specific Vendor+Product-ID. 

# CLI Arguments
For the full argument reference run:
```sh
qc2srgb --help
```

### --config
Config files are the recommended way to use this tool. Configs are the **only** way you can chain multiple displays in customized order and timings. The service default config resides in `/etc/quadcast2Srgb/config.toml`. More about configs can be found in the next section as well. 

To use a config:
```sh
# Simply load the config
qc2srgb --config ~/.config/quadcast2srgb/config.toml

# You can turn on verbosity EITHER by the config or directly
qc2srgb --config ~/.config/quadcast2srgb/config.toml --verbose
```

### --display solid
Display a solid color
```sh
# Constantly display a single color
qc2srgb --display solid --color 290066

# Colors can also be specified with a # and alpha byte (will be ignored; mainly just so you can use the vscode color picker )
qc2srgb --display solid --color '#ff0000ff'
```

### --display pulse
Pulse displays can display colors in pulsating brightness. Optionally with customizable speed, and a customizable bezier curve. (See e.g. https://cubic-bezier.com ):
```sh
# Slow indigo pulse 
qc2srgb --display pulse --color 290066

# With decreased pulse speed (default = 0.025)
qc2srgb --display pulse --color 290066 --pulse-speed 0.015

# With a custom bezier curve to change brightness progression throughout the pulse
qc2srgb --display pulse --color ff1020 --pulse-speed 0.04 --pulse-cubic-bezier 0.4 0.0 0.6 1.0
```

### --display video
Since the QC2S technically has a 12x9 display, it can also be used to display videos. Now, most videos in fact are not made for this tiny resolution, but if you manage to find something that you may want to display, you are able to using the video display:
```sh
# Play a raw RGB video at 24 fps
qc2srgb --display video --video-path ~/leds.raw --video-framerate 24

# Greyscale source file
qc2srgb --display video --video-path ~/leds_grey.raw --video-colors greyscale --video-framerate 30
```
Currently there is no functionality to re-scale a video to 12x9 or support common video formats.

To obtain a raw 12x9 video, one may be able to use ffmpeg, e.g. `ffmpeg -i bad_apple.mp4 \
        -vf "fps=30,scale=12:9:flags=neighbor:eval=frame" \
        -pix_fmt rgb24 \
        -f rawvideo \
        badapple_12x9.raw`

### --display rainbow
Displays a rainbow across all LEDs. Supports a static flat mode or several rolling animation modes:
```sh
# Static flat rainbow
qc2srgb --display rainbow

# Rolling rainbow animating vertically
qc2srgb --display rainbow --rainbow-mode vertical

# Rolling diagonal rainbow at double speed in the opposite direction
qc2srgb --display rainbow --rainbow-mode diagonal --rainbow-speed -2.0
```
Available `--rainbow-mode` values: `flat` (default), `vertical`, `horizontal`, `diagonal`.

### --display transition
Smoothly cycles through 2 or more colors (transitioning in HSV space), with a customizable cubic Bézier easing curve applied to each segment. Colors are specified as a comma-separated list of 6-digit hex values:
```sh
# Cycle red → green → blue → red ...
qc2srgb --display transition --transition-colors ff0000,00aa00,0000ff

# With a slower speed and a custom bezier easing curve (see https://cubic-bezier.com)
qc2srgb --display transition --transition-colors ff0000,00aa00,0000ff --transition-speed 0.003 --transition-cubic-bezier 0.4 0.0 0.6 1.0
```
`--transition-speed` controls how quickly the phase advances per frame per color segment (default `0.005`). `--transition-cubic-bezier` takes four floats `p1x p1y p2x p2y` (default ease-in-out: `0.11 0.0 0.35 1.0`).

### --display glsl
Renders an OpenGL ES 3.00 fragment shader to the LED grid. See the [GLSL Shaders](#glsl-shaders) section for shader conventions and an example.
```sh
# Display a shader at 30 fps
qc2srgb --display glsl --shader-path ~/my_effect.glsl

# 4x supersampling (renders to a 48x36 framebuffer), 15 fps
qc2srgb --display glsl --shader-path ~/my_effect.glsl --shader-fps 15 --shader-scale 4
```
GLSL Shaders also support audio capturing! For a complete documentation check the section [GLSL Shaders](#glsl-shaders).

| Argument | Default | Description |
|---|---|---|
| `--shader-path <path>` | <none> (required) | Path to the `.glsl` fragment shader file |
| `--shader-fps <n>` | `30` | Target frame rate. **Warning**: Any value which results in a frame delta of ~30ms or lower (> ~33 fps) may cause unwanted behavior as the microphone controller cannot work off the requests in time. WIP: Once we add a response-wait communication option instead of blindly sending packets, a frame rate above ~33 fps might not be possible at all. |
| `--shader-scale <n>` | `1` | Supersampling scale: renders at `12·n × 9·n` pixels and block-averages down to 12×9; higher values produce smoother color gradients. Anything above 10 is not recommended due to diminishing return for increased computing cost on such a small display screen. Feel free to figure this one out to your needs though; the smaller the better.  |
### --list-audio-devices
List all available PortAudio audio input/output devices with their ID, name, channel counts and sample rate, then exit. Useful for finding the device ID to pass to `--audio-device-id`:
```sh
qc2srgb --list-audio-devices
```

### Audio capture options
This tool also enables you to expose and capture audio devices to display in real time. Display types which support taking in audio data (currently that is: GLSL through use of uniform `u_audioVolume`). To enable exposure and processing of audio devices `--capture-audio` needs to be enabled, or the config equivalent in [Top-level keys](#top-level-keys).

```sh
# Enable audio capture using the default input device
qc2srgb --capture-audio

# Capture from a specific device (see --list-audio-devices for IDs)
qc2srgb --capture-audio --audio-device-id 6

# Analyse only the right channel of a stereo device
qc2srgb --capture-audio --audio-device-id 6 --audio-channel 1

# Boost quiet signals
qc2srgb --capture-audio --input-gain 80.0

# Disable temporal smoothing for raw per-frame FFT data
qc2srgb --capture-audio --no-audio-smoothing

# Faster response with less smoothing
qc2srgb --capture-audio --audio-smoothing-alpha 0.35
```

| Argument | Default | Description |
|---|---|---|
| `--capture-audio` | off | Enable audio capture (without this flag all audio options are ignored) |
| `--audio-device-id <n>` | default input | PortAudio device index to capture from (run `--list-audio-devices` to see available devices) |
| `--audio-channel <n>` | `0` (first) | Which input channel to analyse — `0` = left/first, `1` = right/second, etc. |
| `--input-gain <float>` | `50.0` | Multiplier applied to all frequency bands after FFT. Higher = more sensitive to quiet sounds |
| `--no-audio-smoothing` | off | Disable EMA smoothing; without this, smoothing is enabled by default |
| `--audio-smoothing-alpha <float>` | `0.15` | EMA smoothing coefficient (0–1); higher = faster response, lower = smoother but may appear laggier |
#### Tips on usage with PulseAudio / PipeWire sound server
Unfortunately capturing specific output audio is not as trivial under linux as it is heavily based on your underlying sound server. To set it up first check your devices:
```
> qc2srgb --list-audio-devices 
...
--- PortAudio Devices (13 total) ---
    ...
  10  "pipewire"  API=ALSA  in=128  out=128  rate=44100
  11  "pulse"  API=ALSA  in=32  out=32  rate=44100 <- this is just a compatibility layer when using pipewire
  12 [default in] [default out]  "default"  API=ALSA  in=128  out=128  rate=44100
```
To use either of these servers with an arbitrary audio monitor check your sound server with `pactl list sources short | grep monitor` (PulseAudio) or `wpctl status` (PipeWire).

To capture the audio of the chosen device, the commands would look similar to this then:
```
# pipewire specific device example
> PIPEWIRE_NODE="64" qc2srgb --capture-audio --audio-device-id 10
# pulseaudio specific device example 
> PULSE_SOURCE="alsa_output.<device of your choice>.monitor" qc2srgb  --capture-audio --audio-device-id 11
```
For the documentations of the envvars, please check the respective [PipeWire](https://docs.pipewire.org/page_man_pipewire-client_conf_5.html) or [PulseAudio](https://www.freedesktop.org/wiki/Software/PulseAudio/FAQ) links. Other servers should also be supported, though you are asked to inform yourself on those and their support in PortAudio.

Unfortunately to my finding there is no easy way to implement this to work uniformly out of the box. If you are using the systemd service you need to adjust the service accordingly. Although I am open to be corrected here, as I do not have much experience with linux sound servers and their intricacies :)!
### --serial `serialid`
In case you have multiple devices which you want to run this tool separately on (per-default it syncs to all Quadcast 2S devices on your computer) you can specify the serial which you can find either via verbose logging (--verbose) or by looking under the physical stand of your microphone:
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
| `no-wait-for-read` | bool | no | `false` | Skip waiting for device response (same as `--no-wait-for-read`) |
| `allowed-serials` | array of strings | no | `[]` (all) | Restrict to these device serial numbers |
| `capture-audio` | bool | no | `false` | Enable audio capture (required for any audio keys below to take effect) |
| `device-id` | integer | no | default input | PortAudio device index for audio capture |
| `channel` | integer | no | `0` | Which input channel to analyse (0 = first) |
| `input-gain` | float | no | `50.0` | FFT band / "volume" multiplier; higher = more sensitive |
| `audio-smoothing` | bool | no | `true` | Enable EMA smoothing of the frequency spectrum |
| `audio-smoothing-alpha` | float | no | `0.15` | EMA coefficient (0-1, higher = faster response) |

```toml
startup-display = "MyDisplay"
verbose = false
allowed-serials = ["12345678", "87654321"]

# Audio capture (requires capture-audio = true)
capture-audio = true
device-id = 6
channel = 0
input-gain = 50.0
audio-smoothing = true
audio-smoothing-alpha = 0.15
```

---

## `[[display]]` entries

Each display is a TOML array-of-tables entry. You can define as many as you like and transition them into each other in form of an arbitrary graph (including cycles for infinite loops).

### Common display options

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `type` | string | **yes** | - | Display type: `"solid"`, `"pulse"`, `"pulse-color"`, `"rainbow"`, `"transition"`, `"video"`, `"glsl"` |
| `name` | string | **yes** | - | Unique identifier used by `startup-display` and `next-display` |
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

### `type = "solid"` - solid color

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

### `type = "pulse"` - pulsing brightness on a single hue

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `color` | string | no | `"#290066"` | Hue to pulse |
| `pulse-speed` | float | no | `0.025` | Phase advance per frame (20 frames/s). Larger → faster |
| `bezier` | array of 4 floats | no | `[0.11, 0.0, 0.35, 1.0]` | CSS cubic-bézier control points `[p1x, p1y, p2x, p2y]` -> https://cubic-bezier.com |

```toml
[[display]]
type        = "pulse"
name        = "SlowPulse"
color       = "#290066"
pulse-speed = 0.015
bezier      = [0.11, 0.0, 0.35, 1.0]
```

---

### `type = "rainbow"` - animated or static rainbow

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `rainbow-mode` | string | no | `"flat"` | Animation mode: `"flat"`, `"vertical"`, `"horizontal"`, `"diagonal"` |
| `rainbow-speed` | float | no | `1.0` | Rotation speed multiplier. Also supports negative values to change the direction of the rainbow. |

```toml
[[display]]
type         = "rainbow"
name         = "MyRainbow"
rainbow-mode = "horizontal"
rainbow-speed = 1.5
end-condition = { type = "time", duration-ms = 10000 }
next-display  = "NextDisplay"
```

---

### `type = "transition"` - smooth N-color HSV transition

Cycles endlessly through 2 or more colors by interpolating in HSV space. A cubic Bézier easing curve is applied per segment.

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `transition-colors` | array of strings | **yes** | - | At least 2 RGB hex color strings to cycle through |
| `transition-speed` | float | no | `0.005` | Phase advance per frame per color segment. Larger → faster |
| `bezier` | array of 4 floats | no | `[0.11, 0.0, 0.35, 1.0]` | CSS cubic-bézier control points `[p1x, p1y, p2x, p2y]` -> https://cubic-bezier.com |

```toml
[[display]]
type               = "transition"
name               = "ColorCycle"
transition-colors  = ["ff0000", "00aa00", "0000ff"]
transition-speed   = 0.005
bezier             = [0.11, 0.0, 0.35, 1.0]
end-condition      = { type = "time", duration-ms = 15000 }
next-display       = "NextDisplay"
```

---

### `type = "video"` - raw video file

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `video-path` | string | **yes** | - | Absolute or relative path to a raw video file |
| `video-framerate` | integer | no | `30` | Playback frame rate in FPS |
| `video-colors` | string | no | `"rgb"` | Pixel format: `"rgb"` (3 bytes/pixel) or `"greyscale"` (1 byte/pixel) |

The expected resolution is **12 × 9** pixels (108 LEDs). See the ffmpeg snippet in [--display video](#--display-video) section for how to produce a compatible file. Supports `end-condition = { type = "video-end" }` to automatically move to the next display when the video ends.

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

### `type = "glsl"` - GLSL fragment shader

Renders an OpenGL ES 3.00 fragment shader to the LED grid. See the [GLSL Shaders](#glsl-shaders) section for uniform conventions and a shader template.

| Key | Type | Required | Default | Description |
|---|---|---|---|---|
| `shader-path` | string | **yes** | - | Absolute or relative path to the `.glsl` fragment shader file |
| `shader-fps` | integer | no | `30` | Target frame rate. **Warning:** values above ~33 fps (due to it resulting in a frame delta < ~30 ms) may cause unwanted behavior as the microphone controller cannot process requests in time. |
| `shader-scale` | integer | no | `1` | Supersampling scale: renders at `12·n × 9·n` pixels and block-averages down to 12×9. Values above 10 are not recommended due to diminishing returns for the increased compute cost, unless needed. |

```toml
[[display]]
type         = "glsl"
name         = "MyShader"
shader-path  = "/usr/share/quadcast2srgb/shaders/example.glsl"
shader-fps   = 30
shader-scale = 2
end-condition = { type = "time", duration-ms = 10000 }
next-display  = "NextDisplay"
```

---

## Example - two solid colors, 5 seconds each

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

# GLSL Shaders

The `glsl` display type renders any OpenGL ES 3.00 fragment shader to the 12×9 LED grid, following the same uniform naming conventions as [Shadertoy](https://www.shadertoy.com). Additional `u_*` aliases are also accepted for compatibility with other web-based editors.

| Uniform | Aliases | Type | Description |
|---|---|---|---|
| `iTime` | `u_time`, `time` | `float` | Seconds since the display started or was last reset |
| `iResolution` | `u_resolution`, `resolution` | `vec2` | Render target size `vec2(12, 9) * shaderScale` |
| `iFrame` | `u_frame`, `frame` | `int` | Frame index since the display started or was last reset |
| `iAudioVolume` | `u_audioVolume`, `audioVolume` | `float` | Current max volume level of the captured audio device (range: 0–1). Only updated when the display sequence was started with audio capture enabled (`capture-audio`). 

Any uniforms that are absent from the shader are silently skipped. None are required. Just like a traditional shader, the output color is determined by writing to an outgoing `vec4` global. The RGB components of this variable determine the LED colors, while the alpha component is ignored.

## Minimal Example

A minimal shader that maps the UV coordinates to red/green and animates blue over time:

```glsl
#version 300 es
precision highp float;

uniform float iTime;
uniform vec2  iResolution;

out vec4 fragColor;

void main()
{
    vec2 uv = gl_FragCoord.xy / iResolution;
    fragColor = vec4(uv.x, uv.y, 0.5 + 0.5 * sin(iTime), 1.0);
}
```

## Pre-packaged shaders
The installation package comes with (very) minimal shaders pre-installed. I will spare myself the list, as it might change throughout the versions, the shaders can be found in `/usr/share/quadcast2srgb/shaders`.

## Audio-reactive shader example

When `--capture-audio` is enabled, `u_audioVolume` pulses with the microphone's volume level:

```glsl
#version 300 es
precision highp float;

uniform float u_time;
uniform vec2  u_resolution;
uniform float u_audioVolume;

out vec4 fragColor;

void main()
{
    vec2 uv = gl_FragCoord.xy / u_resolution;
    // Brightness follows the audio volume, wash with time-based hue
    float brightness = 0.2 + 0.8 * u_audioVolume;
    vec3 color = 0.5 + 0.5 * cos(u_time + uv.xyx + vec3(0, 2, 4));
    fragColor = vec4(color * brightness, 1.0);
}
```

Save it to a file and run:
```sh
qc2srgb --display glsl --shader-path ~/example.glsl --shader-fps 30
```

# Contributing
Contributions are very welcome! If you have any suggestions, ideas, or want to help out with the project, feel free to open an issue or a pull request.

## Example: Custom Display
The architecture of the program is meant to be extendable; meaning that you can add your own display types relatively easy.

Here is an example walkthrough of how to add a new display type that snakes a single lit LED across all 108 LEDs (12×9 grid):

```cpp
// src/display/CSnakeDisplay.h
#pragma once

#include "CQC2SDisplay.h"
#include "DisplayUtils.h" // includes VideoConstants.h for global constants
#include <array>
#include <thread>

class CSnakeDisplay : public CQC2SDisplay
{
    SRGBColor m_color;
    uint32_t  m_position = 0; // current lit LED index (0–107)

public:
    CSnakeDisplay(SRGBColor p_color, String p_name,
                  UniquePtr<CEndCondition> p_pEndCondition,
                  String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition),
                       std::move(p_nextDisplay))
        , m_color(p_color)
    {}

// Initialize required variables here. This is called once right when the display gets parsed from arg or config
    // bool Initialize() override {}

    
    // Called once before displaying starts; return false to abort
    virtual void Reset() { 
        m_position = 0;
        return CQC2SDisplay::Reset();
    }

    // Called once after displaying ends or is aborted
    //virtual void Shutdown(CQuadcast2SCommunicator &/*p_communicator*/) {}

    virtual String GetName() const { return m_name; }
    virtual String GetNextDisplay() const { return m_nextDisplay; }
    virtual void SetNextDisplay(String p_nextDisplay) { m_nextDisplay = std::move(p_nextDisplay); }

    // The "render" function: Called once per frame; return false to manually stop playing this display
    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        // frame data, defaults to 0
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        frame[m_position] = m_color;

        SendColorFrame(p_communicator, frame.data());

        m_position = (m_position + 1) % g_LED_COUNT;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        return true;
    }
};
```
After implementing the display, orient yourself around CQC2SDisplayFactory, ArgParsing.cpp and ConfigParser.cpp to add support for parsing this display from command line or config files with your desired config options.

This example will also show you that the indices on LEDs are "snaking", traversing from columns to column in zig-zag manner (e.g. 0–8 goes down the first column, then 9–17 goes up the second column, etc.; Fairly common in LED matrices due to the wiring... I believe it can also be called [Boustrophedon](https://en.wikipedia.org/wiki/Boustrophedon) Indexing). See VideoProcessing.cpp for a utility function to convert between (x,y) coordinates and LED indices if you want to do display something more axis oriented.


# What else to implement?
- Lua(JIT - for lower overhead) scripting for custom stateful displays?
- Window capture? I'm just imagining playing DOOM on it to be honest...
- Save single display configs directly to device, if possible.
    - I haven't looked into how saving to on board memory works, HyperX NGENUITY does offer the functionality.