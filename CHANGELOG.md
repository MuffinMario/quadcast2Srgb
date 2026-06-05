# Changelog

## [0.4.0] - 2026-06-05

### Added
- Added feature to capture audio. To enable usage requires `--capture-audio` or similar config property.
- Added audio device selection via `--audio-device-id <n>` and config key `device-id`.
  Run `--list-audio-devices` to show available devices with their IDs.
- Added channel selection via `--audio-channel <n>` and config key `channel`
  (0 = left, 1 = right). Requires `--capture-audio`.
- Added `--input-gain <float>` / `input-gain` and `--audio-smoothing-alpha <float>` /
  `audio-smoothing-alpha` config keys for audio capture adjustments.
- Added `--list-audio-devices` flag to list all available PortAudio devices and exit.
- Added default audio-reactive shader `audiosphere.glsl` to the resources.
- Added `u_audioVolume` uniform (also `iAudioVolume`, `audioVolume`) to GLSL shader
  display, reports current capture volume level (0–1).
- Unknown CLI arguments now produce an error and suggest `--help`.
- Displays now wait for the device to acknowledge LED packets by default.
  Added `--no-wait-for-read` / `no-wait-for-read` config key to skip this wait.

## [0.3.0] - 2026-05-30

### Added
- New display `glsl`: renders an OpenGL ES 3.00 fragment shader to the LED grid via EGL headless rendering.
  - CLI: `--shader-path` (required), `--shader-fps` (default 30), `--shader-scale` (default 1, supersampling)
  - Config: `shader-path`, `shader-fps`, `shader-scale`
  - Accepts Shadertoy-style uniform names (`iTime`, `iResolution`, `iFrame`) as well as `u_time`/`u_resolution`/`u_frame` and `time`/`resolution`/`frame` aliases.

## [0.2.1] - 2026-05-28

### Fixed
- Rainbow display: corrected hue wrap-around for negative rotation speeds.
- Default config updated: static solid color replaced with a subtle color transition.

## [0.2.0] - 2026-05-28

### Added
- New display `transition`: cycles through N≥2 colors via HSV interpolation with Bézier easing per segment.
  - CLI: `--transition-colors` (comma-separated hex, e.g. `ff0000,00ff00`), `--transition-speed`, `--transition-cubic-bezier`
  - Config: `transition-colors`, `transition-speed`, `bezier`
- New display `rainbow`: static or rolling rainbow across all 108 LEDs.
  - CLI: `--rainbow-mode` (`flat` | `vertical` | `horizontal` | `diagonal`), `--rainbow-speed`
  - Config: `rainbow-mode`, `rainbow-speed`

## [0.1.0] - 2026-03-18

### Added
- Initial release.
