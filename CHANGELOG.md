# Changelog

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
