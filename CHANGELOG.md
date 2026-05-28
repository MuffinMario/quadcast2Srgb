# Changelog

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
