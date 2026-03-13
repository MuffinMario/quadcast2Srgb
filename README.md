# Quadcast 2S RGB
Currently too lazy to write the readme.

Check out this example config
```toml

# required, must reference a display added below
startup-display = "Solid"
# optional, defaults to false
# verbose = true
# optional, defaults to [], this way you can restrict the program to only connect to a specific serial, in case you have multiple ones.
#                       IF YOU HAVE MULTIPLE DEVICES FEEL FREE TO CONTACT ME! (I cannot test this myself) https://github.com/MuffinMario/quadcast2Srgb
# allowed-serials = ["12345678", "87654321"]

[[display]]
# required
type = "solid"
name = "Solid"
end-condition = { type = "time", duration-ms = 5000 } # optional, if not given display will repeat endlessly
next-display = "Pulse1" # optional, defaults to empty, meaning this display will be the end of the sequence
# end-condition = { type = "time", duration-ms = 5000 } # optional, if not given display will repeat endlessly
# optional, defaults to "#290066" the hash is optional, only here for vscode color picker
# color = "#290066ff" # since vscode color picker adds alpha, it can be included. however it is just ignored.


[[display]]
type = "pulse-color"
name = "Pulse1"
next-display = "Pulse2"
# optional, but you probably want to customize this.
# color = "#290066"
end-condition = { type = "time", duration-ms = 4050 } # optional, if not given display will repeat endlessly
# optional, for custom values check out a generator, e.g. https://cubic-bezier.com
# cubic-bezier = [0.25, 0.1, 0.25, 1.0] # defaults to custom ease in out
# pulse speed is basically the step progress from 0 to 1 in 50ms intervals, meaning we have 20 steps per second
# pulse-speed = 0.025 # optional, defaults to 0.025, bigger number => faster progression.


[[display]]
type = "pulse-color"
name = "Pulse2"
next-display = "Pulse1"
# optional, but you probably want to customize this.
color = "#861e27ff"
end-condition = { type = "time", duration-ms = 2050 } # optional, if not given display will repeat endlessly
cubic-bezier = [0.25, 0.1, 0.25, 1.0]
pulse-speed = 0.05

```
Creates a static light for 5s, and then loops between pulsing indigo & 2x as fast pulsing crimson red forever.
Can be called via `qc2srgb --config example_config.toml`

If you hate configs, you can also `qc2srgb --display solid --color 140040` for single display types.
`--serial ABCDE123` can also be used as a param to only include one singular serial. AYou can also chain it multiple times,
# TODO / New Display ideas?
- GLSL shader?
- Voice volume based response / interface
- Lua(JIT) interface for customization (including all features of the program)
- color transitions with pulse (e.g. #ff0000 to -> #00ff00)