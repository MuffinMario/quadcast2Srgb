#pragma once

#include "../Common.h"
#include "../display/CQC2SDisplayFactory.h"
#include <toml++/toml.h>

struct SConfigParseResult
{
    UniquePtr<CQC2SDisplay> m_pDisplay;
    bool m_verbose = false;
    Option<Set<WString>> m_allowedSerials = std::nullopt;
};
/*
Config parser is a toml++ wrapper to load and parse a config, given that the user wants to use one.

Example config file:
# abstract: this file does the following:
# - defines a display sequence of 4 displays: solid color -> video -> pulse color 1 -> pulse color 2 -> pulse color 1 (looping between the last 2)

# required, must reference a display added below
startup-display = "Solid"
# optional, defaults to false
# verbose = true
# optional, defaults to [], this way you can restrict the program to only connect to a specific serial, in case you have multiple ones.
                        # IF YOU HAVE MULTIPLE DEVICES FEEL FREE TO CONTACT ME! (I cannot test this myself) https://github.com/MuffinMario/quadcast2Srgb
# allowed-serials = ["12345678", "87654321"]

[display]
# required
type = "solid"
name = "Solid"
end-condition = { type = "time", duration-ms = 5000 } # optional, if not given display will repeat endlessly
next-display = "Video" # optional, defaults to empty, meaning this display will be the end of the sequence 
# end-condition = { type = "time", duration-ms = 5000 } # optional, if not given display will repeat endlessly
# optional, defaults to "#290066" the hash is optional, only here for vscode color picker
# color = "#290066ff" # since vscode color picker adds alpha, it can be included. however it is just ignored.

#[display]
# required 
#type = "video"
#name = "Video"
#video-path = "/path/to/video.rgb"
# optional, defaults to empty, meaning this display will loop repeat endlessly. video displays also support video-end, as seen below
#end-condition = { type = "video-end" }
#next-display = "Pulse1"
# optional, defaults to 30
# video-framerate = 30
# optional, defaults to "rgb", currently only supports "rgb" and "greyscale", 8 bit each
# video-colors = "rgb"

[display]
type = "pulse-color"
name = "Pulse1"
next-display = "Pulse1"
# optional, but you probably want to customize this. 
# color = "#290066"
end-condition = { type = "time", duration-ms = 5000 } # optional, if not given display will repeat endlessly
# optional, for custom values check out a generator, e.g. https://cubic-bezier.com
# cubic-bezier = [0.25, 0.1, 0.25, 1.0] # defaults to custom ease in out
# pulse-speed = 0.025 # optional, defaults to 0.025, bigger number => faster progression

[display]
type = "pulse-color"
name = "Pulse2"
next-display = "Pulse1"
# optional, but you probably want to customize this. 
color = "#0c6600"
end-condition = { type = "time", duration-ms = 5000 } # optional, if not given display will repeat endlessly
cubic-bezier = [0.25, 0.1, 0.25, 1.0] 
pulse-speed = 0.05 


*/
class CConfigParser
{
    toml::table m_config;

    // Parse "end-condition" field
    UniquePtr<CEndCondition> ParseEndCondition(const toml::table &p_displayTable, const VideoFrameBuffer *p_pVideoFrames) const;
    // Parse "cubic-bezier" field
    SRGBColor ParseColorWithDefault(const toml::table &p_displayTable, SRGBColor p_defaultColor) const;
    // Parse "cubic-bezier" field
    SCubicBezier ParseBezierWithDefault(const toml::table &p_displayTable, SCubicBezier p_defaultBezier) const;

    // Parse a single [display] table into a CQC2SDisplay instance
    UniquePtr<CQC2SDisplay> ParseSingleDisplay(const toml::table &p_displayTable) const;

public:
    explicit CConfigParser(const String &p_configPath);

    static bool HasConfigArg(int p_argc, char *p_pArgv[]);
    static Option<String> ParseConfigPathArg(int p_argc, char *p_pArgv[]);

    SConfigParseResult Parse() const;
};

