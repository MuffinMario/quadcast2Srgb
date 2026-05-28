// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "ArgParsing.h"
#include <iostream>
#include "../Globals.h"

Option<SRGBColor> ParseHexColor(String p_colorStr)
{
    if (!p_colorStr.empty() && p_colorStr[0] == '#')
        p_colorStr = p_colorStr.substr(1);
    if (
        // accept ff00ff or ff00ff00 (alpha omitted)
        (p_colorStr.size() != 6 && p_colorStr.size() != 8)
        // validate that all characters are hex digits
        || p_colorStr.find_first_not_of("0123456789abcdefABCDEF") != String::npos)
        return std::nullopt;
    try
    {
        return SRGBColor{
            static_cast<uint8_t>(std::stoul(p_colorStr.substr(0, 2), nullptr, 16)),
            static_cast<uint8_t>(std::stoul(p_colorStr.substr(2, 2), nullptr, 16)),
            static_cast<uint8_t>(std::stoul(p_colorStr.substr(4, 2), nullptr, 16))};
    }
    catch (const std::exception &)
    {
        return std::nullopt;
    }
}

Option<Set<WString>> ParseSerialArgs(int p_argc, char *p_pArgv[])
{
    Set<WString> serials;
    for (int i = 1; i < p_argc; ++i)
    {
        if (String(p_pArgv[i]) == "--serial" && i + 1 < p_argc)
        {
            String raw = p_pArgv[++i];
            WString wraw(raw.begin(), raw.end());
            serials.emplace(wraw);
        }
    }
    if (serials.empty())
        return std::nullopt;
    return serials;
}

bool ParseVerbose(int p_argc, char *p_pArgv[])
{
    for (int i = 1; i < p_argc; ++i)
    {
        if (String(p_pArgv[i]) == "--verbose")
        {
            return true;
        }
    }
    return false;
}

bool ParseHelp(int p_argc, char *p_pArgv[])
{
    for (int i = 1; i < p_argc; ++i)
    {
        const String ARG(p_pArgv[i]);
        if (ARG == "--help" || ARG == "-h")
            return true;
    }
    return false;
}

void PrintHelp(const char *p_pProgramName)
{
    LOG(
        L"Usage: " << p_pProgramName << L" [OPTIONS]\n"
        L"\n"
        L"Program to control the RGB LEDs on a HyperX QuadCast 2S microphone.\n"
        L"\n"
        L"General options:\n"
        L"  -h, --help\n"
        L"      Show this help text and exit.\n"
        L"\n"
        L"  --verbose\n"
        L"      Enable verbose logging.\n"
        L"\n"
        L"  --serial <serial>\n"
        L"      Restrict USB device operations to passed device(s) by serial number.\n"
        L"      Can be specified multiple times to allow more than one device.\n"
        L"\n"
        L"  --config <path>\n"
        L"      Load display configuration from a TOML file.\n"
        L"      When provided, ALL --display arguments below are ignored.\n"
        L"\n"
        L"Single(!) display options:\n"
        L"\n"
        L"  --display <type>\n"
        L"      Single Display mode to use. (Currently) Possible values:\n"
        L"        solid        Solid color (default)\n"
        L"        pulse        Pulsing brightness on a single hue\n"
        L"        pulse-color  Alternative name to pulse\n"
        L"        rainbow      Cycling rainbow across all LEDs\n"
        L"        transition   Smooth HSV transition between N>=2 colors\n"
        L"        color-transition  Alternative name to transition\n"
        L"        video        Driven by a raw video file\n"
        L"\n"
        L"  --color <#rrggbb>\n"
        L"      LED color as an RGB hex value, with or without a leading '#'.\n"
        L"      Default: #290066\n"
        L"\n"
        L"Pulse display options:\n"
        L"\n"
        L"  --pulse-speed <float>\n"
        L"      Phase advance per rendered frame, in the range [0, 1].\n"
        L"      Higher values make the pulse faster. Default: 0.025\n"
        L"\n"
        L"  --pulse-cubic-bezier <p1x> <p1y> <p2x> <p2y>\n"
        L"      Four floats defining a CSS-style cubic Bezier easing curve\n"
        L"      applied to the pulse envelope.\n"
        L"      Default: 0.11 0.0 0.35 1.0  (ease-in-out)\n"
        L"\n"
        L"Video display options:\n"
        L"\n"
        L"  --video-path <path>\n"
        L"      Path to a RAW RGB video file. Required when --display video is used.\n"
        L"\n"
        L"  --video-framerate <fps>\n"
        L"      Playback frame rate in frames per second. Default: 30\n"
        L"\n"
        L"  --video-colors <rgb|greyscale>\n"
        L"      Pixel format of the video file.\n"
        L"        rgb        3 bytes per pixel, R G B order (default)\n"
        L"        greyscale  1 byte per pixel, luminance\n"
        L"\n"
        L"Rainbow display options:\n"
        L"\n"
        L"  --rainbow-mode <mode>\n"
        L"      Layout of the rainbow across the LED grid. Possible values:\n"
        L"        flat        All LEDs share the same hue (default)\n"
        L"        vertical    Hue steps per row\n"
        L"        horizontal  Hue steps per column\n"
        L"        diagonal    Hue steps across both axes\n"
        L"\n"
        L"  --rainbow-speed <float>\n"
        L"      Hue rotation in degrees per frame. Default: 1.0\n"
        L"\n"
        L"Color transition display options:\n"
        L"\n"
        L"  --transition-colors <hex1,hex2[,hexN...]>\n"
        L"      Comma-separated list of at least 2 RGB hex colors to cycle through.\n"
        L"      Each token is a 6-digit hex value with or without a leading '#'.\n"
        L"      Example: ff0000,00ff00,0000ff\n"
        L"\n"
        L"  --transition-speed <float>\n"
        L"      Phase advance per frame within one color segment, in the range [0, 1].\n"
        L"      Higher values make the transition faster. Default: 0.005\n"
        L"\n"
        L"  --transition-cubic-bezier <p1x> <p1y> <p2x> <p2y>\n"
        L"      Four floats defining a CSS-style cubic Bezier easing curve applied\n"
        L"      to each color-to-color segment.\n"
        L"      Default: 0.11 0.0 0.35 1.0  (ease-in-out)\n"
        L"\n");
}
