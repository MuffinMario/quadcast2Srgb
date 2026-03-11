#include "ArgParsing.h"
#include <iostream>

Option<SRGBColor> ParseHexColor(String p_colorStr)
{
    if (!p_colorStr.empty() && p_colorStr[0] == '#')
        p_colorStr = p_colorStr.substr(1);
    if (p_colorStr.size() != 6)
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
