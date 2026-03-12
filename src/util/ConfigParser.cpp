#include "ConfigParser.h"

#include "ArgParsing.h"
#include "../display/CMultiDisplay.h"
#include "../display/CEndCondition.h"
#include "../video/VideoProcessing.h"

#include <stdexcept>

namespace
{
String RequireString(const toml::table &p_table, const char *p_pKey)
{
    auto val = p_table[p_pKey].value<String>();
    if (!val)
        throw std::runtime_error(String("Missing or invalid string key: ") + p_pKey);
    return *val;
}

bool OptionalBool(const toml::table &p_table, const char *p_pKey, bool p_default)
{
    auto val = p_table[p_pKey].value<bool>();
    return val.value_or(p_default);
}

int64_t OptionalInt64(const toml::table &p_table, const char *p_pKey, int64_t p_default)
{
    auto val = p_table[p_pKey].value<int64_t>();
    return val.value_or(p_default);
}

float OptionalFloat(const toml::table &p_table, const char *p_pKey, float p_default)
{
    auto val = p_table[p_pKey].value<double>();
    if (val)
        return static_cast<float>(*val);
    return p_default;
}
}

CConfigParser::CConfigParser(const String &p_configPath)
{
    try
    {
        m_config = toml::parse_file(p_configPath);
    }
    catch (const toml::parse_error &e)
    {
        StringStream ss;
        ss << "Failed to parse config '" << p_configPath << "': " << e.description();
        throw std::runtime_error(
            ss.str()
        );
    }
}

bool CConfigParser::HasConfigArg(int p_argc, char *p_pArgv[])
{
    return ParseConfigPathArg(p_argc, p_pArgv).has_value();
}

Option<String> CConfigParser::ParseConfigPathArg(int p_argc, char *p_pArgv[])
{
    for (int i = 1; i < p_argc; ++i)
    {
        String arg = p_pArgv[i];
        if (arg == "--config" && i + 1 < p_argc)
            return String(p_pArgv[i + 1]);
    }
    return std::nullopt;
}

SRGBColor CConfigParser::ParseColorWithDefault(const toml::table &p_displayTable, SRGBColor p_defaultColor) const
{
    auto colorStr = p_displayTable["color"].value<String>();
    if (!colorStr)
        return p_defaultColor;

    auto parsed = ParseHexColor(*colorStr);
    if (!parsed)
        throw std::runtime_error("Invalid 'color' value: expected 6-digit hex string");
    return *parsed;
}

SCubicBezier CConfigParser::ParseBezierWithDefault(const toml::table &p_displayTable, SCubicBezier p_defaultBezier) const
{
    auto pBezierNode = p_displayTable["bezier"].as_array();
    if (!pBezierNode)
        return p_defaultBezier;

    if (pBezierNode->size() != 4)
        throw std::runtime_error("Invalid 'bezier' value: expected array of 4 numbers");

    SCubicBezier bezier = p_defaultBezier;
    for (size_t i = 0; i < 4; ++i)
    {
        auto val = (*pBezierNode)[i].value<double>();
        if (!val)
            throw std::runtime_error("Invalid 'bezier' value: all entries must be numbers");
        switch (i)
        {
        case 0: bezier.m_p1x = static_cast<float>(*val); break;
        case 1: bezier.m_p1y = static_cast<float>(*val); break;
        case 2: bezier.m_p2x = static_cast<float>(*val); break;
        case 3: bezier.m_p2y = static_cast<float>(*val); break;
        }
    }
    return bezier;
}

UniquePtr<CEndCondition> CConfigParser::ParseEndCondition(const toml::table &p_displayTable, const VideoFrameBuffer *p_pVideoFrames) const
{
    auto pEndCondNode = p_displayTable["end-condition"].as_table();
    if (!pEndCondNode)
        return nullptr;

    String endType = RequireString(*pEndCondNode, "type");
    if (endType == "time")
    {
        int64_t durationMs = OptionalInt64(*pEndCondNode, "duration-ms", -1);
        if (durationMs < 0)
            throw std::runtime_error("Invalid end-condition for type 'time': duration-ms must be >= 0");
        return std::make_unique<CTimeEndCondition>(std::chrono::milliseconds(durationMs));
    }

    if (endType == "video-end")
    {
        if (!p_pVideoFrames)
            throw std::runtime_error("end-condition type 'video-end' is only valid for type='video'");
        int64_t loops = OptionalInt64(*pEndCondNode, "loop-count", 1);
        return std::make_unique<CVideoCompletedEndCondition>(p_pVideoFrames->size(), loops);
    }

    throw std::runtime_error("Unknown end-condition type: '" + endType + "'");
}

UniquePtr<CQC2SDisplay> CConfigParser::ParseSingleDisplay(const toml::table &p_displayTable) const
{
    String type = RequireString(p_displayTable, "type");
    String name = RequireString(p_displayTable, "name");
    String nextDisplay = p_displayTable["next-display"].value<String>().value_or("");

    constexpr SRGBColor DEFAULT_COLOR{0x29, 0x00, 0x66};

    if (type == "solid")
    {
        SRGBColor color = ParseColorWithDefault(p_displayTable, DEFAULT_COLOR);
        auto endCondition = ParseEndCondition(p_displayTable, nullptr);
        return CQC2SDisplayFactory::CreateSolidColor(color, name, std::move(endCondition), nextDisplay);
    }

    if (type == "pulse-color" || type == "pulse")
    {
        SRGBColor color = ParseColorWithDefault(p_displayTable, DEFAULT_COLOR);
        float pulseSpeed = OptionalFloat(p_displayTable, "pulse-speed", 0.025f);
        SCubicBezier bezier = ParseBezierWithDefault(p_displayTable, SCubicBezier::EaseInOut());
        auto endCondition = ParseEndCondition(p_displayTable, nullptr);
        return CQC2SDisplayFactory::CreatePulseColor(color, pulseSpeed, name, std::move(endCondition), bezier, nextDisplay);
    }

    if (type == "video")
    {
        String videoPath = RequireString(p_displayTable, "video-path");
        uint32_t fps = static_cast<uint32_t>(OptionalInt64(p_displayTable, "video-framerate", 30));
        String colors = p_displayTable["video-colors"].value<String>().value_or("rgb");

        EVideoFormat format = EVideoFormat::Rgb;
        if (colors == "greyscale" || colors == "grayscale")
            format = EVideoFormat::Greyscale;
        else if (colors != "rgb")
            throw std::runtime_error("Unknown video-colors value: '" + colors + "'");

        VideoFrameBuffer frames = LoadVideoBuffer(videoPath, format);
        if (frames.empty())
            throw std::runtime_error("Video file contains no frames: " + videoPath);

        auto endCondition = ParseEndCondition(p_displayTable, &frames);
        return CQC2SDisplayFactory::CreateVideoDisplay(std::move(frames), fps, name, std::move(endCondition), nextDisplay);
    }

    throw std::runtime_error("Unknown display type: '" + type + "'");
}

SConfigParseResult CConfigParser::Parse() const
{
    SConfigParseResult result{};

    result.m_verbose = OptionalBool(m_config, "verbose", false);

    if (auto pSerialsNode = m_config["allowed-serials"].as_array())
    {
        Set<WString> allowed;
        for (const auto &node : *pSerialsNode)
        {
            auto serialStr = node.value<String>();
            if (!serialStr)
                throw std::runtime_error("Invalid allowed-serials entry: expected string");
            allowed.emplace(serialStr->begin(), serialStr->end());
        }
        if (!allowed.empty())
            result.m_allowedSerials = allowed;
    }

    String startupDisplay = RequireString(m_config, "startup-display");
    auto pMultiBase = CQC2SDisplayFactory::CreateMultiDisplay("config", nullptr, startupDisplay);
    auto *pMulti = dynamic_cast<CMultiDisplay *>(pMultiBase.get());
    if (!pMulti)
        throw std::runtime_error("Internal error: failed to create CMultiDisplay");

    auto pDisplaysNode = m_config["display"].as_array();
    if (!pDisplaysNode || pDisplaysNode->empty())
        throw std::runtime_error("Config must contain at least one [display] table");

    for (const auto &node : *pDisplaysNode)
    {
        auto pTable = node.as_table();
        if (!pTable)
            throw std::runtime_error("Invalid [display] entry: expected table");
        pMulti->AddDisplay(ParseSingleDisplay(*pTable));
    }

    result.m_pDisplay = std::move(pMultiBase);
    return result;
}
