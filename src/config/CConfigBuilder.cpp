// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "CConfigBuilder.h"

#include "../util/ArgParsing.h"
#include "../util/ConfigParser.h"
#include "../display/CQC2SDisplayFactory.h"
#include "../video/VideoProcessing.h"
#include "../Globals.h"

// =============================================================================
//  Internal helpers — parse display-specific CLI arguments
// =============================================================================
namespace
{

struct SDisplayArgs
{
    String              m_displayType      = "solid";
    SRGBColor           m_color            = {0x29, 0x00, 0x66};
    SCubicBezier        m_bezier           = SCubicBezier::EaseInOut();
    float               m_pulseSpeed       = 0.025f;

    String              m_videoPath;
    uint32_t            m_videoFramerate   = 30;
    EVideoFormat        m_videoFormat      = EVideoFormat::Rgb;

    ERainbowMode        m_rainbowMode      = ERainbowMode::Flat;
    double              m_rainbowSpeed     = 1.0;

    DynamicContainer<SRGBColor> m_transitionColors;
    float               m_transitionSpeed  = 0.005f;
    SCubicBezier        m_transitionBezier = SCubicBezier::EaseInOut();

#ifdef USE_GLSL
    String              m_shaderPath;
    uint32_t            m_shaderFps        = 30;
    uint32_t            m_shaderScale      = 1;
#endif
};

/// Parse display-related CLI arguments (everything under the "Single(!) display"
/// section of --help).  Flags that are handled elsewhere (--verbose, --serial,
/// --config, audio group) are intentionally skipped here.
SDisplayArgs ParseDisplayArgs(int p_argc, char *p_pArgv[])
{
    SDisplayArgs args;

    for (int i = 1; i < p_argc; ++i)
    {
        String arg = p_pArgv[i];

        if (arg == "--display" && i + 1 < p_argc)
            args.m_displayType = p_pArgv[++i];
        else if (arg == "--color" && i + 1 < p_argc)
        {
            auto parsed = ParseHexColor(p_pArgv[++i]);
            if (parsed) args.m_color = *parsed;
            else        LOG_ERROR(L"Invalid --color value. Expected 6-digit hex (e.g. ff00dd).");
        }
        else if (arg == "--pulse-speed" && i + 1 < p_argc)
            args.m_pulseSpeed = std::stof(p_pArgv[++i]);
        else if (arg == "--pulse-cubic-bezier" && i + 4 < p_argc)
        {
            args.m_bezier.m_p1x = std::stof(p_pArgv[++i]);
            args.m_bezier.m_p1y = std::stof(p_pArgv[++i]);
            args.m_bezier.m_p2x = std::stof(p_pArgv[++i]);
            args.m_bezier.m_p2y = std::stof(p_pArgv[++i]);
        }
        else if (arg == "--video-path" && i + 1 < p_argc)
            args.m_videoPath = p_pArgv[++i];
        else if (arg == "--video-framerate" && i + 1 < p_argc)
            args.m_videoFramerate = static_cast<uint32_t>(std::stoul(p_pArgv[++i]));
        else if (arg == "--video-colors" && i + 1 < p_argc)
        {
            String fmt = p_pArgv[++i];
            if (fmt == "greyscale" || fmt == "grayscale")
                args.m_videoFormat = EVideoFormat::Greyscale;
            else if (fmt != "rgb")
                LOG_ERROR(L"Unknown --video-colors value '" << WStr(fmt) << L"'");
        }
        else if (arg == "--rainbow-mode" && i + 1 < p_argc)
        {
            String mode = p_pArgv[++i];
            if      (mode == "flat")      args.m_rainbowMode = ERainbowMode::Flat;
            else if (mode == "vertical")  args.m_rainbowMode = ERainbowMode::RollingVertical;
            else if (mode == "horizontal")args.m_rainbowMode = ERainbowMode::RollingHorizontal;
            else if (mode == "diagonal")  args.m_rainbowMode = ERainbowMode::RollingDiagonal;
            else LOG_ERROR(L"Unknown --rainbow-mode value '" << WStr(mode) << L"'. Expected: flat, vertical, horizontal, diagonal.");
        }
        else if (arg == "--rainbow-speed" && i + 1 < p_argc)
            args.m_rainbowSpeed = std::stod(p_pArgv[++i]);
        else if (arg == "--transition-colors" && i + 1 < p_argc)
        {
            String colorsStr = p_pArgv[++i];
            args.m_transitionColors.clear();
            size_t pos = 0;
            while (pos < colorsStr.size())
            {
                const size_t COMMA = colorsStr.find(',', pos);
                const String TOKEN = colorsStr.substr(pos, COMMA == String::npos ? String::npos : COMMA - pos);
                auto parsed = ParseHexColor(TOKEN);
                if (parsed) args.m_transitionColors.push_back(*parsed);
                else        LOG_ERROR(L"Invalid --transition-colors token '" << WStr(TOKEN) << L"'. Expected 6-digit hex.");
                if (COMMA == String::npos) break;
                pos = COMMA + 1;
            }
            if (args.m_transitionColors.size() < 2)
                LOG_ERROR(L"--transition-colors requires at least 2 valid colors.");
        }
        else if (arg == "--transition-speed" && i + 1 < p_argc)
            args.m_transitionSpeed = std::stof(p_pArgv[++i]);
        else if (arg == "--transition-cubic-bezier" && i + 4 < p_argc)
        {
            args.m_transitionBezier.m_p1x = std::stof(p_pArgv[++i]);
            args.m_transitionBezier.m_p1y = std::stof(p_pArgv[++i]);
            args.m_transitionBezier.m_p2x = std::stof(p_pArgv[++i]);
            args.m_transitionBezier.m_p2y = std::stof(p_pArgv[++i]);
        }
#ifdef USE_GLSL
        else if (arg == "--shader-path" && i + 1 < p_argc)
            args.m_shaderPath = p_pArgv[++i];
        else if (arg == "--shader-fps" && i + 1 < p_argc)
            args.m_shaderFps = static_cast<uint32_t>(std::stoul(p_pArgv[++i]));
        else if (arg == "--shader-scale" && i + 1 < p_argc)
            args.m_shaderScale = static_cast<uint32_t>(std::stoul(p_pArgv[++i]));
#endif
    }
    return args;
}

/// Create a display from the parsed CLI arguments (when no config file is given).
UniquePtr<CQC2SDisplay> CreateDisplayFromArgs(const SDisplayArgs &p_args)
{
    const String &type = p_args.m_displayType;

    if (type == "solid")
        return CQC2SDisplayFactory::CreateSolidColor(p_args.m_color, "solid");

    if (type == "pulse" || type == "pulse-color")
        return CQC2SDisplayFactory::CreatePulseColor(p_args.m_color, p_args.m_pulseSpeed, "pulse", nullptr, p_args.m_bezier);

    if (type == "rainbow")
        return CQC2SDisplayFactory::CreateRainbow(p_args.m_rainbowMode, p_args.m_rainbowSpeed, "rainbow");

    if (type == "transition" || type == "color-transition")
    {
        if (p_args.m_transitionColors.size() < 2)
        {
            LOG_ERROR(L"--display transition requires at least 2 colors via --transition-colors. Defaulting to default color.");
            return CQC2SDisplayFactory::CreateSolidColor({0x29, 0x00, 0x66}, "solid");
        }
        DynamicContainer<SHSV> hsvColors;
        hsvColors.reserve(p_args.m_transitionColors.size());
        for (const auto &c : p_args.m_transitionColors)
            hsvColors.push_back(SHSV::FromRGB(c));
        return CQC2SDisplayFactory::CreateColorTransition(std::move(hsvColors), p_args.m_transitionSpeed, "transition", nullptr, p_args.m_transitionBezier);
    }

    if (type == "video")
    {
        if (p_args.m_videoPath.empty())
        {
            LOG_ERROR(L"--display video requires --video-path <path>. Defaulting to default color.");
            return CQC2SDisplayFactory::CreateSolidColor({0x29, 0x00, 0x66}, "solid");
        }
        try
        {
            auto frames = LoadVideoBuffer(p_args.m_videoPath, p_args.m_videoFormat);
            if (frames.empty())
            {
                LOG_ERROR(L"Video file loaded but contains no frames: " << WStr(p_args.m_videoPath) << L". Defaulting to default color.");
                return CQC2SDisplayFactory::CreateSolidColor({0x29, 0x00, 0x66}, "solid");
            }
            return CQC2SDisplayFactory::CreateVideoDisplay(std::move(frames), p_args.m_videoFramerate, "video");
        }
        catch (const std::exception &e)
        {
            LOG_ERROR(L"Failed to load video file '" << WStr(p_args.m_videoPath) << L"': " << WStr(e.what()) << L". Defaulting to default color.");
            return CQC2SDisplayFactory::CreateSolidColor({0x29, 0x00, 0x66}, "solid");
        }
    }

#ifdef USE_GLSL
    if (type == "glsl")
    {
        if (p_args.m_shaderPath.empty())
        {
            LOG_ERROR(L"--display glsl requires --shader-path <path>. Defaulting to default color.");
            return CQC2SDisplayFactory::CreateSolidColor({0x29, 0x00, 0x66}, "solid");
        }
        return CQC2SDisplayFactory::CreateGLSLDisplay(p_args.m_shaderPath, p_args.m_shaderFps, p_args.m_shaderScale, "glsl");
    }
#endif

    LOG_ERROR(L"Unknown display type: " << WStr(type) << L". Defaulting to default color.");
    return CQC2SDisplayFactory::CreateSolidColor({0x29, 0x00, 0x66}, "solid");
}

// ── Audio merging helpers ─────────────────────────────────────────────────

/// Audio CLI flags that override config / defaults.
struct SAudioCliFlags
{
    Option<float> m_inputGain;
    bool          m_noSmoothing  = false;   // --no-audio-smoothing present
    Option<float> m_smoothingAlpha;
    bool          m_captureAudio = false;   // --capture-audio present
};

SAudioCliFlags ParseAudioCliFlags(int p_argc, char *p_pArgv[])
{
    SAudioCliFlags cli;
    cli.m_captureAudio   = ParseFlag(p_argc, p_pArgv, "--capture-audio");
    cli.m_inputGain      = ParseFloatArg(p_argc, p_pArgv, "--input-gain");
    cli.m_noSmoothing    = ParseFlag(p_argc, p_pArgv, "--no-audio-smoothing");
    cli.m_smoothingAlpha = ParseFloatArg(p_argc, p_pArgv, "--audio-smoothing-alpha");
    return cli;
}

void MergeAudioSettings(SProgramConfig &p_cfg, const SAudioCliFlags &p_cli,
                        const SConfigParseResult *p_pConfig)
{
    // enableAudio: CLI --capture-audio OR config flag
    if (p_cli.m_captureAudio)
        p_cfg.m_enableAudio = true;
    // (config may also set it; handled below via the OR read from SConfigParseResult)

    if (p_pConfig)
    {
        // OR for enableAudio
        p_cfg.m_enableAudio = p_cfg.m_enableAudio || p_pConfig->m_enableAudio;

        // inputGain: CLI > Config > Default
        if (!p_cli.m_inputGain.has_value() && p_pConfig->m_inputGain.has_value())
            p_cfg.m_inputGain = *p_pConfig->m_inputGain;

        // smoothing: CLI --no-audio-smoothing can disable regardless of config
        if (!p_cli.m_noSmoothing && p_pConfig->m_audioSmoothing.has_value())
            p_cfg.m_audioSmoothing = *p_pConfig->m_audioSmoothing;

        // smoothingAlpha: CLI > Config > Default
        if (!p_cli.m_smoothingAlpha.has_value() && p_pConfig->m_audioSmoothingAlpha.has_value())
            p_cfg.m_audioSmoothingAlpha = *p_pConfig->m_audioSmoothingAlpha;
    }

    // CLI overrides (highest priority)
    if (p_cli.m_inputGain.has_value())
        p_cfg.m_inputGain = *p_cli.m_inputGain;
    if (p_cli.m_noSmoothing)
        p_cfg.m_audioSmoothing = false;
    if (p_cli.m_smoothingAlpha.has_value())
        p_cfg.m_audioSmoothingAlpha = *p_cli.m_smoothingAlpha;
}

} // anonymous namespace

// =============================================================================
//  CConfigBuilder::Build  —  public entry point
// =============================================================================

SProgramConfig CConfigBuilder::Build(int p_argc, char *p_pArgv[])
{
    SProgramConfig cfg;

    // ── 1. Parse basic CLI flags ──────────────────────────────────────────
    cfg.m_verbose      = ParseFlag(p_argc, p_pArgv, "--verbose");
    cfg.m_noWaitForRead = ParseFlag(p_argc, p_pArgv, "--no-wait-for-read");
    cfg.m_allowedSerials = ParseSerialArgs(p_argc, p_pArgv);

    // ── 2. Check for config file ─────────────────────────────────────────
    auto configPathOpt = CConfigParser::ParseConfigPathArg(p_argc, p_pArgv);

    if (configPathOpt.has_value())
    {
        // ── 2a. Config file present: parse it, then merge ──────────────
        try
        {
            CConfigParser configParser(*configPathOpt);
            auto parseResult = configParser.Parse();

            if (!parseResult.m_pDisplay)
            {
                LOG_ERROR(L"Config parsing failed: no startup display defined.");
                return cfg;   // cfg.m_pDisplay remains nullptr → caller handles it
            }

            // OR-merge toggles
            cfg.m_verbose       = cfg.m_verbose || parseResult.m_verbose;
            cfg.m_noWaitForRead = cfg.m_noWaitForRead || parseResult.m_noWaitForRead;

            // Allowed serials: CLI takes precedence
            if (!cfg.m_allowedSerials.has_value() && parseResult.m_allowedSerials.has_value()
                && !parseResult.m_allowedSerials->empty())
            {
                cfg.m_allowedSerials = parseResult.m_allowedSerials;
            }
            else if (parseResult.m_allowedSerials.has_value())
            {
                LOG_ERROR(L"[Config] Ignoring allowed serials from config because they were also passed via command line arguments.");
            }

            // Take the display from the config
            cfg.m_pDisplay = std::move(parseResult.m_pDisplay);

            // Merge audio settings (config + CLI)
            SAudioCliFlags audioCli = ParseAudioCliFlags(p_argc, p_pArgv);
            MergeAudioSettings(cfg, audioCli, &parseResult);
        }
        catch (const std::exception &e)
        {
            LOG_ERROR(L"Failed to parse config: " << WStr(e.what()));
            // cfg.m_pDisplay stays nullptr
        }
    }
    else
    {
        // ── 2b. No config file: build display from CLI args ────────────
        SDisplayArgs displayArgs = ParseDisplayArgs(p_argc, p_pArgv);
        cfg.m_pDisplay = CreateDisplayFromArgs(displayArgs);

        // Merge audio settings (CLI + defaults only)
        SAudioCliFlags audioCli = ParseAudioCliFlags(p_argc, p_pArgv);
        MergeAudioSettings(cfg, audioCli, nullptr);
    }

    return cfg;
}
