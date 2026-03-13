#include "CQC2SDisplayFactory.h"
#include "../util/ArgParsing.h"
#include "../video/VideoProcessing.h"
#include "../Globals.h"

UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreateSolidColor(SRGBColor p_color, String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay)
{
    return std::make_unique<CSolidColorDisplay>(p_color, std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
}

UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreatePulseColor(SRGBColor p_color, float p_speed, String p_name, UniquePtr<CEndCondition> p_pEndCondition, SCubicBezier p_bezier, String p_nextDisplay)
{
    return std::make_unique<CPulseColorDisplay>(p_color, p_speed, std::move(p_name), std::move(p_pEndCondition), p_bezier, std::move(p_nextDisplay));
}

UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreateMultiDisplay(String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay)
{
    return std::make_unique<CMultiDisplay>(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
}

UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreateVideoDisplay(VideoFrameBuffer p_frames, uint32_t p_fps, String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay)
{
    return std::make_unique<CVideoDisplay>(std::move(p_frames), p_fps, std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
}

UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreateFromArgs(int p_argc, char *p_pArgv[])
{
    // default settings
    String displayType = "solid";
    SRGBColor color = ParseHexColor("#290066").value();
    SCubicBezier bezier = SCubicBezier::EaseInOut();
    float pulseSpeed = 0.025f;
    String videoPath = "";
    uint32_t videoFramerate = 30;
    EVideoFormat videoFormat = EVideoFormat::Rgb;

    for (int i = 1; i < p_argc; ++i)
    {
        String arg = p_pArgv[i];
        LOG_VERBOSE(WString(arg.begin(), arg.end()) << std::endl);
        if (arg == "--display" && i + 1 < p_argc)
        {
            displayType = p_pArgv[++i];
        }
        else if (arg == "--color" && i + 1 < p_argc)
        {
            auto parsed = ParseHexColor(p_pArgv[++i]);
            if (parsed.has_value())
                color = parsed.value();
            else
                LOG_ERROR(L"Invalid --color value. Expected 6-digit hex (e.g. ff00dd).");
        }
        else if (arg == "--pulse-speed" && i + 1 < p_argc)
        {
            pulseSpeed = std::stof(p_pArgv[++i]);
        }
        else if (arg == "--pulse-cubic-bezier" && i + 4 < p_argc)
        {
            bezier.m_p1x = std::stof(p_pArgv[++i]);
            bezier.m_p1y = std::stof(p_pArgv[++i]);
            bezier.m_p2x = std::stof(p_pArgv[++i]);
            bezier.m_p2y = std::stof(p_pArgv[++i]);
        }
        else if (arg == "--video-path" && i + 1 < p_argc)
        {
            videoPath = p_pArgv[++i];
        }
        else if (arg == "--video-framerate" && i + 1 < p_argc)
        {
            videoFramerate = static_cast<uint32_t>(std::stoul(p_pArgv[++i])); // WIP make float
        }
        else if (arg == "--video-colors" && i + 1 < p_argc)
        {
            String fmt = p_pArgv[++i];
            if (fmt == "greyscale" || fmt == "grayscale") // whatever one may address this as
                videoFormat = EVideoFormat::Greyscale;
            else if (fmt == "rgb")
                videoFormat = EVideoFormat::Rgb;
            else
                LOG_ERROR(L"Unknown --video-colors value '" << WStr(fmt) << L"'");
        }
    }

    if (displayType == "solid")
        return CreateSolidColor(color, "solid");

    if (displayType == "pulse" || displayType == "pulse-color")
        return CreatePulseColor(color, pulseSpeed, "pulse", nullptr, bezier);

    if (displayType == "video")
    {
        // check video path given
        if (videoPath.empty())
        {
            LOG_ERROR(L"--display video requires --video-path <path>. Defaulting to default color.");
            return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
        }
        // try to load the video and create the display
        try
        {
            auto frames = LoadVideoBuffer(videoPath, videoFormat);
            if (frames.empty())
            {
                LOG_ERROR(L"Video file loaded but contains no frames: " << WStr(videoPath) << L". Defaulting to default color.");
                return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
            }
            return CreateVideoDisplay(std::move(frames), videoFramerate, "video");
        }
        catch (const std::exception &e)
        {
            LOG_ERROR(L"Failed to load video file '" << WStr(videoPath) << L"': " << WStr(e.what()) << L". Defaulting to default color.");
            return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
        }
    }

    LOG_ERROR(L"Unknown display type: " << WStr(displayType) << L". Defaulting to default color.");
    return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
}
