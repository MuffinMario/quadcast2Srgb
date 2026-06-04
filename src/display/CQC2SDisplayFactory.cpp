// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "CQC2SDisplayFactory.h"

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

UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreateRainbow(ERainbowMode p_mode, double p_rotSpeed, String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay)
{
    return std::make_unique<CRainbowDisplay>(p_mode, p_rotSpeed, std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
}

UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreateColorTransition(DynamicContainer<SHSV> p_colors, float p_speed, String p_name, UniquePtr<CEndCondition> p_pEndCondition, SCubicBezier p_bezier, String p_nextDisplay)
{
    return std::make_unique<CColorTransitionDisplay>(std::move(p_colors), p_speed, std::move(p_name), std::move(p_pEndCondition), p_bezier, std::move(p_nextDisplay));
}

#ifdef USE_GLSL
UniquePtr<CQC2SDisplay> CQC2SDisplayFactory::CreateGLSLDisplay(String p_shaderPath, uint32_t p_fps, uint32_t p_resolutionScale, String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay)
{
    return std::make_unique<CGLSLDisplay>(std::move(p_shaderPath), p_fps, p_resolutionScale, std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
}
#endif

// CreateFromArgs has been moved to src/config/CConfigBuilder.cpp

