// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "CQC2SDisplay.h"
#include "CSolidColorDisplay.h"
#include "CPulseColorDisplay.h"
#include "CVideoDisplay.h"
#include "CMultiDisplay.h"
#include "CRainbowDisplay.h"
#include "CColorTransitionDisplay.h"
#include "CEndCondition.h"
#include "../video/VideoConstants.h"
#ifdef USE_GLSL
#include "CGLSLDisplay.h"
#endif

class CQC2SDisplayFactory
{
public:
    static UniquePtr<CQC2SDisplay> CreateSolidColor(SRGBColor p_color, String p_name = "solid", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreatePulseColor(SRGBColor p_color, float p_speed = 0.05f, String p_name = "pulse", UniquePtr<CEndCondition> p_pEndCondition = nullptr, SCubicBezier p_bezier = SCubicBezier::EaseInOut(), String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreateMultiDisplay(String p_name = "multi", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreateVideoDisplay(VideoFrameBuffer p_frames, uint32_t p_fps = 30, String p_name = "video", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreateRainbow(ERainbowMode p_mode = ERainbowMode::Flat, double p_rotSpeed = 1.0, String p_name = "rainbow", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreateColorTransition(DynamicContainer<SHSV> p_colors, float p_speed = 0.005f, String p_name = "transition", UniquePtr<CEndCondition> p_pEndCondition = nullptr, SCubicBezier p_bezier = SCubicBezier::EaseInOut(), String p_nextDisplay = "");
#ifdef USE_GLSL
    static UniquePtr<CQC2SDisplay> CreateGLSLDisplay(String p_shaderPath, uint32_t p_fps = 30, uint32_t p_resolutionScale = 1, String p_name = "glsl", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");
#endif

    // CreateFromArgs has been moved to CConfigBuilder (src/config/CConfigBuilder.h)
};
