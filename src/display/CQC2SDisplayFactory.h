#pragma once

#include "../Common.h"
#include "CQC2SDisplay.h"
#include "CSolidColorDisplay.h"
#include "CPulseColorDisplay.h"
#include "CVideoDisplay.h"
#include "CMultiDisplay.h"
#include "CEndCondition.h"
#include "../video/VideoConstants.h"

class CQC2SDisplayFactory
{
public:
    static UniquePtr<CQC2SDisplay> CreateSolidColor(SRGBColor p_color, String p_name = "solid", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreatePulseColor(SRGBColor p_color, float p_speed = 0.05f, String p_name = "pulse", UniquePtr<CEndCondition> p_pEndCondition = nullptr, SCubicBezier p_bezier = SCubicBezier::EaseInOut(), String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreateMultiDisplay(String p_name = "multi", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");
    static UniquePtr<CQC2SDisplay> CreateVideoDisplay(VideoFrameBuffer p_frames, uint32_t p_fps = 30, String p_name = "video", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "");

    static UniquePtr<CQC2SDisplay> CreateFromArgs(int p_argc, char *p_pArgv[]);
};
