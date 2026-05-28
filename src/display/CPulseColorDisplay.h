// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CQC2SDisplay.h"
#include "ColorTypes.h"
#include "DisplayUtils.h"
#include <chrono>
#include <thread>

class CPulseColorDisplay : public CQC2SDisplay
{
    SRGBColor m_baseColor;
    float m_t = 0.0f; // linear phase in [0, 1]
    float m_speed;    // advance per frame (in [0,1] units)
    bool m_increasing = true;
    SCubicBezier m_bezier;

    SRGBColor ApplyBrightness(float p_brightness) const
    {
        return {
            static_cast<uint8_t>(m_baseColor.m_red * p_brightness),
            static_cast<uint8_t>(m_baseColor.m_green * p_brightness),
            static_cast<uint8_t>(m_baseColor.m_blue * p_brightness)};
    }

public:
    CPulseColorDisplay(SRGBColor p_color, float p_speed, String p_name, UniquePtr<CEndCondition> p_pEndCondition,
                       SCubicBezier p_bezier = SCubicBezier::EaseInOut(), String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)),
          m_baseColor(p_color), m_speed(p_speed), m_bezier(p_bezier) {}

    void Reset() override
    {
        m_t = 0.0f;
        m_increasing = true;
        
        CQC2SDisplay::Reset(); 
    }
    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        using namespace std::chrono;
        const auto FRAME_DURATION = 50ms;
        const auto FRAME_START = steady_clock::now();

        SRGBColor current = ApplyBrightness(CubicBezierEval(m_bezier, m_t));
        SendMonoColorFrame(p_communicator, current);

        if (m_increasing)
        {
            m_t += m_speed;
            if (m_t >= 1.0f)
            {
                m_t = 1.0f;
                m_increasing = false;
            }
        }
        else
        {
            m_t -= m_speed;
            if (m_t <= 0.0f)
            {
                m_t = 0.0f;
                m_increasing = true;
            }
        }

        const auto ELAPSED = steady_clock::now() - FRAME_START;
        auto remaining = FRAME_DURATION - ELAPSED;
        if (remaining > remaining.zero())
        {
            std::this_thread::sleep_for(remaining);
        }
        return true;
    }
};
