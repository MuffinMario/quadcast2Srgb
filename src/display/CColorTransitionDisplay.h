// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CQC2SDisplay.h"
#include "ColorTypes.h"
#include "DisplayUtils.h"
#include <chrono>
#include <cstdio>
#include <thread>

// Transitions smoothly between N>=2 colors in HSV space using Bézier easing,
// cycling endlessly through the list.  Each color pair forms one segment; the
// Bézier curve is applied per-segment so every transition feels identical.
class CColorTransitionDisplay : public CQC2SDisplay
{
    DynamicContainer<SHSV> m_colors;   // at least 2 entries
    float                  m_speed;    // [0,1] advance per frame within one segment
    SCubicBezier           m_bezier;

    size_t m_segmentIdx = 0;           // current pair: m_colors[idx] → m_colors[next]
    float  m_localT     = 0.0f;        // position within the current segment [0, 1)

public:
    CColorTransitionDisplay(
        DynamicContainer<SHSV> p_colors,
        float                  p_speed,
        String                 p_name,
        UniquePtr<CEndCondition> p_pEndCondition,
        SCubicBezier           p_bezier     = SCubicBezier::EaseInOut(),
        String                 p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)),
          m_colors(std::move(p_colors)), m_speed(p_speed), m_bezier(p_bezier)
    {}

    void Reset() override
    {
        m_segmentIdx = 0;
        m_localT     = 0.0f;
        CQC2SDisplay::Reset();
    }

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        using namespace std::chrono;
        const auto FRAME_DURATION = 50ms;
        const auto FRAME_START    = steady_clock::now();

        const size_t N    = m_colors.size();
        const size_t NEXT = (m_segmentIdx + 1) % N;

        const float EASED   = CubicBezierEval(m_bezier, m_localT);
        const SHSV  CURRENT = LerpHSV(m_colors[m_segmentIdx], m_colors[NEXT],
                                      static_cast<double>(EASED));
                                      
        const SRGBColor COLOR      = CURRENT.ToRGB();
        SendMonoColorFrame(p_communicator, COLOR);

        m_localT += m_speed;
        if (m_localT >= 1.0f)
        {
            m_localT     = 0.0f;
            m_segmentIdx = NEXT;
        }

        const auto ELAPSED   = steady_clock::now() - FRAME_START;
        const auto REMAINING = FRAME_DURATION - ELAPSED;
        if (REMAINING > REMAINING.zero())
            std::this_thread::sleep_for(REMAINING);

        return true;
    }
};
