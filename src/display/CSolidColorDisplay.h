// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CQC2SDisplay.h"
#include "DisplayUtils.h"
#include <chrono>
#include <thread>

class CSolidColorDisplay : public CQC2SDisplay
{
    SRGBColor m_color;

public:
    CSolidColorDisplay(SRGBColor p_color, String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)), m_color(p_color) {}

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        using namespace std::chrono;
        const auto FRAME_DURATION = 50ms;
        const auto FRAME_START = steady_clock::now();

        SendMonoColorFrame(p_communicator, m_color);

        const auto ELAPSED = steady_clock::now() - FRAME_START;
        const auto REMAINING = FRAME_DURATION - ELAPSED;
        if (REMAINING > REMAINING.zero())
            std::this_thread::sleep_for(REMAINING);
        return true;
    }
};
