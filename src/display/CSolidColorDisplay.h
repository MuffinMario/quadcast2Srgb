#pragma once

#include "CQC2SDisplay.h"
#include "DisplayUtils.h"
#include <thread>

class CSolidColorDisplay : public CQC2SDisplay
{
    SRGBColor m_color;

public:
    CSolidColorDisplay(SRGBColor p_color, String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)), m_color(p_color) {}

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        SendMonoColorFrame(p_communicator, m_color);
        std::this_thread::sleep_for(50ms); // solid doesnt need perfect delta timing, so w/e
        return true;
    }
};
