#pragma once

#include "CQC2SDisplay.h"
#include <thread>

class CSolidColorDisplay : public CQC2SDisplay
{
    SRGBColor m_color;

public:
    CSolidColorDisplay(SRGBColor p_color, String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)), m_color(p_color) {}

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        UQuadcast2CommandPacket triggerPacket{};
        triggerPacket.m_colorPacket.m_reportId = 0x44;
        triggerPacket.m_colorPacket.m_devicePart = 1;
        triggerPacket.m_colorPacket.m_subPartId = 6;
        p_communicator.SendCommand(triggerPacket);
        // TBI: Receive response

        for (uint32_t subPart = 0; subPart < 6; ++subPart)
        {
            size_t colorCount = (subPart < 5) ? 20 : 8;
            UQuadcast2CommandPacket colorPacket{};
            colorPacket.m_colorPacket.m_reportId = 0x44;
            colorPacket.m_colorPacket.m_devicePart = 2;
            colorPacket.m_colorPacket.m_subPartId = subPart;

            for (size_t j = 0; j < colorCount; ++j)
                colorPacket.m_colorPacket.m_color[j] = m_color;

            p_communicator.SendCommand(colorPacket);
            // TBI: Receive response
        }
        std::this_thread::sleep_for(50ms); // solid doesnt need perfect delta timing, so w/e
        return true;
    }
};
