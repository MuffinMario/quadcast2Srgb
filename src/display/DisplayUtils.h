// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../communicator/CQuadcast2SCommunicator.h"
#include "../hid/Packets.h"
#include "../video/VideoConstants.h"
#include <cstring>

// Number of sub-parts in one full color frame (0-5)
static constexpr uint32_t g_DISPLAY_SUBPART_COUNT = 6;
// Number of addressable LEDs per sub-part (last sub-part only uses g_DISPLAY_LAST_SUBPART_LED_COUNT)
static constexpr size_t g_DISPLAY_LEDS_PER_SUBPART = 20;
static constexpr size_t g_DISPLAY_LAST_SUBPART_LED_COUNT = 8;

inline void HandleColorSendResponse(CQuadcast2SCommunicator &p_communicator)
{
    // activate erorr reponse headers: if (rand() % 51 != 0) return;
    const bool READ_RESPONSE = true; // config
    if (!READ_RESPONSE)
        return;    
    // wait for response
    const uint32_t RESPONSE_TIMEOUT_MS = 100; // config
    auto responses = p_communicator.ReceiveResponse(sizeof(SQuadcast2ResponseHandshakePacket), RESPONSE_TIMEOUT_MS);

    for (auto response : responses)
    {
        if (response.size() == sizeof(UQuadcast2CommandPacket))
        {
            UQuadcast2CommandPacket *pColorResponse = reinterpret_cast<UQuadcast2CommandPacket *>(response.data());
#ifdef DEBUG
            constexpr bool STRICT_CHECK = true;
            if constexpr (STRICT_CHECK)
            {
                // ensure report device and subpart values are expected
                auto headdata = pColorResponse->m_handshakeResponsePacket.m_reportId == 0xFF &&
                    (pColorResponse->m_handshakeResponsePacket.m_devicePart == 0x1) &&
                    pColorResponse->m_handshakeResponsePacket.m_subPartId == 0x0;
                if (!headdata)
                {
                    LOG_VERBOSE(L"Received unexpected response header data: reportId=" << std::hex << static_cast<int>(pColorResponse->m_handshakeResponsePacket.m_reportId)
                                << " devicePart=" << static_cast<int>(pColorResponse->m_handshakeResponsePacket.m_devicePart)
                                << " subPartId=" << pColorResponse->m_handshakeResponsePacket.m_subPartId << std::dec);
                                break;
                }
                // all bytes except index 0x10 and 0x11 in unknown are 0
                for (size_t i = 0; i < sizeof(pColorResponse->m_handshakeResponsePacket.m_unknown); ++i)
                {
                    auto val = pColorResponse->m_handshakeResponsePacket.m_unknown[i];
                    switch (i)
                    {
                    case 10:
                        // assume value is 0x68
                        if (val != 68)
                        {
                            LOG_VERBOSE(L"Received unexpected value in color response at index " << i << ": " << static_cast<int>(val));
                        }
                        break;
                    case 11:
                        // assume value is device id
// we dont really track which device we send right now, not important right now
                        if (val != 2 && val != 1)
                        {
                            LOG_VERBOSE(L"Received unexpected value in color response at index " << i << ": " << static_cast<int>(val));
                        }
                        break;

                    default:
                        if (val != 0)
                        {
                            LOG_VERBOSE(L"Received unexpected non-zero byte in color response at index " << i << ": " << static_cast<int>(val));
                        }
                        break;
                    }
                }
            }
#endif
        }
        else
        {
            LOG_ERROR(L"Received unexpected response of size " << response.size());
        }
    }
}
// Builds and sends the trigger packet that precedes every color frame.
// devicePart=1, subPartId=6, reportId=0x44
inline void SendColorFrameTrigger(CQuadcast2SCommunicator &p_communicator)
{
    UQuadcast2CommandPacket triggerPacket{};
    triggerPacket.m_colorPacket.m_reportId = 0x44;
    triggerPacket.m_colorPacket.m_devicePart = 1;
    triggerPacket.m_colorPacket.m_subPartId = 6;
    p_communicator.SendCommand(triggerPacket);
    // wait for response
    HandleColorSendResponse(p_communicator);
}

// Send one color to ALL LEDs
inline void SendMonoColorFrame(CQuadcast2SCommunicator &p_communicator, SRGBColor p_color)
{
    SendColorFrameTrigger(p_communicator);

    UQuadcast2CommandPacket colorPacket{};
    colorPacket.m_colorPacket.m_reportId = 0x44;
    colorPacket.m_colorPacket.m_devicePart = 2;

    // fill color data
    colorPacket.m_colorPacket.m_color.fill(p_color);

    for (uint32_t subPart = 0; subPart < g_DISPLAY_SUBPART_COUNT; ++subPart)
    {
        colorPacket.m_colorPacket.m_subPartId = subPart;

        // 0 the trailing bytes on the last subpart
        if (subPart == g_DISPLAY_SUBPART_COUNT - 1)
            std::fill(colorPacket.m_colorPacket.m_color.begin() + g_DISPLAY_LAST_SUBPART_LED_COUNT,
                      colorPacket.m_colorPacket.m_color.end(),
                      SRGBColor{0, 0, 0});

        p_communicator.SendCommand(colorPacket);
        // wait for response
        HandleColorSendResponse(p_communicator);
    }
}

// Sends full color frame data for all LEDs.
inline void SendColorFrame(CQuadcast2SCommunicator &p_communicator, const SRGBColor *p_pFrame)
{
    SendColorFrameTrigger(p_communicator);

    // Single packet instance - header fields set once, only subPartId and color payload change.
    UQuadcast2CommandPacket colorPacket{};
    colorPacket.m_colorPacket.m_reportId = 0x44;
    colorPacket.m_colorPacket.m_devicePart = 2;

    for (uint32_t subPart = 0; subPart < g_DISPLAY_SUBPART_COUNT; ++subPart)
    {
        const size_t LED_COUNT = (subPart < g_DISPLAY_SUBPART_COUNT - 1) ? g_DISPLAY_LEDS_PER_SUBPART
                                                                         : g_DISPLAY_LAST_SUBPART_LED_COUNT;
        const size_t LED_OFFSET = subPart * g_DISPLAY_LEDS_PER_SUBPART;

        colorPacket.m_colorPacket.m_subPartId = subPart;

        std::memcpy(colorPacket.m_colorPacket.m_color.data(),
                    p_pFrame + LED_OFFSET,
                    LED_COUNT * sizeof(SRGBColor));

        // Zero trailing slots on the last sub-part so stale data isn't sent.
        if (LED_COUNT < g_DISPLAY_LEDS_PER_SUBPART)
            std::memset(colorPacket.m_colorPacket.m_color.data() + LED_COUNT,
                        0,
                        (g_DISPLAY_LEDS_PER_SUBPART - LED_COUNT) * sizeof(SRGBColor));

        p_communicator.SendCommand(colorPacket);
        // wait for response
        HandleColorSendResponse(p_communicator);
    }
}
