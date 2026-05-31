// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "HIDTypes.h"
#include "Packets.h"
#include "../Globals.h"
#include <iostream>

class CQuadcast2SHandshaker
{
    static constexpr uint32_t g_HANDSHAKE_TIMEOUT_MS = 1000;

public:
    // Probe p_device with the handshake packet.
    // Returns p_device on success, nullptr if the interface did not respond
    // with the expected 0x11 report-ID.
    static HIDDevicePtr ConnectOne(const HIDDevicePtr &p_device)
    {
        UQuadcast2CommandPacket pkt{};
        pkt.m_handshakePacket.m_reportId = 0x10;
        pkt.m_handshakePacket.m_devicePart = 1;
        pkt.m_handshakePacket.m_subPartId = 0;

        int writeRes = hid_write(p_device.get(), pkt.m_rawData.data(), pkt.m_rawData.size());
        if (writeRes < 0)
        {
            LOG_ERROR(L"[Handshake] Write failed: " << hid_error(p_device.get()));
            ;
            return nullptr;
        }

        DynamicByteContainer buf(sizeof(UQuadcast2CommandPacket));
        int readRes = hid_read_timeout(p_device.get(), buf.data(), buf.size(), g_HANDSHAKE_TIMEOUT_MS);

        if (readRes < 0)
        {
            LOG_ERROR(L"[Handshake] Read failed: " << hid_error(p_device.get()));
            return nullptr;
        }

        if (readRes != static_cast<int>(sizeof(UQuadcast2CommandPacket)))
        {
            LOG_ERROR(L"[Handshake] Incomplete response: got " << readRes << L" bytes.");
            return nullptr;
        }

        const auto *pResp = reinterpret_cast<const UQuadcast2CommandPacket *>(buf.data());
        if (pResp->m_handshakeResponsePacket.m_reportId != 0x11)
        {
            LOG_ERROR(L"[Handshake] Unexpected reportId=0x"
                      << std::hex << static_cast<int>(pResp->m_handshakeResponsePacket.m_reportId)
                      << std::dec);
            return nullptr;
        }

        // nvm [[maybe_unused]] 
        //hid_device_info *pInfo = hid_get_device_info(p_device.get());
        return p_device;
    }
};
