// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "../Globals.h"
#include "../display/CIRenderer.h"
#include "../hid/HIDTypes.h"
#include "../hid/Packets.h"
#include "../video/VideoConstants.h"
#include <algorithm>
#include <cstring>

/// Hardware (HID) implementation of IRenderer.  Sends color frames to one or
/// more connected QuadCast 2S microphones over USB HID.
class CQuadcast2SCommunicator : public CIRenderer
{
    HIDDeviceContainer m_devices;
    mutable Mutex m_mutex;

    // Number of sub-parts in one full color frame (0-5)
    static constexpr uint32_t g_DISPLAY_SUBPART_COUNT = 6;
    // Number of addressable LEDs per sub-part (last sub-part only uses g_DISPLAY_LAST_SUBPART_LED_COUNT)
    static constexpr size_t g_DISPLAY_LEDS_PER_SUBPART = 20;
    static constexpr size_t g_DISPLAY_LAST_SUBPART_LED_COUNT = 8;

    // Send to a single device; caller must NOT hold m_mutex.
    // Returns the hid_write result; on failure the device is removed.
    int Send(HIDDevicePtr p_device, const uint8_t *p_pData, size_t p_size)
    {
        int writeRes = hid_write(p_device.get(), p_pData, p_size);
        if (writeRes < 0)
        {
            LOG_ERROR(L"Failed to write to device: " << hid_error(p_device.get())
                      << L" - removing from list.");
            RemoveDevice(p_device);
        }
        else if (writeRes != static_cast<int>(p_size))
        {
            LOG_ERROR(L"Partial write to device: " << writeRes << L" bytes");
        }
        return writeRes;
    }

    // Read from a single device; caller must NOT hold m_mutex.
    // Returns the hid_read_timeout result; on failure the device is removed.
    int Read(HIDDevicePtr p_device, uint8_t *p_pData, size_t p_size, uint32_t p_timeoutMS)
    {
        int readRes = hid_read_timeout(p_device.get(), p_pData, p_size, p_timeoutMS);
        if (readRes < 0)
        {
            LOG_ERROR(L"Failed to read from device: " << hid_error(p_device.get())
                      << L" - removing from list.");
            RemoveDevice(p_device);
        }
        return readRes;
    }

    // ── IRenderer implementation helpers ──────────────────────────────────

    void HandleColorSendResponse()
    {
        if (g_noWaitForRead)
            return;
        const uint32_t RESPONSE_TIMEOUT_MS = 100;
        auto responses = ReceiveResponse(sizeof(SQuadcast2ResponseHandshakePacket), RESPONSE_TIMEOUT_MS);

        for (auto response : responses)
        {
            if (response.size() == sizeof(UQuadcast2CommandPacket))
            {
                UQuadcast2CommandPacket *pColorResponse = reinterpret_cast<UQuadcast2CommandPacket *>(response.data());
#ifdef DEBUG
                constexpr bool STRICT_CHECK = true;
                if constexpr (STRICT_CHECK)
                {
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
                    for (size_t i = 0; i < sizeof(pColorResponse->m_handshakeResponsePacket.m_unknown); ++i)
                    {
                        auto val = pColorResponse->m_handshakeResponsePacket.m_unknown[i];
                        switch (i)
                        {
                        case 10:
                            if (val != 68)
                                LOG_VERBOSE(L"Received unexpected value in color response at index " << i << ": " << static_cast<int>(val));
                            break;
                        case 11:
                            if (val != 2 && val != 1)
                                LOG_VERBOSE(L"Received unexpected value in color response at index " << i << ": " << static_cast<int>(val));
                            break;
                        default:
                            if (val != 0)
                                LOG_VERBOSE(L"Received unexpected non-zero byte in color response at index " << i << ": " << static_cast<int>(val));
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

    /// Builds and sends the trigger packet that precedes every color frame.
    void SendColorFrameTrigger()
    {
        UQuadcast2CommandPacket triggerPacket{};
        triggerPacket.m_colorPacket.m_reportId = 0x44;
        triggerPacket.m_colorPacket.m_devicePart = 1;
        triggerPacket.m_colorPacket.m_subPartId = 6;
        SendCommand(triggerPacket);
        HandleColorSendResponse();
    }

public:
    CQuadcast2SCommunicator() = default;

    // ── IRenderer interface ────────────────────────────────────────────────

    void RenderMonoFrame(SRGBColor p_color) override
    {
        SendColorFrameTrigger();

        UQuadcast2CommandPacket colorPacket{};
        colorPacket.m_colorPacket.m_reportId = 0x44;
        colorPacket.m_colorPacket.m_devicePart = 2;
        colorPacket.m_colorPacket.m_color.fill(p_color);

        for (uint32_t subPart = 0; subPart < g_DISPLAY_SUBPART_COUNT; ++subPart)
        {
            colorPacket.m_colorPacket.m_subPartId = subPart;

            if (subPart == g_DISPLAY_SUBPART_COUNT - 1)
                std::fill(colorPacket.m_colorPacket.m_color.begin() + g_DISPLAY_LAST_SUBPART_LED_COUNT,
                          colorPacket.m_colorPacket.m_color.end(),
                          SRGBColor{0, 0, 0});

            SendCommand(colorPacket);
            HandleColorSendResponse();
        }
    }

    void RenderFrame(const SRGBColor *p_pFrame) override
    {
        SendColorFrameTrigger();

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

            if (LED_COUNT < g_DISPLAY_LEDS_PER_SUBPART)
                std::memset(colorPacket.m_colorPacket.m_color.data() + LED_COUNT,
                            0,
                            (g_DISPLAY_LEDS_PER_SUBPART - LED_COUNT) * sizeof(SRGBColor));

            SendCommand(colorPacket);
            HandleColorSendResponse();
        }
    }

    // ── Device management ──────────────────────────────────────────────────

    // Add a single verified device.  Ignores duplicates (same pointer).
    void AddDevice(HIDDevicePtr p_device)
    {
        LockGuard lock(m_mutex);
        auto it = std::find(m_devices.begin(), m_devices.end(), p_device);
        if (it == m_devices.end())
            m_devices.push_back(std::move(p_device));
    }

    bool RemoveDevice(const HIDDevicePtr &p_device)
    {
        LockGuard lock(m_mutex);
        auto it = std::find(m_devices.begin(), m_devices.end(), p_device);
        if (it != m_devices.end())
        {
            m_devices.erase(it);
            return true;
        }
        return false;
    }

    Set<WString> GetOpenSerials() const
    {
        LockGuard lock(m_mutex);
        Set<WString> serials;
        for (const auto &pDev : m_devices)
        {
            hid_device_info *pInfo = hid_get_device_info(pDev.get());
            if (pInfo && pInfo->serial_number)
                serials.emplace(pInfo->serial_number);
        }
        return serials;
    }

    bool IsEmpty() const
    {
        LockGuard lock(m_mutex);
        return m_devices.empty();
    }

    std::set<String> GetOpenPaths() const
    {
        LockGuard lock(m_mutex);
        std::set<String> paths;
        for (const auto &pDev : m_devices)
        {
            hid_device_info *pInfo = hid_get_device_info(pDev.get());
            if (pInfo && pInfo->path)
                paths.insert(pInfo->path);
        }
        return paths;
    }

    DynamicContainer<int> SendCommand(UQuadcast2CommandPacket &p_commandPacket)
    {
        return SendCommand(p_commandPacket.m_rawData.data(), p_commandPacket.m_rawData.size());
    }

    DynamicContainer<int> SendCommand(const uint8_t *p_pData, size_t p_size)
    {
        // Snapshot under lock so we don't hold the mutex during HID I/O.
        HIDDeviceContainer snapshot;
        {
            LockGuard lock(m_mutex);
            snapshot = m_devices;
        }
        DynamicContainer<int> results(snapshot.size(), -1);
        for (size_t i = 0; i < snapshot.size(); ++i)
            results[i] = Send(snapshot[i], p_pData, p_size);
        
        return results;
    }

    DynamicContainer<DynamicByteContainer> ReceiveResponse(size_t p_bufferSize, uint32_t p_timeout)
    {
        HIDDeviceContainer snapshot;
        {
            LockGuard lock(m_mutex);
            snapshot = m_devices;
        }
        DynamicContainer<DynamicByteContainer> responses(snapshot.size());
        for (size_t i = 0; i < snapshot.size(); ++i)
        {
            DynamicByteContainer buffer(p_bufferSize);
            // note: if we have multiple devices, timeouts are cumulative... WIP if this is a problem
            // Just use std::async at some point
            // or use thread pools since we dont need to deal with thread creation and destruction overhead
            int res = Read(snapshot[i], buffer.data(), buffer.size(), p_timeout);
            if (res < 0)
            {
                responses[i] = {};
            }
            else if (res > 0)
            {
                buffer.resize(res);
                responses[i] = std::move(buffer);
            }
            // else: timeout, leave responses[i] empty
        }
        return responses;
    }
};
