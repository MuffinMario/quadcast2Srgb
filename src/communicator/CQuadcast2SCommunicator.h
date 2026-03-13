#pragma once

#include "../Common.h"
#include "../Globals.h"
#include "../hid/HIDTypes.h"
#include "../hid/Packets.h"
#include <algorithm>

class CQuadcast2SCommunicator
{
    HIDDeviceContainer m_devices;
    mutable Mutex m_mutex;

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

public:
    CQuadcast2SCommunicator() = default;

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
            int res = hid_read_timeout(snapshot[i].get(), buffer.data(), buffer.size(), p_timeout);
            if (res < 0)
            {
                std::cerr << "Failed to read from device: " << hid_error(snapshot[i].get()) << std::endl;
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

// Forward-declaration-based callback alias used by Display classes.
using FrameCallback = std::function<bool(CQuadcast2SCommunicator &)>;
