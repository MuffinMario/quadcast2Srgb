#pragma once

#include "HIDTypes.h"
#include "../Globals.h"
#include <algorithm>

class CUSBDeviceFinder
{
    bool m_isInitialized = false;

public:
    CUSBDeviceFinder()
    {
        int ret = hid_init();
        if (ret != 0)
            throw CHIDException("Failed to initialize HID API: " + std::to_string(ret));
        m_isInitialized = true;
    }
    CUSBDeviceFinder(const CUSBDeviceFinder &) = delete;
    CUSBDeviceFinder &operator=(const CUSBDeviceFinder &) = delete;
    CUSBDeviceFinder(CUSBDeviceFinder &&) = delete;
    CUSBDeviceFinder &operator=(CUSBDeviceFinder &&) = delete;
    ~CUSBDeviceFinder()
    {
        if (m_isInitialized)
        {
            int ret = hid_exit();
            if (ret != 0)
                LOG_ERROR(L"Failed to exit HID API: " << ret);
        }
    }

    static HIDDeviceContainer FindDevices(
        const SUSBID &p_usbId,
        const Option<Set<WString>> &p_allowedSerials = std::nullopt,
        const Option<Set<WString>> &p_ignoredSerials = std::nullopt)
    {
        hid_device_info *pList = hid_enumerate(p_usbId.m_vendorId, p_usbId.m_productId);
        HIDDeviceContainer found;
        for (hid_device_info *pCur = pList; pCur; pCur = pCur->next)
        {
            WString serial = pCur->serial_number ? WString(pCur->serial_number) : WString{};
            // Serial filter
            if (p_allowedSerials.has_value())
            {
                const auto &allowed = p_allowedSerials.value();
                if (std::find(allowed.begin(), allowed.end(), serial) == allowed.end())
                    continue;
            }
            if (p_ignoredSerials.has_value())
            {
                const auto &ignored = p_ignoredSerials.value();
                if (std::find(ignored.begin(), ignored.end(), serial) != ignored.end())
                    continue;
            }

            hid_device *pDev = hid_open_path(pCur->path);
            if (pDev)
                found.push_back(MakeHIDDevicePtr(pDev));
        }
        hid_free_enumeration(pList);
        return found;
    }
};
