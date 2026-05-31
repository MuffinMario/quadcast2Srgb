// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include <hidapi/hidapi.h>
#include <iostream>
#include <stdexcept>

class CHIDException : public std::runtime_error
{
public:
    CHIDException(const String &p_message)
        : std::runtime_error(p_message) {}
};

struct SUSBID
{
    uint16_t m_vendorId;
    uint16_t m_productId;
};

struct SHIDDeviceDeleter
{
    void operator()(hid_device *p_pDev) const noexcept
    {
        if (p_pDev)
        {
            hid_device_info *pInfo = hid_get_device_info(p_pDev);
            if (pInfo)
            {
                WString manufacturer = pInfo->manufacturer_string ? WString(pInfo->manufacturer_string) : L"";
                WString product = pInfo->product_string ? WString(pInfo->product_string) : L"";
                WString serial = pInfo->serial_number ? WString(pInfo->serial_number) : L"";
                LOG_VERBOSE(
                    L"Closing device: " << manufacturer.c_str() << 
                    L" " << product.c_str() << 
                    L" (VID: " << std::hex << pInfo->vendor_id << 
                    L" PID: " << pInfo->product_id << std::dec << 
                    L" Serial: " << serial.c_str() << L")");
            }
            hid_close(p_pDev);
        }
    }
};

using HIDDevicePtr = std::shared_ptr<hid_device>;
inline HIDDevicePtr MakeHIDDevicePtr(hid_device *p_pDev)
{
    return HIDDevicePtr(p_pDev, SHIDDeviceDeleter{});
}

using HIDDeviceContainer = DynamicContainer<HIDDevicePtr>;
