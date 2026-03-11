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
#if DEBUG
            hid_device_info *pInfo = hid_get_device_info(p_pDev);
            if (pInfo)
                std::wcout << "Closing device: ";
            {
                std::wcout << pInfo->manufacturer_string << " " << pInfo->product_string << " (VID: " << std::hex << pInfo->vendor_id << " PID: " << pInfo->product_id << std::dec << " Serial: " << pInfo->serial_number << ")";
            }
            std::wcout << std::endl;
#endif
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
