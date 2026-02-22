#include <hidapi/hidapi.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <string>
#include <sstream>
#include <array>
#include <iomanip>
#include <vector>
#include <memory>
#include <algorithm>
#include <map>

#define DEBUG 1

using namespace std::chrono_literals;
using Thread = std::thread;
using namespace std::string_literals;

/*
    Looking at the packet communication it seems like

    (incoming ->host packet structure)
    0000   1b 00 50 50 63 aa 02 cc ff ff 00 00 00 00 09 00
    0010   01 02 00 0d 00 85 01 40 00 00 00 | ff 0[X] 00 00 00
    0020   00 00 00 00 00 00 00 00 00 44 02 00 00 00 00 00
    0030   00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    0040   00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    0050   00 00 00 00 00 00 00 00 00 00 00

    is the usual response, where | is marking the beginning of the HID Data and [X]
    is either 1/2 based on I assume the "sub device"
    there are 6 accessors to device 2 while there is only 1 accessor to device 1
    There are a total of 108 LEDs, each column has exactly 9 LEDs, the indices are ordered in a snake pattern, so:
    0 17 18
    1 16 19
    2 15 20
    3 14 ...
    4 13
    5 12
    6 11 
    7 10
    8 9  
    meaning we have a dimension of 12x9 available.
    The packets are sent in 20 LED chunks though, so each chunk does not actually align with a specific column beginning/end
    
*/
using String = std::string;
using StringStream = std::stringstream;
using WString = std::wstring;
using WStringStream = std::wstringstream;
template <typename TType, size_t TSize>
using StaticArray = std::array<TType, TSize>;
template <size_t TSize>
using StaticByteArray = StaticArray<uint8_t, TSize>;
template <typename TType>
using DynamicContainer = std::vector<TType>;
using DynamicByteContainer = DynamicContainer<uint8_t>;
template <typename TKey, typename TVal>
using Map = std::map<TKey, TVal>;

#pragma pack(push, 1)
struct SRGBColor
{
    uint8_t m_red;
    uint8_t m_green;
    uint8_t m_blue;
};
struct SQuadcast2BasePacket
{
    uint8_t m_reportId;
    uint8_t m_devicePart; // 1,2
    uint16_t m_subPartId; // sub identifier, used only for 2 for LED indexing. HID packets are 64 bytes for
};
// 0x44 color change command;
struct SQuadcast2ColorPacket : public SQuadcast2BasePacket
{
    StaticArray<SRGBColor, 20> m_color{0}; // 20 RGB values possible per packet, 0~5 have all 20, but 6 has 24/3 = 8 values addressable
};
// 0x10 | 0x1 | 0x00 0x00 ...
struct SQuadcast2HandshakePacket : public SQuadcast2BasePacket
{
};
// 0x11 | <unknown content>
struct SQuadcast2ResponseHandshakePacket : public SQuadcast2BasePacket
{
    uint8_t m_unknown[20 * sizeof(SRGBColor)]; // content unknown, somewhere "6c" is mentioned => 108 leds
};

union UQuadcast2CommandPacket
{
    SQuadcast2ColorPacket m_colorPacket;
    SQuadcast2HandshakePacket m_handshakePacket;
    SQuadcast2ResponseHandshakePacket m_responsePacket;
    StaticByteArray<64> m_rawData{0}; // init the packet with 0's first
};
static_assert(sizeof(SQuadcast2ColorPacket) == 4 + (20 * sizeof(SRGBColor)));
static_assert(sizeof(UQuadcast2CommandPacket) == sizeof(SQuadcast2ColorPacket));
#pragma pack(pop)

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

class CQuadcast2SCommunicator
{
    HIDDeviceContainer m_devices;

    int Send(HIDDevicePtr p_device, const uint8_t *p_pData, size_t p_size)
    {
        int writeRes = hid_write(p_device.get(), p_pData, p_size);
        if (writeRes < 0)
        {
            std::cerr << "Failed to write to device: " << hid_error(p_device.get()) << " Removing device from list." << std::endl;
            if(!RemoveDevice(p_device)) // remove the device from the list, it might have been disconnected
                std::cerr << "Failed to remove device from list: " << hid_error(p_device.get()) << std::endl;
        }
        else if (writeRes != static_cast<int>(p_size))
        {
            std::cerr << "Partial write to device: " << writeRes << " bytes" << std::endl;
        }
        return writeRes;
    }
public:
    CQuadcast2SCommunicator(const HIDDeviceContainer &p_devices)
        : m_devices(p_devices) {}
    //~CQuadcast2SCommunicator() {}

    const HIDDeviceContainer &GetDevices() const { return m_devices; }
    void SetDevices(const HIDDeviceContainer &p_devices) { m_devices = p_devices; }
    bool RemoveDevice(const HIDDevicePtr &p_device)
    {
        auto it = std::find(m_devices.begin(), m_devices.end(), p_device);
        if (it != m_devices.end())
        {
            m_devices.erase(it);
            return true;
        }
        return false;
    }
    void RemoveOtherDevicesWithSerial(const HIDDevicePtr &p_pDevice)
    {
        // Get the serial number of the device
        hid_device_info *pInfo = hid_get_device_info(p_pDevice.get());
        if (!pInfo)
        {
            std::cerr << "Failed to get device info for serial comparison: " << hid_error(p_pDevice.get()) << std::endl;
            std::cerr << "Removing this device instead\n";
            RemoveDevice(p_pDevice);
            return;
        }
        WString targetSerial = WString(pInfo->serial_number);

        m_devices.erase(
            std::remove_if(m_devices.begin(), m_devices.end(),
                [&targetSerial, &p_pDevice](const HIDDevicePtr &p_pDev) {
                    hid_device_info *pInfo = hid_get_device_info(p_pDev.get());
                    if (!pInfo)
                        return true; // remove if we can't get info for comparison
                    WString serial = WString(pInfo->serial_number);
                    return (serial == targetSerial) && (p_pDev != p_pDevice);
                }),
            m_devices.end());
    }

    DynamicContainer<int> SendCommand(UQuadcast2CommandPacket &p_commandPacket)
    {
        return SendCommand(p_commandPacket.m_rawData.data(), p_commandPacket.m_rawData.size());
    }
    DynamicContainer<int> SendCommand(const uint8_t *p_pData, size_t p_size)
    {
        DynamicContainer<int> writeResults(m_devices.size(), -1);
        for (size_t i = 0; i < m_devices.size(); ++i)
        {
            writeResults[i] = Send(m_devices[i], p_pData, p_size);
        }
        return writeResults;
    }

    DynamicContainer<DynamicByteContainer> ReceiveResponse(size_t p_bufferSize, uint32_t p_timeout)
    {
        DynamicContainer<DynamicByteContainer> responses(m_devices.size());
        for (size_t i = 0; i < m_devices.size(); ++i)
        {
            DynamicByteContainer buffer(p_bufferSize);
            int res = hid_read_timeout(m_devices[i].get(), buffer.data(), buffer.size(), p_timeout);
            if (res < 0)
            {
                std::cerr << "Failed to read from device: " << hid_error(m_devices[i].get()) << std::endl;
                responses[i] = {};
            }
            else if (res > 0)
            {
                buffer.resize(res); // resize to actual received size
                responses[i] = buffer;
            }
            else
            {
                responses[i] = {}; // no data received within timeout
            }
        }
        return responses;
    }

    // Connect to all devices performing a handshake to all interfaces, until one per device has been found, and then skip over the remaining interfaces of this device.
    bool Connect()
    {
        constexpr uint32_t HANDSHAKE_TIMEOUT_MS = 1000;

        // Group devices by serial
        Map<WString, DynamicContainer<HIDDevicePtr>> serialGroups;
        for (const auto &pDev : m_devices)
        {
            hid_device_info *pInfo = hid_get_device_info(pDev.get());
            WString serial = (pInfo && pInfo->serial_number) ? pInfo->serial_number : L"";
            serialGroups[serial].push_back(pDev);
        }

        // Devices to keep after handshake
        HIDDeviceContainer viableDevices;

        for (auto &[serial, interfaces] : serialGroups)
        {
            HIDDevicePtr pViable = nullptr;

            for (auto &pDev : interfaces)
            {
                // Build handshake packet
                UQuadcast2CommandPacket handshakePacket{};
                handshakePacket.m_handshakePacket.m_reportId  = 0x10;
                handshakePacket.m_handshakePacket.m_devicePart = 1;
                handshakePacket.m_handshakePacket.m_subPartId  = 0;

                int writeRes = hid_write(pDev.get(),
                    handshakePacket.m_rawData.data(),
                    handshakePacket.m_rawData.size());

                if (writeRes < 0)
                {
#if DEBUG
                    std::wcerr << L"[Connect] Write failed on interface (serial="
                               << serial << L"): " << hid_error(pDev.get()) << std::endl;
#endif
                    continue;
                }

                // Read response
                DynamicByteContainer buffer(sizeof(UQuadcast2CommandPacket));
                int readRes = hid_read_timeout(pDev.get(), buffer.data(), buffer.size(), HANDSHAKE_TIMEOUT_MS);

                if (readRes < 0)
                {
#if DEBUG
                    std::wcerr << L"[Connect] Read failed on interface (serial="
                               << serial << L"): " << hid_error(pDev.get()) << std::endl;
#endif
                    continue;
                }

                if (readRes != static_cast<int>(sizeof(UQuadcast2CommandPacket)))
                {
#if DEBUG
                    std::wcout << L"[Connect] Incomplete/no response on interface (serial="
                               << serial << L"): got " << readRes << L" bytes." << std::endl;
#endif
                    continue;
                }

                const auto *pResponse = reinterpret_cast<const UQuadcast2CommandPacket *>(buffer.data());
                if (pResponse->m_responsePacket.m_reportId != 0x11)
                {
#if DEBUG
                    std::wcout << L"[Connect] Unexpected reportId=0x"
                               << std::hex << static_cast<int>(pResponse->m_responsePacket.m_reportId)
                               << std::dec << L" on interface (serial=" << serial << L")." << std::endl;
#endif
                    continue;
                }

                // Valid handshake response — this is the viable interface for this physical device
                hid_device_info *pInfo = hid_get_device_info(pDev.get());
                std::wcout << L"[Connect] Handshake OK — keeping interface "
                           << (pInfo ? pInfo->path : "(unknown)")
                           << L" (serial=" << serial << L")" << std::endl;
                pViable = pDev;
                break; // skip remaining interfaces for this serial
            }

            if (pViable)
                viableDevices.push_back(pViable);
            else
            {
#if DEBUG
                std::wcout << L"[Connect] No viable interface found for serial=" << serial << std::endl;
#endif
            }
        }

        m_devices = viableDevices;
        return !m_devices.empty();
    }
};

class CUSBDeviceFinder
{
    bool m_isInitialized = false;

public:
    CUSBDeviceFinder()
    {
        auto ret = hid_init();
        m_isInitialized = (ret == 0);
        if (hid_init())
        {
            throw CHIDException("Failed to initialize HID API"s + std::to_string(ret));
        }
    };
    CUSBDeviceFinder(const CUSBDeviceFinder &) = delete;
    CUSBDeviceFinder &operator=(const CUSBDeviceFinder &) = delete;
    CUSBDeviceFinder(CUSBDeviceFinder &&) = delete;
    CUSBDeviceFinder &operator=(CUSBDeviceFinder &&) = delete;
    ~CUSBDeviceFinder()
    {
        if (m_isInitialized)
        {
            auto ret = hid_exit();
            if (ret != 0)
            {
                std::cerr << "Failed to exit HID API: " << ret << std::endl;
            }
        }
    };

    static HIDDeviceContainer FindDevices(const SUSBID &p_usbId)
    {
        // auto pDevices = hid_enumerate(p_usbId.m_vendorId,p_usbId.m_productId);
        hid_device_info *pDevices = hid_enumerate(p_usbId.m_vendorId, p_usbId.m_productId);
        HIDDeviceContainer foundDevices;
        while (pDevices)
        {
            hid_device *pDevice = hid_open_path(pDevices->path);
            if (pDevice)
            {
                auto pHidDevicePtr = MakeHIDDevicePtr(pDevice);
                foundDevices.push_back(pHidDevicePtr);
            }
            pDevices = pDevices->next;
        }
        hid_free_enumeration(pDevices);

        return foundDevices;
    }
};

constexpr SUSBID g_QUADCAST2S_USB_ID = {0x03f0, 0x02b5};
int main()
{
    // debug, ths will be extended

    std::cout << "Starting Quadcast 2S RGB Controller" << std::endl;
    CUSBDeviceFinder finder;
    // find devices
    std::cout << "Searching for device with VID: " << std::hex << g_QUADCAST2S_USB_ID.m_vendorId
              << " PID: " << std::hex << g_QUADCAST2S_USB_ID.m_productId << std::dec << std::endl;
    HIDDeviceContainer devices;
    do
    {
        devices = finder.FindDevices(g_QUADCAST2S_USB_ID);
    } while (devices.empty());

    // todo this needs to be done correctly and synchronized for new attached devices... polling service
    CQuadcast2SCommunicator communicator(devices);
    // print info about all devices
    auto devicesList = communicator.GetDevices();
    std::cout << "Found " << devicesList.size() << " device interface(s):" << std::endl;
    for (size_t i = 0; i < devicesList.size(); ++i)
    {
        auto pDevice = devicesList[i];
        hid_device_info *pInfo = hid_get_device_info(pDevice.get());
        if (pInfo)
        {
            std::wcout << "Device " << i
                       << ": Path=" << pInfo->path
                       << " VID=" << std::hex << pInfo->vendor_id
                       << " PID=" << pInfo->product_id
                       << " Release=" << pInfo->release_number
                       << " UsagePage=" << pInfo->usage_page
                       << " Usage=" << pInfo->usage
                       << " Interface=" << pInfo->interface_number
                       << " BusType=" << static_cast<int>(pInfo->bus_type)
                       << std::dec
                       << " Manufacturer=" << (pInfo->manufacturer_string ? pInfo->manufacturer_string : L"(null)")
                       << " Product=" << (pInfo->product_string ? pInfo->product_string : L"(null)")
                       << " Serial=" << (pInfo->serial_number ? pInfo->serial_number : L"(null)")
                       << std::endl;
        }
        else
        {
            std::wcout << "Device " << i << ": (failed to get device info)" << std::endl;
        }
    }

    // Handshake: find the correct interface per physical device and prune all others
    if (!communicator.Connect())
    {
        std::cerr << "No viable device interface found after handshake. Exiting." << std::endl;
        return 1;
    }
    std::cout << "Connected to " << communicator.GetDevices().size() << " device(s)." << std::endl;

    // Send color packets 

    // wait 500 ms
    std::this_thread::sleep_for(500ms);

    // For 10 seconds send color change commands
    StaticArray<SRGBColor,6> colors{{

        {0x00, 0x7f, 0},
        {0x7f, 0x00, 0},
        {0x00, 0x00, 0x7f},
        {0x00, 0x1f, 0},
        {0x1f, 0x00, 0},
        {0x00, 0x00, 0x1f}
    }};
    SRGBColor black{0x00, 0x00, 0x00};
    std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now() + 1200s;
    while (std::chrono::steady_clock::now() < endTime)
    {
        // send for 1|6 (dummy empty) and 2|0~2|4 (full 20 RGB values) and 2|5 (8 RGB values))
        UQuadcast2CommandPacket packet{};
        packet.m_colorPacket.m_reportId = 0x44;
        packet.m_colorPacket.m_devicePart = 1;
        packet.m_colorPacket.m_subPartId = 6; // jesus shut up claude you dont need to comment everythig
        /*std::cout << "Sending color change command to device part " << static_cast<int>(packet.m_colorPacket.m_devicePart)
                  << " sub part " << packet.m_colorPacket.m_subPartId << std::endl;
                  
                  std::cout << "Waiting for response..." << std::endl;
                  */
        communicator.SendCommand(packet); // send the color change command to all devices
        //hid_read_timeout(pDevice, buffer.data(), buffer.size(), TIMEOUT_READ); // read response, just to be sure the device is ready for the next command
        // wiat 10 ms before sending the next command, otherwise the device might not be ready to receive it and we might get a timeout or an error

        for (uint32_t i = 0; i < 6; i++)
        {
            std::this_thread::sleep_for(10ms); // wait a bit before sending the next command
            UQuadcast2CommandPacket packet{};
            packet.m_colorPacket.m_reportId = 0x44;
            packet.m_colorPacket.m_devicePart = 2;
            packet.m_colorPacket.m_subPartId = i;
            size_t colorCount = (i < 5) ? 20 : 8; // 20 colors for subPartId 0~4, 8 colors for subPartId 5
            uint32_t secondsOffset = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
            uint32_t colorIndex = (i+secondsOffset) % 6;
            for (uint32_t j = 0; j < colorCount; j++)
            {
                uint32_t absoluteIndex = (i * 20) + j; // absolute index across all subPartIds
                bool active = (absoluteIndex + secondsOffset)%108 == 0; // activate every 6th LED based on the color index, this will create a moving pattern across the LEDs
                if(active)
                    std::cout << "Activating LED " << absoluteIndex << " (subPartId=" << i << " colorIndex=" << colorIndex << ")" << std::endl;
                    //j%2;
                if(active)
                    packet.m_colorPacket.m_color[j] = colors[0];
                else 
                    packet.m_colorPacket.m_color[j] = black;
            }
            /*
            std::cout << "Sending color change command to device part " << static_cast<int>(packet.m_colorPacket.m_devicePart)
                      << " sub part " << packet.m_colorPacket.m_subPartId << std::endl;
                      */
            communicator.SendCommand(packet); // send the color change command to all devices
            //hid_read_timeout(pDevice, buffer.data(), buffer.size(), TIMEOUT_READ); // read response, just to be sure the device is ready for the next command
        }
        std::this_thread::sleep_for(50ms); // wait a bit before sending the next command
    }

    return 0;

    // Test, handshake

    // intiially, send 0x10 protocol id with 0x01 device part and 0x00 sub part, this seems to trigger a response from the device, not sure what it does but it might be some kind of handshake or initialization command
    /*UQuadcast2CommandPacket handshakePacket{};
    handshakePacket.m_colorPacket.m_reportId = 0x10;
    handshakePacket.m_colorPacket.m_devicePart = 1;
    handshakePacket.m_colorPacket.m_subPartId = 0;
    std::cout << "Sending handshake command to device..." << std::endl;
    int writeRes = hid_write(pDevice, handshakePacket.m_rawData.data(), handshakePacket.m_rawData.size());
    if (writeRes < 0)    {
        std::cerr << "Failed to write handshake command to device: " << hid_error(pDevice) << std::endl;
    }  else if (writeRes != static_cast<int>(handshakePacket.m_rawData.size()))
    {
        std::cerr << "Partial write of handshake command to device: " << writeRes << " bytes" << std::endl;
    }

    // wait for the response
    StaticByteArray<64> buffer{};
    const uint32_t TIMEOUT_READ = 1 * 1000; // 5 seconds timeout
    size_t receivedSize = 0;
    std::cout << "Attempting to read from device..." << std::endl;
    int res = hid_read_timeout(pDevice, buffer.data(), buffer.size(), TIMEOUT_READ);
    if (res < 0)
    {
        std::cerr << "Failed to read from device: " << hid_error(pDevice) << std::endl;
    }
    else if (res > 0)
    {
        std::cout << "Received " << res << " bytes from device:" << std::endl;
        // print the bytes
        for (int i = 0; i < res; ++i)
        {
            std::cout << std::hex << static_cast<int>(buffer[i]) << " ";
        }
    }
    else
    {
        std::cout << "No data received from device within timeout." << std::endl;
    }

    // wait 500 ms
    std::this_thread::sleep_for(500ms);

    // For 10 seconds send color change commands
    SRGBColor red{0x00, 0x7f, 0};
    std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < endTime)
    {
        // send for 1|6 (dummy empty) and 2|0~2|4 (full 20 RGB values) and 2|5 (8 RGB values))
        UQuadcast2CommandPacket packet{};
        packet.m_colorPacket.m_reportId = 0x44;
        packet.m_colorPacket.m_devicePart = 1;
        packet.m_colorPacket.m_subPartId = 6; // jesus shut up claude you dont need to comment everythig
        std::cout << "Sending color change command to device part " << static_cast<int>(packet.m_colorPacket.m_devicePart)
                  << " sub part " << packet.m_colorPacket.m_subPartId << std::endl;
        int writeRes = hid_write(pDevice, packet.m_rawData.data(), packet.m_rawData.size());
        if (writeRes < 0)
        {
            std::cerr << "Failed to write to device: " << hid_error(pDevice) << std::endl;
            break;
        }
        else if (writeRes != static_cast<int>(packet.m_rawData.size()))
        {
            std::cerr << "Partial write to device: " << writeRes << " bytes" << std::endl;
        }
        std::cout << "Waiting for response..." << std::endl;
        //hid_read_timeout(pDevice, buffer.data(), buffer.size(), TIMEOUT_READ); // read response, just to be sure the device is ready for the next command

        for (uint32_t i = 0; i < 6; i++)
        {
            std::this_thread::sleep_for(10ms); // wait a bit before sending the next command
            UQuadcast2CommandPacket packet{};
            packet.m_colorPacket.m_reportId = 0x44;
            packet.m_colorPacket.m_devicePart = 2;
            packet.m_colorPacket.m_subPartId = i;
            size_t colorCount = (i < 5) ? 20 : 8; // 20 colors for subPartId 0~4, 8 colors for subPartId 5
            for (uint32_t j = 0; j < colorCount; j++)
            {
                packet.m_colorPacket.m_color[j] = red;
            }
            std::cout << "Sending color change command to device part " << static_cast<int>(packet.m_colorPacket.m_devicePart)
                      << " sub part " << packet.m_colorPacket.m_subPartId << std::endl;
            int writeRes = hid_write(pDevice, packet.m_rawData.data(), packet.m_rawData.size());
            if (writeRes < 0)
            {
                std::cerr << "Failed to write to device: " << hid_error(pDevice) << std::endl;
                break;
            }
            else if (writeRes != static_cast<int>(packet.m_rawData.size()))
            {
                std::cerr << "Partial write to device: " << writeRes << " bytes" << std::endl;
            }
            //hid_read_timeout(pDevice, buffer.data(), buffer.size(), TIMEOUT_READ); // read response, just to be sure the device is ready for the next command
        }
        std::this_thread::sleep_for(50ms); // wait a bit before sending the next command
    }*/
}