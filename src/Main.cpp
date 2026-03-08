#include <hidapi/hidapi.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <string>
#include <cstring>
#include <sstream>
#include <fstream>
#include <array>
#include <iomanip>
#include <vector>
#include <memory>
#include <algorithm>
#include <map>
#include <filesystem>
#include <cmath>
#include <optional>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <queue>
#include <set>
#include <functional>
#include <set>
#include <csignal>

#define DEBUG 1

using namespace std::chrono_literals;
using Thread = std::thread;
using namespace std::string_literals;
template <typename TType>
using Atomic = std::atomic<TType>;

using ConditionVariable = std::condition_variable;

using Mutex = std::mutex;
using UniqueLock = std::unique_lock<Mutex>; // we dont need to template this in our case
using LockGuard = std::lock_guard<Mutex>;
using AtomicBool = Atomic<bool>;
template <typename TVal>
using Queue = std::queue<TVal>;
// Forward declaration needed for FrameCallback alias (defined below the classes).
class CQuadcast2SCommunicator;
using FrameCallback = std::function<bool(CQuadcast2SCommunicator &)>;

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

    There are two device interfaces that response to the what I assume to be the status request packet (0x10 0x01 0 0 0 0 0....)
    However, when you communicate with both at the same time, the LEDs refuse to communicate, we restrict communication with only one interface per usb device
    we split this logic by identifying the device via serial number (hidapi restrictions to not know the exact bus/port/interface ids? to my knowledge)
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
template <typename TKey>
using Set = std::set<TKey>;
template <typename TVal>
using UniquePtr = std::unique_ptr<TVal>;
template <typename TVal>
using Option = std::optional<TVal>;

using IFStream = std::ifstream;
using OFStream = std::ofstream;
using FS = std::filesystem::filesystem_error;
using Path = std::filesystem::path;

static AtomicBool g_verbosity = false;
static AtomicBool g_signalStopRequest;

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

// ---------------------------------------------------------------------------
// CQuadcast2SCommunicator
//
// Owns the set of *verified* (post-handshake) HID device handles and provides
// the sole communication API.  No discovery or handshaking lives here.
//
// Thread-safety: all public methods are guarded by m_mutex so the display
// thread (SendCommand/ReceiveResponse) and the pipeline threads (AddDevice /
// RemoveDevice called during hot-plug) can operate concurrently.
// ---------------------------------------------------------------------------
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
            std::cerr << "Failed to write to device: " << hid_error(p_device.get())
                      << " — removing from list." << std::endl;
            RemoveDevice(p_device);
        }
        else if (writeRes != static_cast<int>(p_size))
        {
            std::cerr << "Partial write to device: " << writeRes << " bytes" << std::endl;
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

    // Return the set of filesystem paths currently held open.
    // ScanThread passes this to EnumerateNew so it never re-enumerates live
    // devices, and so that removed devices automatically become re-discoverable.
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

// ---------------------------------------------------------------------------
// CUSBDeviceFinder
//
// Owns the HID API lifetime (hid_init / hid_exit).
// FindDevices   — enumerate and open *all* matching interfaces.
// EnumerateNew  — same, but skips paths already tracked by the communicator
//                 so that live devices are never re-opened and disconnected
//                 devices automatically become re-discoverable once the
//                 communicator drops their handle.
// ---------------------------------------------------------------------------
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
                std::cerr << "Failed to exit HID API: " << ret << std::endl;
        }
    }

    static HIDDeviceContainer FindDevices(
        const SUSBID &p_usbId,
        const Option<Set<WString>> &p_allowedSerials = std::nullopt,
        const Option<Set<WString>> &p_ignoredSerials = std::nullopt)
    {
        return EnumerateNew(p_usbId, p_ignoredSerials, p_allowedSerials);
    }

    // Open every interface whose path is NOT in p_activePaths.
    // Caller supplies the active-path set from CQuadcast2SCommunicator::GetOpenPaths().
    static HIDDeviceContainer EnumerateNew(
        const SUSBID &p_usbId,
        const Option<Set<WString>> &p_ignoredSerials = std::nullopt,
        const Option<Set<WString>> &p_allowedSerials = std::nullopt)
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

// ---------------------------------------------------------------------------
// CQuadcast2SHandshaker
//
// Takes a bag of raw HID interfaces (all interfaces for potentially multiple
// physical devices) and performs the protocol handshake to identify the single
// correct interface per physical device (serial).
//
// ConnectOne  — handshake a single HIDDevicePtr; returns it on success or
//               nullptr on failure.  Stateless and thread-safe.
// ---------------------------------------------------------------------------
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
            if (g_verbosity)
                std::wcerr << L"[Handshake] Write failed: " << hid_error(p_device.get()) << std::endl;
            return nullptr;
        }

        DynamicByteContainer buf(sizeof(UQuadcast2CommandPacket));
        int readRes = hid_read_timeout(p_device.get(), buf.data(), buf.size(), g_HANDSHAKE_TIMEOUT_MS);

        if (readRes < 0)
        {
            if (g_verbosity)
                std::wcerr << L"[Handshake] Read failed: " << hid_error(p_device.get()) << std::endl;
            return nullptr;
        }

        if (readRes != static_cast<int>(sizeof(UQuadcast2CommandPacket)))
        {
            if (g_verbosity)
                std::wcout << L"[Handshake] Incomplete response: got " << readRes << L" bytes." << std::endl;

            return nullptr;
        }

        const auto *pResp = reinterpret_cast<const UQuadcast2CommandPacket *>(buf.data());
        if (pResp->m_responsePacket.m_reportId != 0x11)
        {
            if (g_verbosity)
                std::wcout << L"[Handshake] Unexpected reportId=0x"
                           << std::hex << static_cast<int>(pResp->m_responsePacket.m_reportId)
                           << std::dec << std::endl;
            return nullptr;
        }

        hid_device_info *pInfo = hid_get_device_info(p_device.get());
        return p_device;
    }
};

constexpr uint32_t g_VIDEO_WIDTH = 12;
constexpr uint32_t g_VIDEO_HEIGHT = 9;
constexpr uint32_t g_LED_COUNT = g_VIDEO_WIDTH * g_VIDEO_HEIGHT; // 108

// Does two things:
// turn row-major frame data into column-major
// flip every odd column because of snake column indicing
DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> AlignIndicesOnVideo(const DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> &p_frames)
{
    DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> alignedFrames;
    alignedFrames.reserve(p_frames.size());

    for (const auto &frame : p_frames)
    {
        StaticArray<SRGBColor, g_LED_COUNT> alignedFrame{};
        for (uint32_t col = 0; col < g_VIDEO_WIDTH; ++col)
        {
            for (uint32_t row = 0; row < g_VIDEO_HEIGHT; ++row)
            {
                // Logical index: row-major input
                uint32_t srcIndex = row * g_VIDEO_WIDTH + col;

                // Physical index: column-major, odd columns reversed
                uint32_t physicalRow = (col % 2 == 1) ? row : (g_VIDEO_HEIGHT - 1 - row);
                uint32_t dstIndex = col * g_VIDEO_HEIGHT + physicalRow;

                alignedFrame[dstIndex] = frame[srcIndex];
            }
        }
        alignedFrames.push_back(alignedFrame);
    }
    return alignedFrames;
}

DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> ProcessGreyscaleVideo(String p_path, size_t p_maxFileSize)
{
    // open the file
    IFStream file(p_path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open video file: " + p_path);
    }
    // Get file size
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    if (fileSize == 0 || fileSize > p_maxFileSize)
    {
        throw std::runtime_error("Invalid video file size: " + std::to_string(fileSize));
    }
    // Warn on file size not being a multiple of LED_COUNT, as this means the last frame will be incomplete and thus discarded
    uint32_t frameCount = fileSize / g_LED_COUNT;
    if (fileSize % g_LED_COUNT != 0)
    {
        std::cerr << "Warning: video file size (" << fileSize << " bytes) is not a multiple of LED count (" << g_LED_COUNT << " bytes). The last " << (fileSize % g_LED_COUNT) << " bytes will be discarded." << std::endl;
    }
    file.seekg(0, std::ios::beg);

    // Read frames
    DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> frames(frameCount);

    for (uint32_t i = 0; i < frameCount; i++)
    {
        StaticByteArray<g_LED_COUNT> frameData{};
        file.read(reinterpret_cast<char *>(frameData.data()), frameData.size());
        // Convert greyscale byte data to SRGBColor, assuming the input is 8-bit greyscale where each byte represents the intensity for R, G, and B equally
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        for (size_t j = 0; j < g_LED_COUNT; ++j)
        {
            uint8_t intensity = frameData[j];
            frame[j] = {intensity, intensity, intensity};
        }
        if (file.gcount() == static_cast<std::streamsize>(frameData.size()))
        {
            frames.push_back(frame);
        }
        else
        {
            if (!file.eof())
            {
                throw std::runtime_error("Error reading video file: " + p_path);
            }
            break; // EOF reached
        }
    }
    file.close();
    return AlignIndicesOnVideo(frames);
}

constexpr size_t g_RGB_FRAME_SIZE = g_LED_COUNT * sizeof(SRGBColor); // 3 bytes per pixel (R, G, B — 8 bits each)
DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> ProcessRGBVideo(String p_path, size_t p_maxFileSize)
{
    IFStream file(p_path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open video file: " + p_path);
    }
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    if (fileSize == 0 || fileSize > p_maxFileSize)
    {
        throw std::runtime_error("Invalid video file size: " + std::to_string(fileSize));
    }
    if (fileSize % g_RGB_FRAME_SIZE != 0)
    {
        std::cerr << "Warning: RGB video file size (" << fileSize << " bytes) is not a multiple of frame size ("
                  << g_RGB_FRAME_SIZE << " bytes). The last " << (fileSize % g_RGB_FRAME_SIZE) << " bytes will be discarded." << std::endl;
    }
    uint32_t frameCount = static_cast<uint32_t>(fileSize / g_RGB_FRAME_SIZE);
    file.seekg(0, std::ios::beg);

    DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> frames;
    frames.reserve(frameCount);

    for (uint32_t i = 0; i < frameCount; ++i)
    {
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        file.read(reinterpret_cast<char *>(frame.data()), g_RGB_FRAME_SIZE);

        if (file.gcount() == static_cast<std::streamsize>(g_RGB_FRAME_SIZE))
        {
            frames.push_back(frame);
        }
        else
        {
            if (!file.eof())
            {
                throw std::runtime_error("Error reading RGB video file: " + p_path);
            }
            break; // incomplete last frame — discard
        }
    }
    file.close();
    return AlignIndicesOnVideo(frames);
}

// Create dummy video that just moves a white column from left to right, for testing purposes
DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> CreateDummyVideoSlide(uint32_t p_frameCount)
{
    DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> frames;
    frames.reserve(p_frameCount);
    for (uint32_t i = 0; i < p_frameCount; ++i)
    {
        // Determine which column is lit: maps frame index linearly across all columns
        uint32_t activeColumn = (i * g_VIDEO_WIDTH) / p_frameCount;

        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        frame.fill({0, 0, 0}); // all black

        // Light up the entire active column (9 LEDs per column)
        for (uint32_t row = 0; row < g_VIDEO_HEIGHT; ++row)
        {
            uint32_t ledIndex = activeColumn * g_VIDEO_HEIGHT + row;
            frame[ledIndex] = {0xff, 0xff, 0xff};
        }

        frames.push_back(frame);
    }
    return AlignIndicesOnVideo(frames);
}

// Create dummy video that scans one pixel at a time left-to-right, top-to-bottom across the entire frame count
DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> CreateDummyVideoPixelScan(uint32_t p_frameCount)
{
    DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>> frames;
    frames.reserve(p_frameCount);
    for (uint32_t i = 0; i < p_frameCount; ++i)
    {
        // Map frame index to a pixel: left-to-right, top-to-bottom
        // pixel index = (i * g_LED_COUNT) / p_frameCount
        uint32_t pixelIndex = (i * g_LED_COUNT) / p_frameCount;
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        frame.fill({0, 0, 0}); // all black
        // light up pixel at x/y in row-major order (video formats)
        frame[pixelIndex] = {0xff, 0xff, 0xff};

        frames.push_back(frame);
    }
    return AlignIndicesOnVideo(frames);
}

class CEndCondition
{
public:
    virtual ~CEndCondition() = default;
    virtual bool IsMet() const = 0;
    virtual void Reset() = 0;
    virtual void NotifyFrameDisplayed() {}
};

class CTimeEndCondition : public CEndCondition
{
    std::chrono::steady_clock::time_point m_endTime;
    std::chrono::milliseconds m_duration;

public:
    explicit CTimeEndCondition(std::chrono::milliseconds p_duration)
        : m_duration(p_duration), m_endTime(std::chrono::steady_clock::now() + p_duration) {}

    bool IsMet() const override
    {
        return std::chrono::steady_clock::now() >= m_endTime;
    }

    void Reset() override
    {
        m_endTime = std::chrono::steady_clock::now() + m_duration;
    }
};

class CVideoCompletedEndCondition : public CEndCondition
{
    size_t m_totalFrames;
    size_t m_renderedFrames = 0;
    int64_t m_loopCount = 1;
    int64_t m_currentLoopCount = 1;

public:
    explicit CVideoCompletedEndCondition(size_t p_totalFrames, int64_t p_loopCount)
        : m_totalFrames(p_totalFrames), m_loopCount(p_loopCount)
    {
    }

    bool IsMet() const override
    {
        return m_currentLoopCount > m_loopCount && m_loopCount >= 0;
    }

    void Reset() override
    {
        m_renderedFrames = 0;
        m_currentLoopCount = 1;
    }

    void NotifyFrameDisplayed() override
    {
        ++m_renderedFrames;
        if (m_renderedFrames >= m_totalFrames)
        {
            m_renderedFrames = 0;
            ++m_currentLoopCount;
        }
    }
};

class CQC2SDisplay
{
protected:
    String m_name;
    String m_nextDisplay; // name of the next display to transition to (empty = none)
    UniquePtr<CEndCondition> m_pEndCondition;

public:
    CQC2SDisplay(String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : m_name(std::move(p_name)), m_nextDisplay(std::move(p_nextDisplay)), m_pEndCondition(std::move(p_pEndCondition))
    {
    }
    virtual ~CQC2SDisplay() = default;

    // Called once before displaying starts; return false to abort
    virtual bool Initialize() { return true; }

    // Called once per frame; return false to stop displaying
    virtual bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) = 0;

    // Called once after displaying ends or is aborted
    virtual void Shutdown(CQuadcast2SCommunicator &p_communicator) {}

    virtual String GetName() const { return m_name; }
    virtual String GetNextDisplay() const { return m_nextDisplay; }
    virtual void SetNextDisplay(String p_nextDisplay) { m_nextDisplay = std::move(p_nextDisplay); }

    virtual void Display(CQuadcast2SCommunicator &p_communicator,
                         const AtomicBool &p_signalStopRequest,
                         FrameCallback p_frameCallback = nullptr)
    {
        // check for end condition
        if (m_pEndCondition.get())
        {
            m_pEndCondition->Reset();
            do
            {
                if (p_frameCallback && !p_frameCallback(p_communicator))
                    break;
                if (p_signalStopRequest.load())
                    break;
                bool displaySuccess = DisplayFrame(p_communicator);
                if (!displaySuccess)
                    break;
                m_pEndCondition->NotifyFrameDisplayed();
            } while (!m_pEndCondition->IsMet() && !p_signalStopRequest.load());
        }
        else
        {
            // no end condition, continue ad infinitum (or until it returns false)
            while (!p_signalStopRequest.load())
            {
                if (p_frameCallback && !p_frameCallback(p_communicator))
                    break;
                if (!DisplayFrame(p_communicator))
                    break;
            }
        }
    }
};

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
        std::this_thread::sleep_for(50ms);
        return true;
    }
};

// Cubic Bézier easing — CSS-style: implicit P0=(0,0), P3=(1,1).
// p_p1 and p_p2 are the two inner control points.
// Given a linear parameter t in [0,1], solves for the Bézier curve parameter s
// such that X(s) == t using Newton–Raphson, then returns Y(s) as the eased value.
struct SCubicBezier
{
    float m_p1x;
    float m_p1y;
    float m_p2x;
    float m_p2y;

    static constexpr SCubicBezier EaseInOut() { return {0.11f, 0.0f, 0.35f, 1.0f}; }
    static constexpr SCubicBezier Linear() { return {0.0f, 0.0f, 1.0f, 1.0f}; }
};

// Evaluate one component of a cubic Bézier with P0=0, P3=1 at parameter s
static inline float CubicBezierComponent(float p_p1, float p_p2, float p_s)
{
    float oneMinusS = 1.0f - p_s;
    // B(s) = 3*(1-s)^2*s*p1 + 3*(1-s)*s^2*p2 + s^3
    return 3.0f * oneMinusS * oneMinusS * p_s * p_p1 + 3.0f * oneMinusS * p_s * p_s * p_p2 + p_s * p_s * p_s;
}

// Derivative of CubicBezierComponent w.r.t. s
static inline float CubicBezierComponentDerivative(float p_p1, float p_p2, float p_s)
{
    float oneMinusS = 1.0f - p_s;
    return 3.0f * oneMinusS * oneMinusS * p_p1 + 6.0f * oneMinusS * p_s * (p_p2 - p_p1) + 3.0f * p_s * p_s * (1.0f - p_p2);
}

// Map linear t -> eased brightness via cubic Bézier
static float CubicBezierEval(const SCubicBezier &p_bezier, float p_t)
{
    if (p_t <= 0.0f)
        return 0.0f;
    if (p_t >= 1.0f)
        return 1.0f;

    // Newton–Raphson: find s such that X(s) == t
    float s = p_t; // initial guess
    for (int iter = 0; iter < 8; ++iter)
    {
        float xS = CubicBezierComponent(p_bezier.m_p1x, p_bezier.m_p2x, s);
        float dxS = CubicBezierComponentDerivative(p_bezier.m_p1x, p_bezier.m_p2x, s);
        if (std::abs(dxS) < 1e-6f)
            break;
        s -= (xS - p_t) / dxS;
        s = std::max(0.0f, std::min(1.0f, s));
    }
    return CubicBezierComponent(p_bezier.m_p1y, p_bezier.m_p2y, s);
}

class CPulseColorDisplay : public CQC2SDisplay
{
    SRGBColor m_baseColor;
    float m_t = 0.0f; // linear phase in [0, 1]
    float m_speed;    // advance per frame (in [0,1] units)
    bool m_increasing = true;
    SCubicBezier m_bezier;

    SRGBColor ApplyBrightness(float p_brightness) const
    {
        return {
            static_cast<uint8_t>(m_baseColor.m_red * p_brightness),
            static_cast<uint8_t>(m_baseColor.m_green * p_brightness),
            static_cast<uint8_t>(m_baseColor.m_blue * p_brightness)};
    }

public:
    CPulseColorDisplay(SRGBColor p_color, float p_speed, String p_name, UniquePtr<CEndCondition> p_pEndCondition,
                       SCubicBezier p_bezier = SCubicBezier::EaseInOut(), String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)),
          m_baseColor(p_color), m_speed(p_speed), m_bezier(p_bezier) {}

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        SRGBColor current = ApplyBrightness(CubicBezierEval(m_bezier, m_t));
        UQuadcast2CommandPacket triggerPacket{};
        triggerPacket.m_colorPacket.m_reportId = 0x44;
        triggerPacket.m_colorPacket.m_devicePart = 1;
        triggerPacket.m_colorPacket.m_subPartId = 6;
        p_communicator.SendCommand(triggerPacket);

        for (uint32_t subPart = 0; subPart < 6; ++subPart)
        {
            size_t colorCount = (subPart < 5) ? 20 : 8;
            UQuadcast2CommandPacket colorPacket{};
            colorPacket.m_colorPacket.m_reportId = 0x44;
            colorPacket.m_colorPacket.m_devicePart = 2;
            colorPacket.m_colorPacket.m_subPartId = subPart;

            for (size_t j = 0; j < colorCount; ++j)
                colorPacket.m_colorPacket.m_color[j] = current;

            p_communicator.SendCommand(colorPacket);
        }

        if (m_increasing)
        {
            m_t += m_speed;
            if (m_t >= 1.0f)
            {
                m_t = 1.0f;
                m_increasing = false;
            }
        }
        else
        {
            m_t -= m_speed;
            if (m_t <= 0.0f)
            {
                m_t = 0.0f;
                m_increasing = true;
            }
        }

        std::this_thread::sleep_for(50ms);
        return true;
    }
};

// Pre-loaded frame buffer type shared across video displays
using VideoFrameBuffer = DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>>;

enum class EVideoFormat
{
    Rgb,
    Greyscale
};

constexpr size_t g_VIDEO_MAX_FILE_SIZE_DEFAULT = 512ULL * 1024 * 1024; // 512 MB

// Load a raw video file from disk into a VideoFrameBuffer, ready for use with CVideoDisplay.
// p_path        : path to the raw binary video file
// p_format      : EVideoFormat::Rgb (3 bytes/pixel) or EVideoFormat::Greyscale (1 byte/pixel)
// p_maxFileSize : maximum allowed file size in bytes (default 512 MB)
VideoFrameBuffer LoadVideoBuffer(const String &p_path, EVideoFormat p_format,
                                 size_t p_maxFileSize = g_VIDEO_MAX_FILE_SIZE_DEFAULT)
{
    switch (p_format)
    {
    case EVideoFormat::Rgb:
        return ProcessRGBVideo(p_path, p_maxFileSize);
    case EVideoFormat::Greyscale:
        return ProcessGreyscaleVideo(p_path, p_maxFileSize);
    default:
        throw std::invalid_argument("Unknown video format");
    }
}

class CVideoDisplay : public CQC2SDisplay
{
    VideoFrameBuffer m_frames;
    uint32_t m_fps;
    size_t m_currentFrame = 0;

public:
    CVideoDisplay(VideoFrameBuffer p_frames, uint32_t p_fps, String p_name,
                  UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)),
          m_frames(std::move(p_frames)), m_fps(p_fps) {}

    bool Initialize() override
    {
        m_currentFrame = 0;
        return !m_frames.empty();
    }

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        if (m_frames.empty())
            return false;

        auto frameStart = std::chrono::steady_clock::now();
        auto frameDuration = std::chrono::milliseconds(1000 / m_fps);

        const auto &frame = m_frames[m_currentFrame];

        UQuadcast2CommandPacket triggerPacket{};
        triggerPacket.m_colorPacket.m_reportId = 0x44;
        triggerPacket.m_colorPacket.m_devicePart = 1;
        triggerPacket.m_colorPacket.m_subPartId = 6;
        p_communicator.SendCommand(triggerPacket);

        UQuadcast2CommandPacket colorPacket{}; // 64 bit aligned
        colorPacket.m_colorPacket.m_reportId = 0x44;
        colorPacket.m_colorPacket.m_devicePart = 2;
        for (uint32_t subPart = 0; subPart < 6; ++subPart)
        {
            size_t colorCount = (subPart < 5) ? 20 : 8;
            size_t ledOffset = subPart * 20;

            colorPacket.m_colorPacket.m_subPartId = subPart;

            /*for (size_t j = 0; j < colorCount; ++j)
                colorPacket.m_colorPacket.m_color[j] = frame[ledOffset + j];*/
            // write directly to memory, data is aligned/prepared beforehand, todo do this for the other displays too...
            std::memcpy(
                colorPacket.m_colorPacket.m_color.data(),
                &frame[ledOffset],
                colorCount * sizeof(SRGBColor));
            // zero fill the remaining leds, just in case
            if (colorCount < 20)
            {
                std::memset(
                    colorPacket.m_colorPacket.m_color.data() + colorCount, // +colorcount * rgbcolor struct
                    0,
                    (20 - colorCount) * sizeof(SRGBColor));
            }

            p_communicator.SendCommand(colorPacket);
        }
        auto elapsed = std::chrono::steady_clock::now() - frameStart;
        #ifdef DEBUG
                //std::wcout << "Displayed frame " << m_currentFrame << " / " << m_frames.size() << " elapsed " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms" << std::endl;
        #endif
        // advance frame, looping when end of video is reached, end condition is decided outside...
        m_currentFrame = (m_currentFrame + 1) % m_frames.size();
        
        // delta wait to align to framerate
        if (elapsed < frameDuration)
            std::this_thread::sleep_for(frameDuration - elapsed);

        return true;
    }
};

class CMultiDisplay : public CQC2SDisplay
{
    DynamicContainer<UniquePtr<CQC2SDisplay>> m_displays;

    String m_initialDisplay;
    Map<String, size_t> m_mapDisplayIndices;
    DynamicContainer<size_t> m_mapIndexTransitions;
    size_t m_currentIndex;

public:
    CMultiDisplay(String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_firstDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_firstDisplay)) {}

    void AddDisplay(UniquePtr<CQC2SDisplay> p_display)
    {
        // add to map
        String name = p_display->GetName();
        if (name.empty())
            throw std::invalid_argument("Display name must not be empty");
        if (m_mapDisplayIndices.find(name) != m_mapDisplayIndices.end())
            throw std::invalid_argument("Display with name '" + name + "' already exists");
        m_mapDisplayIndices[name] = m_displays.size();
        m_displays.push_back(std::move(p_display));
    }

    bool Initialize() override
    {
        // init transition map for multidisplay
        m_mapIndexTransitions.resize(m_mapDisplayIndices.size());
        for (auto &pDisplay : m_displays)
        {
            String thisDisplay = pDisplay->GetName();
            String next = pDisplay->GetNextDisplay();

            auto thisIt = m_mapDisplayIndices.find(thisDisplay);
            if (thisIt == m_mapDisplayIndices.end())
                throw std::runtime_error("Display name '" + thisDisplay + "' not found in index map");

            size_t thisDisplayIndex = thisIt->second;

            // next has been chosen
            if (!next.empty())
            {
                auto nextIt = m_mapDisplayIndices.find(next);
                if (nextIt == m_mapDisplayIndices.end())
                    throw std::runtime_error("Next display name '" + next + "' not found in index map");
                m_mapIndexTransitions[thisDisplayIndex] = nextIt->second;
            }
            // no next given -> assume stop
            else
            {
                m_mapIndexTransitions[thisDisplayIndex] = (size_t)-1;
            }
        }
        // set initial index
        if (m_nextDisplay.empty() || m_mapDisplayIndices.find(m_nextDisplay) == m_mapDisplayIndices.end())
            throw std::runtime_error("Initial display '" + m_nextDisplay + "' not found in display index map");
        m_currentIndex = m_mapDisplayIndices[m_nextDisplay];
        // init all the displays
        for (auto &pDisplay : m_displays)
        {
            if (!pDisplay->Initialize())
                return false;
        }
        return true;
    }

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        // not used!
        return false;
    }

    void Shutdown(CQuadcast2SCommunicator &p_communicator) override
    {
        for (auto &pDisplay : m_displays)
        {
            pDisplay->Shutdown(p_communicator);
        }
    }

    void Display(CQuadcast2SCommunicator &p_communicator,
                 const AtomicBool &p_signalStopRequest,
                 FrameCallback p_frameCallback = nullptr) override
    {
        if (m_displays.empty())
            return;

        do
        {
            // run display of current display
            auto &pDisplay = m_displays[m_currentIndex];
            pDisplay->Display(p_communicator, p_signalStopRequest, p_frameCallback);
            // get next one in line
            auto next = m_mapIndexTransitions[m_currentIndex];
            m_currentIndex = next;
        } while (m_currentIndex != (size_t)-1 && !p_signalStopRequest.load());
    }
};

constexpr SUSBID g_QUADCAST2S_USB_ID = {0x03f0, 0x02b5};

static Option<SRGBColor> ParseHexColor(String p_colorStr)
{
    if (!p_colorStr.empty() && p_colorStr[0] == '#')
        p_colorStr = p_colorStr.substr(1);
    if (p_colorStr.size() != 6)
        return std::nullopt;
    try
    {
        return SRGBColor{
            static_cast<uint8_t>(std::stoul(p_colorStr.substr(0, 2), nullptr, 16)),
            static_cast<uint8_t>(std::stoul(p_colorStr.substr(2, 2), nullptr, 16)),
            static_cast<uint8_t>(std::stoul(p_colorStr.substr(4, 2), nullptr, 16))};
    }
    catch (const std::exception &)
    {
        return std::nullopt;
    }
}

class CQC2SDisplayFactory
{
public:
    static UniquePtr<CQC2SDisplay> CreateSolidColor(SRGBColor p_color, String p_name = "solid", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "")
    {
        return std::make_unique<CSolidColorDisplay>(p_color, std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
    }
    static UniquePtr<CQC2SDisplay> CreatePulseColor(SRGBColor p_color, float p_speed = 0.05f, String p_name = "pulse", UniquePtr<CEndCondition> p_pEndCondition = nullptr, SCubicBezier p_bezier = SCubicBezier::EaseInOut(), String p_nextDisplay = "")
    {
        return std::make_unique<CPulseColorDisplay>(p_color, p_speed, std::move(p_name), std::move(p_pEndCondition), p_bezier, std::move(p_nextDisplay));
    }
    static UniquePtr<CQC2SDisplay> CreateMultiDisplay(String p_name = "multi", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "")
    {
        return std::make_unique<CMultiDisplay>(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
    }
    static UniquePtr<CQC2SDisplay> CreateVideoDisplay(VideoFrameBuffer p_frames, uint32_t p_fps = 30, String p_name = "video", UniquePtr<CEndCondition> p_pEndCondition = nullptr, String p_nextDisplay = "")
    {
        return std::make_unique<CVideoDisplay>(std::move(p_frames), p_fps, std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay));
    }

    static UniquePtr<CQC2SDisplay> CreateFromArgs(int p_argc, char *p_pArgv[])
    {
        // default settings
        String displayType = "solid";
        SRGBColor color = ParseHexColor("#290066").value();
        SCubicBezier bezier = SCubicBezier::EaseInOut();
        float pulseSpeed = 0.025f;
        String videoPath = "";
        uint32_t videoFramerate = 30;
        EVideoFormat videoFormat = EVideoFormat::Rgb;

        for (int i = 1; i < p_argc; ++i)
        {
            String arg = p_pArgv[i];
            std::wcout << WString(arg.begin(), arg.end()) << std::endl;
            if (arg == "--display" && i + 1 < p_argc)
            {
                displayType = p_pArgv[++i];
            }
            else if (arg == "--color" && i + 1 < p_argc)
            {
                auto parsed = ParseHexColor(p_pArgv[++i]);
                if (parsed.has_value())
                    color = parsed.value();
                else
                    std::cerr << "Invalid --color value. Expected 6-digit hex (e.g. ff00dd)." << std::endl;
            }
            else if (arg == "--pulse-speed" && i + 1 < p_argc)
            {
                pulseSpeed = std::stof(p_pArgv[++i]);
            }
            else if (arg == "--pulse-cubic-bezier" && i + 4 < p_argc)
            {
                bezier.m_p1x = std::stof(p_pArgv[++i]);
                bezier.m_p1y = std::stof(p_pArgv[++i]);
                bezier.m_p2x = std::stof(p_pArgv[++i]);
                bezier.m_p2y = std::stof(p_pArgv[++i]);
            }
            else if (arg == "--video-path" && i + 1 < p_argc)
            {
                videoPath = p_pArgv[++i];
            }
            else if (arg == "--video-framerate" && i + 1 < p_argc)
            {
                videoFramerate = static_cast<uint32_t>(std::stoul(p_pArgv[++i])); // WIP make float
            }
            else if (arg == "--video-colors" && i + 1 < p_argc)
            {
                String fmt = p_pArgv[++i];
                if (fmt == "greyscale" || fmt == "grayscale") // whatever one may address this as
                    videoFormat = EVideoFormat::Greyscale;
                else if (fmt == "rgb")
                    videoFormat = EVideoFormat::Rgb;
                else
                    std::cerr << "Unknown --video-colors value '" << fmt << "'" << std::endl;
            }
        }

        if (displayType == "solid")
            return CreateSolidColor(color, "solid");

        if (displayType == "pulse" || displayType == "pulse-color")
            return CreatePulseColor(color, pulseSpeed, "pulse", nullptr, bezier);

        if (displayType == "video")
        {
            // check video path given
            if (videoPath.empty())
            {
                std::cerr << "--display video requires --video-path <path>. Defaulting to default color." << std::endl;
                return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
            }
            // try to load the video and create the display
            try
            {
                auto frames = LoadVideoBuffer(videoPath, videoFormat);
                if (frames.empty())
                {
                    std::cerr << "Video file loaded but contains no frames: " << videoPath << ". Defaulting to default color." << std::endl;
                    return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
                }
                return CreateVideoDisplay(std::move(frames), videoFramerate, "video");
            }
            catch (const std::exception &e)
            {
                std::cerr << "Failed to load video file '" << videoPath << "': " << e.what() << ". Defaulting to default color." << std::endl;
                return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
            }
        }

        std::cerr << "Unknown display type: " << displayType << ". Defaulting to default color." << std::endl;
        return CreateSolidColor({0x29, 0x00, 0x66}, "solid");
    }
};

static Option<Set<WString>> ParseSerialArgs(int p_argc, char *p_pArgv[])
{
    Set<WString> serials;
    for (int i = 1; i < p_argc; ++i)
    {
        if (String(p_pArgv[i]) == "--serial" && i + 1 < p_argc)
        {
            String raw = p_pArgv[++i];
            WString wraw(raw.begin(), raw.end());
            serials.emplace(wraw);
        }
    }
    if (serials.empty())
        return std::nullopt;
    return serials;
}

static bool ParseVerbose(int p_argc, char *p_pArgv[])
{
    for (int i = 1; i < p_argc; ++i)
    {
        if (String(p_pArgv[i]) == "--verbose")
        {
            return true;
        }
    }
    return false;
}

template <typename TKey>
Set<TKey> SetDifference(const Set<TKey> &p_setA, const Set<TKey> &p_setB)
{
    Set<TKey> result;
    // A - B
    std::set_difference(p_setA.begin(), p_setA.end(),
                        p_setB.begin(), p_setB.end(),
                        std::inserter(result, result.end()));
    return result;
}

int main(int p_argc, char *p_pArgv[])
{
    /* minimize the communication breaks by splitting the logic into 3 separate threads
    - device finder
    - device connector (handshake/verify correct interface)
    - device communicator
    this way we can split device array into the pipeline and only create a possible minor hiccup upon connecting to a new device
    keep in mind, we do have the ability to disconnect a device inside the communicator thread, meaning that the finder thread
    needs to have read access to the communicator device list

    finder --> connector --> communicator
     ^                            /
     '---------------------------'

    */
    // "finder" logic
    const auto ALLOWED_SERIALS = ParseSerialArgs(p_argc, p_pArgv);
    g_verbosity =
#ifdef DEBUG
        true;
#else
        ParseVerbose(p_argc, p_pArgv);
#endif

    // Finder -> handshake thread pipeline
    CQuadcast2SHandshaker handshaker;
    HIDDeviceContainer finderHandshakeDevicesPass;
    const auto POLLING_FREQUENCY = 2s; // poll every 2
    ConditionVariable cvNewDevicesFound;
    Mutex finderHandshakeMtx;

    // CV for notification to main thread that devices have been connected to set up sender thread
    ConditionVariable cvConnectedDevicesUpdated;
    Mutex connectedDevicesUpdatedMtx;

    // CV for notification to finder thread that handshake is complete and finding may resume
    ConditionVariable cvHandshakeDone;
    Mutex handshakeDoneMtx;

    // handshake -> sender thread pipeline
    CQuadcast2SCommunicator communicator;
    HIDDeviceContainer connectedHandshakeDevicesPass;
    Mutex handshakeSenderMtx;

    ConditionVariable cvSenderDone;
    Mutex senderDoneMtx;

    auto finderThread = [&]()
    {
        CUSBDeviceFinder finder;
        // lets do that again
        while (!g_signalStopRequest)
        {
            HIDDeviceContainer devices;
            Set<WString> connectedSerials = communicator.GetOpenSerials();

            if (g_verbosity)
            {
                std::wcout << L"[Finder] Current connected serials: ";
                for (const auto &s : connectedSerials)
                    std::wcout << s << L" ";
                std::wcout << std::endl;
            }

            devices = finder.FindDevices(g_QUADCAST2S_USB_ID,
                                         ALLOWED_SERIALS,
                                         connectedSerials);
            // new devices found, pass to connector thread
            if (!devices.empty())
            {
                // pass new devices to the pipe
                std::wcout << L"[Finder] Preparing new devices to connector thread..." << std::endl;
                {
                    LockGuard lg(finderHandshakeMtx);
                    finderHandshakeDevicesPass = devices;
                }
                std::wcout << L"[Finder] Signaling connector thread..." << std::endl;
                cvNewDevicesFound.notify_one();
                // wait for signal from connector thread that it is done completely
                std::wcout << L"[Finder] Waiting for connector thread to finish processing..." << std::endl;
                UniqueLock lock(handshakeDoneMtx);
                cvHandshakeDone.wait(lock, [&connectedHandshakeDevicesPass]()
                                     { return connectedHandshakeDevicesPass.empty() || g_signalStopRequest; });
                std::wcout << L"[Finder] Continuing..." << std::endl;
            }
            std::this_thread::sleep_for(POLLING_FREQUENCY);
        }

        std::wcout << L"[Finder] Finder thread exiting..." << std::endl;
    };

    auto connectorThread = [&]()
    {
        // lets do this again
        while (!g_signalStopRequest)
        {
            if (g_verbosity)
                std::wcout << L"[Handshake] Waiting for new devices from finder thread..." << std::endl;
            UniqueLock lock(finderHandshakeMtx);
            // wait for signal to process new devices (or if we cancel process)
            cvNewDevicesFound.wait(lock,
                                   [&finderHandshakeDevicesPass]()
                                   {
                                       return !finderHandshakeDevicesPass.empty() || g_signalStopRequest;
                                   });
            // if new devices are to be passed, work them through (this is false if its a signal stop request)
            while (!finderHandshakeDevicesPass.empty())
            {
                // Work off the devices
                auto next = finderHandshakeDevicesPass.back();
                finderHandshakeDevicesPass.pop_back();

                // print
                if (g_verbosity)
                {
                    auto pInfo = hid_get_device_info(next.get());
                    if (pInfo)
                    {
                        std::wcout << L"[Handshake] Processing device: "
                                   << (pInfo->manufacturer_string ? pInfo->manufacturer_string : L"(unknown)")
                                   << L" " << (pInfo->product_string ? pInfo->product_string : L"(unknown)")
                                   << L" serial=" << (pInfo->serial_number ? pInfo->serial_number : L"?")
                                   << L" path=" << (pInfo->path ? pInfo->path : "(unknown)")
                                   << std::endl;
                    }
                }
                // only returns viable ptr on successful response from device
                auto addRet = handshaker.ConnectOne(next);
                if (addRet)
                {
                    WString addedSerial;
                    {
                        auto pInfo = hid_get_device_info(addRet.get());
                        if (pInfo)
                            addedSerial = WString(pInfo->serial_number);
                    }

                    if (g_verbosity)
                    {
                        std::wcout << L"[Handshake] Successfully connected to device, adding to communicator pipeline. Serial: " << addedSerial << std::endl;
                    }
                    // add to pipe
                    {
                        LockGuard lg(handshakeSenderMtx);
                        connectedHandshakeDevicesPass.push_back(addRet);
                    }
                    // remove same serials
                    if (!addedSerial.empty())
                    {
                        auto originalSize = finderHandshakeDevicesPass.size();
                        auto updtEnd = std::remove_if(finderHandshakeDevicesPass.begin(), finderHandshakeDevicesPass.end(),
                                       [&addedSerial](const auto &p_device)
                                       {
                                           auto pInfo = hid_get_device_info(p_device.get());
                                           if(!pInfo)
                                               return false;
                                           auto serial = WString(pInfo->serial_number);
                                           auto isDuplicate = serial == addedSerial;
                                           return isDuplicate;
                                       });
                        finderHandshakeDevicesPass.erase(updtEnd, finderHandshakeDevicesPass.end());
                        if (g_verbosity)
                        {
                            auto newSize = finderHandshakeDevicesPass.size();
                            std::wcout << L"[Handshake] Removed " << (originalSize - newSize) << L" duplicate devices with serial " << addedSerial << L" from handshake pipeline due to same serial." << std::endl;
                        }
                    }
                }
                else if (g_verbosity)
                {
                    std::wcout << L"[Handshake] Failed handshake for device, skipping" << std::endl;
                }
            }

            // Notify that we are done
            if (g_verbosity)
                std::wcout << L"[Handshake] Done processing devices, notifying main..." << std::endl;
            // notify main in case we are at init state
            {
                UniqueLock lock(connectedDevicesUpdatedMtx);
                cvConnectedDevicesUpdated.notify_one();
            }
            // wait for sender thread completion notification
            std::wcout << L"[Handshake] Waiting for sender thread to finish processing..." << std::endl;
            {
                UniqueLock lock(senderDoneMtx);
                cvSenderDone.wait(lock, [&connectedHandshakeDevicesPass]()
                                  { return connectedHandshakeDevicesPass.empty() || g_signalStopRequest; });
            }
            std::cout << L"[Handshake] Sender thread done, cascade back to finder complete..." << std::endl;
            cvHandshakeDone.notify_one();
        }

        std::wcout << L"[Handshake] Connector thread exiting..." << std::endl;
    };

    UniquePtr<CQC2SDisplay> pDisplay = CQC2SDisplayFactory::CreateFromArgs(p_argc, p_pArgv);

    auto handleIncomingNewDevices = [&](CQuadcast2SCommunicator &p_communicator)
    {
        {
            LockGuard lg(handshakeSenderMtx);
            // could intersect with the while loop from handshake,
            // but this just means we have less devices for a irrelevant amount of ticks
            if (connectedHandshakeDevicesPass.empty())
                return true;

            while (!connectedHandshakeDevicesPass.empty())
            {
                auto currentDevicePaths = p_communicator.GetOpenPaths();
                // pop device from queue
                auto device = connectedHandshakeDevicesPass.back();
                connectedHandshakeDevicesPass.pop_back();
                // do an extra check if device already connected (prevent duplicates from handshake and possible race condition)
                // any double connected device (device, not interface) will not light up the LEDs, so just to be sure
                auto pInfo = hid_get_device_info(device.get());
                if (!pInfo)
                    continue;
                String path(pInfo->path);
                if (std::find(currentDevicePaths.begin(), currentDevicePaths.end(), path) != currentDevicePaths.end())
                    continue; // already connected, skip
                // add new device to communicate with
                p_communicator.AddDevice(device);
            }
        }
        cvSenderDone.notify_one();
        // return value handles cancellation of display function, we don't really need that.
        return true;
    };
    auto senderThread = [&]()
    {
        pDisplay->Display(
            communicator,
            g_signalStopRequest,
            handleIncomingNewDevices // TODO -> sync new devices here
        );

        std::wcout << L"[Communicator] Display function ended, signaling other threads to stop..." << std::endl;
        g_signalStopRequest = true; // signal other threads to stop as well, in case display ends on its own (e.g. video end condition met) (TODO: Do we want this?)
    };
    std::signal(SIGINT, [](int)
                { g_signalStopRequest = true; });
    std::signal(SIGTERM, [](int)
                { g_signalStopRequest = true; });

    Thread tFinder(finderThread);
    Thread tConnector(connectorThread);

    // Await initial start to have at least one device connected
    {
        UniqueLock lock(handshakeDoneMtx);
        cvConnectedDevicesUpdated.wait(lock, [&connectedHandshakeDevicesPass]()
                             { return !connectedHandshakeDevicesPass.empty() || g_signalStopRequest; });
    }
    if (!pDisplay->Initialize())
        throw std::runtime_error("Failed to initialize display: " + pDisplay->GetName());

    if (g_signalStopRequest)
    {
        if(g_verbosity)
            std::wcout << L"[Main] Stop signal received before starting sender thread, exiting..." << std::endl;
        tFinder.join();
        tConnector.join();
        return 0;
    }

    if (g_verbosity)
        std::wcout << L"[Main] Found device. Starting sender thread..." << std::endl;
    Thread tSender(senderThread);

    // joins here
    tFinder.join();
    tConnector.join();
    tSender.join();

    // shutdown
    pDisplay->Shutdown(communicator);
}