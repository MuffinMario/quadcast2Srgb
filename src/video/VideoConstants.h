#pragma once

#include "../Common.h"
#include "../hid/Packets.h"

constexpr uint32_t g_VIDEO_WIDTH = 12;
constexpr uint32_t g_VIDEO_HEIGHT = 9;
constexpr uint32_t g_LED_COUNT = g_VIDEO_WIDTH * g_VIDEO_HEIGHT; // 108
constexpr size_t g_RGB_FRAME_SIZE = g_LED_COUNT * sizeof(SRGBColor); // 3 bytes per pixel (R, G, B — 8 bits each)

// Pre-loaded frame buffer type shared across video displays
using VideoFrameBuffer = DynamicContainer<StaticArray<SRGBColor, g_LED_COUNT>>;

enum class EVideoFormat
{
    Rgb,
    Greyscale
};

constexpr size_t g_VIDEO_MAX_FILE_SIZE_DEFAULT = 512ULL * 1024 * 1024; // 512 MB
