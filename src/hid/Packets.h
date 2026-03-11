#pragma once

#include "../Common.h"
#include <cstdint>

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
