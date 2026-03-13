#pragma once

#include "CQC2SDisplay.h"
#include "../video/VideoConstants.h"
#include <cstring>
#include <thread>

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
                //LOG_VERBOSE("Displayed frame " << m_currentFrame << " / " << m_frames.size() << " elapsed " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms" << std::endl);
        #endif
        // advance frame, looping when end of video is reached, end condition is decided outside...
        m_currentFrame = (m_currentFrame + 1) % m_frames.size();
        
        // delta wait to align to framerate
        if (elapsed < frameDuration)
            std::this_thread::sleep_for(frameDuration - elapsed);

        return true;
    }
};
