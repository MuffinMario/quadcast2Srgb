#pragma once

#include "CQC2SDisplay.h"
#include "DisplayUtils.h"
#include "../video/VideoConstants.h"
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
        SendColorFrame(p_communicator, frame.data());
        const auto ELAPSED = std::chrono::steady_clock::now() - frameStart;

        // advance frame, looping when end of video is reached, end condition is decided outside...
        m_currentFrame = (m_currentFrame + 1) % m_frames.size();

        // delta wait to align to framerate
        if (ELAPSED < frameDuration)
            std::this_thread::sleep_for(frameDuration - ELAPSED);

        return true;
    }
};
