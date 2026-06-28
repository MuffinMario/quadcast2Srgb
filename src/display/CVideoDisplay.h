// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CQC2SDisplay.h"
#include "CIRenderer.h"
#include "../video/VideoConstants.h"
#include "../video/VideoProcessing.h"
#include <thread>

class CVideoDisplay : public CQC2SDisplay
{
    String m_videoPath;
    EVideoFormat m_videoFormat;
    uint32_t m_fps;

    // Loaded at Initialize() time
    VideoFrameBuffer m_frames;
    size_t m_currentFrame = 0;

public:
    CVideoDisplay(String p_videoPath, EVideoFormat p_format, uint32_t p_fps, String p_name,
                  UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)),
          m_videoPath(std::move(p_videoPath)), m_videoFormat(p_format), m_fps(p_fps) {}

    const String &GetVideoPath()   const { return m_videoPath; }
    void          SetVideoPath(const String &p_path) { m_videoPath = p_path; }
    EVideoFormat  GetFormat()      const { return m_videoFormat; }
    void          SetFormat(EVideoFormat p_format) { m_videoFormat = p_format; }
    uint32_t      GetFPS()         const { return m_fps; }
    void          SetFPS(uint32_t p_fps) { m_fps = p_fps; }
    size_t        GetFrameCount()  const { return m_frames.size(); }

    bool Initialize() override
    {
        m_currentFrame = 0;
        if (m_videoPath.empty())
        {
            LOG(L"[CVideoDisplay] Video Display '" << WStr(m_name) << L"' does not have a video path. Defaulting to empty frame list!");
            m_frames.clear();
            // silently continue
            return true;
        }

        m_frames = LoadVideoBuffer(m_videoPath, m_videoFormat);
        return !m_frames.empty();
    }

    bool DisplayFrame(CIRenderer &p_renderer) override
    {
        if (m_frames.empty())
            return true;

        auto frameStart = std::chrono::steady_clock::now();
        auto frameDuration = std::chrono::milliseconds(1000 / m_fps);

        const auto &frame = m_frames[m_currentFrame];
        p_renderer.RenderFrame(frame.data());
        const auto ELAPSED = std::chrono::steady_clock::now() - frameStart;

        // advance frame, looping when end of video is reached, end condition is decided outside...
        m_currentFrame = (m_currentFrame + 1) % m_frames.size();

        // delta wait to align to framerate
        if (ELAPSED < frameDuration)
            std::this_thread::sleep_for(frameDuration - ELAPSED);

        return true;
    }
};
