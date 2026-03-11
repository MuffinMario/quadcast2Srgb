#pragma once

#include "../Common.h"
#include <chrono>

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
