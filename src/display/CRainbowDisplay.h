// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CQC2SDisplay.h"
#include "ColorTypes.h"
#include "DisplayUtils.h"
#include <chrono>
#include <thread>

enum ERainbowMode
{
    Flat,
    RollingVertical,
    RollingHorizontal,
    RollingDiagonal
};

class CRainbowDisplay : public CQC2SDisplay
{
    StaticArray<SHSV, g_LED_COUNT> m_currentColor;
    StaticArray<SRGBColor, g_LED_COUNT> m_currentRGBColor;
    ERainbowMode m_mode;
    double m_rotSpeed = 1.0; // degrees per frame, placeholder for now

public:
    CRainbowDisplay(
        ERainbowMode p_mode, double p_rotSpeed,
        String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)), m_mode(p_mode), m_rotSpeed(p_rotSpeed)
    {
        m_currentColor.fill(SHSV{0.0, 1.0, 1.0});
    }

    void Reset() override
    {

        switch (m_mode)
        {
        case ERainbowMode::Flat:
            m_currentColor.fill(SHSV{0.0, 1.0, 1.0});
            break;
        case ERainbowMode::RollingVertical:
        {
            const auto ROW_HUE_STEP = 360.0 / g_VIDEO_HEIGHT;
            // sadly indexing for rows isnt as easy as cols, since indices are snaking
            /*
                0   17->repeat
                1   16
                2   15
                3   14
                4   13
                5   12
                6   11   -- -5 diff
                7   10   -- -3 diff
                8   9    -- -1 diff

            */
            for (size_t i = 0; i < g_LED_COUNT; ++i)
            {
                size_t rowNum = i % 18;
                auto sub = (rowNum / 9) * ((i % 9) * 2 + 1); // 9 = -1, 10 = -3, 11 = -5 ...
                rowNum -= sub;
                m_currentColor[i] = SHSV((ROW_HUE_STEP * rowNum), 1.0, 1.0);
            }
        }
        break;
        case ERainbowMode::RollingHorizontal:
        {
            const auto COL_HUE_STEP = 360.0 / g_VIDEO_WIDTH;
            for (size_t i = 0; i < g_LED_COUNT; ++i)
            {
                // every 9 leds, hue steps by COL_HUE_STEP
                auto hue = COL_HUE_STEP * (i / g_VIDEO_HEIGHT);
                m_currentColor[i] = SHSV(hue, 1.0, 1.0);
            }
        }

        break;
        case ERainbowMode::RollingDiagonal:
        {
            // full spectrum along both axes independently:
            // row 0: hue = col * 30  (12 cols -> 0,30,60,...,330)
            // col 0: hue = row * 40  (9 rows  -> 0,40,80,...,320)
            // combined: hue = (col * 30 + row * 40) % 360
            constexpr double COL_STEP = 360.0 / g_VIDEO_WIDTH;  // 30°
            constexpr double ROW_STEP = 360.0 / g_VIDEO_HEIGHT; // 40°
            for (size_t i = 0; i < g_LED_COUNT; ++i)
            {
                const size_t COL = i / g_VIDEO_HEIGHT;
                size_t rowNum = i % 18;
                const auto SUB = (rowNum / 9) * ((i % 9) * 2 + 1);
                rowNum -= SUB;
                const auto HUE = std::fmod(COL * COL_STEP + rowNum * (360.0 - ROW_STEP), 360.0); // - so we have opposite direction
                m_currentColor[i] = SHSV(HUE, 1.0, 1.0);
            }
        }
        }
        for (size_t i = 0; i < g_LED_COUNT; ++i)
            m_currentRGBColor[i] = m_currentColor[i].ToRGB();
            
        CQC2SDisplay::Reset(); 
    }

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        using namespace std::chrono;
        const auto FRAME_DURATION = 50ms;
        const auto FRAME_START = steady_clock::now();

        if (m_mode == ERainbowMode::Flat)
        {
            const auto COLOR = m_currentColor[0].ToRGB();
            SendMonoColorFrame(p_communicator, COLOR);
            m_currentColor[0].m_hue = std::fmod(std::fmod(m_currentColor[0].m_hue + m_rotSpeed, 360.0) + 360.0, 360.0);
        }
        else
        {
            SendColorFrame(p_communicator, m_currentRGBColor.data());
            for (size_t i = 0; i < g_LED_COUNT; ++i)
            {
                m_currentColor[i].m_hue = std::fmod(std::fmod(m_currentColor[i].m_hue + m_rotSpeed, 360.0) + 360.0, 360.0);
                m_currentRGBColor[i] = m_currentColor[i].ToRGB();
            }
        }

        const auto ELAPSED = steady_clock::now() - FRAME_START;
        const auto REMAINING = FRAME_DURATION - ELAPSED;
        if (REMAINING > REMAINING.zero())
            std::this_thread::sleep_for(REMAINING);
        return true;
    }
};
