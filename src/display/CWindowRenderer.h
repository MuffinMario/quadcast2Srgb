// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CIRenderer.h"
#include "../video/VideoConstants.h"
#include "../Globals.h"

#include <SDL2/SDL.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

/// SDL2 window-based implementation of IRenderer.  Renders the 12×9 LED grid
/// to an on-screen window for preview / development purposes.
class CWindowRenderer : public CIRenderer
{
    // scale factor for each LED rectangle (in pixels) on window
    // and separate gap between rectangles (in pixels) (+ wrap around edges)
    static constexpr int g_LED_PIXEL_SIZE = 48;   
    static constexpr int g_GRID_GAP       = 4;    
    static constexpr int g_WINDOW_W = g_VIDEO_WIDTH  * (g_LED_PIXEL_SIZE + g_GRID_GAP) + g_GRID_GAP;
    static constexpr int g_WINDOW_H = g_VIDEO_HEIGHT * (g_LED_PIXEL_SIZE + g_GRID_GAP) + g_GRID_GAP;

    SDL_Window   *m_pWindow   = nullptr;
    SDL_Renderer *m_pRenderer = nullptr;
    std::atomic<bool> m_running{true};

    /// Map a logical LED index (column-major, odd cols reversed) to its
    /// on-screen (x, y) position.
    static void IndexToScreenPos(size_t p_index, int &p_outX, int &p_outY)
    {
        const size_t COL = p_index / g_VIDEO_HEIGHT;
        const size_t ROW_IN_COL = p_index % g_VIDEO_HEIGHT;
        // Odd columns render top→bottom (the physical snake routing)
        const size_t PHYS_ROW = (COL % 2 == 0)
            ? (g_VIDEO_HEIGHT - 1 - ROW_IN_COL)
            : ROW_IN_COL;
        p_outX = g_GRID_GAP + static_cast<int>(COL) * (g_LED_PIXEL_SIZE + g_GRID_GAP);
        p_outY = g_GRID_GAP + static_cast<int>(PHYS_ROW) * (g_LED_PIXEL_SIZE + g_GRID_GAP);
    }

    /// Draw a single colored LED rectangle at the given screen position.
    void DrawLed(int p_x, int p_y, const SRGBColor &p_color)
    {
        SDL_SetRenderDrawColor(m_pRenderer, p_color.m_red, p_color.m_green, p_color.m_blue, 255);
        SDL_Rect rect{p_x, p_y, g_LED_PIXEL_SIZE, g_LED_PIXEL_SIZE};
        SDL_RenderFillRect(m_pRenderer, &rect);
    }

public:
    CWindowRenderer()
    {
        if (SDL_Init(SDL_INIT_VIDEO) != 0)
        {
            LOG_ERROR(L"CWindowRenderer: SDL_Init failed: " << WStr(SDL_GetError()));
            m_running = false;
            return;
        }

        m_pWindow = SDL_CreateWindow("qc2srgb-preview",
                                     SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                     g_WINDOW_W, g_WINDOW_H,
                                     SDL_WINDOW_SHOWN);
        if (!m_pWindow)
        {
            LOG_ERROR(L"CWindowRenderer: SDL_CreateWindow failed: " << WStr(SDL_GetError()));
            m_running = false;
            return;
        }

        m_pRenderer = SDL_CreateRenderer(m_pWindow, -1,
                                         SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (!m_pRenderer)
        {
            LOG_ERROR(L"CWindowRenderer: SDL_CreateRenderer failed: " << WStr(SDL_GetError()));
            m_running = false;
            return;
        }
    }

    ~CWindowRenderer() override
    {
        if (m_pRenderer) SDL_DestroyRenderer(m_pRenderer);
        if (m_pWindow)   SDL_DestroyWindow(m_pWindow);
        SDL_Quit();
    }

    CWindowRenderer(const CWindowRenderer &) = delete;
    CWindowRenderer &operator=(const CWindowRenderer &) = delete;

    /// Check whether the window has been closed by the user.
    bool IsRunning() const { return m_running.load(); }

    /// Poll SDL events; returns false when the window is closed.
    bool PollEvents()
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                m_running = false;
                return false;
            }
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
            {
                m_running = false;
                return false;
            }
        }
        return m_running.load();
    }

    // ── IRenderer interface ────────────────────────────────────────────────

    void RenderMonoFrame(SRGBColor p_color) override
    {
        if (!m_pRenderer) return;

        SDL_SetRenderDrawColor(m_pRenderer, p_color.m_red, p_color.m_green, p_color.m_blue, 255);
        SDL_RenderClear(m_pRenderer);
        SDL_RenderPresent(m_pRenderer);
    }

    void RenderFrame(const SRGBColor *p_pFrame) override
    {
        if (!m_pRenderer) return;

        // Dark background
        SDL_SetRenderDrawColor(m_pRenderer, 16, 16, 16, 255);
        SDL_RenderClear(m_pRenderer);

        for (size_t i = 0; i < g_LED_COUNT; ++i)
        {
            int x, y;
            IndexToScreenPos(i, x, y);
            DrawLed(x, y, p_pFrame[i]);
        }

        SDL_RenderPresent(m_pRenderer);
    }
};
