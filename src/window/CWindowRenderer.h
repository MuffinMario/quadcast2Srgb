// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../display/CIRenderer.h"
#include "../video/VideoConstants.h"
#include "../Globals.h"

#include <SDL2/SDL.h>
#include <GLES3/gl3.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

/// SDL2 window-based implementation of IRenderer.  Creates an OpenGL ES 3.0
/// context via SDL and renders the 12×9 LED grid with raw GL.  Other GL
/// consumers (e.g. CGLSLDisplay) can share this context without EGL conflicts.
class CWindowRenderer : public CIRenderer
{
    // scale factor for each LED rectangle (in pixels) on window
    // and separate gap between rectangles (in pixels) (+ wrap around edges)
    static constexpr int g_LED_PIXEL_SIZE = 48;
    static constexpr int g_GRID_GAP = 4;
    static constexpr int g_EXTRA_W = 300;
    static constexpr int g_EXTRA_H = 250;
    static constexpr int g_WINDOW_W = g_VIDEO_WIDTH * (g_LED_PIXEL_SIZE + g_GRID_GAP) + g_GRID_GAP + g_EXTRA_W;
    static constexpr int g_WINDOW_H = g_VIDEO_HEIGHT * (g_LED_PIXEL_SIZE + g_GRID_GAP) + g_GRID_GAP  + g_EXTRA_H;

    SDL_Window *m_pWindow = nullptr;
    SDL_GLContext m_pGLContext = nullptr;
    std::atomic<bool> m_running{true};

    // ── GL resources for rectangle rendering ────────────────────────────
    GLuint m_rectProgram = 0;
    GLuint m_rectVAO = 0;
    GLuint m_rectVBO = 0;
    GLint m_uColorLoc = -1;
    GLint m_uOffsetLoc = -1;

    // ── Grid FBO + texture (rendered by displays, displayed by ImGui) ──
    GLuint m_gridFBO     = 0;
    GLuint m_gridTexture = 0;

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

    bool InitGL()
    {
        // Simple shader: colored quad with pixel offset
        const char *pVertSrc =
            "#version 300 es\n"
            "in vec2 aPos;\n"
            "uniform vec2 uOffset;\n"
            "uniform vec2 uScale;\n"
            "void main() {\n"
            "    vec2 pos = aPos * uScale + uOffset;\n"
            "    gl_Position = vec4(pos, 0.0, 1.0);\n"
            "}\n";

        const char *pFragSrc =
            "#version 300 es\n"
            "precision mediump float;\n"
            "uniform vec3 uColor;\n"
            "out vec4 fragColor;\n"
            "void main() {\n"
            "    fragColor = vec4(uColor, 1.0);\n"
            "}\n";

        GLuint vShader = glCreateShader(GL_VERTEX_SHADER);
        GLuint fShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(vShader, 1, &pVertSrc, nullptr);
        glShaderSource(fShader, 1, &pFragSrc, nullptr);
        glCompileShader(vShader);
        glCompileShader(fShader);

        m_rectProgram = glCreateProgram();
        glAttachShader(m_rectProgram, vShader);
        glAttachShader(m_rectProgram, fShader);
        glLinkProgram(m_rectProgram);
        glDeleteShader(vShader);
        glDeleteShader(fShader);

        m_uColorLoc = glGetUniformLocation(m_rectProgram, "uColor");
        m_uOffsetLoc = glGetUniformLocation(m_rectProgram, "uOffset");
        GLint uScaleLoc = glGetUniformLocation(m_rectProgram, "uScale");

        // Unit quad in NDC space (will be scaled by uScale and offset by uOffset)
        const float QUAD[] = {0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 1.f, 1.f};
        glGenVertexArrays(1, &m_rectVAO);
        glGenBuffers(1, &m_rectVBO);
        glBindVertexArray(m_rectVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_rectVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(QUAD), QUAD, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

        // Set constant scale (LED pixel size in NDC)
        glUseProgram(m_rectProgram);
        glUniform2f(uScaleLoc,
                    2.f * g_LED_PIXEL_SIZE / g_WINDOW_W,
                    2.f * g_LED_PIXEL_SIZE / g_WINDOW_H);

        // ── Grid FBO + texture ─────────────────────────────────
        glGenTextures(1, &m_gridTexture);
        glBindTexture(GL_TEXTURE_2D, m_gridTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
                     g_WINDOW_W, g_WINDOW_H,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glGenFramebuffers(1, &m_gridFBO);
        glBindFramebuffer(GL_FRAMEBUFFER, m_gridFBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, m_gridTexture, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        return true;
    }

    void DrawLedGL(int p_x, int p_y, const SRGBColor &p_color)
    {
        // Normalize pixel offset to NDC [-1, 1]
        float ox = -1.f + 2.f * p_x / g_WINDOW_W;
        float oy = 1.f - 2.f * p_y / g_WINDOW_H - 2.f * g_LED_PIXEL_SIZE / g_WINDOW_H;
        glUniform2f(m_uOffsetLoc, ox, oy);
        glUniform3f(m_uColorLoc, p_color.m_red / 255.f, p_color.m_green / 255.f, p_color.m_blue / 255.f);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
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

        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);

        m_pWindow = SDL_CreateWindow("qc2srgb-preview",
                                     SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                     g_WINDOW_W, g_WINDOW_H,
                                     SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        if (!m_pWindow)
        {
            LOG_ERROR(L"CWindowRenderer: SDL_CreateWindow failed: " << WStr(SDL_GetError()));
            m_running = false;
            return;
        }

        m_pGLContext = SDL_GL_CreateContext(m_pWindow);
        if (!m_pGLContext)
        {
            LOG_ERROR(L"CWindowRenderer: SDL_GL_CreateContext failed: " << WStr(SDL_GetError()));
            m_running = false;
            return;
        }

        SDL_GL_MakeCurrent(m_pWindow, m_pGLContext);

        if (!InitGL())
        {
            LOG_ERROR(L"CWindowRenderer: GL init failed");
            m_running = false;
            return;
        }

        // ── Dear ImGui init ──────────────────────────────────────
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();

        ImGuiIO &io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.IniFilename = nullptr; // prevent saving imgui.ini; layout is set programmatically
        ImGui_ImplSDL2_InitForOpenGL(m_pWindow, m_pGLContext);
        ImGui_ImplOpenGL3_Init("#version 300 es");

        // Load custom font (JetBrains Mono)
        {
            const char *pPaths[] = {
                "/usr/share/fonts/truetype/JetBrainsMono/JetBrainsMono-Regular.ttf",
                "/usr/share/fonts/TTF/JetBrainsMono-Regular.ttf",
                "/usr/share/fonts/jetbrains-mono/JetBrainsMono-Regular.ttf",
            };
            const char *pHome = getenv("HOME");
            String homePath;
            if (pHome)
            {
                homePath = String(pHome) + "/.local/share/fonts/JetBrainsMono-Regular.ttf";
                pPaths[0] = homePath.c_str(); // reuse first slot for ~/.local
            }

            bool loaded = false;
            for (const char *pPath : pPaths)
            {
                FILE *pFile = fopen(pPath, "rb");
                if (pFile) { fclose(pFile); io.Fonts->AddFontFromFileTTF(pPath, 14.0f); break; }
            }
        }
    }

    ~CWindowRenderer() override
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        if (m_rectVAO)
            glDeleteVertexArrays(1, &m_rectVAO);
        if (m_rectVBO)
            glDeleteBuffers(1, &m_rectVBO);
        if (m_rectProgram)
            glDeleteProgram(m_rectProgram);
        if (m_pGLContext)
            SDL_GL_DeleteContext(m_pGLContext);
        if (m_pWindow)
            SDL_DestroyWindow(m_pWindow);
        SDL_Quit();
    }

    CWindowRenderer(const CWindowRenderer &) = delete;
    CWindowRenderer &operator=(const CWindowRenderer &) = delete;

    bool IsRunning() const { return m_running.load(); }

    bool PollEvents()
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT ||
                (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(m_pWindow)))
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

    void NewFrame()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
    }

    /// ImTextureID for ImGui::Image to display the LED grid.
    ImTextureID GetGridTexture() const
    {
        return (ImTextureID)(intptr_t)m_gridTexture;
    }

    /// Present the frame: render ImGui, swap buffers.
    void Present()
    {
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(m_pWindow);
    }

    // ── IRenderer interface ────────────────────────────────────────

    void RenderMonoFrame(SRGBColor p_color) override
    {
        if (!m_pGLContext)
            return;

        glBindFramebuffer(GL_FRAMEBUFFER, m_gridFBO);
        glViewport(0, 0, g_WINDOW_W, g_WINDOW_H);
        glClearColor(p_color.m_red / 255.f, p_color.m_green / 255.f, p_color.m_blue / 255.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void RenderFrame(const SRGBColor *p_pFrame) override
    {
        if (!m_pGLContext)
            return;

        glBindFramebuffer(GL_FRAMEBUFFER, m_gridFBO);
        glViewport(0, 0, g_WINDOW_W, g_WINDOW_H);

        glClearColor(16.f / 255.f, 16.f / 255.f, 16.f / 255.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(m_rectProgram);
        glBindVertexArray(m_rectVAO);
        for (size_t i = 0; i < g_LED_COUNT; ++i)
        {
            int x, y;
            IndexToScreenPos(i, x, y);
            DrawLedGL(x, y, p_pFrame[i]);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
};
