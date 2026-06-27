// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CWindowRenderer.h"
#include "../config/CConfigBuilder.h"
#include "../audio/CAudioProcessor.h"
#include "../display/CSolidColorDisplay.h"
#include "../display/CPulseColorDisplay.h"
#include "../display/CRainbowDisplay.h"
#include "../display/CColorTransitionDisplay.h"
#include "../display/CVideoDisplay.h"
#ifdef USE_GLSL
#include "../display/CGLSLDisplay.h"
#endif
#include "../display/CMultiDisplay.h"
#include "../display/ColorTypes.h"
#include "imgui.h"

/// Manages the preview window, display loop, and ImGui UI.
/// Run() blocks until the window is closed or the display ends.
class CPreviewApp
{
    CWindowRenderer m_renderer;
    CAudioProcessor m_audioProcessor;
    SProgramConfig  m_config;

public:
    CPreviewApp(SProgramConfig p_config)
        : m_config(std::move(p_config))
    {
    }

    bool Initialize()
    {
        if (!m_renderer.IsRunning())
        {
            LOG_ERROR(L"Failed to create preview window.");
            return false;
        }

        // ── Audio capture ───────────────────────────────────────
        if (m_config.m_enableAudio)
        {
            if (!m_audioProcessor.Initialize(1024, m_config.m_audioDeviceId, m_config.m_audioChannel))
            {
                LOG("[CPreviewApp] Failed to initialize audio processor.");
                return false;
            }
            m_audioProcessor.SetInputGain(m_config.m_inputGain);
            m_audioProcessor.SetSmoothing(m_config.m_audioSmoothing, m_config.m_audioSmoothingAlpha);
            m_config.m_pDisplay->SetAudioProcessor(&m_audioProcessor);
        }

        if (!m_config.m_pDisplay->Initialize())
        {
            LOG_ERROR(L"Failed to initialize display: " + WStr(m_config.m_pDisplay->GetName()));
            return false;
        }

        return true;
    }

    static void ShowColor(const char *p_pLabel, SRGBColor p_color)
    {
        ImGui::Text("%s: #%02X%02X%02X", p_pLabel,
                    p_color.m_red, p_color.m_green, p_color.m_blue);
    }

    static void ShowBezier(const char *p_pLabel, const SCubicBezier &p_bezier)
    {
        ImGui::Text("%s: (%.2f, %.2f) -> (%.2f, %.2f)", p_pLabel,
                    p_bezier.m_p1x, p_bezier.m_p1y,
                    p_bezier.m_p2x, p_bezier.m_p2y);
    }

    void ShowDisplayOptions(CQC2SDisplay *p_pDisplay)
    {
        if (auto pSolid = dynamic_cast<CSolidColorDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: solid");
            ShowColor("Color", pSolid->GetColor());
        }
        else if (auto pPulse = dynamic_cast<CPulseColorDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: pulse");
            ShowColor("Color", pPulse->GetColor());
            ImGui::Text("Speed: %.3f", pPulse->GetSpeed());
            ShowBezier("Bezier", pPulse->GetBezier());
        }
        else if (auto pRainbow = dynamic_cast<CRainbowDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: rainbow");
            ImGui::Text("Speed: %.1f °/frame", pRainbow->GetSpeed());
            const char *pModeName = "?";
            switch (pRainbow->GetMode())
            {
            case ERainbowMode::Flat:            pModeName = "flat";       break;
            case ERainbowMode::RollingVertical:  pModeName = "vertical";  break;
            case ERainbowMode::RollingHorizontal:pModeName = "horizontal";break;
            case ERainbowMode::RollingDiagonal:  pModeName = "diagonal";  break;
            }
            ImGui::Text("Mode: %s", pModeName);
        }
        else if (auto pTransition = dynamic_cast<CColorTransitionDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: transition");
            ImGui::Text("Colors: %zu", pTransition->GetColors().size());
            ImGui::Text("Speed: %.4f", pTransition->GetSpeed());
            ShowBezier("Bezier", pTransition->GetBezier());
        }
        else if (auto pVideo = dynamic_cast<CVideoDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: video");
            ImGui::Text("FPS: %u", pVideo->GetFPS());
            ImGui::Text("Frames: %zu", pVideo->GetFrameCount());
        }
#ifdef USE_GLSL
        else if (auto pGLSL = dynamic_cast<CGLSLDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: glsl");
            ImGui::Text("Shader: %s", pGLSL->GetShaderPath().c_str());
            ImGui::Text("FPS: %u", pGLSL->GetFPS());
            ImGui::Text("Scale: %u", pGLSL->GetScale());
        }
#endif
        else if (dynamic_cast<CMultiDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: multi-display");
        }
        else
        {
            ImGui::Text("Type: unknown");
        }
    }

    void ShowDisplayInfo(CQC2SDisplay *p_pDisplay)
    {
        if (!p_pDisplay)
            return;

        if (auto pMulti = dynamic_cast<CMultiDisplay *>(p_pDisplay))
        {
            // ── Multi-display: tree of children ─────────────────
            for (size_t i = 0; i < pMulti->GetDisplayCount(); ++i)
            {
                CQC2SDisplay *pChild = pMulti->GetDisplay(i);
                bool open = ImGui::TreeNode(pChild->GetName().c_str());
                if (open)
                {
                    ImGui::Text("Next: %s", pChild->GetNextDisplay().empty()
                        ? "(stop)" : pChild->GetNextDisplay().c_str());
                    ShowDisplayOptions(pChild);
                    ImGui::TreePop();
                }
            }
        }
        else
        {
            // ── Single display ──────────────────────────────────
            ImGui::Text("Name: %s", p_pDisplay->GetName().c_str());
            if (!p_pDisplay->GetNextDisplay().empty())
                ImGui::Text("Next: %s", p_pDisplay->GetNextDisplay().c_str());
            ImGui::Separator();
            ShowDisplayOptions(p_pDisplay);
        }
    }

    void Run()
    {
        LOG(L"[CPreviewApp] Starting display with window preview...");

        auto callback = [&](CIRenderer &)
        {
            m_renderer.NewFrame();

            // ── Docking layout ───────────────────────────────────
            ImGui::DockSpaceOverViewport();

            ImGui::Begin("LED Grid");
            ImVec2 avail = ImGui::GetContentRegionAvail();
            ImGui::Image(m_renderer.GetGridTexture(), avail,
                         ImVec2(0, 1), ImVec2(1, 0)); // flip Y
            ImGui::End();

            ImGui::Begin("Display");
            ShowDisplayInfo(m_config.m_pDisplay.get());
            ImGui::End();

            ImGui::Begin("Shader");
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            if (ImGui::Button("Reload"))
            {
                // TODO: reload active shader
            }
            ImGui::End();

            ImGui::Begin("General");
            ImGui::Text("Verbose logging: %s", m_config.m_verbose ? "Yes" : "No");
            ImGui::Text("Skip device response: %s", m_config.m_noWaitForRead ? "Yes" : "No");
            if (m_config.m_allowedSerials.has_value() && !m_config.m_allowedSerials->empty())
            {
                ImGui::Text("Allowed serials:");
                for (const auto &serial : *m_config.m_allowedSerials)
                {
                    String narrow(serial.begin(), serial.end());
                    ImGui::Text("  %s", narrow.c_str());
                }
            }
            else
            {
                ImGui::Text("Allowed serials: all");
            }

            ImGui::SeparatorText("Audio");
            ImGui::Text("Capture: %s", m_config.m_enableAudio ? "enabled" : "disabled");
            ImGui::Text("Smoothing: %s", m_config.m_audioSmoothing ? "Yes" : "No");
            ImGui::Text("Smoothing alpha: %.3f", m_config.m_audioSmoothingAlpha);
            ImGui::Text("Input gain: %.1f", m_config.m_inputGain);
            ImGui::Text("Device ID: %s", m_config.m_audioDeviceId.has_value()
                ? std::to_string(*m_config.m_audioDeviceId).c_str() : "default");
            ImGui::Text("Channel: %s", m_config.m_audioChannel.has_value()
                ? std::to_string(*m_config.m_audioChannel).c_str() : "0");
            ImGui::End();

            auto continueDisplaying = m_renderer.PollEvents();
            if (!continueDisplaying)
                g_signalStopRequest = true;

            m_renderer.Present();
            return continueDisplaying;
        };

        m_config.m_pDisplay->Display(m_renderer, g_signalStopRequest, std::move(callback));
    }

    void Shutdown()
    {
        m_audioProcessor.Shutdown();
        m_config.m_pDisplay->Shutdown(m_renderer);
    }
};
