// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CWindowRenderer.h"
#include "../config/CConfigBuilder.h"
#include "../audio/CAudioProcessor.h"
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

            ImGui::Begin("Controls");
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::End();

            ImGui::Begin("Shader");
            if (ImGui::Button("Reload"))
            {
                // TODO: reload active shader
            }
            ImGui::End();

            ImGui::Begin("Info");
            ImGui::Text("Display: %s", m_config.m_pDisplay->GetName().c_str());
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
