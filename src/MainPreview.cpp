// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Common.h"
#include "Globals.h"
#include "config/CConfigBuilder.h"
#include "display/CIRenderer.h"
#include "display/CWindowRenderer.h"
#include "audio/CAudioProcessor.h"
#include "util/ArgParsing.h"

#include <csignal>
#include <iostream>
#include <thread>

int main(int p_argc, char *p_pArgv[])
{
    // ── Handle early-exit flags ───────────────────────────────────────
    if (ParseFlag(p_argc, p_pArgv, {"--help", "-h"}))
    {
        PrintHelp(p_argc > 0 ? p_pArgv[0] : "qc2srgb-preview");
        return EXIT_SUCCESS;
    }
    if (ParseFlag(p_argc, p_pArgv, {"--list-audio-devices"}))
    {
        CAudioProcessor::PrintDevices();
        return EXIT_SUCCESS;
    }

    // ── Build unified configuration (same as the hardware executable) ──
    SProgramConfig cfg = CConfigBuilder::Build(p_argc, p_pArgv);

    g_verbosity =
#ifdef DEBUG
        true;
#else
        cfg.m_verbose;
#endif
    g_noWaitForRead.store(true); // preview: no HID response to wait for

    if (!cfg.m_pDisplay)
    {
        LOG_ERROR(L"No startup display could be created. Exiting.");
        return EXIT_FAILURE;
    }

    // ── Audio capture (same as hardware path) ─────────────────────────
    CAudioProcessor audioProcessor;
    if (cfg.m_enableAudio)
    {
        if (!audioProcessor.Initialize(1024, cfg.m_audioDeviceId, cfg.m_audioChannel))
        {
            LOG("[MainPreview] Failed to initialize audio processor.");
            return EXIT_FAILURE;
        }
        audioProcessor.SetInputGain(cfg.m_inputGain);
        audioProcessor.SetSmoothing(cfg.m_audioSmoothing, cfg.m_audioSmoothingAlpha);
        cfg.m_pDisplay->SetAudioProcessor(&audioProcessor);
    }

    // ── Create the window renderer ────────────────────────────────────
    CWindowRenderer windowRenderer;
    if (!windowRenderer.IsRunning())
    {
        LOG_ERROR(L"Failed to create preview window.");
        return EXIT_FAILURE;
    }

    if (!cfg.m_pDisplay->Initialize())
    {
        LOG_ERROR(L"Failed to initialize display: " + WStr(cfg.m_pDisplay->GetName()));
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, [](int)
                { g_signalStopRequest = true; });
    std::signal(SIGTERM, [](int)
                { g_signalStopRequest = true; });

    LOG(L"[MainPreview] Starting display with window preview...");

    // Run the display in a separate thread so we can poll SDL events on the main thread.
    Thread displayThread([&]()
    {
        cfg.m_pDisplay->Display(windowRenderer, g_signalStopRequest);
        g_signalStopRequest = true;
    });

    // Main thread: poll SDL events (window close, Escape key, Ctrl+C).
    while (!g_signalStopRequest.load() && windowRenderer.PollEvents())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 Hz event polling
    }

    g_signalStopRequest = true;
    displayThread.join();

    // ── Shutdown ──────────────────────────────────────────────────────
    audioProcessor.Shutdown();
    cfg.m_pDisplay->Shutdown(windowRenderer);

    LOG(L"[MainPreview] Exiting.");
    return EXIT_SUCCESS;
}
