// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Common.h"
#include "Globals.h"
#include "config/CConfigBuilder.h"
#include "window/CPreviewApp.h"
#include "audio/CAudioProcessor.h"
#include "util/ArgParsing.h"

#include <csignal>
#include <iostream>

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

    // ── Build unified configuration ──────────────────────────────────
    SProgramConfig cfg = CConfigBuilder::Build(p_argc, p_pArgv);

    g_verbosity =
#ifdef DEBUG
        true;
#else
        cfg.m_verbose;
#endif
    g_noWaitForRead.store(true);

    if (!cfg.m_pDisplay)
    {
        LOG_ERROR(L"No startup display could be created. Exiting.");
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, [](int) { g_signalStopRequest = true; });
    std::signal(SIGTERM, [](int) { g_signalStopRequest = true; });

    CPreviewApp app(std::move(cfg));

    if (!app.Initialize())
        return EXIT_FAILURE;

    app.Run();
    app.Shutdown();

    LOG(L"[MainPreview] Exiting.");
    return EXIT_SUCCESS;
}
