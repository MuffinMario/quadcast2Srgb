// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "SProgramConfig.h"

// ──────────────────────────────────────────────────────────────────────────────
/// Builds a unified SProgramConfig by parsing CLI arguments and (optionally)
/// loading a config file, then merging them according to the documented priority.
///
/// Usage:
///   auto cfg = CConfigBuilder::Build(argc, argv);
///   g_verbosity.store(cfg.m_verbose);
///   ...
// ──────────────────────────────────────────────────────────────────────────────
class CConfigBuilder
{
public:
    /// Parse CLI arguments, optionally load a config file, merge, and return the
    /// unified program configuration.
    static SProgramConfig Build(int p_argc, char *p_pArgv[]);
};
