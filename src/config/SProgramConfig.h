// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "../display/CQC2SDisplay.h"
#include "../display/ColorTypes.h"

#include <set>

// ──────────────────────────────────────────────────────────────────────────────
/// Unified program configuration, merged from CLI arguments and/or config file.
///
/// Priority (highest to lowest):
///   1. CLI arguments
///   2. Config file values
///   3. Built-in defaults
///
/// Notable exceptions:
///   - m_verbose / m_noWaitForRead are OR'd (if either source enables it, it's on).
///   - m_audioSmoothing can be DISABLED by --no-audio-smoothing regardless of config.
// ──────────────────────────────────────────────────────────────────────────────
struct SProgramConfig
{
    // ── Top-level flags ───────────────────────────────────────────────────
    bool m_verbose = false;
    bool m_noWaitForRead = false;
    Option<Set<WString>> m_allowedSerials;

    // ── Audio capture ─────────────────────────────────────────────────────
    bool  m_enableAudio         = false;
    float m_inputGain           = 50.0f;
    bool  m_audioSmoothing      = true;
    float m_audioSmoothingAlpha = 0.15f;

    // ── Startup display ───────────────────────────────────────────────────
    UniquePtr<CQC2SDisplay> m_pDisplay;
};
