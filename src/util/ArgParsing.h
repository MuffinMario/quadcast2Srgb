// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "../hid/Packets.h"

// ── Generic argument parsers ──────────────────────────────────────────────────

/// Returns true if @p p_flagName appears anywhere in argv.
inline bool ParseFlag(int p_argc, char *p_pArgv[], const String &p_flagName)
{
    for (int i = 1; i < p_argc; ++i)
        if (String(p_pArgv[i]) == p_flagName)
            return true;
    return false;
}

/// Returns true if any of the flag names in @p p_names appears in argv.
inline bool ParseFlag(int p_argc, char *p_pArgv[], std::initializer_list<const char *> p_names)
{
    for (int i = 1; i < p_argc; ++i)
    {
        const String ARG(p_pArgv[i]);
        for (const char *pName : p_names)
            if (ARG == pName)
                return true;
    }
    return false;
}

/// Returns the value following @p p_flagName as a float, or nullopt.
inline Option<float> ParseFloatArg(int p_argc, char *p_pArgv[], const String &p_flagName)
{
    for (int i = 1; i < p_argc; ++i)
    {
        if (String(p_pArgv[i]) == p_flagName && i + 1 < p_argc)
        {
            try { return std::stof(p_pArgv[i + 1]); }
            catch (...) { return std::nullopt; }
        }
    }
    return std::nullopt;
}

/// Returns the value following @p p_flagName as an int, or nullopt.
inline Option<int> ParseIntArg(int p_argc, char *p_pArgv[], const String &p_flagName)
{
    for (int i = 1; i < p_argc; ++i)
    {
        if (String(p_pArgv[i]) == p_flagName && i + 1 < p_argc)
        {
            try { return static_cast<int>(std::stol(p_pArgv[i + 1])); }
            catch (...) { return std::nullopt; }
        }
    }
    return std::nullopt;
}

// ── Specialised parsers (kept for non-trivial logic) ─────────────────────────

Option<SRGBColor> ParseHexColor(String p_colorStr);
Option<Set<WString>> ParseSerialArgs(int p_argc, char *p_pArgv[]);
void PrintHelp(const char *p_pProgramName);
