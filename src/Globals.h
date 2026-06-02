// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "Common.h"

// Global verbosity flag (set from --verbose or DEBUG build)
extern AtomicBool g_verbosity;

// Global no-wait-for-read flag (set from --no-wait-for-read or config)
extern AtomicBool g_noWaitForRead;

// Global stop request flag (set by SIGINT/SIGTERM)
extern AtomicBool g_signalStopRequest;

#define LOG_VERBOSE(msg) \
    do { \
        if (g_verbosity) { \
            WStringStream wss; \
            wss << msg << L'\n'; \
            std::wclog << wss.str(); \
        } \
    } while(0)

#define LOG_ERROR(msg) \
    do { \
        WStringStream wss; \
        wss << L"[ERROR] " << msg << L'\n'; \
        std::wclog << wss.str(); \
    } while(0)

#define LOG(msg) \
    do { \
        WStringStream wss; \
        wss << msg << L'\n'; \
        std::wclog << wss.str(); \
    } while(0)

// Widen a narrow string for use with std::wclog
inline WString WStr(const String &p_str) { return WString(p_str.begin(), p_str.end()); }
inline WString WStr(const char *p_pStr) { String str(p_pStr); return WString(str.begin(), str.end()); }

#include "hid/HIDTypes.h"
// HyperX QuadCast 2S USB vendor/product ID
constexpr SUSBID g_QUADCAST2S_USB_ID = {0x03f0, 0x02b5};