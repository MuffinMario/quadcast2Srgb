#pragma once

#include "Common.h"
#include "hid/HIDTypes.h"

// Global verbosity flag (set from --verbose or DEBUG build)
extern AtomicBool g_verbosity;

// Global stop request flag (set by SIGINT/SIGTERM)
extern AtomicBool g_signalStopRequest;

// HyperX QuadCast 2S USB vendor/product ID
constexpr SUSBID g_QUADCAST2S_USB_ID = {0x03f0, 0x02b5};

#define LOG_VERBOSE(msg) \
    do { \
        if (g_verbosity) \
            std::wclog << msg << std::endl; \
    } while(0)

#define LOG_ERROR(msg) \
    do { \
        std::wclog << L"[ERROR] " << msg << std::endl; \
    } while(0)

#define LOG(msg) \
    do { \
        std::wclog << msg << std::endl; \
    } while(0)

// Widen a narrow string for use with std::wclog
inline WString WStr(const String &p_str) { return WString(p_str.begin(), p_str.end()); }
inline WString WStr(const char *p_pStr) { String str(p_pStr); return WString(str.begin(), str.end()); }