#pragma once

#include "Common.h"
#include "hid/HIDTypes.h"

// Global verbosity flag (set from --verbose or DEBUG build)
extern AtomicBool g_verbosity;

// Global stop request flag (set by SIGINT/SIGTERM)
extern AtomicBool g_signalStopRequest;

// HyperX QuadCast 2S USB vendor/product ID
constexpr SUSBID g_QUADCAST2S_USB_ID = {0x03f0, 0x02b5};
