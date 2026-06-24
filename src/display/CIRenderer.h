// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "../video/VideoConstants.h"
#include "ColorTypes.h"

/// Abstract interface for rendering computed color frames to a target.
/// Simple interface with two utility render methods which can be invoked
/// by varying display implementations (primarily -> the communicator for the microphone)
class CIRenderer
{
public:
    virtual ~CIRenderer() = default;

    /// Render a full per-LED color frame (g_LED_COUNT = 108 entries).
    virtual void RenderFrame(const SRGBColor *p_pFrame) = 0;

    /// Render a single color applied to every LED.
    virtual void RenderMonoFrame(SRGBColor p_color) = 0;
};
