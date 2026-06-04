// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"

/// Holds the result of an FFT applied to a captured audio frame.
/// Normalized amplitudes per frequency band, plus capture metadata.
struct SAudioSpectrum
{
    /// Normalized amplitude per frequency band, range [0.0, 1.0].
    DynamicContainer<float> m_bands;
    /// Frequency width of each band in Hz, or 0 if unknown.
    float m_bandWidthHz = 0.0f;
    /// Sample rate of the audio capture in Hz.
    uint32_t m_sampleRate = 0;
    /// FFT window size (number of samples per transform).
    uint32_t m_fftSize = 0;
    /// Maximum amplitude across all bands (pre-computed, useful as a volume proxy).
    float m_maxBand = 0.0f;

    /// Number of frequency bands (equal to fftSize/2 for a real-only FFT).
    size_t GetNumBands() const { return m_bands.size(); }

    /// Convenience: true when spectrum data is available.
    bool IsValid() const { return !m_bands.empty() && m_sampleRate > 0; }
};
