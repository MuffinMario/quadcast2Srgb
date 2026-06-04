// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "AudioTypes.h"
#include <portaudio.h>
#include <fftw3.h>

/// Wraps PortAudio capture + FFTW processing to produce SAudioSpectrum data
/// from the default input device.  The PortAudio callback runs on a separate
/// high-priority thread; FFT processing happens on the same callback thread
/// once enough samples have been accumulated.
class CAudioProcessor
{
    // ---- PortAudio ---------------------------------------------------------------
    PaStream *m_pStream = nullptr;

    // ---- FFTW --------------------------------------------------------------------
    fftw_plan m_fftPlan{};               // r2c plan, created once
    double *m_pFftInput = nullptr;       // fftw_malloc'd, size m_fftSize
    fftw_complex *m_pFftOutput = nullptr;// fftw_malloc'd, size m_fftSize/2+1
    uint32_t m_fftSize = 1024;           // samples per FFT window (power of 2)

    // ---- PCM accumulation --------------------------------------------------------
    DynamicContainer<float> m_pcmBuffer; // circular-ish: fills up to m_fftSize
    size_t m_pcmWritePos = 0;            // next free slot in m_pcmBuffer

    // ---- Windowing ---------------------------------------------------------------
    DynamicContainer<double> m_window;   // Hann window coefficients, size m_fftSize
    float m_inputGain = 10.0f;           // multiplier applied to each band after FFT

    // ---- Smoothing (exponential moving average) ----------------------------------
    bool  m_smoothingEnabled = true;     // when true, apply EMA to band amplitudes
    float m_smoothingAlpha   = 0.15f;    // EMA coefficient (0–1); higher = faster response
    DynamicContainer<float> m_smoothedBands;  // previous EMA output per band
    float m_smoothedMaxBand  = 0.0f;     // previous EMA output for max band

    // ---- Output ------------------------------------------------------------------
    mutable Mutex m_spectrumMutex;       // guards m_spectrum
    SAudioSpectrum m_spectrum;           // latest FFT result
    uint32_t m_sampleRate = 48000;       // actual rate reported by PortAudio
    bool m_initialized = false;

    // ---- PortAudio callback ------------------------------------------------------
    static int PaCallback(const void *p_pInput, void * /*p_pOutput*/,
                          unsigned long p_frameCount,
                          const PaStreamCallbackTimeInfo * /*p_pTimeInfo*/,
                          PaStreamCallbackFlags /*p_statusFlags*/,
                          void *p_pUserData);

    /// Run FFTW on the accumulated PCM buffer and update m_spectrum.
    /// Called from the PortAudio callback thread once m_pcmBuffer is full.
    void ProcessFFT();

public:
    CAudioProcessor() = default;
    ~CAudioProcessor();

    // Non-copyable, non-movable (owns PortAudio/FFTW resources)
    CAudioProcessor(const CAudioProcessor &) = delete;
    CAudioProcessor &operator=(const CAudioProcessor &) = delete;
    CAudioProcessor(CAudioProcessor &&) = delete;
    CAudioProcessor &operator=(CAudioProcessor &&) = delete;

    /// Open the default input device and start capturing.
    /// @param p_fftSize  FFT window size (must be a power of 2, default 1024).
    /// @return true on success.
    bool Initialize(uint32_t p_fftSize = 1024);

    /// Stop capture, destroy FFTW plan, free buffers.
    void Shutdown();

    /// Thread-safe copy of the latest spectrum.
    SAudioSpectrum GetSpectrum() const;

    /// True after a successful Initialize() and before Shutdown().
    bool IsInitialized() const { return m_initialized; }

    /// Set the input gain multiplier applied to each frequency band after FFT.
    /// Typical microphone signals at speaking distance need 10× – 50× to
    /// reach useful levels (default: 10.0).
    void SetInputGain(float p_gain) { m_inputGain = p_gain; }

    /// Enable or disable exponential moving average smoothing of the spectrum.
    /// @param p_enabled  true = smooth (default), false = instant per-frame values.
    /// @param p_alpha    EMA coefficient [0–1]; higher = faster response (default 0.15).
    void SetSmoothing(bool p_enabled, float p_alpha = 0.15f)
    {
        m_smoothingEnabled = p_enabled;
        if (p_alpha > 0.0f)
            m_smoothingAlpha = p_alpha;
    }
};
