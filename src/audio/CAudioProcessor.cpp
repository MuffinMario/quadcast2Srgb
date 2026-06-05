// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "CAudioProcessor.h"
#include "../Globals.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <stdexcept>

// ── PortAudio callback ─────────────────────────────────────────────────────────

int CAudioProcessor::PaCallback(const void *p_pInput, void * /*p_pOutput*/, 
                                unsigned long p_frameCount,
                                const PaStreamCallbackTimeInfo * /*p_pTimeInfo*/,
                                PaStreamCallbackFlags /*p_statusFlags*/,
                                void *p_pUserData)
{
    auto *pThis = static_cast<CAudioProcessor *>(p_pUserData);
    auto *pStream = p_pInput;
    if (!p_pInput || !pStream || p_frameCount == 0)
        return paContinue;

    const auto *pSamples = static_cast<const float *>(pStream);

    // Accumulate PCM samples (de-interleave if the stream has multiple channels)
    while (p_frameCount > 0)
    {
        size_t remaining = pThis->m_fftSize - pThis->m_pcmWritePos;
        size_t toCopy    = std::min(remaining, static_cast<size_t>(p_frameCount));

        if (pThis->m_numChannels > 1)
        {
            // Pick only the desired channel from interleaved data
            float *pBuf = pThis->m_pcmBuffer.data() + pThis->m_pcmWritePos;
            for (size_t j = 0; j < toCopy; ++j)
                pBuf[j] = pSamples[j * pThis->m_numChannels + pThis->m_channelIndex];
            for(size_t c = 0; c < pThis->m_numChannels; ++c)
                for (size_t j = 0; j < toCopy; ++j)
                {

                    auto val = pSamples[j * pThis->m_numChannels + c];
                    if(val != 0.0f)
                    {
                        
                        LOG_VERBOSE(L"CAudioProcessor: sample[" << j << "][" << c << "] = " << val);
                        break;
                    }
                }
            pSamples += toCopy * pThis->m_numChannels;
        }
        else
        {
            std::memcpy(pThis->m_pcmBuffer.data() + pThis->m_pcmWritePos,
                        pSamples,
                        toCopy * sizeof(float));
            pSamples += toCopy;
        }
        pThis->m_pcmWritePos += toCopy;
        p_frameCount         -= toCopy;

        if (pThis->m_pcmWritePos >= pThis->m_fftSize)
        {
            pThis->ProcessFFT();
            pThis->m_pcmWritePos = 0;
        }
    }

    return paContinue;
}

// ── FFT processing ─────────────────────────────────────────────────────────────

void CAudioProcessor::ProcessFFT()
{
    // 1. Apply Hann window and copy float → double into FFTW input
    for (size_t i = 0; i < static_cast<size_t>(m_fftSize); ++i)
    {
        m_pFftInput[i] = static_cast<double>(m_pcmBuffer[i]) * m_window[i];
    }

    // 2. Execute FFTW plan (real → complex)
    // Given m_pFftInput as real-valued input, 
    // m_pFftOutput will contain N/2+1 complex bins 
    fftw_execute(m_fftPlan);

    // 3. Compute magnitude spectrum (only first N/2 bins are meaningful for r2c)
    const size_t NUM_BINS = m_fftSize / 2;
    // the ft used a hann window applied input, 
    // we need to correct the amp by reverting the scaling effect of the window again
    // also correct ofr fft scaling (1/N)
    const double SCALE = 2.0 / static_cast<double>(m_fftSize);

    // ── update shared spectrum under lock ──
    {
        LockGuard lock(m_spectrumMutex);

        m_spectrum.m_bands.resize(NUM_BINS);
        m_spectrum.m_sampleRate  = m_sampleRate;
        m_spectrum.m_fftSize     = m_fftSize;
        m_spectrum.m_bandWidthHz = static_cast<float>(m_sampleRate) / static_cast<float>(m_fftSize);

        const float GAIN  = m_inputGain;
        const bool  SMOOTH = m_smoothingEnabled;
        const float ALPHA  = m_smoothingAlpha;
        const float ONE_MINUS_ALPHA = 1.0f - ALPHA;

        // Ensure smoothed state is sized correctly (first run or FFT size changed)
        if (SMOOTH && m_smoothedBands.size() != NUM_BINS)
            m_smoothedBands.resize(NUM_BINS, 0.0f);

        for (size_t i = 0; i < NUM_BINS; ++i)
        {
            double real = m_pFftOutput[i][0];
            double imag = m_pFftOutput[i][1];
            // calculate the amplitude (r² + i²) and we revert the scaling applied by the hann window
            // and also add user configurable input gain as another scaling factor
            double mag  = std::sqrt(real * real + imag * imag) * SCALE * GAIN;

            // normalize values by clamping to range [0,1]
            float raw = static_cast<float>(std::clamp(mag, 0.0, 1.0));

            // the final band value we set depends on whether smoothing is enabled or not
            // apply EMA if enabled, else we get the raw per-frame value directly from the FFT
            if (SMOOTH)
            {
                m_smoothedBands[i] = ALPHA * raw + ONE_MINUS_ALPHA * m_smoothedBands[i];
                m_spectrum.m_bands[i] = m_smoothedBands[i];
            }
            else
            {
                m_spectrum.m_bands[i] = raw;
            }
        }

        // Pre-compute max band for direct usage on some displays
        float maxBand = 0.0f;
        for (size_t i = 0; i < NUM_BINS; ++i)
            maxBand = std::max(maxBand, m_spectrum.m_bands[i]);

        // this is already smoothed if enabled... applying smoothing on a second layer might create a nice effect though
        m_smoothedMaxBand = maxBand; 
        m_spectrum.m_maxBand = m_smoothedMaxBand;
    }
}

// ── Device enumeration ─────────────────────────────────────────────────────────

void CAudioProcessor::PrintDevices()
{
    PaError paErr = Pa_Initialize();
    if (paErr != paNoError)
    {
        LOG_ERROR(L"CAudioProcessor: Pa_Initialize failed during device listing: "
                  << WStr(Pa_GetErrorText(paErr)));
        return;
    }

    PaDeviceIndex defaultInput  = Pa_GetDefaultInputDevice();
    PaDeviceIndex defaultOutput = Pa_GetDefaultOutputDevice();
    PaDeviceIndex numDevices    = Pa_GetDeviceCount();

    if (numDevices < 0)
    {
        LOG_ERROR(L"CAudioProcessor: Pa_GetDeviceCount returned error");
        Pa_Terminate();
        return;
    }

    LOG(L"--- PortAudio Devices (" << numDevices << L" total) ---");

    for (PaDeviceIndex i = 0; i < numDevices; ++i)
    {
        const PaDeviceInfo *pInfo = Pa_GetDeviceInfo(i);
        if (!pInfo)
        {
            LOG(L"  " << i << L"  (null device info)");
            continue;
        }

        const PaHostApiInfo *pHostApi = Pa_GetHostApiInfo(pInfo->hostApi);
        StringStream ss;
        ss << "  " << i;
        if (i == defaultInput)  ss << " [default in]";
        if (i == defaultOutput) ss << " [default out]";
        ss << "  \"" << pInfo->name << "\""
           << "  API=" << (pHostApi ? pHostApi->name : "?")
           << "  in="  << pInfo->maxInputChannels
           << "  out=" << pInfo->maxOutputChannels
           << "  rate=" << pInfo->defaultSampleRate;
        LOG(WStr(ss.str()));
    }
    LOG(L"---");

    Pa_Terminate();
}

// ── Public interface ───────────────────────────────────────────────────────────

CAudioProcessor::~CAudioProcessor()
{
    Shutdown();
}

bool CAudioProcessor::Initialize(uint32_t p_fftSize, Option<int> p_deviceId, Option<int> p_channel)
{
    if (m_initialized)
    {
        LOG_ERROR(L"CAudioProcessor: Already initialized.");
        return false;
    }

    // Validate FFT size
    if (p_fftSize == 0 || (p_fftSize & (p_fftSize - 1)) != 0)
    {
        LOG_ERROR(L"CAudioProcessor: FFT size must be a power of 2, got " << p_fftSize);
        return false;
    }
    m_fftSize = p_fftSize;

    // ── PortAudio initialisation ────────────────────────────────────────
    PaError paErr = Pa_Initialize();
    if (paErr != paNoError)
    {
        LOG_ERROR(L"CAudioProcessor: PortAudio init failed: " << WStr(Pa_GetErrorText(paErr)));
        return false;
    }

    // ── Store channel selection ──────────────────────────────────────────
    m_channelIndex = p_channel.value_or(0);

    // ── Open the requested device (or default if none specified) ────────
    if (p_deviceId.has_value())
    {
        PaDeviceIndex requested = static_cast<PaDeviceIndex>(*p_deviceId);
        const PaDeviceInfo *pReqInfo = Pa_GetDeviceInfo(requested);
        if (!pReqInfo)
        {
            LOG_ERROR(L"CAudioProcessor: Device #" << *p_deviceId << L" not found.");
            Pa_Terminate();
            return false;
        }
        if (pReqInfo->maxInputChannels < 1)
        {
            LOG_ERROR(L"CAudioProcessor: Device #" << *p_deviceId << L" \""
                      << WStr(pReqInfo->name) << L"\" has no input channels.");
            Pa_Terminate();
            return false;
        }

        // Open enough channels to reach the requested index plus one
        m_numChannels = std::min(static_cast<int>(pReqInfo->maxInputChannels),
                                 m_channelIndex + 1);

        if (m_channelIndex >= m_numChannels)
        {
            LOG_ERROR(L"CAudioProcessor: Channel #" << m_channelIndex
                      << L" out of range for device #" << requested << L" (max input channels="
                      << pReqInfo->maxInputChannels << L").");
            Pa_Terminate();
            return false;
        }

        PaStreamParameters inputParams;
        inputParams.device                    = requested;
        inputParams.channelCount              = static_cast<int>(m_numChannels);
        inputParams.sampleFormat              = paFloat32;
        inputParams.suggestedLatency           = pReqInfo->defaultLowInputLatency;
        inputParams.hostApiSpecificStreamInfo = nullptr;

        paErr = Pa_OpenStream(&m_pStream,
                              &inputParams,
                              nullptr,           // no output
                              m_sampleRate,
                              paFramesPerBufferUnspecified,
                              paNoFlag,
                              PaCallback,
                              this);
        if (paErr != paNoError)
        {
            LOG_ERROR(L"CAudioProcessor: Failed to open device #" << requested
                      << L" \"" << WStr(pReqInfo->name) << L"\" ("
                      << WStr(Pa_GetErrorText(paErr)) << L").");
            Pa_Terminate();
            return false;
        }

        LOG(L"CAudioProcessor: Opened device #" << requested
            << L" \"" << WStr(pReqInfo->name) << L"\""
            << L" (" << m_numChannels << L" ch, sampling ch " << m_channelIndex << L")");

        // Query actual sample rate
        const PaStreamInfo *pInfo = Pa_GetStreamInfo(m_pStream);
        if (pInfo)
            m_sampleRate = static_cast<uint32_t>(pInfo->sampleRate);

        LOG_VERBOSE(L"CAudioProcessor: Using input device #" << requested
                    << L" \"" << WStr(pReqInfo->name) << L"\""
                    << L" | channels=" << m_numChannels
                    << L" | sampling channel=" << m_channelIndex
                    << L" | defaultSampleRate=" << pReqInfo->defaultSampleRate << L" Hz"
                    << L" | actualSampleRate=" << m_sampleRate << L" Hz");
    }
    else
    {
        m_numChannels = 1;

        paErr = Pa_OpenDefaultStream(&m_pStream,
                                     1,          // input channels  (mono)
                                     0,          // output channels (none)
                                     paFloat32,  // sample format
                                     m_sampleRate,
                                     paFramesPerBufferUnspecified,
                                     PaCallback,
                                     this);
        if (paErr != paNoError)
        {
            LOG_ERROR(L"CAudioProcessor: Pa_OpenDefaultStream failed: " << WStr(Pa_GetErrorText(paErr)));
            Pa_Terminate();
            return false;
        }

        // Query actual sample rate
        const PaStreamInfo *pInfo = Pa_GetStreamInfo(m_pStream);
        if (pInfo)
            m_sampleRate = static_cast<uint32_t>(pInfo->sampleRate);

        PaDeviceIndex defaultDev = Pa_GetDefaultInputDevice();
        const PaDeviceInfo *pDevInfo = Pa_GetDeviceInfo(defaultDev);
        if (pDevInfo)
        {
            LOG_VERBOSE(L"CAudioProcessor: Using default input device #" << defaultDev
                        << L" \"" << WStr(pDevInfo->name) << L"\""
                        << L" | defaultSampleRate=" << pDevInfo->defaultSampleRate << L" Hz"
                        << L" | actualSampleRate=" << m_sampleRate << L" Hz");
        }
    }

    // ── FFTW preparation ────────────────────────────────────────────────
    m_pFftInput  = fftw_alloc_real(m_fftSize);
    m_pFftOutput = fftw_alloc_complex(m_fftSize / 2 + 1);
    if (!m_pFftInput || !m_pFftOutput)
    {
        LOG_ERROR(L"CAudioProcessor: fftw_alloc failed.");
        Pa_CloseStream(m_pStream);
        m_pStream = nullptr;
        Pa_Terminate();
        return false;
    }
    // FFTW_MEASURE: plan creation is slow but produces an optimised plan
    m_fftPlan = fftw_plan_dft_r2c_1d(static_cast<int>(m_fftSize),
                                     m_pFftInput, m_pFftOutput,
                                     FFTW_MEASURE);

    // ── Hann window ─────────────────────────────────────────────────────
    // https://en.wikipedia.org/wiki/Hann_function
    m_window.resize(m_fftSize);
    const double TWO_PI_OVER_N = 2.0 * M_PI / static_cast<double>(m_fftSize - 1);
    for (size_t i = 0; i < static_cast<size_t>(m_fftSize); ++i)
        m_window[i] = 0.5 * (1.0 - std::cos(TWO_PI_OVER_N * static_cast<double>(i)));

    // ── PCM buffer ──────────────────────────────────────────────────────
    m_pcmBuffer.resize(m_fftSize, 0.0f);
    m_pcmWritePos = 0;

    // ── Start stream ────────────────────────────────────────────────────
    paErr = Pa_StartStream(m_pStream);
    if (paErr != paNoError)
    {
        LOG_ERROR(L"CAudioProcessor: Pa_StartStream failed: " << WStr(Pa_GetErrorText(paErr)));
        fftw_destroy_plan(m_fftPlan);
        fftw_free(m_pFftInput);
        fftw_free(m_pFftOutput);
        Pa_CloseStream(m_pStream);
        m_pStream = nullptr;
        Pa_Terminate();
        return false;
    }

    m_initialized = true;
    LOG_VERBOSE(L"CAudioProcessor: Initialized with sampleRate=" << m_sampleRate
                << L" Hz, fftSize=" << m_fftSize);
    return true;
}

void CAudioProcessor::Shutdown()
{
    if (!m_initialized)
        return;

    // Stop PortAudio stream
    if (m_pStream)
    {
        Pa_StopStream(m_pStream);
        Pa_CloseStream(m_pStream);
        m_pStream = nullptr;
    }

    // Release FFTW resources
    if (m_fftPlan)
    {
        fftw_destroy_plan(m_fftPlan);
        m_fftPlan = nullptr;
    }
    if (m_pFftInput)
    {
        fftw_free(m_pFftInput);
        m_pFftInput = nullptr;
    }
    if (m_pFftOutput)
    {
        fftw_free(m_pFftOutput);
        m_pFftOutput = nullptr;
    }

    Pa_Terminate();

    m_initialized = false;
    LOG_VERBOSE(L"CAudioProcessor: Shut down.");
}

SAudioSpectrum CAudioProcessor::GetSpectrum() const
{
    if (!m_initialized)
        return {};

    LockGuard lock(m_spectrumMutex);
    return m_spectrum;
}
