// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "../Globals.h"
#include "CIRenderer.h"
#include "../audio/AudioTypes.h"
#include "../audio/CAudioProcessor.h"
#include "CEndCondition.h"

/// Callback invoked once per frame before DisplayFrame(); return false to abort the display loop.
/// The renderer parameter is the same object passed to Display().
using FrameCallback = std::function<bool(CIRenderer &)>;

class CQC2SDisplay
{
protected:
    String m_name;
    String m_nextDisplay; // name of the next display to transition to (empty = none)
    UniquePtr<CEndCondition> m_pEndCondition;

    // ---- audio capture -----------------------------------------------------------
    CAudioProcessor *m_pAudioProcessor = nullptr;  ///< Non-owning pointer to the shared audio processor
    // ------------------------------------------------------------------------------

public:
    CQC2SDisplay(String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : m_name(std::move(p_name)), m_nextDisplay(std::move(p_nextDisplay)), m_pEndCondition(std::move(p_pEndCondition))
    {
    }
    virtual ~CQC2SDisplay() = default;

    // Called once before displaying starts; return false to abort
    virtual bool Initialize() { return true; }
    virtual void Reset() { if (m_pEndCondition) m_pEndCondition->Reset(); }

    // Called once per frame; return false to stop displaying
    virtual bool DisplayFrame(CIRenderer &/*p_renderer*/) = 0;

    // Called once after displaying ends or is aborted
    virtual void Shutdown(CIRenderer &/*p_renderer*/) {}

    // ---- audio capture accessor --------------------------------------------------
    /// Set the shared audio processor.  Called once by main() before Display().
    /// CMultiDisplay overrides this to propagate to all child displays.
    virtual void SetAudioProcessor(CAudioProcessor *p_pProcessor) { m_pAudioProcessor = p_pProcessor; }

    /// Thread-safe copy of the latest audio spectrum.
    /// Returns an empty/invalid spectrum when no processor has been set.
    SAudioSpectrum GetAudioSpectrum() const
    {
        if (!m_pAudioProcessor)
            return {};
        return m_pAudioProcessor->GetSpectrum();
    }

    /// Whether a shared audio processor has been set.
    bool IsAudioEnabled() const { return m_pAudioProcessor != nullptr; }
    // ------------------------------------------------------------------------------

    virtual String GetName() const { return m_name; }
    virtual void SetName(const String &p_name) { m_name = p_name; }
    virtual String GetNextDisplay() const { return m_nextDisplay; }
    virtual void SetNextDisplay(String p_nextDisplay) { m_nextDisplay = std::move(p_nextDisplay); }

    virtual CEndCondition *GetEndCondition() const { return m_pEndCondition.get(); }
    virtual void SetEndCondition(UniquePtr<CEndCondition> p_pEndCondition) { m_pEndCondition = std::move(p_pEndCondition); }

    virtual void Display(CIRenderer &p_renderer,
                         const AtomicBool &p_signalStopRequest,
                         FrameCallback p_frameCallback = nullptr)
    {
        Reset();
        // check for end condition
        if (m_pEndCondition.get())
        {
            do
            {
                if (p_frameCallback && !p_frameCallback(p_renderer))
                    break;
                if (p_signalStopRequest.load())
                    break;
                bool displaySuccess = DisplayFrame(p_renderer);
                if (!displaySuccess)
                    break;
                m_pEndCondition->NotifyFrameDisplayed();
            } while (!m_pEndCondition->IsMet() && !p_signalStopRequest.load());
        }
        else
        {
            // no end condition, continue ad infinitum (or until it returns false)
            while (!p_signalStopRequest.load())
            {
                if (p_frameCallback && !p_frameCallback(p_renderer))
                    break;
                if (!DisplayFrame(p_renderer))
                    break;
            }
        }
    }
};
