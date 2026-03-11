#pragma once

#include "CQC2SDisplay.h"
#include <iostream>
#include <stdexcept>

class CMultiDisplay : public CQC2SDisplay
{
    DynamicContainer<UniquePtr<CQC2SDisplay>> m_displays;

    String m_initialDisplay;
    Map<String, size_t> m_mapDisplayIndices;
    DynamicContainer<size_t> m_mapIndexTransitions;
    size_t m_currentIndex;

public:
    CMultiDisplay(String p_name, UniquePtr<CEndCondition> p_pEndCondition, String p_firstDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_firstDisplay)) {}

    void AddDisplay(UniquePtr<CQC2SDisplay> p_display)
    {
        // add to map
        String name = p_display->GetName();
        if (name.empty())
            throw std::invalid_argument("Display name must not be empty");
        if (m_mapDisplayIndices.find(name) != m_mapDisplayIndices.end())
            throw std::invalid_argument("Display with name '" + name + "' already exists");
        m_mapDisplayIndices[name] = m_displays.size();
        m_displays.push_back(std::move(p_display));
    }

    bool Initialize() override
    {
        // init transition map for multidisplay
        m_mapIndexTransitions.resize(m_mapDisplayIndices.size());
        for (auto &pDisplay : m_displays)
        {
            String thisDisplay = pDisplay->GetName();
            String next = pDisplay->GetNextDisplay();

            auto thisIt = m_mapDisplayIndices.find(thisDisplay);
            if (thisIt == m_mapDisplayIndices.end())
                throw std::runtime_error("Display name '" + thisDisplay + "' not found in index map");

            size_t thisDisplayIndex = thisIt->second;

            // next has been chosen
            if (!next.empty())
            {
                auto nextIt = m_mapDisplayIndices.find(next);
                if (nextIt == m_mapDisplayIndices.end())
                    throw std::runtime_error("Next display name '" + next + "' not found in index map");
                m_mapIndexTransitions[thisDisplayIndex] = nextIt->second;
            }
            // no next given -> assume stop
            else
            {
                m_mapIndexTransitions[thisDisplayIndex] = (size_t)-1;
            }
        }
        // set initial index
        if (m_nextDisplay.empty() || m_mapDisplayIndices.find(m_nextDisplay) == m_mapDisplayIndices.end())
            throw std::runtime_error("Initial display '" + m_nextDisplay + "' not found in display index map");
        m_currentIndex = m_mapDisplayIndices[m_nextDisplay];
        // init all the displays
        for (auto &pDisplay : m_displays)
        {
            if (!pDisplay->Initialize())
                return false;
        }
        return true;
    }

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        // not used!
        return false;
    }

    void Shutdown(CQuadcast2SCommunicator &p_communicator) override
    {
        for (auto &pDisplay : m_displays)
        {
            pDisplay->Shutdown(p_communicator);
        }
    }

    void Display(CQuadcast2SCommunicator &p_communicator,
                 const AtomicBool &p_signalStopRequest,
                 FrameCallback p_frameCallback = nullptr) override
    {
        if (m_displays.empty())
            return;

        do
        {
            // run display of current display
            auto &pDisplay = m_displays[m_currentIndex];
            pDisplay->Display(p_communicator, p_signalStopRequest, p_frameCallback);
            // get next one in line
            auto next = m_mapIndexTransitions[m_currentIndex];
            m_currentIndex = next;
        } while (m_currentIndex != (size_t)-1 && !p_signalStopRequest.load());
    }
};
