#pragma once

#include "CQC2SDisplay.h"
#include "DisplayUtils.h"
#include <cmath>
#include <chrono>
#include <thread>

// Cubic Bézier easing - CSS-style: implicit P0=(0,0), P3=(1,1).
// p_p1 and p_p2 are the two inner control points.
// Given a linear parameter t in [0,1], solves for the Bézier curve parameter s
// such that X(s) == t using Newton–Raphson, then returns Y(s) as the eased value.
struct SCubicBezier
{
    float m_p1x;
    float m_p1y;
    float m_p2x;
    float m_p2y;

    static constexpr SCubicBezier EaseInOut() { return {0.11f, 0.0f, 0.35f, 1.0f}; }
    static constexpr SCubicBezier Linear() { return {0.0f, 0.0f, 1.0f, 1.0f}; }
};

// Evaluate one component of a cubic Bézier with P0=0, P3=1 at parameter s
inline float CubicBezierComponent(float p_p1, float p_p2, float p_s)
{
    float oneMinusS = 1.0f - p_s;
    // B(s) = 3*(1-s)^2*s*p1 + 3*(1-s)*s^2*p2 + s^3
    return 3.0f * oneMinusS * oneMinusS * p_s * p_p1 + 3.0f * oneMinusS * p_s * p_s * p_p2 + p_s * p_s * p_s;
}

// Derivative of CubicBezierComponent w.r.t. s
inline float CubicBezierComponentDerivative(float p_p1, float p_p2, float p_s)
{
    float oneMinusS = 1.0f - p_s;
    return 3.0f * oneMinusS * oneMinusS * p_p1 + 6.0f * oneMinusS * p_s * (p_p2 - p_p1) + 3.0f * p_s * p_s * (1.0f - p_p2);
}

// Map linear t -> eased brightness via cubic Bézier
inline float CubicBezierEval(const SCubicBezier &p_bezier, float p_t)
{
    if (p_t <= 0.0f)
        return 0.0f;
    if (p_t >= 1.0f)
        return 1.0f;

    // Newton–Raphson: find s such that X(s) == t
    float s = p_t; // initial guess
    for (int iter = 0; iter < 8; ++iter)
    {
        float xS = CubicBezierComponent(p_bezier.m_p1x, p_bezier.m_p2x, s);
        float dxS = CubicBezierComponentDerivative(p_bezier.m_p1x, p_bezier.m_p2x, s);
        if (std::abs(dxS) < 1e-6f)
            break;
        s -= (xS - p_t) / dxS;
        s = std::max(0.0f, std::min(1.0f, s));
    }
    return CubicBezierComponent(p_bezier.m_p1y, p_bezier.m_p2y, s);
}

class CPulseColorDisplay : public CQC2SDisplay
{
    SRGBColor m_baseColor;
    float m_t = 0.0f; // linear phase in [0, 1]
    float m_speed;    // advance per frame (in [0,1] units)
    bool m_increasing = true;
    SCubicBezier m_bezier;

    SRGBColor ApplyBrightness(float p_brightness) const
    {
        return {
            static_cast<uint8_t>(m_baseColor.m_red * p_brightness),
            static_cast<uint8_t>(m_baseColor.m_green * p_brightness),
            static_cast<uint8_t>(m_baseColor.m_blue * p_brightness)};
    }

public:
    CPulseColorDisplay(SRGBColor p_color, float p_speed, String p_name, UniquePtr<CEndCondition> p_pEndCondition,
                       SCubicBezier p_bezier = SCubicBezier::EaseInOut(), String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)),
          m_baseColor(p_color), m_speed(p_speed), m_bezier(p_bezier) {}

    void Reset() override
    {
        m_t = 0.0f;
        m_increasing = true;
        
        CQC2SDisplay::Reset(); 
    }
    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        using namespace std::chrono;
        const auto FRAME_DURATION = 50ms;
        const auto FRAME_START = steady_clock::now();

        SRGBColor current = ApplyBrightness(CubicBezierEval(m_bezier, m_t));
        SendMonoColorFrame(p_communicator, current);

        if (m_increasing)
        {
            m_t += m_speed;
            if (m_t >= 1.0f)
            {
                m_t = 1.0f;
                m_increasing = false;
            }
        }
        else
        {
            m_t -= m_speed;
            if (m_t <= 0.0f)
            {
                m_t = 0.0f;
                m_increasing = true;
            }
        }

        const auto ELAPSED = steady_clock::now() - FRAME_START;
        auto remaining = FRAME_DURATION - ELAPSED;
        if (remaining > remaining.zero())
        {
            std::this_thread::sleep_for(remaining);
        }
        return true;
    }
};
