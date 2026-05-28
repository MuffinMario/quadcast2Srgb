// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../hid/Packets.h"
#include <algorithm>
#include <cmath>

// ── HSV color type ────────────────────────────────────────────────────────────

struct SHSV
{
    // [0, 360)
    double m_hue;
    // [0, 1]
    double m_saturation;
    // [0, 1]
    double m_value;

    SHSV() : m_hue(0.0), m_saturation(1.0), m_value(1.0) {}
    SHSV(double p_hue, double p_saturation, double p_value)
        : m_hue(p_hue), m_saturation(p_saturation), m_value(p_value) {}

    static SHSV FromRGB(SRGBColor p_color)
    {
        const double R     = p_color.m_red   / 255.0;
        const double G     = p_color.m_green / 255.0;
        const double B     = p_color.m_blue  / 255.0;
        const double CMAX  = std::max({R, G, B});
        const double CMIN  = std::min({R, G, B});
        const double DELTA = CMAX - CMIN;

        double hue = 0.0;
        if (DELTA > 0.0)
        {
            if (CMAX == R)      hue = 60.0 * std::fmod((G - B) / DELTA, 6.0);
            else if (CMAX == G) hue = 60.0 * ((B - R) / DELTA + 2.0);
            else                hue = 60.0 * ((R - G) / DELTA + 4.0);
            if (hue < 0.0) hue += 360.0;
        }
        return SHSV{hue, (CMAX > 0.0) ? DELTA / CMAX : 0.0, CMAX};
    }

    SRGBColor ToRGB() const
    {
        // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
        const auto CHROMA = m_value * m_saturation;
        const auto HDER   = m_hue / 60.0;
        const auto X      = CHROMA * (1 - std::abs(std::fmod(HDER, 2) - 1));

        // C on 0~1 5~6 | X on 1~2 4~5 | 0 on 2~3
        double r1 = 0, g1 = 0, b1 = 0;
        if (0 <= HDER && HDER < 1)
        {
            r1 = CHROMA;
            g1 = X;
        }
        else if (1 <= HDER && HDER < 2)
        {
            r1 = X;
            g1 = CHROMA;
        }
        else if (2 <= HDER && HDER < 3)
        {
            g1 = CHROMA;
            b1 = X;
        }
        else if (3 <= HDER && HDER < 4)
        {
            g1 = X;
            b1 = CHROMA;
        }
        else if (4 <= HDER && HDER < 5)
        {
            r1 = X;
            b1 = CHROMA;
        }
        else if (5 <= HDER && HDER < 6)
        {
            r1 = CHROMA;
            b1 = X;
        } // else, black

        const auto M = m_value - CHROMA;
        return SRGBColor{
            static_cast<uint8_t>((r1 + M) * 255),
            static_cast<uint8_t>((g1 + M) * 255),
            static_cast<uint8_t>((b1 + M) * 255)};
    }
};

// Interpolate two HSV values along the shortest hue path.
inline SHSV LerpHSV(const SHSV &p_a, const SHSV &p_b, double p_t)
{
    double hueDiff = p_b.m_hue - p_a.m_hue;
    if (hueDiff >  180.0) hueDiff -= 360.0;
    if (hueDiff < -180.0) hueDiff += 360.0;
    // Since we dont handle negative hues when converting to rgb -> just like in rainbow display we (x+360)&360
    const double HUE = std::fmod(p_a.m_hue + hueDiff * p_t + 360.0, 360.0);
    return SHSV{
        HUE,
        p_a.m_saturation + (p_b.m_saturation - p_a.m_saturation) * p_t,
        p_a.m_value      + (p_b.m_value      - p_a.m_value)      * p_t};
}

// ── Cubic Bézier easing ───────────────────────────────────────────────────────
// CSS-style: implicit P0=(0,0), P3=(1,1).
// p_p1 and p_p2 are the two inner control points.
// Given a linear parameter t in [0,1], solves for the Bézier curve parameter s
// such that X(s) == t using Newton–Raphson, then returns Y(s) as the eased value.

struct SCubicBezier
{
    float m_p1x, m_p1y, m_p2x, m_p2y;

    static constexpr SCubicBezier EaseInOut() { return {0.11f, 0.0f, 0.35f, 1.0f}; }
    static constexpr SCubicBezier Linear()    { return {0.0f,  0.0f, 1.0f,  1.0f}; }
};

// Evaluate one component of a cubic Bézier with P0=0, P3=1 at parameter s
inline float CubicBezierComponent(float p_p1, float p_p2, float p_s)
{
    const float OMS = 1.0f - p_s;
    // B(s) = 3*(1-s)^2*s*p1 + 3*(1-s)*s^2*p2 + s^3
    return 3.0f * OMS * OMS * p_s * p_p1 + 3.0f * OMS * p_s * p_s * p_p2 + p_s * p_s * p_s;
}

// Derivative of CubicBezierComponent w.r.t. s
inline float CubicBezierComponentDerivative(float p_p1, float p_p2, float p_s)
{
    const float OMS = 1.0f - p_s;
    return 3.0f * OMS * OMS * p_p1 + 6.0f * OMS * p_s * (p_p2 - p_p1) + 3.0f * p_s * p_s * (1.0f - p_p2);
}

// Map linear t -> eased value via cubic Bézier
inline float CubicBezierEval(const SCubicBezier &p_bezier, float p_t)
{
    if (p_t <= 0.0f) return 0.0f;
    if (p_t >= 1.0f) return 1.0f;

    // Newton–Raphson: find s such that X(s) == t
    float s = p_t;
    for (int iter = 0; iter < 8; ++iter)
    {
        const float X_S  = CubicBezierComponent(p_bezier.m_p1x, p_bezier.m_p2x, s);
        const float DX_S = CubicBezierComponentDerivative(p_bezier.m_p1x, p_bezier.m_p2x, s);
        if (std::abs(DX_S) < 1e-6f) break;
        s -= (X_S - p_t) / DX_S;
        s  = std::max(0.0f, std::min(1.0f, s));
    }
    return CubicBezierComponent(p_bezier.m_p1y, p_bezier.m_p2y, s);
}
