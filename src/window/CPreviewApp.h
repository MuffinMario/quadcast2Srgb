// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "CWindowRenderer.h"
#include "../config/CConfigBuilder.h"
#include "../audio/CAudioProcessor.h"
#include "../display/CSolidColorDisplay.h"
#include "../display/CPulseColorDisplay.h"
#include "../display/CRainbowDisplay.h"
#include "../display/CColorTransitionDisplay.h"
#include "../display/CVideoDisplay.h"
#ifdef USE_GLSL
#include "../display/CGLSLDisplay.h"
#include "../../external/ImGuiFileDialog/ImGuiFileDialog.h"
#endif
#include "../display/CMultiDisplay.h"
#include "../display/CQC2SDisplayFactory.h"
#include "../display/ColorTypes.h"
#include "imgui.h"
#include <cstdio>
#include <filesystem>

/// Manages the preview window, display loop, and ImGui UI.
/// Run() blocks until the window is closed or the display ends.
class CPreviewApp
{
    CWindowRenderer m_renderer;
    CAudioProcessor m_audioProcessor;
    SProgramConfig m_config;
    DynamicContainer<char> m_cmdBuffer;
    bool m_restartRequested = false;
    String m_pendingType;
    DynamicContainer<SAudioDeviceInfo> m_audioDevices;
    DynamicContainer<String> m_audioDeviceNames; // combo labels, cached alongside m_audioDevices
    bool m_audioDevicesCached = false;
    std::filesystem::file_time_type m_cachedShaderMtime = std::filesystem::file_time_type::min();

public:
    CPreviewApp(SProgramConfig p_config)
        : m_config(std::move(p_config))
    {
    }

    bool Initialize()
    {
        if (!m_renderer.IsRunning())
        {
            LOG_ERROR(L"Failed to create preview window.");
            return false;
        }

        // ── Audio capture ───────────────────────────────────────
        if (m_config.m_enableAudio)
        {
            if (!m_audioProcessor.Initialize(1024, m_config.m_audioDeviceId, m_config.m_audioChannel))
            {
                LOG("[CPreviewApp] Failed to initialize audio processor.");
                return false;
            }
            m_audioProcessor.SetInputGain(m_config.m_inputGain);
            m_audioProcessor.SetSmoothing(m_config.m_audioSmoothing, m_config.m_audioSmoothingAlpha);
            m_config.m_pDisplay->SetAudioProcessor(&m_audioProcessor);
        }

        if (!m_config.m_pDisplay->Initialize())
        {
            LOG_ERROR(L"Failed to initialize display: " + WStr(m_config.m_pDisplay->GetName()));
            return false;
        }

        return true;
    }

    static void ShowColor(const char *p_pLabel, SRGBColor p_color)
    {
        ImGui::Text("%s: #%02X%02X%02X", p_pLabel,
                    p_color.m_red, p_color.m_green, p_color.m_blue);
    }

    static void ShowBezier(const char *p_pLabel, const SCubicBezier &p_bezier)
    {
        ImGui::Text("%s: (%.2f, %.2f) -> (%.2f, %.2f)", p_pLabel,
                    p_bezier.m_p1x, p_bezier.m_p1y,
                    p_bezier.m_p2x, p_bezier.m_p2y);
    }

    static bool ShowColorPicker(const char *p_pLabel, SRGBColor &p_color)
    {
        float col[3] = {p_color.m_red / 255.f, p_color.m_green / 255.f, p_color.m_blue / 255.f};
        if (ImGui::ColorEdit3(p_pLabel, col))
        {
            p_color = {static_cast<uint8_t>(col[0] * 255.f),
                       static_cast<uint8_t>(col[1] * 255.f),
                       static_cast<uint8_t>(col[2] * 255.f)};
            return true;
        }
        return false;
    }

    static void ShowBezierEditor(const char *p_pLabel, SCubicBezier &p_bezier)
    {
        ImGui::SameLine();
        if (ImGui::Button("Modify"))
            ImGui::OpenPopup(p_pLabel);

        if (ImGui::BeginPopupModal(p_pLabel, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            // Work on a local copy so Cancel discards changes.
            static SCubicBezier s_bezier; // persisted across frames while popup is open
            if (ImGui::IsWindowAppearing())
                s_bezier = p_bezier;

            const float SIZE = 200.f;
            const float DOT_RADIUS = 6.f;
            const ImVec2 ORIGIN = ImGui::GetCursorScreenPos();

            // Reserve graph space first so drag overlays don't affect layout.
            ImGui::Dummy(ImVec2(SIZE + DOT_RADIUS, SIZE));

            ImDrawList *pDraw = ImGui::GetWindowDrawList();

            // Background
            pDraw->AddRectFilled(ORIGIN, ImVec2(ORIGIN.x + SIZE, ORIGIN.y + SIZE), IM_COL32(32, 32, 32, 255));
            pDraw->AddRect(ORIGIN, ImVec2(ORIGIN.x + SIZE, ORIGIN.y + SIZE), IM_COL32(128, 128, 128, 255));

            // Grid
            for (int i = 0; i <= 10; ++i)
            {
                float f = i / 10.f;
                pDraw->AddLine(ImVec2(ORIGIN.x + f * SIZE, ORIGIN.y), ImVec2(ORIGIN.x + f * SIZE, ORIGIN.y + SIZE), IM_COL32(64, 64, 64, 255));
                pDraw->AddLine(ImVec2(ORIGIN.x, ORIGIN.y + f * SIZE), ImVec2(ORIGIN.x + SIZE, ORIGIN.y + f * SIZE), IM_COL32(64, 64, 64, 255));
            }

            // Point helpers
            auto scr = [&](float p_x, float p_y)
            { return ImVec2(ORIGIN.x + p_x * SIZE, ORIGIN.y + (1.f - p_y) * SIZE); };
            auto drawPt = [&](ImVec2 p_pt, ImU32 p_col)
            { pDraw->AddCircleFilled(p_pt, DOT_RADIUS-1.f, p_col); pDraw->AddCircle(p_pt, DOT_RADIUS, IM_COL32_WHITE); };
            auto drawPtOutline = [&](ImVec2 p_pt, ImU32 p_col)
            { pDraw->AddCircle(p_pt, DOT_RADIUS, IM_COL32_WHITE); };

            // Fixed endpoints
            drawPtOutline(scr(0.f, 0.f), IM_COL32(255, 0, 0, 255));
            drawPtOutline(scr(1.f, 1.f), IM_COL32(255, 0, 0, 255));

            // Draggable control points on the local copy
            ImVec2 p1 = scr(s_bezier.m_p1x, s_bezier.m_p1y);
            ImVec2 p2 = scr(s_bezier.m_p2x, s_bezier.m_p2y);
            ImVec2 cursorSave = ImGui::GetCursorPos();

            auto drag = [&](ImVec2 &p_pt, float &p_outX, float &p_outY, const char *p_pId)
            {
                ImGui::PushID(p_pId);
                ImGui::SetCursorScreenPos(ImVec2(p_pt.x - 8, p_pt.y - 8));
                ImGui::InvisibleButton("##drag", ImVec2(16, 16));
                if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                {
                    p_pt.x += ImGui::GetIO().MouseDelta.x;
                    p_pt.y += ImGui::GetIO().MouseDelta.y;
                    p_outX = std::clamp((p_pt.x - ORIGIN.x) / SIZE, 0.f, 1.f);
                    p_outY = std::clamp(1.f - (p_pt.y - ORIGIN.y) / SIZE, 0.f, 1.f);
                    p_pt = scr(p_outX, p_outY);
                }
                ImGui::PopID();
            };

            drag(p1, s_bezier.m_p1x, s_bezier.m_p1y, "p1");
            drag(p2, s_bezier.m_p2x, s_bezier.m_p2y, "p2");

            ImGui::SetCursorPos(cursorSave);

            // Draw control points and lines
            drawPt(p1, IM_COL32(0, 255, 0, 255));
            drawPt(p2, IM_COL32(0, 128, 255, 255));
            pDraw->AddLine(scr(0.f, 0.f), p1, IM_COL32(0, 255, 0, 128));
            pDraw->AddLine(scr(1.f, 1.f), p2, IM_COL32(0, 128, 255, 128));

            // Bezier curve (sampled from local copy)
            for (int i = 0; i < 50; ++i)
            {
                float t0 = i / 50.f, t1 = (i + 1) / 50.f;
                float y0 = CubicBezierEval(s_bezier, t0);
                float y1 = CubicBezierEval(s_bezier, t1);
                ImVec2 a = scr(t0, y0), b = scr(t1, y1);
                pDraw->AddLine(a, b, IM_COL32(255, 255, 0, 255), 2.f);
            }

            ImGui::Spacing();
            ImGui::SeparatorText("Presets");

            auto presetBtn = [&](const char *p_pName, float p_p1x, float p_p1y, float p_p2x, float p_p2y)
            {
                if (ImGui::Button(p_pName))
                {
                    s_bezier.m_p1x = p_p1x;
                    s_bezier.m_p1y = p_p1y;
                    s_bezier.m_p2x = p_p2x;
                    s_bezier.m_p2y = p_p2y;
                }
            };

            int btnCount = 0;
            auto sep = [&]
            { if (++btnCount % 3 != 0) ImGui::SameLine(); };

            // https://easings.net/
            presetBtn("Linear", 0.0f, 0.0f, 1.0f, 1.0f);
            sep();
            presetBtn("Ease In", 0.32f, 0.0f, 0.67f, 0.0f);
            sep();
            presetBtn("Ease Out", 0.33f, 1.0f, 0.68f, 1.0f);
            sep();

            ImGui::Spacing();
            ImGui::Separator();

            if (ImGui::Button("Set"))
            {
                p_bezier = s_bezier;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
                ImGui::CloseCurrentPopup();

            ImGui::EndPopup();
        }
    }

    void ShowDisplayOptions(CQC2SDisplay *p_pDisplay)
    {
        if (auto pSolid = dynamic_cast<CSolidColorDisplay *>(p_pDisplay))
        {
            SRGBColor color = pSolid->GetColor();
            if (ShowColorPicker("Color", color))
                pSolid->SetColor(color);
        }
        else if (auto pPulse = dynamic_cast<CPulseColorDisplay *>(p_pDisplay))
        {
            SRGBColor color = pPulse->GetColor();
            if (ShowColorPicker("Color", color))
                pPulse->SetColor(color);
            float speed = pPulse->GetSpeed();
            ImGui::SliderFloat("Speed", &speed, 0.001f, 0.5f, "%.4f");
            pPulse->SetSpeed(speed);
            SCubicBezier bezier = pPulse->GetBezier();
            ImGui::Text("Bezier: (%.2f, %.2f) -> (%.2f, %.2f)",
                        bezier.m_p1x, bezier.m_p1y,
                        bezier.m_p2x, bezier.m_p2y);
            ShowBezierEditor("Pulse Bezier", bezier);
            pPulse->SetBezier(bezier);
        }
        else if (auto pRainbow = dynamic_cast<CRainbowDisplay *>(p_pDisplay))
        {

            // ── Mode combo ────────────────────
            const char *rainbowModes[] = {"Flat", "Rolling Vertical", "Rolling Horizontal", "Rolling Diagonal"};
            int modeIdx = static_cast<int>(pRainbow->GetMode());
            if (ImGui::Combo("Mode", &modeIdx, rainbowModes, IM_ARRAYSIZE(rainbowModes)))
                pRainbow->SetMode(static_cast<ERainbowMode>(modeIdx));

            // ── Speed slider with invert ──────
            bool invert = (pRainbow->GetSpeed() < 0.0);
            float absSpeed = static_cast<float>(std::abs(pRainbow->GetSpeed()));
            if (ImGui::SliderFloat("Speed (°/frame)", &absSpeed, 0.0f, 30.0f, "%.1f"))
                pRainbow->SetSpeed(invert ? -static_cast<double>(absSpeed) : static_cast<double>(absSpeed));
            // ImGui::SameLine();
            if (ImGui::Checkbox("Invert Rainbow Direction", &invert))
                pRainbow->SetSpeed(invert ? -static_cast<double>(absSpeed) : static_cast<double>(absSpeed));
        }
        else if (auto pTransition = dynamic_cast<CColorTransitionDisplay *>(p_pDisplay))
        {

            // ── Speed ────────────────────────
            float speed = pTransition->GetSpeed();
            ImGui::SliderFloat("Speed", &speed, 0.0001f, 0.05f, "%.4f");
            pTransition->SetSpeed(speed);

            // ── Bezier ───────────────────────
            SCubicBezier bezier = pTransition->GetBezier();
            ImGui::Text("Bezier: (%.2f, %.2f) -> (%.2f, %.2f)",
                        bezier.m_p1x, bezier.m_p1y,
                        bezier.m_p2x, bezier.m_p2y);
            ShowBezierEditor("Transition Bezier", bezier);
            pTransition->SetBezier(bezier);

            // ── Color list ───────────────────
            const auto &colors = pTransition->GetColors();
            int removeIdx = -1;
            for (size_t i = 0; i < colors.size(); ++i)
            {
                ImGui::PushID(static_cast<int>(i));
                SRGBColor rgb = colors[i].ToRGB();
                char label[32];
                snprintf(label, sizeof(label), "Color %zu", i + 1);
                if (ShowColorPicker(label, rgb))
                    pTransition->SetColor(i, SHSV::FromRGB(rgb));
                ImGui::SameLine();
                if (ImGui::Button("-") && colors.size() > 2)
                    removeIdx = static_cast<int>(i);
                ImGui::PopID();
            }
            if (removeIdx >= 0)
                pTransition->RemoveColor(static_cast<size_t>(removeIdx));
            if (ImGui::Button("+ Add Color"))
            {
                // pick a hue offset from the last color for variety
                float hue = 0.0f;
                if (!colors.empty())
                    hue = std::fmod(colors.back().m_hue + 60.0, 360.0);
                pTransition->AddColor(SHSV{hue, 1.0, 1.0});
            }
        }
        else if (auto pVideo = dynamic_cast<CVideoDisplay *>(p_pDisplay))
        {
            // ── Video path ──────────────────
            {
                constexpr size_t PATH_BUF_SIZE = 512;
                char pathBuf[PATH_BUF_SIZE] = {};
                String curPath = pVideo->GetVideoPath();
                std::copy_n(curPath.begin(), std::min(curPath.size(), PATH_BUF_SIZE - 1), pathBuf);

                const float BROWSE_W = ImGui::CalcTextSize("Browse").x + ImGui::GetStyle().FramePadding.x * 2;
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - BROWSE_W - ImGui::GetStyle().ItemSpacing.x);
                ImGui::InputText("##videoPath", pathBuf, PATH_BUF_SIZE, ImGuiInputTextFlags_ReadOnly);
                ImGui::SameLine();
                if (ImGui::Button("Browse"))
                {
                    IGFD::FileDialogConfig config;
                    config.path = curPath.empty() ? "." : curPath;
                    ImGuiFileDialog::Instance()->OpenDialog("VideoFile", "Choose Video File", ".rgbffmpeg,.*", config);
                }
                if (ImGuiFileDialog::Instance()->Display("VideoFile", ImGuiWindowFlags_NoCollapse, ImVec2(300, 250)))
                {
                    if (ImGuiFileDialog::Instance()->IsOk())
                    {
                        String filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
                        pVideo->SetVideoPath(filePathName);
                        pVideo->Initialize();
                    }
                    ImGuiFileDialog::Instance()->Close();
                }
            }

            // ── Format ────────────────────────
            {
                int fmt = (pVideo->GetFormat() == EVideoFormat::Rgb) ? 0 : 1;
                if (ImGui::RadioButton("RGB", &fmt, 0)) { pVideo->SetFormat(EVideoFormat::Rgb); pVideo->Initialize(); }
                ImGui::SameLine();
                if (ImGui::RadioButton("Greyscale", &fmt, 1)) { pVideo->SetFormat(EVideoFormat::Greyscale); pVideo->Initialize(); }
            }

            // ── FPS ─────────────────────────
            int fps = static_cast<int>(pVideo->GetFPS());
            if (ImGui::SliderInt("FPS", &fps, 1, 60))
                pVideo->SetFPS(static_cast<uint32_t>(fps));

            ImGui::Text("Frames: %zu", pVideo->GetFrameCount());
        }
#ifdef USE_GLSL
        else if (auto pGLSL = dynamic_cast<CGLSLDisplay *>(p_pDisplay))
        {
            // ── Shader path ──────────────────
            {
                constexpr size_t PATH_BUF_SIZE = 512;
                char pathBuf[PATH_BUF_SIZE] = {};
                String curPath = pGLSL->GetShaderPath();
                std::copy_n(curPath.begin(), std::min(curPath.size(), PATH_BUF_SIZE - 1), pathBuf);

                const float BROWSE_W = ImGui::CalcTextSize("Browse").x + ImGui::GetStyle().FramePadding.x * 2;
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - BROWSE_W - ImGui::GetStyle().ItemSpacing.x);
                ImGui::InputText("##shaderPath", pathBuf, PATH_BUF_SIZE, ImGuiInputTextFlags_ReadOnly);
                ImGui::SameLine();
                if (ImGui::Button("Browse"))
                {
                    IGFD::FileDialogConfig config;
                    config.path = curPath.empty() ? "." : curPath;

                    ImGuiFileDialog::Instance()->OpenDialog("GLSLShaderFile", "Choose GLSL Shader", ".glsl,.frag,.txt,.*", config);
                }
                if (ImGuiFileDialog::Instance()->Display("GLSLShaderFile", ImGuiWindowFlags_NoCollapse, ImVec2(300, 250)))
                {
                    if (ImGuiFileDialog::Instance()->IsOk())
                    {
                        String filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
                        pGLSL->SetShaderPath(filePathName);

#ifdef USE_GLSL
                        if (auto pGLSL = dynamic_cast<CGLSLDisplay *>(m_config.m_pDisplay.get()))
                            pGLSL->Initialize();
#endif
                    }
                    ImGuiFileDialog::Instance()->Close();
                }
            }

            // ── Reload + Auto-reload ────────
            ImGui::Checkbox("Auto Recompile on Change", &m_config.m_glslAutoReload);
            if (ImGui::Button("Recompile"))
                pGLSL->Initialize();

            // ── FPS ─────────────────────────
            int fps = static_cast<int>(pGLSL->GetFPS());
            if (ImGui::SliderInt("FPS", &fps, 1, 60))
                pGLSL->SetFPS(static_cast<uint32_t>(fps));
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Affects the preview window frame rate as well");

            // ── Scale ───────────────────────
            
            int scale = static_cast<int>(pGLSL->GetScale());
            if (ImGui::SliderInt("Superscale", &scale, 1, 50))
            {
                pGLSL->SetScale(static_cast<uint32_t>(scale));
                // needs to recompile since fbo info has changed
                pGLSL->Initialize();
            }
        }
#endif
        else if (dynamic_cast<CMultiDisplay *>(p_pDisplay))
        {
            ImGui::Text("Type: multi-display");
        }
        else
        {
            ImGui::Text("Type: unknown");
        }
    }

    static String GetDisplayTypeName(CQC2SDisplay *p_pDisplay)
    {
        if (!p_pDisplay)
            return "";
        if (dynamic_cast<CSolidColorDisplay *>(p_pDisplay))
            return "solid";
        if (dynamic_cast<CPulseColorDisplay *>(p_pDisplay))
            return "pulse";
        if (dynamic_cast<CRainbowDisplay *>(p_pDisplay))
            return "rainbow";
        if (dynamic_cast<CColorTransitionDisplay *>(p_pDisplay))
            return "transition";
        if (dynamic_cast<CVideoDisplay *>(p_pDisplay))
            return "video";
        if (dynamic_cast<CMultiDisplay *>(p_pDisplay))
            return "multi";
#ifdef USE_GLSL
        if (dynamic_cast<CGLSLDisplay *>(p_pDisplay))
            return "glsl";
#endif
        return "";
    }

    static UniquePtr<CQC2SDisplay> CreateDefaultDisplay(const String &p_type)
    {
        constexpr SRGBColor DEFAULT_COLOR{0x29, 0x00, 0x66};
        if (p_type == "solid")
            return CQC2SDisplayFactory::CreateSolidColor(DEFAULT_COLOR, p_type);
        if (p_type == "pulse")
            return CQC2SDisplayFactory::CreatePulseColor(DEFAULT_COLOR, 0.025f, p_type);
        if (p_type == "rainbow")
            return CQC2SDisplayFactory::CreateRainbow(ERainbowMode::Flat, 1.0, p_type);
        if (p_type == "transition")
        {
            DynamicContainer<SHSV> colors;
            colors.push_back(SHSV::FromRGB(DEFAULT_COLOR));
            colors.push_back(SHSV::FromRGB({0x4F, 0x31, 0x91}));
            return CQC2SDisplayFactory::CreateColorTransition(std::move(colors), 0.005f, p_type);
        }
        if (p_type == "video")
            return CQC2SDisplayFactory::CreateVideoDisplay("", EVideoFormat::Rgb, 30, p_type);
#ifdef USE_GLSL
        if (p_type == "glsl")
            return CQC2SDisplayFactory::CreateGLSLDisplay("", 30, 1, p_type, nullptr, "", true);
#endif
        return nullptr;
    }

    void ShowDisplayInfo(CQC2SDisplay *p_pDisplay)
    {
        if (!p_pDisplay)
            return;

        if (auto pMulti = dynamic_cast<CMultiDisplay *>(p_pDisplay))
        {
            // ── Multi-display: tree of children ─────────────────
            for (size_t i = 0; i < pMulti->GetDisplayCount(); ++i)
            {
                CQC2SDisplay *pChild = pMulti->GetDisplay(i);
                bool open = ImGui::TreeNode(pChild->GetName().c_str());
                if (open)
                {
                    ImGui::Text("Next: %s", pChild->GetNextDisplay().empty()
                                                ? "(stop)"
                                                : pChild->GetNextDisplay().c_str());
                    ShowDisplayOptions(pChild);
                    ImGui::TreePop();
                }
            }
        }
        else
        {
            // ── Single display ──────────────────────────────────
            ImGui::Text("Name: %s", p_pDisplay->GetName().c_str());
            if (!p_pDisplay->GetNextDisplay().empty())
                ImGui::Text("Next: %s", p_pDisplay->GetNextDisplay().c_str());
            ImGui::Separator();
            ShowDisplayOptions(p_pDisplay);
        }
    }

    String BuildCommandLine() const
    {
        StringStream ss;
        ss << "qc2srgb";

        if (m_config.m_verbose)
            ss << " --verbose";
        if (m_config.m_noWaitForRead)
            ss << " --no-wait-for-read";
        if (m_config.m_allowedSerials.has_value())
            for (const auto &s : *m_config.m_allowedSerials)
                ss << " --serial " << String(s.begin(), s.end());

        if (m_config.m_enableAudio)
        {
            ss << " --capture-audio";
            if (m_config.m_inputGain != 50.0f)
                ss << " --input-gain " << m_config.m_inputGain;
            if (!m_config.m_audioSmoothing)
                ss << " --no-audio-smoothing";
            if (m_config.m_audioSmoothingAlpha != 0.15f)
                ss << " --audio-smoothing-alpha " << m_config.m_audioSmoothingAlpha;
            if (m_config.m_audioDeviceId.has_value())
                ss << " --audio-device-id " << *m_config.m_audioDeviceId;
            if (m_config.m_audioChannel.has_value() && *m_config.m_audioChannel != 0)
                ss << " --audio-channel " << *m_config.m_audioChannel;
        }

        ss << BuildDisplayArgs(m_config.m_pDisplay.get());
        return ss.str();
    }

    String BuildDisplayArgs(CQC2SDisplay *p_pDisplay) const
    {
        if (!p_pDisplay || dynamic_cast<CMultiDisplay *>(p_pDisplay))
            return "";

        StringStream ss;

        if (auto pSolid = dynamic_cast<CSolidColorDisplay *>(p_pDisplay))
            ss << " --display solid --color " << ColorToHex(pSolid->GetColor());
        else if (auto pPulse = dynamic_cast<CPulseColorDisplay *>(p_pDisplay))
            ss << " --display pulse --color " << ColorToHex(pPulse->GetColor())
               << " --pulse-speed " << pPulse->GetSpeed()
               << " --pulse-cubic-bezier "
               << pPulse->GetBezier().m_p1x << " " << pPulse->GetBezier().m_p1y << " "
               << pPulse->GetBezier().m_p2x << " " << pPulse->GetBezier().m_p2y;
        else if (auto pRainbow = dynamic_cast<CRainbowDisplay *>(p_pDisplay))
        {
            const char *pMode = "flat";
            switch (pRainbow->GetMode())
            {
            case ERainbowMode::RollingVertical:
                pMode = "vertical";
                break;
            case ERainbowMode::RollingHorizontal:
                pMode = "horizontal";
                break;
            case ERainbowMode::RollingDiagonal:
                pMode = "diagonal";
                break;
            default:
                break;
            }
            ss << " --display rainbow --rainbow-mode " << pMode
               << " --rainbow-speed " << pRainbow->GetSpeed();
        }
        else if (auto pTransition = dynamic_cast<CColorTransitionDisplay *>(p_pDisplay))
        {
            ss << " --display transition --transition-colors ";
            const auto &colors = pTransition->GetColors();
            for (size_t i = 0; i < colors.size(); ++i)
            {
                if (i)
                    ss << ",";
                ss << ColorToHex(colors[i].ToRGB());
            }
            ss << " --transition-speed " << pTransition->GetSpeed()
               << " --transition-cubic-bezier "
               << pTransition->GetBezier().m_p1x << " " << pTransition->GetBezier().m_p1y << " "
               << pTransition->GetBezier().m_p2x << " " << pTransition->GetBezier().m_p2y;
        }
        else if (auto pVideo = dynamic_cast<CVideoDisplay *>(p_pDisplay))
            ss << " --display video --video-path " << pVideo->GetVideoPath()
               << " --video-framerate " << pVideo->GetFPS();
#ifdef USE_GLSL
        else if (auto pGLSL = dynamic_cast<CGLSLDisplay *>(p_pDisplay))
            ss << " --display glsl --shader-path " << pGLSL->GetShaderPath()
               << " --shader-fps " << pGLSL->GetFPS()
               << " --shader-scale " << pGLSL->GetScale();
#endif
        return ss.str();
    }

    static String ColorToHex(SRGBColor p_color)
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X%02X%02X",
                 p_color.m_red, p_color.m_green, p_color.m_blue);
        return buf;
    }

    void Run()
    {
        LOG(L"[CPreviewApp] Starting display with window preview...");

        do
        {
            m_restartRequested = false;

            /* There is a clear problem with callback-based rendering
                We solely rely on the displays ability to display. Sure, one issue is when setting fps
                to a low number the response time goes down... But the main problem here is that
                when the primary display's Display()/DisplayFrame() returns false, we practically exit the tool.
                Yes, this is mainly a preview tool, but the user might lose their configuration if something messes up.
                TODO/WIP!
            */
           // TODO: Remember that when multi-display is implemented, final exit condition (none/nonexistent display)
           //       should be replaced with the initial display!
            auto callback = [&](CIRenderer &)
            {
                m_renderer.NewFrame();

                // ── Docking layout ───────────────────────────────────
                ImGui::DockSpaceOverViewport();

                ImGui::Begin("LED Grid");
                ImVec2 avail = ImGui::GetContentRegionAvail();
                ImGui::Image(m_renderer.GetGridTexture(), avail,
                             ImVec2(0, 1), ImVec2(1, 0)); // flip Y
                ImGui::End();

                ImGui::Begin("Display");
                bool isMulti = dynamic_cast<CMultiDisplay *>(m_config.m_pDisplay.get()) != nullptr;
                if (!isMulti)
                {
                    String currentType = GetDisplayTypeName(m_config.m_pDisplay.get());
                    const char *types[] = {"solid", "pulse", "rainbow", "transition", "video"
#ifdef USE_GLSL
                                           ,
                                           "glsl"
#endif
                    };
                    int typeIdx = -1;
                    for (int i = 0; i < IM_ARRAYSIZE(types); ++i)
                        if (currentType == types[i])
                        {
                            typeIdx = i;
                            break;
                        }
                    if (ImGui::Combo("Type", &typeIdx, types, IM_ARRAYSIZE(types)))
                    {
                        m_pendingType = types[typeIdx];
                        m_restartRequested = true;
                        g_signalStopRequest = true;
                    }
                }
                ShowDisplayInfo(m_config.m_pDisplay.get());
                ImGui::End();

                // ── Auto-reload GLSL shader when file changes ─────
#ifdef USE_GLSL
                if (m_config.m_glslAutoReload)
                {
                    if (auto pGLSL = dynamic_cast<CGLSLDisplay *>(m_config.m_pDisplay.get()))
                    {
                        String shaderPath = pGLSL->GetShaderPath();
                        if (!shaderPath.empty())
                        {
                            std::error_code ec;
                            auto mtime = std::filesystem::last_write_time(shaderPath, ec);
                            if (!ec)
                            {
                                if (mtime != m_cachedShaderMtime)
                                {
                                    m_cachedShaderMtime = mtime;
                                    pGLSL->Initialize();
                                }
                            }
                        }
                    }
                }
                else
                {
                    // Reset cache when auto-reload is off so it re-triggers when turned on
                    m_cachedShaderMtime = std::filesystem::file_time_type::min();
                }
#endif

                ImGui::Begin("Info");
                ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
                if(m_config.m_enableAudio && m_audioProcessor.IsInitialized())
                {
                    auto spectrum = m_audioProcessor.GetSpectrum();
                    ImGui::Text("Max Audio Band Volume: %.1f",spectrum.m_maxBand);
                }
#ifdef USE_GLSL
                if (auto pGLSL = dynamic_cast<CGLSLDisplay *>(m_config.m_pDisplay.get()))
                {
                    uint32_t scale = pGLSL->GetScale();
                    ImGui::Text("GL Render Viewport: %u x %u (%u pixel)", g_VIDEO_WIDTH * scale, g_VIDEO_HEIGHT * scale,g_VIDEO_WIDTH * scale * g_VIDEO_HEIGHT * scale);
                }
#endif
                ImGui::End();

                ImGui::Begin("Command Line");

                if (isMulti)
                {
                    // if its multi dont display anything meaningful
                    String cmd = "---";
                    m_cmdBuffer.assign(cmd.begin(), cmd.end());
                    m_cmdBuffer.push_back('\0');
                }
                else
                {
                    String cmd = BuildCommandLine();
                    m_cmdBuffer.assign(cmd.begin(), cmd.end());
                    m_cmdBuffer.push_back('\0');
                }

                // stretch input text field to grow with window size, subtract its size by round about the button on the same row
                const float BUTTON_W = ImGui::CalcTextSize("Copy").x + ImGui::GetStyle().FramePadding.x * 2;
                const float GOAL_INPUT_WIDTH = ImGui::GetContentRegionAvail().x - BUTTON_W - ImGui::GetStyle().ItemSpacing.x;
                ImGui::SetNextItemWidth(GOAL_INPUT_WIDTH);
                ImGui::InputText("##cmd", m_cmdBuffer.data(), m_cmdBuffer.size(),
                                 ImGuiInputTextFlags_ReadOnly);

                // copy button, disable if multi display
                ImGui::SameLine();
                if (isMulti)
                    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                if (ImGui::Button("Copy"))
                {
                    String cmd(m_cmdBuffer.begin(), m_cmdBuffer.end());
                    ImGui::SetClipboardText(cmd.c_str());
                }
                ImGui::End();

                ImGui::Begin("General");
                ImGui::Checkbox("Verbose logging", &m_config.m_verbose);
                ImGui::Checkbox("Skip device response", &m_config.m_noWaitForRead);
                ImGui::SetItemTooltip("This enables potential higher FPS, but risks overloading the device with requests.\nDon't use this unless you really need to (probably not). The preview does not simulate USB transactions and thus cannot be tested here!\nUSE AT YOUR OWN RISK!");
                if (m_config.m_allowedSerials.has_value() && !m_config.m_allowedSerials->empty())
                {
                    ImGui::Text("Allowed serials:");
                    for (const auto &serial : *m_config.m_allowedSerials)
                    {
                        String narrow(serial.begin(), serial.end());
                        ImGui::Text("  %s", narrow.c_str());
                    }
                }
                else
                {
                    ImGui::Text("Allowed serials: all");
                }

                ImGui::SeparatorText("Audio");
                if (ImGui::Checkbox("Capture Audio", &m_config.m_enableAudio))
                {
                    m_audioDevicesCached = false; // refresh device list on toggle
                    auto &audioOn = m_config.m_enableAudio;
                    // audio processor is off -> turn on
                    if (audioOn)
                    {
                        if (m_audioProcessor.IsInitialized())
                            LOG("[CPreviewApp] WARNING: Audio is already on");
                        else
                        {
                            if (!m_audioProcessor.Initialize(1024, m_config.m_audioDeviceId, m_config.m_audioChannel))
                            {
                                LOG("[CPreviewApp] Failed to initialize audio processor.");
                            }
                            else
                            {
                                m_audioProcessor.SetInputGain(m_config.m_inputGain);
                                m_audioProcessor.SetSmoothing(m_config.m_audioSmoothing, m_config.m_audioSmoothingAlpha);
                                m_config.m_pDisplay->SetAudioProcessor(&m_audioProcessor);
                            }
                        }
                    }
                    else
                    {
                        if (!m_audioProcessor.IsInitialized())
                            LOG("[CPreviewApp] WARNING: Audio is NOT initialized");
                        else
                        {
                            m_audioProcessor.Shutdown();
                            m_config.m_pDisplay->SetAudioProcessor(nullptr);
                        }
                    }
                }
                ImGui::BeginDisabled(!m_config.m_enableAudio);
                if (ImGui::Checkbox("Smoothing", &m_config.m_audioSmoothing))
                {
                    if (m_audioProcessor.IsInitialized())
                        m_audioProcessor.SetSmoothing(m_config.m_audioSmoothing, m_config.m_audioSmoothingAlpha);
                }
                ImGui::SetItemTooltip("Smooth the volume bands by using EMA");
                if (ImGui::SliderFloat("Smoothing Alpha", &m_config.m_audioSmoothingAlpha, 0.0f, 1.0f, "%.3f"))
                {
                    if (m_audioProcessor.IsInitialized())
                        m_audioProcessor.SetSmoothing(m_config.m_audioSmoothing, m_config.m_audioSmoothingAlpha);
                }
                ImGui::SetItemTooltip("The smoothing factor of the bands. The smaller the more gradual the transition.");
                if (ImGui::SliderFloat("Input Gain", &m_config.m_inputGain, 0.0f, 100.0f, "%.1f"))
                {
                    if (m_audioProcessor.IsInitialized())
                        m_audioProcessor.SetInputGain(m_config.m_inputGain);
                }

                // ── Refresh device cache if dirty ─────────────────
                if (!m_audioDevicesCached)
                {
                    auto audioDevices = CAudioProcessor::GetDevices();
                    m_audioDevices.clear();
                    std::copy_if(
                        audioDevices.begin(),
                        audioDevices.end(),
                        std::back_inserter(m_audioDevices),
                        [](const SAudioDeviceInfo &p_device)
                        { return p_device.m_maxInputChannels > 0; });

                    // Build combo labels: [0] = "Default (auto)", [1..N] = devices
                    m_audioDeviceNames.clear();
                    m_audioDeviceNames.reserve(m_audioDevices.size() + 1);
                    m_audioDeviceNames.push_back("Default (auto)");
                    for (const auto &d : m_audioDevices)
                    {
                        StringStream ss;
                        ss << d.m_deviceId << ": " << d.m_name << " (" << d.m_hostApiName << ")";
                        m_audioDeviceNames.push_back(ss.str());
                    }

                    m_audioDevicesCached = true;
                }

                // ── Device combo ────────────────────────────────────
                {
                    // Build a C-string array from cached labels (cheap, no formatting)
                    DynamicContainer<const char *> items;
                    items.reserve(m_audioDeviceNames.size());
                    for (const auto &n : m_audioDeviceNames)
                        items.push_back(n.c_str());

                    // Determine current selection index
                    int selIdx = 0;
                    if (m_config.m_audioDeviceId.has_value())
                    {
                        int target = *m_config.m_audioDeviceId;
                        for (size_t i = 0; i < m_audioDevices.size(); ++i)
                        {
                            if (m_audioDevices[i].m_deviceId == target)
                            {
                                selIdx = static_cast<int>(i + 1);
                                break;
                            }
                        }
                    }

                    const float DEVICE_W = ImGui::CalcTextSize("Device").x;

                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - DEVICE_W - ImGui::GetStyle().ItemSpacing.x);
                    if (ImGui::Combo("Device", &selIdx, items.data(), static_cast<int>(items.size())))
                    {
                        Option<int> newDeviceId;
                        if (selIdx > 0)
                            newDeviceId = m_audioDevices[static_cast<size_t>(selIdx - 1)].m_deviceId;

                        if (newDeviceId != m_config.m_audioDeviceId)
                        {
                            // Shutdown audio if running, update device id, restart
                            bool wasInitialized = m_audioProcessor.IsInitialized();
                            if (wasInitialized)
                            {
                                m_audioProcessor.Shutdown();
                                m_config.m_pDisplay->SetAudioProcessor(nullptr);
                            }

                            m_config.m_audioDeviceId = newDeviceId;
                            m_config.m_audioChannel.reset(); // reset channel to 0 on device change

                            if (wasInitialized)
                            {
                                if (!m_audioProcessor.Initialize(1024, m_config.m_audioDeviceId, m_config.m_audioChannel))
                                {
                                    LOG("[CPreviewApp] Failed to re-initialize audio processor after device change.");
                                }
                                else
                                {
                                    m_audioProcessor.SetInputGain(m_config.m_inputGain);
                                    m_audioProcessor.SetSmoothing(m_config.m_audioSmoothing, m_config.m_audioSmoothingAlpha);
                                    m_config.m_pDisplay->SetAudioProcessor(&m_audioProcessor);
                                }
                            }
                        }
                    }
                    ImGui::SetItemTooltip("Only shows the devices with capturable input channels.");
                }
                // ── Channel number ───────────────────────────────────
                {
                    // Determine max input channels for the currently selected device
                    int maxChannels = 0;
                    if (m_config.m_audioDeviceId.has_value())
                    {
                        int target = *m_config.m_audioDeviceId;
                        for (const auto &d : m_audioDevices)
                        {
                            if (d.m_deviceId == target)
                            {
                                maxChannels = d.m_maxInputChannels;
                                break;
                            }
                        }
                    }
                    // At minimum offer channel 0 (mono default)
                    int numChannels = maxChannels > 0 ? maxChannels : 1;

                    // Build channel labels
                    DynamicContainer<String> channelLabels;
                    channelLabels.reserve(static_cast<size_t>(numChannels));
                    for (int i = 0; i < numChannels; ++i)
                    {
                        char buf[16];
                        snprintf(buf, sizeof(buf), "%d", i);
                        channelLabels.push_back(buf);
                    }
                    DynamicContainer<const char *> channelItems;
                    channelItems.reserve(channelLabels.size());
                    for (const auto &l : channelLabels)
                        channelItems.push_back(l.c_str());

                    int channel = m_config.m_audioChannel.value_or(0);
                    if (ImGui::Combo("Channel", &channel, channelItems.data(), static_cast<int>(channelItems.size())))
                    {
                        channel = std::clamp(channel, 0, maxChannels > 0 ? maxChannels - 1 : 0);
                        if (channel != m_config.m_audioChannel.value_or(0))
                        {
                            bool wasInitialized = m_audioProcessor.IsInitialized();
                            if (wasInitialized)
                            {
                                m_audioProcessor.Shutdown();
                                m_config.m_pDisplay->SetAudioProcessor(nullptr);
                            }

                            if (channel == 0)
                                m_config.m_audioChannel.reset();
                            else
                                m_config.m_audioChannel = channel;

                            if (wasInitialized)
                            {
                                if (!m_audioProcessor.Initialize(1024, m_config.m_audioDeviceId, m_config.m_audioChannel))
                                {
                                    LOG("[CPreviewApp] Failed to re-initialize audio processor after channel change.");
                                }
                                else
                                {
                                    m_audioProcessor.SetInputGain(m_config.m_inputGain);
                                    m_audioProcessor.SetSmoothing(m_config.m_audioSmoothing, m_config.m_audioSmoothingAlpha);
                                    m_config.m_pDisplay->SetAudioProcessor(&m_audioProcessor);
                                }
                            }
                        }
                    }
                }
                ImGui::EndDisabled();
                ImGui::End();

                auto continueDisplaying = m_renderer.PollEvents();
                if (!continueDisplaying)
                    g_signalStopRequest = true;

                m_renderer.Present();
                return continueDisplaying;
            };

            m_config.m_pDisplay->Display(m_renderer, g_signalStopRequest, std::move(callback));

            if (m_restartRequested && !m_pendingType.empty())
            {
                m_config.m_pDisplay->Shutdown(m_renderer);
                auto pNewDisplay = CreateDefaultDisplay(m_pendingType);
                if (pNewDisplay)
                {
                    m_config.m_pDisplay = std::move(pNewDisplay);
                    if (m_config.m_enableAudio)
                        m_config.m_pDisplay->SetAudioProcessor(&m_audioProcessor);
                    if (!m_config.m_pDisplay->Initialize())
                    {
                        LOG_ERROR(L"Failed to initialize new display: " + WStr(m_pendingType));
                        // fallback to solid
                        m_config.m_pDisplay = CQC2SDisplayFactory::CreateSolidColor({0x29, 0x00, 0x66}, "solid");
                        m_config.m_pDisplay->Initialize();
                    }
                    g_signalStopRequest = false;
                }
            }
        } while (m_restartRequested && !g_signalStopRequest.load());
    }

    void Shutdown()
    {
        m_audioProcessor.Shutdown();
        m_config.m_pDisplay->Shutdown(m_renderer);
    }
};
