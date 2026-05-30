// Copyright (c) 2026 Mario T
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#ifdef USE_GLSL

#include "CQC2SDisplay.h"
#include "DisplayUtils.h"
#include "../video/VideoConstants.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <thread>

class CGLSLDisplay : public CQC2SDisplay
{
    String   m_shaderPath;
    uint32_t m_fps;
    uint32_t m_resolutionScale = 1;

    // EGL handles
    EGLDisplay m_eglDisplay = EGL_NO_DISPLAY;
    EGLSurface m_eglSurface = EGL_NO_SURFACE;
    EGLContext m_eglContext = EGL_NO_CONTEXT;

    // GL fb/va object handles
    GLuint m_program    = 0;
    GLuint m_fbo        = 0;
    GLuint m_fboTexture = 0;
    GLuint m_vao        = 0; // empty VAO; positions come from gl_VertexID in vertex shader

    // Uniform locations, -1 meaning not present in shader
    GLint m_uTime       = -1;
    GLint m_uResolution = -1;
    GLint m_uFrame      = -1;

    int m_frameCount = 0;
    std::chrono::steady_clock::time_point m_startTime;

    // Compile a shader and log any errors.  Returns true on success.
    static bool CompileShader(GLuint p_shader, const char *p_pSource)
    {
        glShaderSource(p_shader, 1, &p_pSource, nullptr);
        glCompileShader(p_shader);

        GLint ok = GL_FALSE;
        glGetShaderiv(p_shader, GL_COMPILE_STATUS, &ok);
        if (!ok)
        {
            GLint len = 0;
            glGetShaderiv(p_shader, GL_INFO_LOG_LENGTH, &len);
            DynamicContainer<GLchar> log(static_cast<size_t>(len));
            glGetShaderInfoLog(p_shader, len, nullptr, log.data());
            LOG_ERROR(L"CGLSLDisplay: shader compile error:\n" << WStr(log.data()));
        }
        return ok == GL_TRUE;
    }

    // Delete all GL objects.  Safe to call multiple times or before Initialize.
    // Must be called while the EGL context is still current.
    void CleanupGL()
    {
        if (m_vao)        { glDeleteVertexArrays(1, &m_vao);        m_vao        = 0; }
        if (m_fboTexture) { glDeleteTextures(1,     &m_fboTexture); m_fboTexture = 0; }
        if (m_fbo)        { glDeleteFramebuffers(1, &m_fbo);        m_fbo        = 0; }
        if (m_program)    { glDeleteProgram(m_program);             m_program    = 0; }
    }

    // Unbind and release all EGL resources.  Safe to call multiple times.
    void CleanupEGL()
    {
        if (m_eglDisplay == EGL_NO_DISPLAY)
            return;

        eglMakeCurrent(m_eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

        if (m_eglContext != EGL_NO_CONTEXT)
        {
            eglDestroyContext(m_eglDisplay, m_eglContext);
            m_eglContext = EGL_NO_CONTEXT;
        }
        if (m_eglSurface != EGL_NO_SURFACE)
        {
            eglDestroySurface(m_eglDisplay, m_eglSurface);
            m_eglSurface = EGL_NO_SURFACE;
        }

        eglTerminate(m_eglDisplay);
        m_eglDisplay = EGL_NO_DISPLAY;
    }

public:
    CGLSLDisplay(String p_shaderPath, uint32_t p_fps, uint32_t p_resolutionScale, String p_name,
                 UniquePtr<CEndCondition> p_pEndCondition, String p_nextDisplay = "")
        : CQC2SDisplay(std::move(p_name), std::move(p_pEndCondition), std::move(p_nextDisplay)),
          m_shaderPath(std::move(p_shaderPath)), m_fps(p_fps),
          m_resolutionScale(std::max(1u, p_resolutionScale))
    {}

    CGLSLDisplay(const CGLSLDisplay &) = delete;
    CGLSLDisplay &operator=(const CGLSLDisplay &) = delete;

    // Ensure cleanup even when Shutdown() is never called (e.g. on early abort).
    ~CGLSLDisplay() override
    {
        if (m_eglDisplay != EGL_NO_DISPLAY && m_eglContext != EGL_NO_CONTEXT
            && m_eglSurface != EGL_NO_SURFACE)
            eglMakeCurrent(m_eglDisplay, m_eglSurface, m_eglSurface, m_eglContext);
        CleanupGL();
        CleanupEGL();
    }

    

    bool Initialize() override
    {
        // EGL init display
        m_eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (m_eglDisplay == EGL_NO_DISPLAY)
        {
            LOG_ERROR(L"CGLSLDisplay: eglGetDisplay failed");
            return false;
        }

        if (!eglInitialize(m_eglDisplay, nullptr, nullptr))
        {
            LOG_ERROR(L"CGLSLDisplay: eglInitialize failed");
            CleanupEGL();
            return false;
        }

        if (!eglBindAPI(EGL_OPENGL_ES_API))
        {
            LOG_ERROR(L"CGLSLDisplay: eglBindAPI(EGL_OPENGL_ES_API) failed");
            CleanupEGL();
            return false;
        }

        // EGL_OPENGL_ES3_BIT is defined in <EGL/eglext.h> for EGL < 1.5;
        // it is available in core EGL 1.5.
        const EGLint CONFIG_ATTRIBS[] = {
            EGL_SURFACE_TYPE,    EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
            EGL_RED_SIZE,        8,
            EGL_GREEN_SIZE,      8,
            EGL_BLUE_SIZE,       8,
            EGL_NONE
        };

        EGLConfig pConfig{};
        EGLint    numConfigs = 0;
        if (!eglChooseConfig(m_eglDisplay, CONFIG_ATTRIBS, &pConfig, 1, &numConfigs)
            || numConfigs < 1)
        {
            LOG_ERROR(L"CGLSLDisplay: no suitable EGL config found");
            CleanupEGL();
            return false;
        }

        // The surface is only required to make the context current.  All actual
        // rendering targets the 12x9 FBO created below.
        const EGLint PBUFFER_ATTRIBS[] = {
            EGL_WIDTH,  1,
            EGL_HEIGHT, 1,
            EGL_NONE
        };
        m_eglSurface = eglCreatePbufferSurface(m_eglDisplay, pConfig, PBUFFER_ATTRIBS);
        if (m_eglSurface == EGL_NO_SURFACE)
        {
            LOG_ERROR(L"CGLSLDisplay: eglCreatePbufferSurface failed");
            CleanupEGL();
            return false;
        }

        // eg 3 context creation
        const EGLint CONTEXT_ATTRIBS[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3,
            EGL_NONE
        };
        m_eglContext = eglCreateContext(m_eglDisplay, pConfig, EGL_NO_CONTEXT, CONTEXT_ATTRIBS);
        if (m_eglContext == EGL_NO_CONTEXT)
        {
            LOG_ERROR(L"CGLSLDisplay: eglCreateContext failed");
            CleanupEGL();
            return false;
        }

        // make context current to set up GL objects, later we will release it again to
        if (!eglMakeCurrent(m_eglDisplay, m_eglSurface, m_eglSurface, m_eglContext))
        {
            LOG_ERROR(L"CGLSLDisplay: eglMakeCurrent failed");
            CleanupEGL();
            return false;
        }

        // Load fragment shader
        IFStream shaderFile(m_shaderPath);
        if (!shaderFile.is_open())
        {
            LOG_ERROR(L"CGLSLDisplay: cannot open shader file: " << WStr(m_shaderPath));
            CleanupEGL();
            return false;
        }
        StringStream ss;
        ss << shaderFile.rdbuf();
        const String FRAG_SOURCE = ss.str();

        // Compile shader (with a simple default vertex shader) and link program
        // Vertex shader: full-screen triangle via gl_VertexID — no vertex buffer needed.
        const char *pVertexSource =
            "#version 300 es\n"
            "void main() {\n"
            "    const vec2 pos[3] = vec2[3](\n"
            "        vec2(-1.0, -1.0),\n"
            "        vec2( 3.0, -1.0),\n"
            "        vec2(-1.0,  3.0)\n"
            "    );\n"
            "    gl_Position = vec4(pos[gl_VertexID], 0.0, 1.0);\n"
            "}\n";

        // create shaders & exit on compile or link error
        const GLuint VERT_SHADER = glCreateShader(GL_VERTEX_SHADER);
        const GLuint FRAG_SHADER = glCreateShader(GL_FRAGMENT_SHADER);

        const char *pFragSrc = FRAG_SOURCE.c_str();
        const bool VERT_OK   = CompileShader(VERT_SHADER, pVertexSource);
        const bool FRAG_OK   = CompileShader(FRAG_SHADER, pFragSrc);

        if (!VERT_OK || !FRAG_OK)
        {
            glDeleteShader(VERT_SHADER);
            glDeleteShader(FRAG_SHADER);
            CleanupEGL();
            return false;
        }

        m_program = glCreateProgram();
        glAttachShader(m_program, VERT_SHADER);
        glAttachShader(m_program, FRAG_SHADER);
        glLinkProgram(m_program);

        // Shaders are no longer needed once linked -> detach before deleting.
        glDetachShader(m_program, VERT_SHADER);
        glDetachShader(m_program, FRAG_SHADER);
        glDeleteShader(VERT_SHADER);
        glDeleteShader(FRAG_SHADER);

        GLint linkOk = GL_FALSE;
        glGetProgramiv(m_program, GL_LINK_STATUS, &linkOk);
        if (!linkOk)
        {
            GLint len = 0;
            glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &len);
            DynamicContainer<char> log(static_cast<size_t>(len));
            glGetProgramInfoLog(m_program, len, nullptr, log.data());
            LOG_ERROR(L"CGLSLDisplay: shader link error:\n" << WStr(log.data()));
            CleanupGL();
            CleanupEGL();
            return false;
        }
        
        // support not just shadertoy convention but also
        // webgl / generic u_time, u_resolution, u_frame naming
        // whichever comes first in the linked program wins
        auto resolveUniform = [&](std::initializer_list<const char *> p_names) -> GLint {
            for (const char *pName : p_names)
            {
                const GLint LOC = glGetUniformLocation(m_program, pName);
                if (LOC != -1)
                    return LOC;
            }
            return -1;
        };

        m_uTime       = resolveUniform({"iTime",       "u_time",       "time"});
        m_uResolution = resolveUniform({"iResolution", "u_resolution", "resolution"});
        m_uFrame      = resolveUniform({"iFrame",      "u_frame",      "frame"});

        // Set up FBO 
        glGenTextures(1, &m_fboTexture);
        glBindTexture(GL_TEXTURE_2D, m_fboTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
                     static_cast<GLsizei>(g_VIDEO_WIDTH  * m_resolutionScale),
                     static_cast<GLsizei>(g_VIDEO_HEIGHT * m_resolutionScale),
                     0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glBindTexture(GL_TEXTURE_2D, 0);

        glGenFramebuffers(1, &m_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, m_fboTexture, 0);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            LOG_ERROR(L"CGLSLDisplay: framebuffer incomplete");
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            CleanupGL();
            CleanupEGL();
            return false;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // es 3.0 requires non zero vao, even if we dont 
        glGenVertexArrays(1, &m_vao);

        // actual display related variables
        m_frameCount = 0;
        m_startTime  = std::chrono::steady_clock::now();

        // Initialize may be called from a different thread than the rendering logic in DisplayFrame (main(), to be precise)
        // we release the display egl rendering context here, 
        eglMakeCurrent(m_eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

        return true;
    }

    void Reset() override
    {
        CQC2SDisplay::Reset();
        m_frameCount = 0;
        m_startTime  = std::chrono::steady_clock::now();
        // while we could transfer the context here, reset is not just called in the display thread
    }

    // To render frames, the display thread must first acquire the EGL context in Display()
    // before calling DisplayFrame().  We override the original Display() function to ensure that the acquisition happens in the display thread
    void Display(CQuadcast2SCommunicator &p_communicator,
                 const AtomicBool        &p_signalStopRequest,
                 FrameCallback            p_frameCallback = nullptr) override
    {
        if (!eglMakeCurrent(m_eglDisplay, m_eglSurface, m_eglSurface, m_eglContext))
        {
            LOG_ERROR(L"CGLSLDisplay: eglMakeCurrent failed in display thread");
            return;
        }
        CQC2SDisplay::Display(p_communicator, p_signalStopRequest, std::move(p_frameCallback));
        // Release context from this thread so Shutdown() can re-acquire it for cleanup.
        eglMakeCurrent(m_eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }

    bool DisplayFrame(CQuadcast2SCommunicator &p_communicator) override
    {
        const auto FRAME_START    = std::chrono::steady_clock::now();
        const auto FRAME_DURATION = std::chrono::milliseconds(1000 / m_fps);

        // update uniforms
        glUseProgram(m_program);

        if (m_uTime != -1)
        {
            const float ELAPSED = std::chrono::duration<float>(FRAME_START - m_startTime).count();
            glUniform1f(m_uTime, ELAPSED);
        }
        if (m_uResolution != -1)
            glUniform2f(m_uResolution,
                        static_cast<float>(g_VIDEO_WIDTH  * m_resolutionScale),
                        static_cast<float>(g_VIDEO_HEIGHT * m_resolutionScale));
        if (m_uFrame != -1)
            glUniform1i(m_uFrame, m_frameCount);

        // render to our FBO
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glViewport(0, 0,
                   static_cast<GLsizei>(g_VIDEO_WIDTH  * m_resolutionScale),
                   static_cast<GLsizei>(g_VIDEO_HEIGHT * m_resolutionScale));
                   
        // WIP: some issues caused the diodes to be flashing white at 60 fps
        // though I believe this is mostly because we were sending
        // frames faster than the display could handle, causing it to fail processing

        //glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        //glClear(GL_COLOR_BUFFER_BIT);

        glBindVertexArray(m_vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        // now, read back the rendered frame
        // glReadPixels stores rows bottom-to-top (row 0 = GL bottom).  When
        // m_resolutionScale > 1, each logical LED maps to a scale² block
        // in the high-res buffer; we average that block down to one color
        const uint32_t TOTAL_WIDTH  = g_VIDEO_WIDTH  * m_resolutionScale;
        const uint32_t TOTAL_HEIGHT = g_VIDEO_HEIGHT * m_resolutionScale;
        const uint32_t BLOCK        = m_resolutionScale * m_resolutionScale;

        DynamicContainer<uint8_t> pixels(static_cast<size_t>(TOTAL_WIDTH * TOTAL_HEIGHT * 4u));
        glReadPixels(0, 0,
                     static_cast<GLsizei>(TOTAL_WIDTH),
                     static_cast<GLsizei>(TOTAL_HEIGHT),
                     GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // downscale the frame to 12x9 by averaging blocks based on the scale
        StaticArray<SRGBColor, g_LED_COUNT> frame{};
        for (uint32_t col = 0; col < g_VIDEO_WIDTH; ++col)
        {
            for (uint32_t row = 0; row < g_VIDEO_HEIGHT; ++row)
            {
                uint32_t rSum = 0;
                uint32_t gSum = 0;
                uint32_t bSum = 0;

                // GL bottom-up row base for display row (0 = top)
                const uint32_t GL_ROW_BASE = (g_VIDEO_HEIGHT - 1u - row) * m_resolutionScale;
                const uint32_t COL_BASE    = col * m_resolutionScale;

                for (uint32_t dy = 0; dy < m_resolutionScale; ++dy)
                {
                    for (uint32_t dx = 0; dx < m_resolutionScale; ++dx)
                    {
                        const size_t SRC = ((GL_ROW_BASE + dy) * TOTAL_WIDTH + COL_BASE + dx) * 4u;
                        rSum += pixels[SRC];
                        gSum += pixels[SRC + 1u];
                        bSum += pixels[SRC + 2u];
                    }
                }

                // display index mapping: odd cols are top-to-bottom, even cols are bottom-to-top
                const uint32_t PHYS_ROW = (col % 2u == 1u) ? row : (g_VIDEO_HEIGHT - 1u - row);
                const uint32_t DST_IDX  = col * g_VIDEO_HEIGHT + PHYS_ROW;

                frame[DST_IDX] = {
                    static_cast<uint8_t>(rSum / BLOCK),
                    static_cast<uint8_t>(gSum / BLOCK),
                    static_cast<uint8_t>(bSum / BLOCK)
                };
            }
        }

        const auto ELAPSED_CALCULATION = std::chrono::steady_clock::now() - FRAME_START;
        // WIP stuff due to high fps... calculate average color 
        //auto totalR = 0u, totalG = 0u, totalB = 0u;
        //for (const auto &color : frame)        {
        //    totalR += color.m_red;
        //    totalG += color.m_green;
        //    totalB += color.m_blue;
        //}
        //auto avgR = totalR / g_LED_COUNT;
        //auto avgG = totalG / g_LED_COUNT;
        //auto avgB = totalB / g_LED_COUNT;
        //LOG_VERBOSE(L"CGLSLDisplay: avg color: (" << avgR << ", " << avgG << ", " << avgB << ")");
        SendColorFrame(p_communicator, frame.data());

        // next frame prep
        ++m_frameCount;

        const auto ELAPSED = std::chrono::steady_clock::now() - FRAME_START;
        //LOG_VERBOSE(L"CGLSLDisplay: frame " << m_frameCount << " time: "
        //            << std::chrono::duration_cast<std::chrono::milliseconds>(ELAPSED).count() << " ms "
        //            << "(calc " << std::chrono::duration_cast<std::chrono::milliseconds>(ELAPSED_CALCULATION).count() << " ms)"
        //            << " waiting " << std::chrono::duration_cast<std::chrono::milliseconds>(FRAME_DURATION - ELAPSED).count() << " ms"
        //        );
        if (ELAPSED < FRAME_DURATION)
            std::this_thread::sleep_for(FRAME_DURATION - ELAPSED);

        return true;
    }

    void Shutdown(CQuadcast2SCommunicator & /*p_communicator*/) override
    {
        // Re-acquire the context in the calling (main) thread for proper GL cleanup.
        if (m_eglDisplay != EGL_NO_DISPLAY && m_eglContext != EGL_NO_CONTEXT
            && m_eglSurface != EGL_NO_SURFACE)
            eglMakeCurrent(m_eglDisplay, m_eglSurface, m_eglSurface, m_eglContext);
        CleanupGL();
        CleanupEGL();
    }
};

#endif // USE_GLSL
