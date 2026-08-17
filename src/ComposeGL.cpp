#include "ComposeGL.hpp"
#include "Log.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <format>
#include <string_view>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

namespace hxc {

    namespace {

        // A single oversized triangle addressed by gl_VertexID: no attribute buffer,
        // no element buffer, no per-frame upload.
        constexpr const char* VERTEX_SOURCE = R"GLSL(#version 300 es
void main() {
    vec2 corner = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(corner * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

        // The inverse-warp kernel. This is a faithful translation of the CPU
        // functions in Math.cpp — fovRay/projectPinhole/fovProject — and the
        // end-to-end tests assert the two agree by comparing rendered marker
        // positions against CPU-computed predictions. GLSL ES has no fp64, so the
        // agreement is to a fraction of a pixel, not bit-exact.
        constexpr const char* FRAGMENT_SOURCE = R"GLSL(#version 300 es
precision highp float;
precision highp int;

uniform ivec2 uPaneOrigin;
uniform ivec2 uPaneSize;

uniform vec4  uOutFovTan;     // tan(l), tan(r), tan(u), tan(d)
uniform mat3  uOutRot;        // output camera local -> world
uniform vec3  uOutPos;

uniform int   uBgMode;        // 0 solid, 1 checker, 2 camera
uniform vec4  uSolid;
uniform mat3  uCamRotInv;     // world -> camera local
uniform vec3  uCamPos;
uniform vec4  uCamIntr;       // fx, fy, cx, cy
uniform vec3  uDistortionA;   // k1, k2, p1
uniform vec2  uDistortionB;   // p2, k3
uniform vec2  uCamSize;
uniform float uBgDepth;       // <= 0 means infinite

uniform int   uHasFg;
uniform mat3  uFgRotInv;
uniform vec3  uFgPos;
uniform vec4  uFgFovTan;
uniform float uFgDepth;
uniform int   uPremultiplied;

uniform sampler2D uBgTex;
uniform sampler2D uFgTex;

out vec4 fragColor;

// The image's row 0 is its top; GL's row 0 is the bottom. Everything below works
// in the CPU convention and readback() flips on the way out, which is the same
// arrangement the portal demo's offscreen path uses.
ivec2 panePixel() {
    ivec2 frag = ivec2(gl_FragCoord.xy) - uPaneOrigin;
    return ivec2(frag.x, uPaneSize.y - 1 - frag.y);
}

vec3 rayFromFov(vec4 tangents, ivec2 pixel, ivec2 size) {
    float u = (float(pixel.x) + 0.5) / float(size.x);
    float v = (float(pixel.y) + 0.5) / float(size.y);
    return vec3(tangents.x + u * (tangents.y - tangents.x),
                tangents.z + v * (tangents.w - tangents.z),
                -1.0);
}

// Where the output pixel's ray puts the content, in world space. With an
// infinite depth the caller uses the direction instead of a point, which is what
// the two branches below encode.
vec3 sourceLocal(mat3 rotInv, vec3 sourcePos, vec3 dirWorld, float depth) {
    if (depth > 0.0)
        return rotInv * ((uOutPos + normalize(dirWorld) * depth) - sourcePos);
    return rotInv * dirWorld;
}

vec4 sampleCamera(vec3 dirWorld) {
    vec3  local = sourceLocal(uCamRotInv, uCamPos, dirWorld, uBgDepth);
    float zcv   = -local.z;
    if (zcv <= 1e-6)
        return uSolid;

    float xn = local.x / zcv;
    float yn = -local.y / zcv;

    float k1 = uDistortionA.x, k2 = uDistortionA.y, p1 = uDistortionA.z;
    float p2 = uDistortionB.x, k3 = uDistortionB.y;

    float r2     = xn * xn + yn * yn;
    float radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    float xd     = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn);
    float yd     = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn;

    vec2 pixel = vec2(uCamIntr.x * xd + uCamIntr.z, uCamIntr.y * yd + uCamIntr.w);
    vec2 uv    = pixel / uCamSize;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0)
        return uSolid;

    return vec4(texture(uBgTex, uv).rgb, 1.0);
}

// A world-locked checker for takes with no camera source. It is drawn in
// direction space, so it shows the output camera's rotation honestly: with
// --framing stabilized the checker stops shaking, which is the whole point.
vec4 checkerBackground(vec3 dirWorld) {
    vec3  d     = normalize(dirWorld);
    float yaw   = atan(d.x, -d.z);
    float pitch = asin(clamp(d.y, -1.0, 1.0));

    const float CELL = 0.13089969; // 7.5 degrees
    int   cellX = int(floor(yaw / CELL + 1024.0));
    int   cellY = int(floor(pitch / CELL + 1024.0));
    bool  dark  = ((cellX + cellY) & 1) == 0;

    vec3 color = dark ? vec3(0.055, 0.070, 0.086) : vec3(0.105, 0.125, 0.145);
    // A horizon line and a forward mark give the eye something absolute to hold.
    if (abs(pitch) < 0.006)
        color = vec3(0.28, 0.34, 0.40);
    if (abs(yaw) < 0.006 && abs(pitch) < 0.25)
        color = vec3(0.34, 0.26, 0.22);
    return vec4(color, 1.0);
}

void main() {
    ivec2 pixel    = panePixel();
    vec3  dirWorld = uOutRot * rayFromFov(uOutFovTan, pixel, uPaneSize);

    vec4 background = uSolid;
    if (uBgMode == 1)
        background = checkerBackground(dirWorld);
    else if (uBgMode == 2)
        background = sampleCamera(dirWorld);

    vec4 overlay = vec4(0.0);
    if (uHasFg == 1) {
        vec3 local = sourceLocal(uFgRotInv, uFgPos, dirWorld, uFgDepth);
        if (local.z < 0.0) {
            float tx = local.x / -local.z;
            float ty = local.y / -local.z;
            float u  = (tx - uFgFovTan.x) / (uFgFovTan.y - uFgFovTan.x);
            float v  = (ty - uFgFovTan.z) / (uFgFovTan.w - uFgFovTan.z);
            if (u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0)
                overlay = texture(uFgTex, vec2(u, v));
        }
    }

    vec3 premultiplied = uPremultiplied == 1 ? overlay.rgb : overlay.rgb * overlay.a;
    fragColor          = vec4(background.rgb * (1.0 - overlay.a) + premultiplied, 1.0);
}
)GLSL";

        bool extensionListed(const char* list, std::string_view wanted) {
            if (!list)
                return false;
            std::string_view remaining = list;
            while (!remaining.empty()) {
                const size_t END   = remaining.find(' ');
                const auto   TOKEN = remaining.substr(0, END);
                if (TOKEN == wanted)
                    return true;
                if (END == std::string_view::npos)
                    return false;
                remaining = remaining.substr(END + 1);
            }
            return false;
        }

        std::string infoLog(GLuint object, bool program) {
            GLint length = 0;
            if (program)
                glGetProgramiv(object, GL_INFO_LOG_LENGTH, &length);
            else
                glGetShaderiv(object, GL_INFO_LOG_LENGTH, &length);

            std::vector<char> log(static_cast<size_t>(length > 0 ? length : 1), '\0');
            if (program)
                glGetProgramInfoLog(object, static_cast<GLsizei>(log.size()), nullptr, log.data());
            else
                glGetShaderInfoLog(object, static_cast<GLsizei>(log.size()), nullptr, log.data());
            return log.data();
        }

        GLuint compileStage(GLenum stage, const char* source, std::string& error) {
            const GLuint SHADER = glCreateShader(stage);
            if (SHADER == 0) {
                error = "glCreateShader failed";
                return 0;
            }
            glShaderSource(SHADER, 1, &source, nullptr);
            glCompileShader(SHADER);

            GLint compiled = GL_FALSE;
            glGetShaderiv(SHADER, GL_COMPILE_STATUS, &compiled);
            if (compiled == GL_TRUE)
                return SHADER;

            error = std::format("shader compilation failed: {}", infoLog(SHADER, false));
            glDeleteShader(SHADER);
            return 0;
        }

        GLuint linkProgram(std::string& error) {
            const GLuint VERTEX = compileStage(GL_VERTEX_SHADER, VERTEX_SOURCE, error);
            if (VERTEX == 0)
                return 0;
            const GLuint FRAGMENT = compileStage(GL_FRAGMENT_SHADER, FRAGMENT_SOURCE, error);
            if (FRAGMENT == 0) {
                glDeleteShader(VERTEX);
                return 0;
            }

            const GLuint PROGRAM = glCreateProgram();
            glAttachShader(PROGRAM, VERTEX);
            glAttachShader(PROGRAM, FRAGMENT);
            glLinkProgram(PROGRAM);
            glDeleteShader(VERTEX);
            glDeleteShader(FRAGMENT);

            GLint linked = GL_FALSE;
            glGetProgramiv(PROGRAM, GL_LINK_STATUS, &linked);
            if (linked == GL_TRUE)
                return PROGRAM;

            error = std::format("program link failed: {}", infoLog(PROGRAM, true));
            glDeleteProgram(PROGRAM);
            return 0;
        }

        std::string lowercase(std::string text) {
            std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return text;
        }

        std::array<float, 9> toFloat9(const std::array<double, 9>& source) {
            std::array<float, 9> out{};
            for (size_t i = 0; i < 9; ++i)
                out[i] = static_cast<float>(source[i]);
            return out;
        }

        // Column-major inverse of a rotation matrix is its transpose.
        std::array<float, 9> inverseRotation(const SQuat& rotation) {
            return toFloat9(rotation.inverse().toMat3ColumnMajor());
        }

    }

    struct CComposeGL::SState {
        EGLDisplay display = EGL_NO_DISPLAY;
        EGLContext context = EGL_NO_CONTEXT;
        EGLConfig  config  = nullptr;

        GLuint      program      = 0;
        GLuint      vertexArray  = 0;
        GLuint      framebuffer  = 0;
        GLuint      renderbuffer = 0;

        int         paneWidth  = 0;
        int         paneHeight = 0;
        int         paneCount  = 1;
        std::string description;

        struct STexture {
            GLuint id     = 0;
            int    width  = 0;
            int    height = 0;
        };
        std::vector<STexture> backgroundTextures;
        std::vector<STexture> overlayTextures;
        std::vector<uint8_t>  readbackScratch;

        bool                  makeCurrent() const {
            return eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context) == EGL_TRUE;
        }

        bool uploadTexture(std::vector<STexture>& slots, int pane, const uint8_t* rgba, int width, int height) {
            if (pane < 0 || pane >= static_cast<int>(slots.size()) || !rgba || width <= 0 || height <= 0)
                return false;
            if (!makeCurrent())
                return false;

            auto& slot = slots[static_cast<size_t>(pane)];
            if (slot.id == 0) {
                glGenTextures(1, &slot.id);
                glBindTexture(GL_TEXTURE_2D, slot.id);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            } else
                glBindTexture(GL_TEXTURE_2D, slot.id);

            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            // Row 0 of the upload becomes v = 0, so the texture's v axis runs top to
            // bottom in image order, matching the CPU pixel convention the shader
            // computes its uv in.
            if (slot.width != width || slot.height != height) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
                slot.width  = width;
                slot.height = height;
            } else
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba);

            return glGetError() == GL_NO_ERROR;
        }

        bool chooseConfigAndContext(std::string& error) {
            const std::array<EGLint, 13> ATTRIBUTES = {
                EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE,
            };
            EGLint count = 0;
            if (eglChooseConfig(display, ATTRIBUTES.data(), &config, 1, &count) != EGL_TRUE || count <= 0) {
                error = "no EGL config offers an 8-bit RGBA ES3 renderable surface";
                return false;
            }
            if (eglBindAPI(EGL_OPENGL_ES_API) != EGL_TRUE) {
                error = "eglBindAPI(EGL_OPENGL_ES_API) failed";
                return false;
            }
            const std::array<EGLint, 3> CONTEXT_ATTRIBUTES = {EGL_CONTEXT_MAJOR_VERSION, 3, EGL_NONE};
            context                                        = eglCreateContext(display, config, EGL_NO_CONTEXT, CONTEXT_ATTRIBUTES.data());
            if (context == EGL_NO_CONTEXT) {
                error = std::format("eglCreateContext for OpenGL ES 3 failed (0x{:x})", static_cast<uint32_t>(eglGetError()));
                return false;
            }
            if (!makeCurrent()) {
                error = std::format("eglMakeCurrent without a surface failed (0x{:x})", static_cast<uint32_t>(eglGetError()));
                return false;
            }
            return true;
        }

        void teardown() {
            if (display == EGL_NO_DISPLAY)
                return;
            if (context != EGL_NO_CONTEXT && makeCurrent()) {
                for (auto& TEXTURE : backgroundTextures) {
                    if (TEXTURE.id)
                        glDeleteTextures(1, &TEXTURE.id);
                }
                for (auto& TEXTURE : overlayTextures) {
                    if (TEXTURE.id)
                        glDeleteTextures(1, &TEXTURE.id);
                }
                if (renderbuffer)
                    glDeleteRenderbuffers(1, &renderbuffer);
                if (framebuffer)
                    glDeleteFramebuffers(1, &framebuffer);
                if (vertexArray)
                    glDeleteVertexArrays(1, &vertexArray);
                if (program)
                    glDeleteProgram(program);
            }
            eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (context != EGL_NO_CONTEXT)
                eglDestroyContext(display, context);
            eglTerminate(display);
            display = EGL_NO_DISPLAY;
        }
    };

    CComposeGL::~CComposeGL() {
        if (m_state)
            m_state->teardown();
    }

    namespace {

        struct SDeviceCandidate {
            EGLDeviceEXT device = nullptr;
            std::string  label;
        };

        std::vector<SDeviceCandidate> enumerateDevices() {
            std::vector<SDeviceCandidate> found;

            const auto QUERY_DEVICES = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
            const auto QUERY_STRING  = reinterpret_cast<PFNEGLQUERYDEVICESTRINGEXTPROC>(eglGetProcAddress("eglQueryDeviceStringEXT"));
            if (!QUERY_DEVICES || !QUERY_STRING)
                return found;

            EGLint total = 0;
            if (QUERY_DEVICES(0, nullptr, &total) != EGL_TRUE || total <= 0)
                return found;

            std::vector<EGLDeviceEXT> devices(static_cast<size_t>(total));
            if (QUERY_DEVICES(total, devices.data(), &total) != EGL_TRUE)
                return found;

            for (EGLint i = 0; i < total; ++i) {
                SDeviceCandidate candidate;
                candidate.device      = devices[static_cast<size_t>(i)];
                const char* DRM_NODE  = QUERY_STRING(devices[static_cast<size_t>(i)], EGL_DRM_DEVICE_FILE_EXT);
                const char* RENDER    = QUERY_STRING(devices[static_cast<size_t>(i)], 0x3377 /* EGL_DRM_RENDER_NODE_FILE_EXT */);
                const char* VENDOR    = QUERY_STRING(devices[static_cast<size_t>(i)], EGL_VENDOR);
                candidate.label       = std::format("device {}: drm={} render={} vendor={}", i, DRM_NODE ? DRM_NODE : "-", RENDER ? RENDER : "-", VENDOR ? VENDOR : "-");
                found.push_back(candidate);
            }
            return found;
        }

    }

    std::unique_ptr<CComposeGL> CComposeGL::create(int paneWidth, int paneHeight, int paneCount, const std::string& gpuHint, std::string& error) {
        error.clear();
        if (paneWidth <= 0 || paneHeight <= 0 || paneCount <= 0) {
            error = std::format("invalid render target {}x{} x{} panes", paneWidth, paneHeight, paneCount);
            return nullptr;
        }

        const auto GET_PLATFORM_DISPLAY = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
        if (!GET_PLATFORM_DISPLAY) {
            error = "eglGetPlatformDisplayEXT is unavailable";
            return nullptr;
        }

        auto  compositor = std::unique_ptr<CComposeGL>(new CComposeGL());
        compositor->m_state = std::make_unique<SState>();
        auto& state      = *compositor->m_state;

        const std::string HINT = lowercase(gpuHint);

        if (HINT.empty()) {
            if (!extensionListed(eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS), "EGL_MESA_platform_surfaceless")) {
                error = "EGL_MESA_platform_surfaceless is unavailable";
                return nullptr;
            }
            state.display = GET_PLATFORM_DISPLAY(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
            if (state.display == EGL_NO_DISPLAY) {
                error = "eglGetPlatformDisplayEXT(EGL_PLATFORM_SURFACELESS_MESA) failed";
                return nullptr;
            }
            EGLint major = 0, minor = 0;
            if (eglInitialize(state.display, &major, &minor) != EGL_TRUE) {
                error         = std::format("eglInitialize on the surfaceless platform failed (0x{:x})", static_cast<uint32_t>(eglGetError()));
                state.display = EGL_NO_DISPLAY;
                return nullptr;
            }
            if ((major < 1 || (major == 1 && minor < 5)) && !extensionListed(eglQueryString(state.display, EGL_EXTENSIONS), "EGL_KHR_surfaceless_context")) {
                error = "EGL is older than 1.5 and lacks EGL_KHR_surfaceless_context";
                return nullptr;
            }
            if (!state.chooseConfigAndContext(error))
                return nullptr;
        } else {
            // A hinted run enumerates devices and keeps the first whose strings
            // match, so a machine with both a Mesa and an NVIDIA GLVND vendor can be
            // pinned without environment gymnastics.
            const auto CANDIDATES = enumerateDevices();
            if (CANDIDATES.empty()) {
                error = "EGL_EXT_device_enumeration found no devices; drop --gpu to use the default surfaceless display";
                return nullptr;
            }

            std::string roster;
            for (const auto& CANDIDATE : CANDIDATES) {
                if (!roster.empty())
                    roster += "\n  ";
                roster += CANDIDATE.label;

                if (HINT == "list")
                    continue;

                EGLDisplay display = GET_PLATFORM_DISPLAY(EGL_PLATFORM_DEVICE_EXT, CANDIDATE.device, nullptr);
                if (display == EGL_NO_DISPLAY)
                    continue;
                if (eglInitialize(display, nullptr, nullptr) != EGL_TRUE)
                    continue;

                state.display = display;
                std::string ignored;
                if (!state.chooseConfigAndContext(ignored)) {
                    eglTerminate(display);
                    state.display = EGL_NO_DISPLAY;
                    state.context = EGL_NO_CONTEXT;
                    continue;
                }

                const auto* RENDERER = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
                const auto* VENDOR   = eglQueryString(display, EGL_VENDOR);
                const std::string HAYSTACK = lowercase(std::format("{} {} {}", RENDERER ? RENDERER : "", VENDOR ? VENDOR : "", CANDIDATE.label));
                roster += std::format(" renderer=\"{}\"", RENDERER ? RENDERER : "-");
                if (HAYSTACK.find(HINT) != std::string::npos)
                    break;

                state.teardown();
                state.context = EGL_NO_CONTEXT;
                state.display = EGL_NO_DISPLAY;
            }

            if (state.display == EGL_NO_DISPLAY || state.context == EGL_NO_CONTEXT) {
                error = std::format("no EGL device matches --gpu \"{}\". Devices seen:\n  {}", gpuHint, roster);
                return nullptr;
            }
        }

        if (!state.makeCurrent()) {
            error = "eglMakeCurrent failed after context creation";
            return nullptr;
        }

        state.program = linkProgram(error);
        if (state.program == 0)
            return nullptr;

        glGenVertexArrays(1, &state.vertexArray);
        glBindVertexArray(state.vertexArray);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDisable(GL_SCISSOR_TEST);

        glGenFramebuffers(1, &state.framebuffer);
        glGenRenderbuffers(1, &state.renderbuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, state.renderbuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, paneWidth * paneCount, paneHeight);
        glBindFramebuffer(GL_FRAMEBUFFER, state.framebuffer);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, state.renderbuffer);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            error = std::format("could not allocate a {}x{} RGBA8 render target", paneWidth * paneCount, paneHeight);
            return nullptr;
        }

        state.paneWidth  = paneWidth;
        state.paneHeight = paneHeight;
        state.paneCount  = paneCount;
        state.backgroundTextures.resize(static_cast<size_t>(paneCount));
        state.overlayTextures.resize(static_cast<size_t>(paneCount));
        state.readbackScratch.assign(static_cast<size_t>(paneWidth) * paneCount * paneHeight * 4, 0);

        const auto* RENDERER = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
        const auto* VENDOR   = eglQueryString(state.display, EGL_VENDOR);
        state.description    = std::format("{} via {}", RENDERER ? RENDERER : "unknown renderer", VENDOR ? VENDOR : "unknown EGL vendor");
        return compositor;
    }

    bool CComposeGL::uploadBackground(int pane, const uint8_t* rgba, int width, int height) {
        return m_state->uploadTexture(m_state->backgroundTextures, pane, rgba, width, height);
    }

    bool CComposeGL::uploadOverlay(int pane, const uint8_t* rgba, int width, int height) {
        return m_state->uploadTexture(m_state->overlayTextures, pane, rgba, width, height);
    }

    bool CComposeGL::drawPane(int pane, const SPaneDraw& draw, std::string& error) {
        auto& state = *m_state;
        if (pane < 0 || pane >= state.paneCount) {
            error = std::format("pane {} is out of range (target holds {})", pane, state.paneCount);
            return false;
        }
        if (!state.makeCurrent()) {
            error = "eglMakeCurrent failed";
            return false;
        }
        if (!draw.outputFov.sane()) {
            error = "the output camera's fov is not a usable frustum";
            return false;
        }

        const GLint ORIGIN_X = pane * state.paneWidth;
        glBindFramebuffer(GL_FRAMEBUFFER, state.framebuffer);
        glViewport(ORIGIN_X, 0, state.paneWidth, state.paneHeight);
        glUseProgram(state.program);
        glBindVertexArray(state.vertexArray);

        const auto UNIFORM = [&](const char* name) { return glGetUniformLocation(state.program, name); };

        glUniform2i(UNIFORM("uPaneOrigin"), ORIGIN_X, 0);
        glUniform2i(UNIFORM("uPaneSize"), state.paneWidth, state.paneHeight);

        const auto OUT_TAN = draw.outputFov.tangents();
        glUniform4f(UNIFORM("uOutFovTan"), static_cast<float>(OUT_TAN[0]), static_cast<float>(OUT_TAN[1]), static_cast<float>(OUT_TAN[2]), static_cast<float>(OUT_TAN[3]));
        const auto OUT_ROT = toFloat9(draw.outputCamera.rot.toMat3ColumnMajor());
        glUniformMatrix3fv(UNIFORM("uOutRot"), 1, GL_FALSE, OUT_ROT.data());
        glUniform3f(UNIFORM("uOutPos"), static_cast<float>(draw.outputCamera.pos.x), static_cast<float>(draw.outputCamera.pos.y), static_cast<float>(draw.outputCamera.pos.z));

        glUniform1i(UNIFORM("uBgMode"), static_cast<int>(draw.backgroundMode));
        glUniform4f(UNIFORM("uSolid"), draw.solidColor[0], draw.solidColor[1], draw.solidColor[2], draw.solidColor[3]);

        const auto CAM_ROT_INV = inverseRotation(draw.cameraPose.rot);
        glUniformMatrix3fv(UNIFORM("uCamRotInv"), 1, GL_FALSE, CAM_ROT_INV.data());
        glUniform3f(UNIFORM("uCamPos"), static_cast<float>(draw.cameraPose.pos.x), static_cast<float>(draw.cameraPose.pos.y), static_cast<float>(draw.cameraPose.pos.z));
        glUniform4f(UNIFORM("uCamIntr"), static_cast<float>(draw.intrinsics.fx), static_cast<float>(draw.intrinsics.fy), static_cast<float>(draw.intrinsics.cx), static_cast<float>(draw.intrinsics.cy));
        const auto DISTORTION = draw.intrinsics.distortion5();
        glUniform3f(UNIFORM("uDistortionA"), static_cast<float>(DISTORTION[0]), static_cast<float>(DISTORTION[1]), static_cast<float>(DISTORTION[2]));
        glUniform2f(UNIFORM("uDistortionB"), static_cast<float>(DISTORTION[3]), static_cast<float>(DISTORTION[4]));
        glUniform2f(UNIFORM("uCamSize"), static_cast<float>(draw.backgroundWidth), static_cast<float>(draw.backgroundHeight));
        glUniform1f(UNIFORM("uBgDepth"), depthIsInfinite(draw.backgroundDepth) ? -1.0F : static_cast<float>(draw.backgroundDepth));

        glUniform1i(UNIFORM("uHasFg"), draw.hasOverlay ? 1 : 0);
        const auto FG_ROT_INV = inverseRotation(draw.overlayPose.rot);
        glUniformMatrix3fv(UNIFORM("uFgRotInv"), 1, GL_FALSE, FG_ROT_INV.data());
        glUniform3f(UNIFORM("uFgPos"), static_cast<float>(draw.overlayPose.pos.x), static_cast<float>(draw.overlayPose.pos.y), static_cast<float>(draw.overlayPose.pos.z));
        const auto FG_TAN = draw.overlayFov.tangents();
        glUniform4f(UNIFORM("uFgFovTan"), static_cast<float>(FG_TAN[0]), static_cast<float>(FG_TAN[1]), static_cast<float>(FG_TAN[2]), static_cast<float>(FG_TAN[3]));
        glUniform1f(UNIFORM("uFgDepth"), depthIsInfinite(draw.overlayDepth) ? -1.0F : static_cast<float>(draw.overlayDepth));
        glUniform1i(UNIFORM("uPremultiplied"), draw.premultipliedAlpha ? 1 : 0);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, state.backgroundTextures[static_cast<size_t>(pane)].id);
        glUniform1i(UNIFORM("uBgTex"), 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, state.overlayTextures[static_cast<size_t>(pane)].id);
        glUniform1i(UNIFORM("uFgTex"), 1);

        glDrawArrays(GL_TRIANGLES, 0, 3);

        const GLenum ERROR = glGetError();
        if (ERROR != GL_NO_ERROR) {
            error = std::format("GL error 0x{:x} while drawing pane {}", static_cast<uint32_t>(ERROR), pane);
            return false;
        }
        return true;
    }

    bool CComposeGL::readback(std::vector<uint8_t>& rgba) {
        auto& state = *m_state;
        if (!state.makeCurrent())
            return false;

        const int WIDTH  = state.paneWidth * state.paneCount;
        const int HEIGHT = state.paneHeight;
        rgba.resize(static_cast<size_t>(WIDTH) * HEIGHT * 4);

        glBindFramebuffer(GL_FRAMEBUFFER, state.framebuffer);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, state.readbackScratch.data());
        if (glGetError() != GL_NO_ERROR)
            return false;

        // glReadPixels hands back the bottom row first.
        const size_t STRIDE = static_cast<size_t>(WIDTH) * 4;
        for (int y = 0; y < HEIGHT; ++y)
            std::memcpy(rgba.data() + static_cast<size_t>(y) * STRIDE, state.readbackScratch.data() + static_cast<size_t>(HEIGHT - 1 - y) * STRIDE, STRIDE);
        return true;
    }

    int CComposeGL::width() const {
        return m_state->paneWidth * m_state->paneCount;
    }

    int CComposeGL::height() const {
        return m_state->paneHeight;
    }

    std::string CComposeGL::description() const {
        return m_state->description;
    }

}
