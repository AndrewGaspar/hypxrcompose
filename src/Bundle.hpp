#pragma once

// The in-memory model of a `.hypxrtake` bundle, and the loader that is also the
// validator. `hypxrcompose validate` is exactly this loader run with media
// probing on and every diagnostic printed; `render` is the same loader with the
// diagnostics filtered to errors. There is deliberately no second parser, so the
// thing that composes and the thing that arbitrates the format cannot drift.
//
// The contract, restated (see README for the normative copy):
//
//   manifest.json    {"take_id", "host":{...},
//                     "sources":{"overlay","app_audio","cameras","mic"},
//                     "overlay":{"width","height","format","encoder","target_hz",
//                                "eye_count"},
//                     "notes":[]}
//   telemetry.jsonl  {"t_host_ns","frame",
//                     "eyes":[{"pose":{"pos":[3],"quat":[4]},
//                              "fov":{"l","r","u","d"}} x eye_count],
//                     "stage_correction":{...}|null, "blend_mode"}
//   clock.jsonl      {"t_host_ns","offset_ns","rtt_us"}   (device = host + offset)
//   overlay/eye0.mkv, overlay/eye1.mkv   RGBA lossless, container pts = t_host_ns
//   audio/app.flac + audio/app.json      {"start_t_host_ns","sample_rate_hz",
//                                         "channels"}
//   cameras/<prefix>-cam{L,R}.mp4 + cameras/<prefix>-cameras.jsonl
//   audio/<prefix>-mic.flac + audio/<prefix>-mic.json {"start_t_device_ns", ...}
//
// Every place this loader had to interpret something the contract does not pin
// down is marked `INTERPRETATION:` in Bundle.cpp and listed in README's
// "Interpretations" section, because the capture-side producers have to converge
// on the same reading.

#include "Math.hpp"
#include "Ffmpeg.hpp"
#include "Timeline.hpp"

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace nlohmann {
    // Forward declaration would need the full template signature; the loader keeps
    // the raw manifest as a string instead so this header stays json-free.
}

namespace hxc {

    struct SDiag {
        bool        error = true;
        std::string where;
        std::string message;
    };

    class CDiagnostics {
      public:
        template <typename... Args>
        void error(std::string where, std::format_string<Args...> fmt, Args&&... args) {
            m_diags.push_back({true, std::move(where), std::format(fmt, std::forward<Args>(args)...)});
        }
        template <typename... Args>
        void warn(std::string where, std::format_string<Args...> fmt, Args&&... args) {
            m_diags.push_back({false, std::move(where), std::format(fmt, std::forward<Args>(args)...)});
        }

        bool                        hasErrors() const;
        size_t                      errorCount() const;
        size_t                      warningCount() const;
        const std::vector<SDiag>&   all() const {
            return m_diags;
        }
        void print() const;

      private:
        std::vector<SDiag> m_diags;
    };

    struct STelemetryEye {
        SPose pose;
        SFov  fov;
    };

    // One composition layer as the session submitted it, recorded for grade-B
    // replay. v1 does not consume these - it composites the recorded matte, not
    // re-rendered quads - but validate checks them, because telemetry not recorded
    // correctly today is telemetry v2 cannot use.
    //
    // TWO LOAD-BEARING SEMANTICS, both from the producer:
    //
    //   1. `pose` is HEAD-RELATIVE. The layer's pose in STAGE space at time t is
    //      head(t) composed with it. Reading it as a world pose puts every quad in
    //      the wrong place the moment the wearer moves.
    //   2. `viewSpace` distinguishes what that means over time. true = head-locked:
    //      the layer stays at this head-relative pose always. false = room-anchored:
    //      the layer had a fixed pose in STAGE, and what was recorded is that pose
    //      expressed relative to the head at t. A replay must re-anchor it, not
    //      carry it along with the head.
    // OpenXR's XrEyeVisibility, which is what the host producer stamps into
    // `visibility` - PINNED, see README's interpretations table. A layer is
    // composed into both eyes, or into one of them only: a stereo-depth desktop
    // emits a per-eye pair of quads sharing a pose and taking opposite halves of
    // a side-by-side swapchain, while a HUD is `both`. `NONE` is the producer's
    // fourth spelling, for a layer that was submitted but composed nowhere.
    enum class eEyeVisibility {
        BOTH,
        LEFT,
        RIGHT,
        NONE,
    };

    std::string toString(eEyeVisibility visibility);

    struct SQuadRecord {
        int64_t                    index = 0; // composition order, back to front
        std::optional<std::string> name;      // currently always null from the producer
        SPose                      pose;      // head-relative, see above
        double                     width      = 0.0; // metres
        double                     height     = 0.0;
        // The canonical reading is `eyeVisibility`: an XrEyeVisibility spelled
        // as a string. `visibility` survives for the deprecated boolean/numeric
        // spelling, which older bundles use as an opacity and which the loader
        // still accepts with a warning. `composedInEye` is the question a
        // compositor should ask; it answers both spellings at once.
        double                     visibility    = 1.0;
        eEyeVisibility             eyeVisibility = eEyeVisibility::BOTH;
        bool                       viewSpace  = false;
        int64_t                    swapchain  = -1; // stable within a session only
        int64_t                    image      = -1;
        int64_t                    arrayLayer = 0;
        std::array<double, 4>      rect{};   // x, y, w, h in swapchain image pixels
        bool                       hasRect = false;

        // The layer's pose in STAGE space, given the head pose at the same instant.
        SPose                      worldPose(const SPose& head) const {
            return head.compose(pose);
        }

        // Was this layer composed into the given eye's view? eye 0 = left,
        // eye 1 = right, matching the telemetry's `eyes` order. A grade-B replay
        // draws a quad into a pane exactly when this is true for that pane's eye.
        bool                       composedInEye(int eye) const {
            if (!(visibility > 0.0))
                return false;
            switch (eyeVisibility) {
                case eEyeVisibility::LEFT: return eye == 0;
                case eEyeVisibility::RIGHT: return eye == 1;
                case eEyeVisibility::NONE: return false;
                default: return true;
            }
        }
    };

    struct STelemetryFrame {
        int64_t                    tHostNs = 0;
        int64_t                    frame   = 0;
        std::vector<STelemetryEye> eyes;
        std::optional<SPose>       stageCorrection;
        std::string                blendMode;
        // "this frame has no pixels in the overlay video" - either decimated down to
        // overlay.target_hz or lost to the readback queue. The records *without* it
        // are what the video's frames correspond to, in order.
        bool                       dropped = false;
        // The producer records this per record, at the same instant as the eyes and
        // in the same (STAGE) space. It is what camera extrinsics and quad poses are
        // relative to. Absent only in bundles written before it existed, where
        // headPose() falls back to the midpoint of the eyes.
        std::optional<SPose>       head;
        std::vector<SQuadRecord>   quads;
        bool                       hasQuadsArray = false;

        SPose                      headPose() const;
    };

    struct SOverlayInfo {
        int         width    = 0;
        int         height   = 0;
        std::string format   = "rgba";
        std::string encoder;
        double      targetHz = 0.0;
        int         eyeCount = 2;
        // INTERPRETATION: not in the stated contract. "straight" unless the manifest
        // says otherwise; see README.
        std::string alpha    = "straight";

        std::vector<std::string> videoPaths;
        std::vector<SVideoInfo>  videoInfo;

        // The alignment rule, and the only one: the n-th frame of each eye's video
        // is the n-th telemetry record that is not `dropped`, and that record's
        // t_host_ns is the frame's true time. Container pts carry a uniform nominal
        // timeline at target_hz and are never used to align - Matroska's 1 ms
        // timestamp scale could not carry t_host_ns even if a producer wanted it to.
        //
        // Both eyes share these, because both eyes share the telemetry.
        std::vector<size_t>      frameTelemetryIndex;
        std::vector<int64_t>     frameHostNs;
    };

    struct SCameraFrame {
        int64_t tDeviceNs  = 0;
        int64_t exposureNs = 0;
        int64_t frame      = 0;
    };

    struct SCamera {
        std::string               key;   // normalized to "L" or "R"
        int                       eye = 0;
        SCameraIntrinsics         intrinsics;
        SPose                     headToCamera;
        std::string               timestampSource;
        std::string               videoPath;
        SVideoInfo                video;
        std::vector<SCameraFrame> frames;
        // Device timestamps mapped into host time, mid-exposure, ready for nearest
        // matching against the output timeline.
        std::vector<int64_t>      hostNs;
    };

    struct SAudioTrack {
        std::string path;
        std::string sidecarPath;
        std::string role;        // "app" or "mic"
        int64_t     startNs     = 0;
        bool        deviceClock = false;  // true -> startNs is in the device domain
        int64_t     startHostNs = 0;      // resolved through the clock map
        int         sampleRate  = 0;
        int         channels    = 0;
        int64_t     durationNs  = 0;
    };

    struct SLoadOptions {
        // Probing runs ffprobe over every media file; render needs it, a purely
        // structural check can skip it.
        bool        probeMedia = true;
        // INDEX counts the container's packets, DEEP decodes every frame. The
        // alignment rule is enforced identically either way - what changes is
        // only how the frame count on its left-hand side was arrived at, and
        // whether the run takes seconds or a quarter of an hour. See
        // eProbeDepth.
        eProbeDepth probeDepth = eProbeDepth::INDEX;
        // Hash every decoded frame. Implies a full decode, so it is only ever
        // asked for alongside DEEP.
        bool        checksum   = false;
        // Consulted before, and filled in after, every video probe. A segmented
        // render's parent hands one of these to its workers so K processes do
        // not each demux the same gigabytes to count the same frames.
        std::shared_ptr<CProbeCache> probeCache;
    };

    struct SBundle {
        std::filesystem::path        root;
        std::string                  takeId;
        std::string                  manifestText;

        struct SSources {
            bool overlay  = false;
            bool appAudio = false;
            bool cameras  = false;
            bool mic      = false;
        } sources;

        SOverlayInfo                 overlay;
        std::vector<STelemetryFrame> telemetry;
        std::vector<int64_t>         telemetryHostNs;
        CClockMap                    clock;
        std::vector<SCamera>         cameras;
        std::optional<SAudioTrack>   appAudio;
        std::optional<SAudioTrack>   mic;
        std::vector<std::string>     notes;

        int64_t                      firstHostNs() const {
            return telemetryHostNs.empty() ? 0 : telemetryHostNs.front();
        }
        int64_t lastHostNs() const {
            return telemetryHostNs.empty() ? 0 : telemetryHostNs.back();
        }
        // Median inter-frame interval of the telemetry track, in nanoseconds.
        int64_t medianTelemetryIntervalNs() const;

        const SCamera* cameraForEye(int eye) const;

        // Returns nullopt only when the bundle is unusable; recoverable problems
        // land in `diags` and still yield a bundle so validate can report them all
        // in one pass rather than one per run.
        static std::optional<SBundle> load(const std::filesystem::path& root, CDiagnostics& diags, const SLoadOptions& options = {});
    };

}
