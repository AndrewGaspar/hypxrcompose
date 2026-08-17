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
        // INTERPRETATION: the contract carries per-eye poses only, but a camera
        // extrinsic is head-relative, so a head pose is needed. An optional `head`
        // pose is read when the producer supplies one; otherwise head() returns the
        // midpoint of the eyes, which is where OpenXR's VIEW space sits.
        std::optional<SPose>       head;

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
        bool probeMedia = true;
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
