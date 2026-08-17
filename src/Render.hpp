#pragma once

// `hypxrcompose render <take> --out out.mp4`.
//
// The output timeline is constant-rate: frame k sits at t0 + k/fps where t0 is
// the first telemetry stamp. Every source is then resampled onto it by
// nearest-in-host-time selection, which for the common case (output rate equal
// to the capture rate) is an exact 1:1 map and for the 30 Hz cameras is the
// least-wrong choice available without optical flow.
//
// Poses are the exception: the camera pose for a given source frame is
// *interpolated* between the telemetry records bracketing that frame's
// mid-exposure instant, because a 30 Hz camera frame lands between 90 Hz
// telemetry records and rounding to the nearest would inject up to half a
// telemetry interval of head motion into the geometry.

#include "Math.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace hxc {

    enum class eEyeSelection {
        LEFT,
        RIGHT,
        STEREO_SBS,
    };

    enum class eFraming {
        ASIS,       // the recorded eye poses, exactly as the renderer used them
        STABILIZED, // the zero-phase Gaussian smoothing described in Stabilize.hpp
    };

    enum class eBackgroundChoice {
        AUTO,    // camera when the take has one, checker when it does not
        CAMERA,
        CHECKER,
        SOLID,
    };

    struct SRenderOptions {
        std::filesystem::path take;
        std::string           outPath;

        eEyeSelection         eye     = eEyeSelection::LEFT;
        eFraming              framing = eFraming::ASIS;
        eBackgroundChoice     background = eBackgroundChoice::AUTO;

        // Per-eye pane size. Stereo SBS output is therefore 2*width x height, which
        // keeps each eye's geometry undistorted rather than squeezing two eyes into
        // one frame. 0 means "take the overlay's size, or the camera's".
        int                   width  = 0;
        int                   height = 0;
        double                fps    = 0.0; // 0 means "manifest target_hz, else the telemetry rate"

        double                backgroundDepth = 2.0;
        // Infinite by default: with the output camera at the recorded eye pose the
        // overlay reprojection is exact at any depth, and with a stabilized camera a
        // rotation-only warp is the honest choice absent depth data.
        double                overlayDepth    = std::numeric_limits<double>::infinity();

        int64_t               stabilizeSigmaNs = 200LL * 1000000LL;

        bool                  noAudio  = false;
        double                micGain  = 1.0;
        bool                  noLimiter = false;

        std::string           videoCodec = "libx264";
        int                   crf        = 18;
        std::string           preset     = "medium";

        std::string           gpuHint;
        std::string           framesDir;  // when set, every output frame is also written as a PNG
        std::string           reportPath; // when set, the render report is written as JSON
        size_t                limitFrames = 0;

        // ---- segmented rendering -------------------------------------------------
        //
        // The composite is embarrassingly parallel in time: output frame k depends
        // on the take and on k, never on frame k-1. So the timeline is cut into
        // `jobs` contiguous chunks, each rendered by its own worker process, and
        // the encoded chunks are stream-copied together at the end.
        //
        // Workers are processes, not threads. The decisive reason is that a worker
        // is then *the same code path* as a single-job render - it runs the same
        // loop over a different range of k - so "the parallel output equals the
        // serial output" is a property of the arithmetic rather than a hope about
        // two GL contexts in one address space. (The others: EGL's surfaceless
        // display is a per-process singleton whose eglTerminate would be a
        // use-after-free for a sibling context, and a worker that dies takes only
        // its own segment with it.)
        //
        // 0 = choose; 1 = render in this process, no workers, no temporary files.
        int                   jobs          = 0;
        // Threads each worker's ffmpeg subprocesses may use. 0 = divide the
        // machine by `jobs`. Left alone in a single-job run, where ffmpeg's own
        // default - all of them - is right.
        int                   decodeThreads = 0;

        // Worker mode: render only chunk `segmentIndex` of `segmentCount`. Audio
        // is never muxed into a segment; the parent folds it into the join.
        int                   segmentIndex  = -1;
        int                   segmentCount  = 0;
        // What the parent spawns for a worker. Empty means /proc/self/exe, which
        // is right for the installed binary and wrong for the test executable -
        // hence the override.
        std::string           workerBinary;
        // Where the parent left its probe results for the workers to read, so K
        // processes do not each demux the take's gigabytes to count the same
        // frames. Set by the parent; nothing else needs to know about it.
        std::string           probeCachePath;
    };

    // The half-open range of output frames segment `index` of `count` owns.
    // Shared by the parent that spawns workers and the workers themselves, so the
    // two cannot disagree about where a chunk begins. Remainder frames go to the
    // earliest segments, one each.
    std::pair<size_t, size_t> segmentRange(size_t frameCount, int index, int count);

    struct SRenderReport {
        struct SFrameRecord {
            size_t                 index          = 0;
            int64_t                tHostNs        = 0;
            size_t                 telemetryIndex = 0;
            std::array<int64_t, 2> overlayFrame{-1, -1};
            // Which telemetry record that overlay frame was rendered from - the
            // ordinal rule's answer, which differs from `telemetryIndex` whenever
            // frames were dropped.
            std::array<int64_t, 2> overlayTelemetryIndex{-1, -1};
            std::array<int64_t, 2> cameraFrame{-1, -1};
            std::array<int64_t, 2> cameraHostNs{0, 0};
        };

        int                       paneWidth  = 0;
        int                       paneHeight = 0;
        int                       paneCount  = 1;
        double                    fps        = 0.0;
        int64_t                   firstHostNs = 0;
        std::string               gpu;
        std::string               framing;
        // How many workers produced this. 1 for a single-job render, and for a
        // worker's own report of its own segment.
        int                       jobs       = 1;
        std::vector<SFrameRecord> frames;

        double                    wallSeconds    = 0.0;
        double                    decodeSeconds  = 0.0;
        double                    gpuSeconds     = 0.0;
        double                    encodeSeconds  = 0.0;
        double                    framesPerSecond = 0.0;
        double                    megapixelsPerSecond = 0.0;

        struct SAudioPlacement {
            std::string role;
            int64_t     startSample = 0;
            double      gain        = 1.0;
        };
        std::vector<SAudioPlacement> audio;

        std::string toJson() const;
        // Reads back what toJson() wrote. The parent of a segmented render uses
        // it to fold its workers' reports into one, so `--report` describes the
        // whole take whatever `--jobs` was.
        static bool fromJson(const std::string& text, SRenderReport& out, std::string& error);
    };

    // 0 on success. `report` is filled even on some failures, so the tests can see
    // how far the pipeline got.
    int runRender(const SRenderOptions& options, SRenderReport* report = nullptr);

}
