#include "Validate.hpp"
#include "Bundle.hpp"
#include "Log.hpp"

#include <cstdio>
#include <map>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace hxc {

    std::string describeBundle(const SBundle& bundle) {
        std::string out;
        out += std::format("take `{}` at {}\n", bundle.takeId.empty() ? "(unnamed)" : bundle.takeId, bundle.root.string());

        const int64_t SPAN = bundle.lastHostNs() - bundle.firstHostNs();
        out += std::format("  telemetry : {} records over {:.3f} s (median interval {:.3f} ms, {:.1f} Hz)\n", bundle.telemetry.size(), static_cast<double>(SPAN) * 1e-9,
                           static_cast<double>(bundle.medianTelemetryIntervalNs()) * 1e-6, bundle.medianTelemetryIntervalNs() > 0 ? 1e9 / static_cast<double>(bundle.medianTelemetryIntervalNs()) : 0.0);

        if (bundle.clock.empty())
            out += "  clock     : no samples; host and device time treated as one clock\n";
        else
            out += std::format("  clock     : {} samples, offset {:.3f} ms at the start and {:.3f} ms at the end (device = host + offset)\n", bundle.clock.size(),
                               static_cast<double>(bundle.clock.samples().front().offsetNs) * 1e-6, static_cast<double>(bundle.clock.samples().back().offsetNs) * 1e-6);

        if (!bundle.sources.overlay)
            out += "  overlay   : absent\n";
        else {
            const size_t DROPPED = bundle.telemetry.size() - bundle.overlay.frameTelemetryIndex.size();
            out += std::format("  overlay   : {}x{} {} ({}), {} eye(s), target {:.1f} Hz, alpha {}\n", bundle.overlay.width, bundle.overlay.height, bundle.overlay.format, bundle.overlay.encoder,
                               bundle.overlay.videoPaths.size(), bundle.overlay.targetHz, bundle.overlay.alpha);
            out += std::format("              {} record(s) carry pixels, {} marked dropped\n", bundle.overlay.frameTelemetryIndex.size(), DROPPED);
            for (size_t eye = 0; eye < bundle.overlay.videoInfo.size(); ++eye) {
                if (bundle.overlay.videoPaths[eye].empty())
                    continue;
                out += std::format("              eye{}: {} frames, pix_fmt {}\n", eye, bundle.overlay.videoInfo[eye].ptsNs.size(), bundle.overlay.videoInfo[eye].pixelFormat);
            }
        }

        // The quad records are not composited yet (NEXT-STEPS gap 1), so the only
        // thing validate can usefully say about them is that they parsed and how
        // the producer spells the two fields that have more than one spelling.
        {
            size_t withQuads = 0, quadCount = 0;
            std::map<std::string, size_t> byEye;
            for (const auto& RECORD : bundle.telemetry) {
                if (!RECORD.hasQuadsArray)
                    continue;
                ++withQuads;
                quadCount += RECORD.quads.size();
                for (const auto& QUAD : RECORD.quads)
                    ++byEye[toString(QUAD.eyeVisibility)];
            }
            if (withQuads > 0) {
                std::string spread;
                for (const auto& [NAME, COUNT] : byEye)
                    spread += std::format("{}{}x{}", spread.empty() ? "" : ", ", COUNT, NAME);
                out += std::format("  quads     : {} layer record(s) over {} telemetry record(s), eye visibility {}\n", quadCount, withQuads, spread);
            }
        }

        if (bundle.cameras.empty())
            out += "  cameras   : absent\n";
        else {
            for (const auto& CAM : bundle.cameras) {
                out += std::format("  camera {}  : {}x{}, {} frames, fx={:.1f} fy={:.1f} ({:.1f} deg horizontal), lens at {} from the head\n", CAM.key, CAM.video.width, CAM.video.height,
                                   CAM.frames.size(), CAM.intrinsics.fx, CAM.intrinsics.fy, CAM.intrinsics.hfovDegrees(CAM.video.width), toString(CAM.headToCamera.pos));
            }
        }

        for (const auto* TRACK : {bundle.appAudio ? &*bundle.appAudio : nullptr, bundle.mic ? &*bundle.mic : nullptr}) {
            if (!TRACK)
                continue;
            out += std::format("  audio {:<4}: {} Hz x{}, {:.3f} s, starts {:+.3f} ms into the take ({} clock)\n", TRACK->role, TRACK->sampleRate, TRACK->channels,
                               static_cast<double>(TRACK->durationNs) * 1e-9, static_cast<double>(TRACK->startHostNs - bundle.firstHostNs()) * 1e-6, TRACK->deviceClock ? "device" : "host");
        }

        for (const auto& NOTE : bundle.notes)
            out += std::format("  note      : {}\n", NOTE);

        return out;
    }

    int runValidate(const SValidateOptions& options) {
        CDiagnostics diags;
        SLoadOptions load;
        load.probeMedia = !options.skipMedia;

        const auto BUNDLE = SBundle::load(options.root, diags, load);

        if (options.jsonOutput) {
            json report;
            report["take"]  = options.root.string();
            report["ok"]    = BUNDLE.has_value() && !diags.hasErrors() && (!options.strict || diags.warningCount() == 0);
            report["errors"]   = json::array();
            report["warnings"] = json::array();
            for (const auto& D : diags.all())
                report[D.error ? "errors" : "warnings"].push_back({{"where", D.where}, {"message", D.message}});
            if (BUNDLE) {
                report["summary"] = {
                    {"take_id", BUNDLE->takeId},
                    {"telemetry_records", BUNDLE->telemetry.size()},
                    {"clock_samples", BUNDLE->clock.size()},
                    {"first_t_host_ns", BUNDLE->firstHostNs()},
                    {"last_t_host_ns", BUNDLE->lastHostNs()},
                    {"cameras", BUNDLE->cameras.size()},
                    {"overlay_eyes", BUNDLE->overlay.videoPaths.size()},
                    {"app_audio", BUNDLE->appAudio.has_value()},
                    {"mic", BUNDLE->mic.has_value()},
                };
            }
            std::printf("%s\n", report.dump(2).c_str());
        } else {
            if (BUNDLE)
                std::fputs(describeBundle(*BUNDLE).c_str(), stdout);
            diags.print();
            if (diags.hasErrors())
                HXC_ERR("{} error(s), {} warning(s)", diags.errorCount(), diags.warningCount());
            else if (diags.warningCount() > 0)
                HXC_WARN("no errors, {} warning(s)", diags.warningCount());
            else
                HXC_INFO("bundle validates clean");
        }

        if (!BUNDLE)
            return 2;
        if (diags.hasErrors())
            return 1;
        if (options.strict && diags.warningCount() > 0)
            return 1;
        return 0;
    }

}
