// validate is the format's arbiter, so its error cases are the specification's
// test suite. Each case below is a way a producer can get the contract wrong.

#include "Bundle.hpp"
#include "Harness.hpp"
#include "Log.hpp"
#include "Validate.hpp"

#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace hxc;
using namespace hxctest;
using json  = nlohmann::json;
namespace fs = std::filesystem;

namespace {

    bool hasDiagnostic(const CDiagnostics& diags, bool error, std::string_view needle) {
        for (const auto& D : diags.all()) {
            if (D.error == error && (D.message.find(needle) != std::string::npos || D.where.find(needle) != std::string::npos))
                return true;
        }
        return false;
    }

    std::string describe(const CDiagnostics& diags) {
        std::string out;
        for (const auto& D : diags.all())
            out += std::format("\n  [{}] {}: {}", D.error ? "error" : "warn", D.where, D.message);
        return out;
    }

    json minimalManifest() {
        return json{
            {"take_id", "unit"},
            {"host", {{"tool", "test"}}},
            {"sources", {{"overlay", false}, {"app_audio", false}, {"cameras", false}, {"mic", false}}},
            {"notes", json::array()},
        };
    }

    std::string poseLine(double qw) {
        return std::format(R"("pose":{{"pos":[0,0,0],"quat":[0,0,0,{}]}})", qw);
    }

    std::string telemetryLine(int64_t tHostNs, int64_t frame, int eyes = 2, double qw = 1.0, const char* fov = R"({"l":-0.9,"r":0.9,"u":0.8,"d":-0.8})") {
        std::string body;
        for (int i = 0; i < eyes; ++i) {
            if (i > 0)
                body += ",";
            body += std::format(R"({{{},"fov":{}}})", poseLine(qw), fov);
        }
        return std::format(R"({{"t_host_ns":{},"frame":{},"eyes":[{}],"stage_correction":null,"blend_mode":"alpha"}})", tHostNs, frame, body);
    }

    // Writes a take with the given pieces and returns its path.
    fs::path writeTake(const std::string& name, const json& manifest, const std::vector<std::string>& telemetry, const std::vector<std::string>& clock, bool writeClock = true) {
        const fs::path ROOT = scratchRoot() / ("validate-" + name);
        std::error_code ec;
        fs::remove_all(ROOT, ec);
        fs::create_directories(ROOT, ec);

        std::ofstream(ROOT / "manifest.json") << manifest.dump(2) << "\n";
        {
            std::ofstream stream(ROOT / "telemetry.jsonl");
            for (const auto& LINE : telemetry)
                stream << LINE << "\n";
        }
        if (writeClock) {
            std::ofstream stream(ROOT / "clock.jsonl");
            for (const auto& LINE : clock)
                stream << LINE << "\n";
        }
        return ROOT;
    }

    std::vector<std::string> goodTelemetry() {
        return {telemetryLine(1000000000, 0), telemetryLine(1016666666, 1), telemetryLine(1033333333, 2)};
    }

    std::vector<std::string> goodClock() {
        return {R"({"t_host_ns":900000000,"offset_ns":250000000,"rtt_us":1800})", R"({"t_host_ns":1100000000,"offset_ns":250004000,"rtt_us":1750})"};
    }

    CDiagnostics loadAndCollect(const fs::path& root) {
        CDiagnostics diags;
        SBundle::load(root, diags, {});
        return diags;
    }

}

TEST(Validate, AMinimalHostOnlyTakeIsAccepted) {
    const auto ROOT  = writeTake("minimal", minimalManifest(), goodTelemetry(), goodClock());
    const auto DIAGS = loadAndCollect(ROOT);
    EXPECT_FALSE(DIAGS.hasErrors()) << describe(DIAGS);
}

TEST(Validate, AMissingDirectoryIsReportedRatherThanCrashing) {
    CDiagnostics diags;
    EXPECT_FALSE(SBundle::load(scratchRoot() / "does-not-exist", diags, {}).has_value());
    EXPECT_TRUE(hasDiagnostic(diags, true, "not a directory"));

    SValidateOptions options;
    options.root = scratchRoot() / "does-not-exist";
    setLogLevel(eLogLevel::ERR);
    EXPECT_EQ(runValidate(options), 2);
    setLogLevel(eLogLevel::INFO);
}

TEST(Validate, AManifestThatIsNotJsonSaysSo) {
    const auto ROOT = writeTake("badjson", minimalManifest(), goodTelemetry(), goodClock());
    std::ofstream(ROOT / "manifest.json") << "{ this is not json";

    CDiagnostics diags;
    EXPECT_FALSE(SBundle::load(ROOT, diags, {}).has_value());
    EXPECT_TRUE(hasDiagnostic(diags, true, "not valid JSON")) << describe(diags);
}

TEST(Validate, MissingSourcesBlockIsAnError) {
    json manifest = minimalManifest();
    manifest.erase("sources");
    const auto DIAGS = loadAndCollect(writeTake("nosources", manifest, goodTelemetry(), goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "`sources`")) << describe(DIAGS);
}

TEST(Validate, MissingHostBlockIsAnError) {
    json manifest = minimalManifest();
    manifest.erase("host");
    const auto DIAGS = loadAndCollect(writeTake("nohost", manifest, goodTelemetry(), goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "`host`")) << describe(DIAGS);
}

TEST(Validate, ASourceFlagWithNoMediaIsAnError) {
    json manifest             = minimalManifest();
    manifest["sources"]["overlay"] = true;
    manifest["overlay"]       = {{"width", 640}, {"height", 480}, {"format", "rgba"}, {"encoder", "ffv1"}, {"target_hz", 60}, {"eye_count", 2}};
    const auto DIAGS          = loadAndCollect(writeTake("noverlay", manifest, goodTelemetry(), goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "missing, but manifest.sources.overlay is true")) << describe(DIAGS);
}

TEST(Validate, ANonUnitQuaternionIsRejectedWithItsNorm) {
    auto telemetry = goodTelemetry();
    telemetry[1]   = telemetryLine(1016666666, 1, 2, 0.5);
    const auto DIAGS = loadAndCollect(writeTake("badquat", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "norm")) << describe(DIAGS);
}

TEST(Validate, AnInsideOutFrustumIsRejected) {
    auto telemetry = goodTelemetry();
    telemetry[1]   = telemetryLine(1016666666, 1, 2, 1.0, R"({"l":0.9,"r":-0.9,"u":0.8,"d":-0.8})");
    const auto DIAGS = loadAndCollect(writeTake("badfov", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "not a usable frustum")) << describe(DIAGS);
}

TEST(Validate, TimestampsThatDoNotAdvanceAreRejected) {
    auto telemetry = goodTelemetry();
    telemetry[2]   = telemetryLine(1010000000, 2);
    const auto DIAGS = loadAndCollect(writeTake("backwards", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "does not advance")) << describe(DIAGS);
}

TEST(Validate, FrameNumbersThatDoNotAdvanceAreRejected) {
    auto telemetry = goodTelemetry();
    telemetry[2]   = telemetryLine(1033333333, 1);
    const auto DIAGS = loadAndCollect(writeTake("dupframe", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "`frame`")) << describe(DIAGS);
}

TEST(Validate, AnEyeCountMismatchAgainstTheManifestIsRejected) {
    json manifest             = minimalManifest();
    manifest["sources"]["overlay"] = false;
    manifest["overlay"]       = {{"width", 640}, {"height", 480}, {"format", "rgba"}, {"encoder", "ffv1"}, {"target_hz", 60}, {"eye_count", 2}};
    auto telemetry            = goodTelemetry();
    telemetry[1]              = telemetryLine(1016666666, 1, 1);
    const auto DIAGS          = loadAndCollect(writeTake("eyecount", manifest, telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "eye_count")) << describe(DIAGS);
}

TEST(Validate, AMalformedTelemetryLineIsReportedWithItsLineNumber) {
    auto telemetry = goodTelemetry();
    telemetry[1]   = "{not json at all";
    const auto DIAGS = loadAndCollect(writeTake("badline", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "telemetry.jsonl:2")) << describe(DIAGS);
}

TEST(Validate, AMissingClockIsOnlyAWarningForAHostOnlyTake) {
    const auto ROOT  = writeTake("noclock", minimalManifest(), goodTelemetry(), {}, false);
    const auto DIAGS = loadAndCollect(ROOT);
    EXPECT_FALSE(DIAGS.hasErrors()) << describe(DIAGS);
    EXPECT_TRUE(hasDiagnostic(DIAGS, false, "same clock")) << describe(DIAGS);
}

TEST(Validate, AMissingClockIsAnErrorWhenTheTakeClaimsDeviceSources) {
    json manifest                  = minimalManifest();
    manifest["sources"]["cameras"] = true;
    const auto DIAGS               = loadAndCollect(writeTake("noclockcam", manifest, goodTelemetry(), {}, false));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "device timestamps cannot be placed")) << describe(DIAGS);
}

TEST(Validate, ClockCoverageShorterThanTheTakeIsWarnedAbout) {
    const std::vector<std::string> CLOCK{R"({"t_host_ns":1020000000,"offset_ns":250000000,"rtt_us":1800})"};
    const auto                     DIAGS = loadAndCollect(writeTake("shortclock", minimalManifest(), goodTelemetry(), CLOCK));
    EXPECT_FALSE(DIAGS.hasErrors()) << describe(DIAGS);
    EXPECT_TRUE(hasDiagnostic(DIAGS, false, "held constant outside")) << describe(DIAGS);
}

TEST(Validate, TheSyntheticBundleValidatesCleanEndToEnd) {
    SValidateOptions options;
    options.root   = fixture().take;
    options.strict = true;
    EXPECT_EQ(runValidate(options), 0);
}

TEST(Validate, AnAudioSidecarNeedsExactlyOneClockDomain) {
    // Copy the fixture and break only the app audio sidecar.
    const fs::path ROOT = scratchRoot() / "validate-audiodomain";
    std::error_code ec;
    fs::remove_all(ROOT, ec);
    fs::copy(fixture().take, ROOT, fs::copy_options::recursive, ec);
    ASSERT_FALSE(ec) << ec.message();

    std::ofstream(ROOT / "audio" / "app.json") << R"({"start_t_host_ns":1,"start_t_device_ns":2,"sample_rate_hz":48000,"channels":1})";
    const auto BOTH = loadAndCollect(ROOT);
    EXPECT_TRUE(hasDiagnostic(BOTH, true, "one clock domain")) << describe(BOTH);

    std::ofstream(ROOT / "audio" / "app.json") << R"({"sample_rate_hz":48000,"channels":1})";
    const auto NEITHER = loadAndCollect(ROOT);
    EXPECT_TRUE(hasDiagnostic(NEITHER, true, "start_t_host_ns")) << describe(NEITHER);
}

TEST(Validate, ACameraSidecarThatDisagreesWithItsVideoIsRejected) {
    const fs::path ROOT = scratchRoot() / "validate-camcount";
    std::error_code ec;
    fs::remove_all(ROOT, ec);
    fs::copy(fixture().take, ROOT, fs::copy_options::recursive, ec);
    ASSERT_FALSE(ec) << ec.message();

    // Drop the last per-frame record for camera L.
    fs::path sidecar;
    for (const auto& ENTRY : fs::directory_iterator(ROOT / "cameras")) {
        if (ENTRY.path().extension() == ".jsonl")
            sidecar = ENTRY.path();
    }
    ASSERT_FALSE(sidecar.empty());

    std::vector<std::string> lines;
    {
        std::ifstream stream(sidecar);
        std::string   line;
        while (std::getline(stream, line))
            lines.push_back(line);
    }
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
        if (it->find("\"cam\":\"L\"") != std::string::npos) {
            lines.erase(std::next(it).base());
            break;
        }
    }
    {
        std::ofstream stream(sidecar);
        for (const auto& LINE : lines)
            stream << LINE << "\n";
    }

    const auto DIAGS = loadAndCollect(ROOT);
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "one-for-one in decode order")) << describe(DIAGS);
}

TEST(Validate, AnOverlayVideoWhoseSizeContradictsTheManifestIsRejected) {
    const fs::path ROOT = scratchRoot() / "validate-overlaysize";
    std::error_code ec;
    fs::remove_all(ROOT, ec);
    fs::copy(fixture().take, ROOT, fs::copy_options::recursive, ec);
    ASSERT_FALSE(ec) << ec.message();

    std::ifstream in(ROOT / "manifest.json");
    json          manifest;
    in >> manifest;
    in.close();
    manifest["overlay"]["width"] = 1234;
    std::ofstream(ROOT / "manifest.json") << manifest.dump(2);

    const auto DIAGS = loadAndCollect(ROOT);
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "manifest.overlay says")) << describe(DIAGS);
}

TEST(Validate, AnUnknownAlphaAssociationIsRejected) {
    const fs::path ROOT = scratchRoot() / "validate-alpha";
    std::error_code ec;
    fs::remove_all(ROOT, ec);
    fs::copy(fixture().take, ROOT, fs::copy_options::recursive, ec);
    ASSERT_FALSE(ec) << ec.message();

    std::ifstream in(ROOT / "manifest.json");
    json          manifest;
    in >> manifest;
    in.close();
    manifest["overlay"]["alpha"] = "associated";
    std::ofstream(ROOT / "manifest.json") << manifest.dump(2);

    const auto DIAGS = loadAndCollect(ROOT);
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "straight")) << describe(DIAGS);
}

TEST(Validate, TheOrdinalAlignmentCountIsEnforced) {
    // The rule: the n-th frame of each eye's video is the n-th telemetry record
    // without "dropped". Flip one record's flag and the counts no longer agree.
    const fs::path ROOT = scratchRoot() / "validate-ordinal";
    std::error_code ec;
    fs::remove_all(ROOT, ec);
    fs::copy(fixture().take, ROOT, fs::copy_options::recursive, ec);
    ASSERT_FALSE(ec) << ec.message();

    std::vector<std::string> lines;
    {
        std::ifstream stream(ROOT / "telemetry.jsonl");
        std::string   line;
        while (std::getline(stream, line))
            lines.push_back(line);
    }
    ASSERT_GT(lines.size(), 4u);

    bool flipped = false;
    for (auto& line : lines) {
        const size_t AT = line.find("\"dropped\":false");
        if (AT == std::string::npos)
            continue;
        line.replace(AT, std::strlen("\"dropped\":false"), "\"dropped\":true");
        flipped = true;
        break;
    }
    ASSERT_TRUE(flipped) << "the fixture should carry explicit dropped flags";
    {
        std::ofstream stream(ROOT / "telemetry.jsonl");
        for (const auto& LINE : lines)
            stream << LINE << "\n";
    }

    const auto DIAGS = loadAndCollect(ROOT);
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "the n-th frame is the n-th undropped record")) << describe(DIAGS);
}

TEST(Validate, ANonBooleanDroppedFlagIsRejected) {
    auto telemetry = goodTelemetry();
    telemetry[1]   = std::format(R"({{"t_host_ns":1016666666,"frame":1,"eyes":[{{{},"fov":{{"l":-0.9,"r":0.9,"u":0.8,"d":-0.8}}}},{{{},"fov":{{"l":-0.9,"r":0.9,"u":0.8,"d":-0.8}}}}],"dropped":"yes","blend_mode":"alpha"}})",
                                 poseLine(1.0), poseLine(1.0));
    const auto DIAGS = loadAndCollect(writeTake("droppedtype", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "`dropped` must be a boolean")) << describe(DIAGS);
}

TEST(Validate, AnObsoletePtsEpochIsCalledOut) {
    const fs::path ROOT = scratchRoot() / "validate-ptsepoch";
    std::error_code ec;
    fs::remove_all(ROOT, ec);
    fs::copy(fixture().take, ROOT, fs::copy_options::recursive, ec);
    ASSERT_FALSE(ec) << ec.message();

    std::ifstream in(ROOT / "manifest.json");
    json          manifest;
    in >> manifest;
    in.close();
    manifest["overlay"]["pts_epoch_ns"] = 14400000000000LL;
    std::ofstream(ROOT / "manifest.json") << manifest.dump(2);

    const auto DIAGS = loadAndCollect(ROOT);
    EXPECT_FALSE(DIAGS.hasErrors()) << describe(DIAGS);
    EXPECT_TRUE(hasDiagnostic(DIAGS, false, "obsolete")) << describe(DIAGS);
}

TEST(Validate, StrictTurnsWarningsIntoAFailure) {
    // A take whose clock does not span it warns; --strict makes that fatal.
    const std::vector<std::string> CLOCK{R"({"t_host_ns":1020000000,"offset_ns":250000000,"rtt_us":1800})"};
    const auto                     ROOT = writeTake("strict", minimalManifest(), goodTelemetry(), CLOCK);

    SValidateOptions options;
    options.root = ROOT;
    setLogLevel(eLogLevel::ERR);
    EXPECT_EQ(runValidate(options), 0);
    options.strict = true;
    EXPECT_EQ(runValidate(options), 1);
    setLogLevel(eLogLevel::INFO);
}
