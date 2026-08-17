// validate is the format's arbiter, so its error cases are the specification's
// test suite. Each case below is a way a producer can get the contract wrong.

#include "Bundle.hpp"
#include "Harness.hpp"
#include "Log.hpp"
#include "Validate.hpp"

#include <array>
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

namespace {

    // A telemetry line with one quad record, whose fields the caller can break.
    std::string quadTelemetryLine(int64_t tHostNs, int64_t frame, const std::string& quadBody) {
        return std::format(R"({{"t_host_ns":{},"frame":{},"eyes":[{{{},"fov":{{"l":-0.9,"r":0.9,"u":0.8,"d":-0.8}}}},{{{},"fov":{{"l":-0.9,"r":0.9,"u":0.8,"d":-0.8}}}}],)"
                           R"("head":{{"pos":[0,0,0],"quat":[0,0,0,1]}},"quads":[{}],"blend_mode":"alpha"}})",
                           tHostNs, frame, poseLine(1.0), poseLine(1.0), quadBody);
    }

    // The canonical spelling: `visibility` is an XrEyeVisibility string.
    constexpr const char* GOOD_QUAD = R"({"index":0,"name":null,"pose":{"pos":[0,0,-1],"quat":[0,0,0,1]},"size":[0.7,0.42],"visibility":"both","view_space":false,)"
                                      R"("swapchain":7,"image":0,"array_layer":0,"rect":[0,0,640,480]})";

    // Same record with `visibility` replaced by whatever the caller wants to try.
    std::string quadWithVisibility(const std::string& spelling) {
        std::string  quad = GOOD_QUAD;
        const size_t AT   = quad.find(R"("visibility":"both")");
        quad.replace(AT, std::strlen(R"("visibility":"both")"), std::format(R"("visibility":{})", spelling));
        return quad;
    }

    std::vector<std::string> quadTelemetry(const std::string& quadBody) {
        return {quadTelemetryLine(1000000000, 0, quadBody), quadTelemetryLine(1016666666, 1, quadBody), quadTelemetryLine(1033333333, 2, quadBody)};
    }

}

TEST(Validate, AWellFormedQuadRecordIsAccepted) {
    const auto DIAGS = loadAndCollect(writeTake("quadgood", minimalManifest(), quadTelemetry(GOOD_QUAD), goodClock()));
    EXPECT_FALSE(DIAGS.hasErrors()) << describe(DIAGS);
}

TEST(Validate, AQuadWithoutViewSpaceIsRejected) {
    std::string quad = GOOD_QUAD;
    const size_t AT  = quad.find(R"("view_space":false,)");
    ASSERT_NE(AT, std::string::npos);
    quad.erase(AT, std::strlen(R"("view_space":false,)"));

    const auto DIAGS = loadAndCollect(writeTake("quadnoview", minimalManifest(), quadTelemetry(quad), goodClock()));
    // view_space decides whether a layer is head-locked or room-anchored, so a
    // record without it cannot be replayed at all.
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "`view_space`")) << describe(DIAGS);
}

TEST(Validate, AQuadWithoutAPoseIsRejectedWithTheHeadRelativeSemanticsSpelledOut) {
    const std::string QUAD = R"({"index":0,"name":null,"size":[0.7,0.42],"visibility":"both","view_space":false,"swapchain":7,"image":0})";
    const auto        DIAGS = loadAndCollect(writeTake("quadnopose", minimalManifest(), quadTelemetry(QUAD), goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "head-relative")) << describe(DIAGS);
}

TEST(Validate, AQuadWithANonPositiveSizeIsRejected) {
    std::string quad = GOOD_QUAD;
    const size_t AT  = quad.find(R"("size":[0.7,0.42])");
    ASSERT_NE(AT, std::string::npos);
    quad.replace(AT, std::strlen(R"("size":[0.7,0.42])"), R"("size":[0.7,0])");

    const auto DIAGS = loadAndCollect(writeTake("quadbadsize", minimalManifest(), quadTelemetry(quad), goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "positive in both axes")) << describe(DIAGS);
}

TEST(Validate, AQuadIndexThatDisagreesWithCompositionOrderIsRejected) {
    std::string quad = GOOD_QUAD;
    const size_t AT  = quad.find(R"("index":0)");
    ASSERT_NE(AT, std::string::npos);
    quad.replace(AT, std::strlen(R"("index":0)"), R"("index":3)");

    const auto DIAGS = loadAndCollect(writeTake("quadbadindex", minimalManifest(), quadTelemetry(quad), goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "composition order back-to-front")) << describe(DIAGS);
}

// `visibility` is an XrEyeVisibility. The producer emits one of four strings,
// and reading a string as "not a visibility" is what made v1 reject every real
// take ever recorded - so each spelling gets an assertion.
TEST(Validate, QuadEyeVisibilityReadsAllFourSpellings) {
    const std::array<std::pair<const char*, eEyeVisibility>, 4> CASES{{
        {R"("both")", eEyeVisibility::BOTH},
        {R"("left")", eEyeVisibility::LEFT},
        {R"("right")", eEyeVisibility::RIGHT},
        {R"("none")", eEyeVisibility::NONE},
    }};

    for (const auto& [SPELLING, EXPECTED] : CASES) {
        CDiagnostics diags;
        const auto   BUNDLE = SBundle::load(writeTake(std::format("quadvis{}", static_cast<int>(EXPECTED)), minimalManifest(), quadTelemetry(quadWithVisibility(SPELLING)), goodClock()), diags,
                                            SLoadOptions{.probeMedia = false, .probeDepth = eProbeDepth::INDEX, .checksum = false, .probeCache = nullptr});
        ASSERT_TRUE(BUNDLE.has_value()) << SPELLING;
        EXPECT_FALSE(diags.hasErrors()) << SPELLING << describe(diags);
        ASSERT_FALSE(BUNDLE->telemetry.empty());
        ASSERT_EQ(BUNDLE->telemetry.front().quads.size(), 1u);
        EXPECT_EQ(BUNDLE->telemetry.front().quads.front().eyeVisibility, EXPECTED) << SPELLING;
    }
}

TEST(Validate, QuadEyeVisibilityDecidesWhichEyeComposedTheLayer) {
    CDiagnostics diags;
    const auto   BUNDLE = SBundle::load(writeTake("quadviseye", minimalManifest(), quadTelemetry(quadWithVisibility(R"("right")")), goodClock()), diags, SLoadOptions{.probeMedia = false, .probeDepth = eProbeDepth::INDEX, .checksum = false, .probeCache = nullptr});
    ASSERT_TRUE(BUNDLE.has_value());
    const auto& QUAD = BUNDLE->telemetry.front().quads.front();
    EXPECT_FALSE(QUAD.composedInEye(0)) << "a right-eye layer is not in the left eye's view";
    EXPECT_TRUE(QUAD.composedInEye(1));

    CDiagnostics none;
    const auto   NOWHERE = SBundle::load(writeTake("quadvisnone", minimalManifest(), quadTelemetry(quadWithVisibility(R"("none")")), goodClock()), none, SLoadOptions{.probeMedia = false, .probeDepth = eProbeDepth::INDEX, .checksum = false, .probeCache = nullptr});
    ASSERT_TRUE(NOWHERE.has_value());
    EXPECT_FALSE(NOWHERE->telemetry.front().quads.front().composedInEye(0));
    EXPECT_FALSE(NOWHERE->telemetry.front().quads.front().composedInEye(1));
}

TEST(Validate, AnUnknownQuadVisibilityStringIsRejectedRatherThanTreatedAsBoth) {
    const auto DIAGS = loadAndCollect(writeTake("quadvisjunk", minimalManifest(), quadTelemetry(quadWithVisibility(R"("stereo")")), goodClock()));
    // Guessing "both" would put a one-eye layer in front of both eyes, which is
    // worse than refusing the file.
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "XrEyeVisibility")) << describe(DIAGS);
}

TEST(Validate, TheDeprecatedNumericVisibilityIsStillReadButWarns) {
    CDiagnostics diags;
    const auto   BUNDLE = SBundle::load(writeTake("quadvisnum", minimalManifest(), quadTelemetry(quadWithVisibility("0.5")), goodClock()), diags, SLoadOptions{.probeMedia = false, .probeDepth = eProbeDepth::INDEX, .checksum = false, .probeCache = nullptr});
    ASSERT_TRUE(BUNDLE.has_value());
    EXPECT_FALSE(diags.hasErrors()) << describe(diags);
    EXPECT_TRUE(hasDiagnostic(diags, false, "XrEyeVisibility")) << describe(diags);
    const auto& QUAD = BUNDLE->telemetry.front().quads.front();
    EXPECT_DOUBLE_EQ(QUAD.visibility, 0.5) << "the old spelling still means an opacity";
    EXPECT_EQ(QUAD.eyeVisibility, eEyeVisibility::BOTH) << "and leaves the eye mask alone";

    const auto BOOLEAN = loadAndCollect(writeTake("quadvisbool", minimalManifest(), quadTelemetry(quadWithVisibility("true")), goodClock()));
    EXPECT_FALSE(BOOLEAN.hasErrors()) << describe(BOOLEAN);
    EXPECT_TRUE(hasDiagnostic(BOOLEAN, false, "XrEyeVisibility")) << describe(BOOLEAN);
}

TEST(Validate, AQuadVisibilityOutsideZeroToOneIsRejected) {
    const auto DIAGS = loadAndCollect(writeTake("quadbadvis", minimalManifest(), quadTelemetry(quadWithVisibility("1.4")), goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "0..1")) << describe(DIAGS);
}

TEST(Validate, ATakeWithoutQuadRecordsWarnsThatGradeBReplayIsForeclosed) {
    const auto DIAGS = loadAndCollect(writeTake("noquads", minimalManifest(), goodTelemetry(), goodClock()));
    EXPECT_FALSE(DIAGS.hasErrors()) << describe(DIAGS);
    EXPECT_TRUE(hasDiagnostic(DIAGS, false, "grade-B replay")) << describe(DIAGS);
}

TEST(Validate, QuadsOnSomeRecordsButNotOthersIsAProducerBug) {
    auto telemetry = quadTelemetry(GOOD_QUAD);
    telemetry[1]   = telemetryLine(1016666666, 1);
    const auto DIAGS = loadAndCollect(writeTake("quadpartial", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "must be present on all of them or none")) << describe(DIAGS);
}

TEST(Validate, AHeadPoseOnSomeRecordsButNotOthersIsRejected) {
    auto telemetry = quadTelemetry(GOOD_QUAD);
    // Strip the head from the middle record only.
    const size_t AT = telemetry[1].find(R"("head":{"pos":[0,0,0],"quat":[0,0,0,1]},)");
    ASSERT_NE(AT, std::string::npos);
    telemetry[1].erase(AT, std::strlen(R"("head":{"pos":[0,0,0],"quat":[0,0,0,1]},)"));

    const auto DIAGS = loadAndCollect(writeTake("headpartial", minimalManifest(), telemetry, goodClock()));
    EXPECT_TRUE(hasDiagnostic(DIAGS, true, "`head` pose; it must be present on all")) << describe(DIAGS);
}

TEST(Validate, ATakeWithoutHeadPosesFallsBackToTheEyeMidpointAndSaysSo) {
    const auto DIAGS = loadAndCollect(writeTake("nohead", minimalManifest(), goodTelemetry(), goodClock()));
    EXPECT_FALSE(DIAGS.hasErrors()) << describe(DIAGS);
    EXPECT_TRUE(hasDiagnostic(DIAGS, false, "midpoint of the eyes")) << describe(DIAGS);
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
