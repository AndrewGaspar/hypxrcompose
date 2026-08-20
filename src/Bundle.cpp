#include "Bundle.hpp"
#include "Log.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string_view>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace hxc {

    bool CDiagnostics::hasErrors() const {
        return errorCount() > 0;
    }

    size_t CDiagnostics::errorCount() const {
        return static_cast<size_t>(std::count_if(m_diags.begin(), m_diags.end(), [](const SDiag& d) { return d.error; }));
    }

    size_t CDiagnostics::warningCount() const {
        return m_diags.size() - errorCount();
    }

    void CDiagnostics::print() const {
        // One schema disagreement in telemetry.jsonl is one diagnostic per record,
        // and a real take holds thousands of records. Printing all of them buries
        // the *other* diagnostics under half a megabyte of the same sentence, so
        // repeats of a message are collapsed after a few. The counts reported by
        // errorCount()/warningCount() are untouched: this is a printing decision,
        // not a filtering one, and --json still emits every diagnostic.
        constexpr size_t                SHOWN_PER_MESSAGE = 3;
        std::map<std::string_view, size_t> seen;

        for (const auto& D : m_diags) {
            const size_t COUNT = ++seen[D.message];
            if (COUNT <= SHOWN_PER_MESSAGE)
                logf(D.error ? eLogLevel::ERR : eLogLevel::WARN, "{}: {}", D.where, D.message);
        }
        for (const auto& [MESSAGE, COUNT] : seen) {
            if (COUNT > SHOWN_PER_MESSAGE)
                logf(eLogLevel::WARN, "(and {} more like it): {}", COUNT - SHOWN_PER_MESSAGE, MESSAGE);
        }
    }

    std::string toString(eEyeVisibility visibility) {
        switch (visibility) {
            case eEyeVisibility::LEFT: return "left";
            case eEyeVisibility::RIGHT: return "right";
            case eEyeVisibility::NONE: return "none";
            default: return "both";
        }
    }

    SPose STelemetryFrame::headPose() const {
        if (head)
            return *head;
        if (eyes.empty())
            return {};
        if (eyes.size() == 1)
            return eyes[0].pose;

        SPose midpoint;
        midpoint.pos = (eyes[0].pose.pos + eyes[1].pose.pos) * 0.5;
        midpoint.rot = slerp(eyes[0].pose.rot, eyes[1].pose.rot, 0.5);
        return midpoint;
    }

    int64_t SBundle::medianTelemetryIntervalNs() const {
        if (telemetryHostNs.size() < 2)
            return 0;
        std::vector<int64_t> deltas;
        deltas.reserve(telemetryHostNs.size() - 1);
        for (size_t i = 1; i < telemetryHostNs.size(); ++i)
            deltas.push_back(telemetryHostNs[i] - telemetryHostNs[i - 1]);
        std::nth_element(deltas.begin(), deltas.begin() + static_cast<long>(deltas.size() / 2), deltas.end());
        return deltas[deltas.size() / 2];
    }

    const SCamera* SBundle::cameraForEye(int eye) const {
        for (const auto& CAM : cameras) {
            if (CAM.eye == eye)
                return &CAM;
        }
        return nullptr;
    }

    namespace {

        // Keeps a repeated per-record complaint from burying the one-line problems.
        struct SCap {
            size_t emitted    = 0;
            size_t suppressed = 0;
            size_t limit      = 5;

            bool   allow() {
                if (emitted < limit) {
                    ++emitted;
                    return true;
                }
                ++suppressed;
                return false;
            }
            void flush(CDiagnostics& diags, const std::string& where, const std::string& what) {
                if (suppressed > 0)
                    diags.warn(where, "... and {} more record(s) with {}", suppressed, what);
            }
        };

        bool readWholeFile(const fs::path& path, std::string& out, std::string& error) {
            std::ifstream stream(path, std::ios::binary);
            if (!stream) {
                error = "cannot be opened";
                return false;
            }
            std::ostringstream buffer;
            buffer << stream.rdbuf();
            out = buffer.str();
            return true;
        }

        struct SJsonLine {
            int         number = 0;
            std::string text;
        };

        bool readJsonLines(const fs::path& path, std::vector<SJsonLine>& out, std::string& error) {
            std::ifstream stream(path);
            if (!stream) {
                error = "cannot be opened";
                return false;
            }
            std::string line;
            int         number = 0;
            while (std::getline(stream, line)) {
                ++number;
                // Trailing whitespace and blank lines are tolerated: a producer that
                // flushes line-at-a-time will sometimes leave a partial final line,
                // and that is a warning at worst, not a parse failure.
                const size_t END = line.find_last_not_of(" \t\r");
                if (END == std::string::npos)
                    continue;
                out.push_back({number, line.substr(0, END + 1)});
            }
            return true;
        }

        const json* member(const json& node, const char* key) {
            if (!node.is_object())
                return nullptr;
            const auto IT = node.find(key);
            return IT == node.end() ? nullptr : &(*IT);
        }

        std::string typeName(const json& node) {
            return node.type_name();
        }

        bool wantNumber(const json& node, const char* key, const std::string& where, CDiagnostics& diags, double& out, bool required = true) {
            const json* FOUND = member(node, key);
            if (!FOUND) {
                if (required)
                    diags.error(where, "missing required number `{}`", key);
                return false;
            }
            if (!FOUND->is_number()) {
                diags.error(where, "`{}` must be a number, found {}", key, typeName(*FOUND));
                return false;
            }
            out = FOUND->get<double>();
            if (!std::isfinite(out)) {
                diags.error(where, "`{}` is not finite", key);
                return false;
            }
            return true;
        }

        bool wantInt(const json& node, const char* key, const std::string& where, CDiagnostics& diags, int64_t& out, bool required = true) {
            const json* FOUND = member(node, key);
            if (!FOUND) {
                if (required)
                    diags.error(where, "missing required integer `{}`", key);
                return false;
            }
            if (!FOUND->is_number_integer()) {
                if (FOUND->is_number()) {
                    diags.error(where, "`{}` must be an integer, found the fractional value {}", key, FOUND->get<double>());
                    return false;
                }
                diags.error(where, "`{}` must be an integer, found {}", key, typeName(*FOUND));
                return false;
            }
            out = FOUND->get<int64_t>();
            return true;
        }

        bool wantString(const json& node, const char* key, const std::string& where, CDiagnostics& diags, std::string& out, bool required = true) {
            const json* FOUND = member(node, key);
            if (!FOUND) {
                if (required)
                    diags.error(where, "missing required string `{}`", key);
                return false;
            }
            if (!FOUND->is_string()) {
                diags.error(where, "`{}` must be a string, found {}", key, typeName(*FOUND));
                return false;
            }
            out = FOUND->get<std::string>();
            return true;
        }

        bool wantBool(const json& node, const char* key, const std::string& where, CDiagnostics& diags, bool& out, bool required = true) {
            const json* FOUND = member(node, key);
            if (!FOUND) {
                if (required)
                    diags.error(where, "missing required boolean `{}`", key);
                return false;
            }
            if (!FOUND->is_boolean()) {
                diags.error(where, "`{}` must be a boolean, found {}", key, typeName(*FOUND));
                return false;
            }
            out = FOUND->get<bool>();
            return true;
        }

        bool wantDoubleArray(const json& node, const char* key, size_t count, const std::string& where, CDiagnostics& diags, std::vector<double>& out) {
            const json* FOUND = member(node, key);
            if (!FOUND) {
                diags.error(where, "missing required array `{}`", key);
                return false;
            }
            if (!FOUND->is_array()) {
                diags.error(where, "`{}` must be an array, found {}", key, typeName(*FOUND));
                return false;
            }
            if (count != 0 && FOUND->size() != count) {
                diags.error(where, "`{}` must hold {} numbers, found {}", key, count, FOUND->size());
                return false;
            }
            out.clear();
            for (size_t i = 0; i < FOUND->size(); ++i) {
                const json& ELEMENT = (*FOUND)[i];
                if (!ELEMENT.is_number() || !std::isfinite(ELEMENT.get<double>())) {
                    diags.error(where, "`{}`[{}] must be a finite number", key, i);
                    return false;
                }
                out.push_back(ELEMENT.get<double>());
            }
            return true;
        }

        std::optional<SPose> parsePose(const json& node, const std::string& where, CDiagnostics& diags) {
            if (!node.is_object()) {
                diags.error(where, "a pose must be an object with `pos` and `quat`, found {}", typeName(node));
                return std::nullopt;
            }

            std::vector<double> pos, quat;
            if (!wantDoubleArray(node, "pos", 3, where, diags, pos))
                return std::nullopt;
            if (!wantDoubleArray(node, "quat", 4, where, diags, quat))
                return std::nullopt;

            SPose pose;
            pose.pos = {pos[0], pos[1], pos[2]};
            pose.rot = {quat[0], quat[1], quat[2], quat[3]};

            const double NORM = pose.rot.norm();
            if (!(NORM > 0.0)) {
                diags.error(where, "`quat` is the zero quaternion");
                return std::nullopt;
            }
            if (std::abs(NORM - 1.0) > 1e-3) {
                diags.error(where, "`quat` has norm {:.6f}; unit quaternions are required (xyzw order)", NORM);
                return std::nullopt;
            }
            pose.rot = pose.rot.normalized();
            return pose;
        }

        std::optional<SFov> parseFov(const json& node, const std::string& where, CDiagnostics& diags) {
            if (!node.is_object()) {
                diags.error(where, "an fov must be an object with `l`,`r`,`u`,`d`, found {}", typeName(node));
                return std::nullopt;
            }
            SFov fov;
            if (!wantNumber(node, "l", where, diags, fov.l) || !wantNumber(node, "r", where, diags, fov.r) || !wantNumber(node, "u", where, diags, fov.u) ||
                !wantNumber(node, "d", where, diags, fov.d))
                return std::nullopt;
            if (!fov.sane()) {
                diags.error(where, "fov (l={:.4f} r={:.4f} u={:.4f} d={:.4f}) is not a usable frustum; angles are radians with l<r, d<u, and |angle| < 1.55", fov.l, fov.r, fov.u, fov.d);
                return std::nullopt;
            }
            return fov;
        }

        // "left", "XR_EYE_VISIBILITY_LEFT", "Left" - all the same enumerant. An
        // unknown spelling returns nullopt rather than silently meaning "both",
        // because a layer shown to the wrong eye is worse than a loud failure.
        std::optional<eEyeVisibility> parseEyeVisibility(std::string text) {
            std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            constexpr std::string_view PREFIX = "xr_eye_visibility_";
            if (text.starts_with(PREFIX))
                text = text.substr(PREFIX.size());
            if (text == "both")
                return eEyeVisibility::BOTH;
            if (text == "left")
                return eEyeVisibility::LEFT;
            if (text == "right")
                return eEyeVisibility::RIGHT;
            if (text == "none")
                return eEyeVisibility::NONE;
            return std::nullopt;
        }

        // Reads one composition-layer record. The fields the producer named are
        // required; the shapes it did not pin down are accepted in every plausible
        // spelling and flagged in README.
        std::optional<SQuadRecord> parseQuad(const json& node, const std::string& where, CDiagnostics& diags) {
            if (!node.is_object()) {
                diags.error(where, "a quad record must be an object, found {}", typeName(node));
                return std::nullopt;
            }

            SQuadRecord quad;
            if (!wantInt(node, "index", where, diags, quad.index))
                return std::nullopt;

            if (const json* NAME = member(node, "name")) {
                if (NAME->is_string())
                    quad.name = NAME->get<std::string>();
                else if (!NAME->is_null()) {
                    diags.error(where, "`name` must be a string or null, found {}", typeName(*NAME));
                    return std::nullopt;
                }
            }

            const json* POSE = member(node, "pose");
            if (!POSE) {
                diags.error(where, "missing required `pose` (head-relative: the layer's STAGE pose is head * pose)");
                return std::nullopt;
            }
            const auto PARSED = parsePose(*POSE, where + " /pose", diags);
            if (!PARSED)
                return std::nullopt;
            quad.pose = *PARSED;

            // INTERPRETATION: `size` is in metres, but its shape is unstated. Both a
            // two-number array and an object are read.
            const json* SIZE = member(node, "size");
            if (!SIZE) {
                diags.error(where, "missing required `size` (quad extent in metres)");
                return std::nullopt;
            }
            if (SIZE->is_array()) {
                std::vector<double> extent;
                if (!wantDoubleArray(node, "size", 2, where, diags, extent))
                    return std::nullopt;
                quad.width  = extent[0];
                quad.height = extent[1];
            } else if (SIZE->is_object()) {
                const bool WIDE = member(*SIZE, "width") != nullptr;
                if (!wantNumber(*SIZE, WIDE ? "width" : "w", where + " /size", diags, quad.width) || !wantNumber(*SIZE, WIDE ? "height" : "h", where + " /size", diags, quad.height))
                    return std::nullopt;
            } else {
                diags.error(where, "`size` must be [width, height] in metres or an object with width/height, found {}", typeName(*SIZE));
                return std::nullopt;
            }
            if (!(quad.width > 0.0) || !(quad.height > 0.0)) {
                diags.error(where, "`size` must be positive in both axes, found {} x {} m", quad.width, quad.height);
                return std::nullopt;
            }

            // PINNED: `visibility` is an XrEyeVisibility, spelled as one of the
            // four strings "both", "left", "right", "none". It says *which eye the
            // layer was composed into*, not how opaque it was - a stereo-depth
            // desktop submits a per-eye pair sharing one pose and taking opposite
            // halves of a side-by-side swapchain, and a HUD submits "both".
            //
            // A boolean or a 0..1 number is the deprecated spelling, read as an
            // opacity with the eye mask left at BOTH. It is still accepted, with a
            // warning, because bundles written against the earlier reading exist.
            //
            // v1 rejected strings outright, which meant it rejected every real
            // take ever recorded; see README's interpretations table.
            if (const json* VISIBILITY = member(node, "visibility")) {
                if (VISIBILITY->is_string()) {
                    const auto PARSED_EYES = parseEyeVisibility(VISIBILITY->get<std::string>());
                    if (!PARSED_EYES) {
                        diags.error(where, "`visibility` is the string \"{}\"; it is an XrEyeVisibility and must be `both`, `left`, `right`, or `none`", VISIBILITY->get<std::string>());
                        return std::nullopt;
                    }
                    quad.eyeVisibility = *PARSED_EYES;
                } else if (VISIBILITY->is_boolean()) {
                    quad.visibility = VISIBILITY->get<bool>() ? 1.0 : 0.0;
                    diags.warn(where, "`visibility` is a boolean; it is an XrEyeVisibility and the spelling is one of `both`/`left`/`right`/`none`. Reading it as an opacity for compatibility "
                                      "with bundles written before that was settled");
                } else if (VISIBILITY->is_number()) {
                    quad.visibility = VISIBILITY->get<double>();
                    if (!(quad.visibility >= 0.0 && quad.visibility <= 1.0)) {
                        diags.error(where, "`visibility` is {}; a numeric visibility is the deprecated opacity spelling and must be in 0..1", quad.visibility);
                        return std::nullopt;
                    }
                    diags.warn(where, "`visibility` is a number; it is an XrEyeVisibility and the spelling is one of `both`/`left`/`right`/`none`. Reading it as an opacity for compatibility "
                                      "with bundles written before that was settled");
                } else {
                    diags.error(where, "`visibility` must be one of `both`/`left`/`right`/`none` (or, deprecated, a boolean or a number in 0..1), found {}", typeName(*VISIBILITY));
                    return std::nullopt;
                }
            } else {
                diags.error(where, "missing required `visibility`");
                return std::nullopt;
            }

            if (!wantBool(node, "view_space", where, diags, quad.viewSpace))
                return std::nullopt;

            wantInt(node, "swapchain", where, diags, quad.swapchain);
            wantInt(node, "image", where, diags, quad.image);
            wantInt(node, "array_layer", where, diags, quad.arrayLayer, false);
            if (quad.arrayLayer < 0) {
                diags.error(where, "`array_layer` is negative ({})", quad.arrayLayer);
                return std::nullopt;
            }

            // INTERPRETATION: `rect` is the sub-image the layer samples, in swapchain
            // pixels. Array and object spellings are both read.
            if (const json* RECT = member(node, "rect"); RECT && !RECT->is_null()) {
                if (RECT->is_array()) {
                    std::vector<double> values;
                    if (!wantDoubleArray(node, "rect", 4, where, diags, values))
                        return std::nullopt;
                    quad.rect    = {values[0], values[1], values[2], values[3]};
                    quad.hasRect = true;
                } else if (RECT->is_object()) {
                    const bool WIDE = member(*RECT, "width") != nullptr;
                    double     x = 0.0, y = 0.0, w = 0.0, h = 0.0;
                    if (!wantNumber(*RECT, "x", where + " /rect", diags, x) || !wantNumber(*RECT, "y", where + " /rect", diags, y) ||
                        !wantNumber(*RECT, WIDE ? "width" : "w", where + " /rect", diags, w) || !wantNumber(*RECT, WIDE ? "height" : "h", where + " /rect", diags, h))
                        return std::nullopt;
                    quad.rect    = {x, y, w, h};
                    quad.hasRect = true;
                } else {
                    diags.error(where, "`rect` must be [x, y, w, h] in swapchain pixels or an object, found {}", typeName(*RECT));
                    return std::nullopt;
                }
                if (!(quad.rect[2] > 0.0) || !(quad.rect[3] > 0.0)) {
                    diags.error(where, "`rect` has a non-positive extent ({} x {})", quad.rect[2], quad.rect[3]);
                    return std::nullopt;
                }
            }

            return quad;
        }

        // INTERPRETATION: the contract names camera files `-camL`/`-camR` but does
        // not say what the per-frame `cam` field holds. Everything plausible is
        // accepted and normalized to "L"/"R" with eye 0/1.
        std::optional<std::pair<std::string, int>> normalizeCameraKey(const json& value) {
            if (value.is_number_integer()) {
                const int64_t INDEX = value.get<int64_t>();
                if (INDEX == 0)
                    return std::pair<std::string, int>{"L", 0};
                if (INDEX == 1)
                    return std::pair<std::string, int>{"R", 1};
                return std::nullopt;
            }
            if (!value.is_string())
                return std::nullopt;

            std::string text = value.get<std::string>();
            std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (text == "l" || text == "left" || text == "cam0" || text == "caml" || text == "0" || text == "eye0")
                return std::pair<std::string, int>{"L", 0};
            if (text == "r" || text == "right" || text == "cam1" || text == "camr" || text == "1" || text == "eye1")
                return std::pair<std::string, int>{"R", 1};
            return std::nullopt;
        }

        std::vector<fs::path> globSuffix(const fs::path& directory, std::string_view suffix) {
            std::vector<fs::path> found;
            std::error_code       ec;
            if (!fs::is_directory(directory, ec))
                return found;
            for (const auto& ENTRY : fs::directory_iterator(directory, ec)) {
                if (!ENTRY.is_regular_file())
                    continue;
                const std::string NAME = ENTRY.path().filename().string();
                if (NAME.size() >= suffix.size() && NAME.compare(NAME.size() - suffix.size(), suffix.size(), suffix) == 0)
                    found.push_back(ENTRY.path());
            }
            std::sort(found.begin(), found.end());
            return found;
        }

        bool containsCaseInsensitive(const std::string& haystack, std::string_view needle) {
            std::string lower = haystack;
            std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            std::string want{needle};
            std::transform(want.begin(), want.end(), want.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return lower.find(want) != std::string::npos;
        }

    }

    std::optional<SBundle> SBundle::load(const fs::path& root, CDiagnostics& diags, const SLoadOptions& options) {
        std::error_code ec;
        if (!fs::is_directory(root, ec)) {
            diags.error(root.string(), "not a directory; a .hypxrtake bundle is a directory of files");
            return std::nullopt;
        }

        SBundle bundle;
        bundle.root = root;

        // ---- manifest.json ------------------------------------------------------
        const fs::path MANIFEST_PATH = root / "manifest.json";
        std::string    manifestText, readError;
        if (!readWholeFile(MANIFEST_PATH, manifestText, readError)) {
            diags.error("manifest.json", "{}", readError);
            return std::nullopt;
        }
        bundle.manifestText = manifestText;

        json manifest;
        try {
            manifest = json::parse(manifestText);
        } catch (const std::exception& e) {
            diags.error("manifest.json", "is not valid JSON: {}", e.what());
            return std::nullopt;
        }
        if (!manifest.is_object()) {
            diags.error("manifest.json", "the top level must be an object, found {}", typeName(manifest));
            return std::nullopt;
        }

        wantString(manifest, "take_id", "manifest.json", diags, bundle.takeId);

        if (const json* HOST = member(manifest, "host"); !HOST)
            diags.error("manifest.json", "missing required object `host`");
        else if (!HOST->is_object())
            diags.error("manifest.json /host", "must be an object, found {}", typeName(*HOST));

        if (const json* NOTES = member(manifest, "notes")) {
            if (!NOTES->is_array())
                diags.error("manifest.json /notes", "must be an array, found {}", typeName(*NOTES));
            else {
                for (const auto& NOTE : *NOTES)
                    bundle.notes.push_back(NOTE.is_string() ? NOTE.get<std::string>() : NOTE.dump());
            }
        } else
            diags.warn("manifest.json", "no `notes` array; the contract lists it as always present (may be empty)");

        if (const json* SOURCES = member(manifest, "sources"); !SOURCES || !SOURCES->is_object())
            diags.error("manifest.json", "missing required object `sources` with the four booleans overlay/app_audio/cameras/mic");
        else {
            wantBool(*SOURCES, "overlay", "manifest.json /sources", diags, bundle.sources.overlay);
            wantBool(*SOURCES, "app_audio", "manifest.json /sources", diags, bundle.sources.appAudio);
            wantBool(*SOURCES, "cameras", "manifest.json /sources", diags, bundle.sources.cameras);
            wantBool(*SOURCES, "mic", "manifest.json /sources", diags, bundle.sources.mic);
        }

        if (const json* OVERLAY = member(manifest, "overlay"); !OVERLAY || !OVERLAY->is_object()) {
            if (bundle.sources.overlay)
                diags.error("manifest.json", "`sources.overlay` is true but there is no `overlay` block");
        } else {
            const std::string WHERE = "manifest.json /overlay";
            int64_t           value = 0;
            if (wantInt(*OVERLAY, "width", WHERE, diags, value))
                bundle.overlay.width = static_cast<int>(value);
            if (wantInt(*OVERLAY, "height", WHERE, diags, value))
                bundle.overlay.height = static_cast<int>(value);
            wantString(*OVERLAY, "format", WHERE, diags, bundle.overlay.format);
            wantString(*OVERLAY, "encoder", WHERE, diags, bundle.overlay.encoder);
            double hz = 0.0;
            if (wantNumber(*OVERLAY, "target_hz", WHERE, diags, hz))
                bundle.overlay.targetHz = hz;
            if (wantInt(*OVERLAY, "eye_count", WHERE, diags, value))
                bundle.overlay.eyeCount = static_cast<int>(value);

            if (bundle.overlay.width <= 0 || bundle.overlay.height <= 0)
                diags.error(WHERE, "width/height must be positive, found {}x{}", bundle.overlay.width, bundle.overlay.height);
            if (bundle.overlay.format != "rgba")
                diags.warn(WHERE, "`format` is \"{}\"; v1 composes RGBA and treats anything else as RGBA anyway", bundle.overlay.format);
            if (bundle.overlay.eyeCount != 2)
                diags.warn(WHERE, "`eye_count` is {}; v1 composes eye 0 as left and eye 1 as right and ignores any others", bundle.overlay.eyeCount);
            if (!(bundle.overlay.targetHz > 0.0))
                diags.error(WHERE, "`target_hz` must be positive, found {}", bundle.overlay.targetHz);

            // INTERPRETATION (not in the contract): alpha association.
            if (const json* ALPHA = member(*OVERLAY, "alpha"); ALPHA && ALPHA->is_string())
                bundle.overlay.alpha = ALPHA->get<std::string>();
            if (bundle.overlay.alpha != "straight" && bundle.overlay.alpha != "premultiplied")
                diags.error(WHERE, "`alpha` must be \"straight\" or \"premultiplied\", found \"{}\"", bundle.overlay.alpha);

            if (member(*OVERLAY, "pts_epoch_ns"))
                diags.warn(WHERE, "`pts_epoch_ns` is obsolete: overlay frames align to telemetry by ordinal (the n-th frame is the n-th record without `dropped`), not by container pts");
        }

        // ---- telemetry.jsonl ----------------------------------------------------
        std::vector<SJsonLine> lines;
        if (!readJsonLines(root / "telemetry.jsonl", lines, readError))
            diags.error("telemetry.jsonl", "{}", readError);
        else if (lines.empty())
            diags.error("telemetry.jsonl", "holds no records; a take without telemetry cannot be composed");

        {
            SCap    parseCap, orderCap, eyeCap, blendCap, quadCap;
            int64_t previousHost  = 0;
            int64_t previousFrame = 0;
            bool    first         = true;
            for (const auto& LINE : lines) {
                const std::string WHERE = std::format("telemetry.jsonl:{}", LINE.number);
                json              record;
                try {
                    record = json::parse(LINE.text);
                } catch (const std::exception& e) {
                    if (parseCap.allow())
                        diags.error(WHERE, "is not valid JSON: {}", e.what());
                    continue;
                }
                if (!record.is_object()) {
                    if (parseCap.allow())
                        diags.error(WHERE, "each line must be a JSON object, found {}", typeName(record));
                    continue;
                }

                STelemetryFrame frame;
                if (!wantInt(record, "t_host_ns", WHERE, diags, frame.tHostNs))
                    continue;
                if (!wantInt(record, "frame", WHERE, diags, frame.frame))
                    continue;

                const json* EYES = member(record, "eyes");
                if (!EYES || !EYES->is_array()) {
                    if (eyeCap.allow())
                        diags.error(WHERE, "missing required array `eyes`");
                    continue;
                }
                if (bundle.overlay.eyeCount > 0 && static_cast<int>(EYES->size()) != bundle.overlay.eyeCount) {
                    if (eyeCap.allow())
                        diags.error(WHERE, "`eyes` holds {} entries but manifest.overlay.eye_count is {}", EYES->size(), bundle.overlay.eyeCount);
                    continue;
                }

                bool eyesOk = true;
                for (size_t e = 0; e < EYES->size(); ++e) {
                    const std::string EYE_WHERE = std::format("{} /eyes/{}", WHERE, e);
                    const json&       EYE       = (*EYES)[e];
                    const json*       POSE      = member(EYE, "pose");
                    const json*       FOV       = member(EYE, "fov");
                    if (!POSE || !FOV) {
                        if (eyeCap.allow())
                            diags.error(EYE_WHERE, "each eye needs both `pose` and `fov`");
                        eyesOk = false;
                        break;
                    }
                    const auto PARSED_POSE = parsePose(*POSE, EYE_WHERE + " /pose", diags);
                    const auto PARSED_FOV  = parseFov(*FOV, EYE_WHERE + " /fov", diags);
                    if (!PARSED_POSE || !PARSED_FOV) {
                        eyesOk = false;
                        break;
                    }
                    frame.eyes.push_back({*PARSED_POSE, *PARSED_FOV});
                }
                if (!eyesOk)
                    continue;

                if (const json* HEAD = member(record, "head"); HEAD && !HEAD->is_null()) {
                    // The producer records this at the same instant as the eyes, in
                    // STAGE space. It is what quad poses and camera extrinsics are
                    // relative to, so when it is present nothing derives a head from
                    // the eyes any more.
                    const json* POSE = member(*HEAD, "pose");
                    frame.head       = parsePose(POSE ? *POSE : *HEAD, WHERE + " /head", diags);
                }

                if (const json* QUADS = member(record, "quads"); QUADS && !QUADS->is_null()) {
                    if (!QUADS->is_array()) {
                        if (quadCap.allow())
                            diags.error(WHERE, "`quads` must be an array of composition layers, found {}", typeName(*QUADS));
                    } else {
                        frame.hasQuadsArray = true;
                        for (size_t q = 0; q < QUADS->size(); ++q) {
                            const auto PARSED = parseQuad((*QUADS)[q], std::format("{} /quads/{}", WHERE, q), diags);
                            if (!PARSED)
                                break;
                            frame.quads.push_back(*PARSED);
                        }
                        // The array is in composition order, back to front, and
                        // `index` names that order. A disagreement means one of the
                        // two is not what the consumer will think it is.
                        for (size_t q = 0; q < frame.quads.size(); ++q) {
                            if (frame.quads[q].index != static_cast<int64_t>(q) && quadCap.allow())
                                diags.error(std::format("{} /quads/{}", WHERE, q), "`index` is {} at array position {}; the array is composition order back-to-front and `index` must agree",
                                            frame.quads[q].index, q);
                        }
                    }
                }

                if (const json* CORRECTION = member(record, "stage_correction"); CORRECTION && !CORRECTION->is_null()) {
                    // The contract writes this as `{...}|null` without fixing the
                    // shape. A pose is the reading research 27 footnote 2 implies;
                    // anything else is carried as "present but unread".
                    if (member(*CORRECTION, "pos") && member(*CORRECTION, "quat")) {
                        CDiagnostics quiet;
                        frame.stageCorrection = parsePose(*CORRECTION, WHERE + " /stage_correction", quiet);
                        if (!frame.stageCorrection && parseCap.allow())
                            diags.error(WHERE + " /stage_correction", "looks like a pose but does not parse as one");
                    } else if (blendCap.allow())
                        diags.warn(WHERE, "`stage_correction` is an object without pos/quat; v1 records it but cannot interpret it");
                }

                if (const json* DROPPED = member(record, "dropped")) {
                    if (!DROPPED->is_boolean()) {
                        if (parseCap.allow())
                            diags.error(WHERE, "`dropped` must be a boolean, found {}", typeName(*DROPPED));
                    } else
                        frame.dropped = DROPPED->get<bool>();
                }

                wantString(record, "blend_mode", WHERE, diags, frame.blendMode);
                if (!frame.blendMode.empty() && frame.blendMode != "alpha" && frame.blendMode != "alpha_blend" && frame.blendMode != "additive" && frame.blendMode != "opaque") {
                    if (blendCap.allow())
                        diags.warn(WHERE, "unrecognized `blend_mode` \"{}\"", frame.blendMode);
                }

                if (!first) {
                    if (frame.tHostNs <= previousHost && orderCap.allow())
                        diags.error(WHERE, "`t_host_ns` {} does not advance past the previous record's {}", frame.tHostNs, previousHost);
                    if (frame.frame <= previousFrame && orderCap.allow())
                        diags.error(WHERE, "`frame` {} does not advance past the previous record's {}", frame.frame, previousFrame);
                }
                first         = false;
                previousHost  = frame.tHostNs;
                previousFrame = frame.frame;

                bundle.telemetryHostNs.push_back(frame.tHostNs);
                bundle.telemetry.push_back(std::move(frame));
            }
            parseCap.flush(diags, "telemetry.jsonl", "a JSON parse failure");
            quadCap.flush(diags, "telemetry.jsonl", "a malformed `quads` entry");
            orderCap.flush(diags, "telemetry.jsonl", "an out-of-order timestamp or frame number");
            eyeCap.flush(diags, "telemetry.jsonl", "a malformed `eyes` array");
            blendCap.flush(diags, "telemetry.jsonl", "an unrecognized field value");
        }

        if (!bundle.telemetry.empty() && bundle.telemetry.front().eyes.size() < 2)
            diags.warn("telemetry.jsonl", "only {} eye(s) per record; stereo output needs two", bundle.telemetry.front().eyes.size());

        if (!bundle.telemetry.empty()) {
            const size_t WITH_HEAD = static_cast<size_t>(std::count_if(bundle.telemetry.begin(), bundle.telemetry.end(), [](const STelemetryFrame& f) { return f.head.has_value(); }));
            if (WITH_HEAD == 0)
                diags.warn("telemetry.jsonl", "no record carries a `head` pose; falling back to the midpoint of the eyes, which is where OpenXR's VIEW space sits but is not what "
                                              "the producer recorded");
            else if (WITH_HEAD != bundle.telemetry.size())
                diags.error("telemetry.jsonl", "{} of {} records carry a `head` pose; it must be present on all of them or none", WITH_HEAD, bundle.telemetry.size());

            // Quad records are v2's input, not v1's, but a take recorded without
            // them can never be replayed - so their absence is worth saying out
            // loud, and a half-present array is a producer bug.
            const size_t WITH_QUADS = static_cast<size_t>(std::count_if(bundle.telemetry.begin(), bundle.telemetry.end(), [](const STelemetryFrame& f) { return f.hasQuadsArray; }));
            if (WITH_QUADS == 0)
                diags.warn("telemetry.jsonl", "no record carries a `quads` array; the take composes fine, but grade-B replay (re-rendering the layers rather than reusing the matte) "
                                              "will never be possible from it");
            else if (WITH_QUADS != bundle.telemetry.size())
                diags.error("telemetry.jsonl", "{} of {} records carry a `quads` array; it must be present on all of them or none", WITH_QUADS, bundle.telemetry.size());
        }

        // ---- clock.jsonl --------------------------------------------------------
        {
            std::vector<SJsonLine> clockLines;
            const fs::path         CLOCK_PATH = root / "clock.jsonl";
            if (!fs::exists(CLOCK_PATH)) {
                // A host-only take genuinely has no device clock to reconcile.
                if (bundle.sources.cameras || bundle.sources.mic)
                    diags.error("clock.jsonl", "missing, but the manifest declares device-side sources; device timestamps cannot be placed on the host timeline without it");
                else
                    diags.warn("clock.jsonl", "missing; host and device time will be treated as the same clock");
            } else if (!readJsonLines(CLOCK_PATH, clockLines, readError))
                diags.error("clock.jsonl", "{}", readError);

            std::vector<SClockSample> samples;
            SCap                      cap;
            int64_t                   previous = 0;
            bool                      first    = true;
            for (const auto& LINE : clockLines) {
                const std::string WHERE = std::format("clock.jsonl:{}", LINE.number);
                json              record;
                try {
                    record = json::parse(LINE.text);
                } catch (const std::exception& e) {
                    if (cap.allow())
                        diags.error(WHERE, "is not valid JSON: {}", e.what());
                    continue;
                }
                SClockSample sample;
                if (!wantInt(record, "t_host_ns", WHERE, diags, sample.tHostNs))
                    continue;
                if (!wantInt(record, "offset_ns", WHERE, diags, sample.offsetNs))
                    continue;
                double rtt = 0.0;
                if (wantNumber(record, "rtt_us", WHERE, diags, rtt, false)) {
                    if (rtt < 0.0 && cap.allow())
                        diags.error(WHERE, "`rtt_us` is negative ({})", rtt);
                    sample.rttUs = rtt;
                }
                if (!first && sample.tHostNs < previous && cap.allow())
                    diags.error(WHERE, "`t_host_ns` {} goes backwards from {}", sample.tHostNs, previous);
                first    = false;
                previous = sample.tHostNs;
                samples.push_back(sample);
            }
            cap.flush(diags, "clock.jsonl", "a malformed sample");
            bundle.clock = CClockMap(std::move(samples));

            if (!bundle.clock.empty() && !bundle.telemetryHostNs.empty()) {
                if (bundle.clock.firstHostNs() > bundle.firstHostNs() || bundle.clock.lastHostNs() < bundle.lastHostNs())
                    diags.warn("clock.jsonl", "samples span [{}, {}] but telemetry spans [{}, {}]; the offset is held constant outside the sampled span", bundle.clock.firstHostNs(),
                               bundle.clock.lastHostNs(), bundle.firstHostNs(), bundle.lastHostNs());
            }
        }

        // ---- overlay videos -----------------------------------------------------
        //
        // The alignment rule is ordinal, not temporal: the n-th decoded frame of an
        // eye's video is the n-th telemetry record that is not `dropped`. Container
        // pts carry a uniform nominal timeline at target_hz and cannot carry
        // t_host_ns at all - Matroska's timestamp scale is fixed at 1 ms - so they
        // are read only to sanity-check that nominal cadence, never to align.
        if (bundle.sources.overlay) {
            for (size_t i = 0; i < bundle.telemetry.size(); ++i) {
                if (bundle.telemetry[i].dropped)
                    continue;
                bundle.overlay.frameTelemetryIndex.push_back(i);
                bundle.overlay.frameHostNs.push_back(bundle.telemetry[i].tHostNs);
            }
            if (bundle.overlay.frameTelemetryIndex.empty() && !bundle.telemetry.empty())
                diags.error("telemetry.jsonl", "every record is marked `dropped`, so no overlay frame has a stamped pose");

            const fs::path OVERLAY_DIR = root / "overlay";
            const int      EYES        = std::max(1, bundle.overlay.eyeCount);
            for (int eye = 0; eye < EYES; ++eye) {
                // The prompt's contract says overlay/eye{0,1}.mkv; research 27 wrote
                // overlay/{left,right}.mkv. Both are accepted, the former preferred.
                const std::vector<fs::path> CANDIDATES = {
                    OVERLAY_DIR / std::format("eye{}.mkv", eye),
                    OVERLAY_DIR / (eye == 0 ? "left.mkv" : "right.mkv"),
                };
                fs::path chosen;
                for (const auto& CANDIDATE : CANDIDATES) {
                    if (fs::exists(CANDIDATE)) {
                        chosen = CANDIDATE;
                        break;
                    }
                }
                if (chosen.empty()) {
                    diags.error(std::format("overlay/eye{}.mkv", eye), "missing, but manifest.sources.overlay is true");
                    bundle.overlay.videoPaths.emplace_back();
                    bundle.overlay.videoInfo.emplace_back();
                    continue;
                }
                if (chosen.filename().string() != std::format("eye{}.mkv", eye))
                    diags.warn(chosen.filename().string(), "the contract names this file eye{}.mkv; accepting the research-27 spelling", eye);

                bundle.overlay.videoPaths.push_back(chosen.string());
                bundle.overlay.videoInfo.emplace_back();

                if (!options.probeMedia)
                    continue;

                const std::string NAME = chosen.filename().string();
                std::string       probeError;
                if (!probeVideoCached(chosen.string(), bundle.overlay.videoInfo.back(), probeError, options.probeDepth, options.probeCache)) {
                    diags.error(NAME, "{}", probeError);
                    continue;
                }
                const auto& INFO = bundle.overlay.videoInfo.back();
                if (bundle.overlay.width > 0 && (INFO.width != bundle.overlay.width || INFO.height != bundle.overlay.height))
                    diags.error(NAME, "is {}x{} but manifest.overlay says {}x{}", INFO.width, INFO.height, bundle.overlay.width, bundle.overlay.height);
                if (INFO.ptsNs.empty())
                    diags.error(NAME, "ffprobe reported no frames");

                // A packet count is a frame count only where a packet is a frame.
                // For the overlay that is guaranteed - the contract's encoders are
                // all intra-only - and an encoder that breaks the guarantee has to
                // say so out loud, because the alignment check below is about to
                // trust the number.
                if (options.probeDepth == eProbeDepth::INDEX && !INFO.intraOnly && !INFO.codecName.empty())
                    diags.warn(NAME, "codec `{}` is not one of the known intra-only overlay encoders, so its frame count was counted from the container index rather than decoded; "
                                     "run `validate --deep` to count by decoding",
                               INFO.codecName);

                if (options.checksum) {
                    std::string digest;
                    if (!checksumVideo(chosen.string(), 0, digest, probeError))
                        diags.error(NAME, "{}", probeError);
                    else
                        HXC_INFO("{}: {} frames decode clean, md5 {}", NAME, INFO.ptsNs.size(), digest);
                }

                // The decoded pixel format is a property of the encoder the producer
                // chose, and every accepted one must carry alpha - a matte is the
                // entire reason this source exists. (x264rgb is absent on purpose:
                // it cannot carry alpha at all.)
                static const std::map<std::string, std::string> ALPHA_FORMAT_FOR_ENCODER{
                    {"ffv1", "bgra"},
                    {"png", "rgba"},
                    {"utvideo", "gbrap"},
                };
                const auto EXPECTED = ALPHA_FORMAT_FOR_ENCODER.find(bundle.overlay.encoder);
                if (EXPECTED == ALPHA_FORMAT_FOR_ENCODER.end())
                    diags.warn(NAME, "manifest.overlay.encoder is \"{}\"; the known alpha-carrying encoders are ffv1 (bgra), png (rgba), and utvideo (gbrap)", bundle.overlay.encoder);
                else if (!INFO.pixelFormat.empty() && INFO.pixelFormat != EXPECTED->second)
                    diags.warn(NAME, "decodes as `{}` but the {} encoder is expected to yield `{}`", INFO.pixelFormat, bundle.overlay.encoder, EXPECTED->second);
                if (INFO.pixelFormat.find('a') == std::string::npos)
                    diags.error(NAME, "pixel format `{}` carries no alpha channel; the overlay matte is the whole point of this source", INFO.pixelFormat);

                // THE alignment check. Everything downstream trusts this count.
                if (!bundle.telemetry.empty() && INFO.ptsNs.size() != bundle.overlay.frameTelemetryIndex.size())
                    diags.error(NAME, "holds {} frames but telemetry holds {} record(s) without `dropped`; the n-th frame is the n-th undropped record, so the two counts must be equal",
                                INFO.ptsNs.size(), bundle.overlay.frameTelemetryIndex.size());

                // The nominal cadence is a soft check: a wrong target_hz does not
                // break alignment, but it does mean the manifest is lying.
                if (INFO.ptsNs.size() > 2 && bundle.overlay.targetHz > 0.0) {
                    const int64_t SPAN     = INFO.ptsNs.back() - INFO.ptsNs.front();
                    const double  NOMINAL  = static_cast<double>(INFO.ptsNs.size() - 1) * 1e9 / bundle.overlay.targetHz;
                    if (NOMINAL > 0.0 && std::abs(static_cast<double>(SPAN) - NOMINAL) > 0.02 * NOMINAL)
                        diags.warn(NAME, "container pts span {:.3f} s, but {} frames at the manifest's {:.1f} Hz would span {:.3f} s; pts are nominal and unused for alignment, so this is "
                                         "cosmetic - but one of the two is wrong",
                                   static_cast<double>(SPAN) * 1e-9, INFO.ptsNs.size(), bundle.overlay.targetHz, NOMINAL * 1e-9);
                }
            }

            // A dropped-frame budget worth surfacing: losing most of the session is
            // legal but almost certainly not intended.
            if (!bundle.telemetry.empty()) {
                const size_t KEPT = bundle.overlay.frameTelemetryIndex.size();
                if (KEPT * 4 < bundle.telemetry.size())
                    diags.warn("telemetry.jsonl", "{} of {} records are marked `dropped`; the overlay covers under a quarter of the session", bundle.telemetry.size() - KEPT,
                               bundle.telemetry.size());
            }
        }

        // ---- cameras ------------------------------------------------------------
        if (!bundle.sources.cameras) {
            // A take can carry a calibration sidecar and still declare no camera
            // source: the first real joined take does exactly that, because the
            // cameras disconnected mid-session and captured nothing. That is not
            // a contradiction to complain about - the manifest is right, there
            // are no camera pixels - but it is worth saying out loud, because
            // "cameras: absent" on its own invites the question of where the
            // sidecar went.
            auto orphans = globSuffix(root / "cameras", "cameras.jsonl");
            if (orphans.empty())
                orphans = globSuffix(root, "cameras.jsonl");
            if (!orphans.empty())
                bundle.loaderNotes.push_back(std::format("{} is present but manifest.sources.cameras is false; the take has calibration but no camera frames, so nothing composites from it",
                                                         orphans.front().filename().string()));
        }

        if (bundle.sources.cameras) {
            const fs::path CAMERA_DIR = root / "cameras";
            auto           headers    = globSuffix(CAMERA_DIR, "cameras.jsonl");
            // PINNED by the first real joined takes: the join drops the sidecar at
            // the take ROOT while the videos it describes stay under cameras/.
            // Only the `cameras.jsonl` suffix is looked for at the root - a bare
            // `.jsonl` sweep there would swallow telemetry.jsonl and clock.jsonl.
            if (headers.empty())
                headers = globSuffix(root, "cameras.jsonl");
            if (headers.empty())
                headers = globSuffix(CAMERA_DIR, ".jsonl");

            if (headers.empty())
                diags.error("cameras/", "manifest.sources.cameras is true but no `*-cameras.jsonl` is present, at the take root or under cameras/");
            else {
                if (headers.size() > 1)
                    diags.warn("cameras/", "{} camera sidecars present; using {}", headers.size(), headers.front().filename().string());

                const fs::path         SIDECAR = headers.front();
                std::vector<SJsonLine> camLines;
                if (!readJsonLines(SIDECAR, camLines, readError))
                    diags.error(SIDECAR.filename().string(), "{}", readError);
                else if (camLines.empty())
                    diags.error(SIDECAR.filename().string(), "is empty; the first line must be the calibration header");
                else {
                    const std::string HEADER_WHERE = std::format("{}:1", SIDECAR.filename().string());
                    json              header;
                    try {
                        header = json::parse(camLines.front().text);
                    } catch (const std::exception& e) {
                        diags.error(HEADER_WHERE, "the calibration header is not valid JSON: {}", e.what());
                        header = json::object();
                    }

                    std::string headerTimestampSource;
                    wantString(header, "timestamp_source", HEADER_WHERE, diags, headerTimestampSource, false);

                    // INTERPRETATION: the contract says the header carries intrinsics
                    // and extrinsics "per cam" without fixing the container. All three
                    // plausible shapes are read: a `cameras` array of entries, a
                    // `cameras` object keyed by cam, or the cam keys at the top level.
                    std::vector<std::pair<std::string, const json*>> entries;
                    if (const json* CAMS = member(header, "cameras")) {
                        if (CAMS->is_array()) {
                            for (const auto& ENTRY : *CAMS) {
                                const json* KEY = member(ENTRY, "cam");
                                entries.emplace_back(KEY ? (KEY->is_string() ? KEY->get<std::string>() : KEY->dump()) : std::string{}, &ENTRY);
                            }
                        } else if (CAMS->is_object()) {
                            for (auto it = CAMS->begin(); it != CAMS->end(); ++it)
                                entries.emplace_back(it.key(), &it.value());
                        }
                    }
                    if (entries.empty()) {
                        for (auto it = header.begin(); it != header.end(); ++it) {
                            if (it.value().is_object() && member(it.value(), "intrinsics"))
                                entries.emplace_back(it.key(), &it.value());
                        }
                    }

                    // FOURTH SHAPE, and the one the producer actually writes: the
                    // header nests the other way round. Instead of one object per
                    // camera holding its intrinsics and extrinsics, there are two
                    // parallel maps - `intrinsics: {L:{...}, R:{...}}` and
                    // `extrinsics_head_to_camera: {L:{...}, R:{...}}` - with
                    // `axes`, `timestamp_source`, `t_device_ns_domain` and
                    // `clock_anchor` shared once at the top level. Each camera's
                    // entry is assembled from the two maps plus the shared fields,
                    // so everything downstream sees the shape it already knew.
                    std::vector<json>        assembled;
                    std::vector<std::string> assembledKeys;
                    if (entries.empty()) {
                        const json* INTRINSICS_MAP = member(header, "intrinsics");
                        const json* EXTRINSICS_MAP = member(header, "extrinsics_head_to_camera");
                        if (INTRINSICS_MAP && INTRINSICS_MAP->is_object() && EXTRINSICS_MAP && EXTRINSICS_MAP->is_object()) {
                            for (auto it = INTRINSICS_MAP->begin(); it != INTRINSICS_MAP->end(); ++it) {
                                if (!it.value().is_object())
                                    continue;
                                json entry          = json::object();
                                entry["intrinsics"] = it.value();
                                if (const json* EXTRINSICS = member(*EXTRINSICS_MAP, it.key().c_str()))
                                    entry["extrinsics_head_to_camera"] = *EXTRINSICS;
                                // The raw Android pose is keyed by camera too, and
                                // it is the authority - see below.
                                if (const json* RAW_MAP = member(header, "extrinsics_android_raw")) {
                                    if (const json* RAW = member(*RAW_MAP, it.key().c_str()))
                                        entry["extrinsics_android_raw"] = *RAW;
                                }
                                for (const char* SHARED : {"timestamp_source", "axes", "t_device_ns_domain", "clock_anchor"}) {
                                    if (const json* VALUE = member(header, SHARED))
                                        entry[SHARED] = *VALUE;
                                }
                                assembled.push_back(std::move(entry));
                                assembledKeys.push_back(it.key());
                            }
                            // Addresses are taken only after the vector has stopped
                            // growing, or they would dangle.
                            for (size_t i = 0; i < assembled.size(); ++i)
                                entries.emplace_back(assembledKeys[i], &assembled[i]);
                        }
                    }

                    if (entries.empty())
                        diags.error(HEADER_WHERE, "no camera calibration entries found; expected either a `cameras` array/object or per-cam objects each holding `intrinsics` and "
                                                  "`extrinsics_head_to_camera`");

                    for (const auto& [RAW_KEY, ENTRY] : entries) {
                        const auto NORMALIZED = normalizeCameraKey(json(RAW_KEY));
                        if (!NORMALIZED) {
                            diags.error(HEADER_WHERE, "camera key \"{}\" is not one of L/R (or left/right, cam0/cam1, 0/1)", RAW_KEY);
                            continue;
                        }

                        SCamera camera;
                        camera.key             = NORMALIZED->first;
                        camera.eye             = NORMALIZED->second;
                        camera.timestampSource = headerTimestampSource;

                        const std::string ENTRY_WHERE = std::format("{} /{}", HEADER_WHERE, RAW_KEY);
                        wantString(*ENTRY, "timestamp_source", ENTRY_WHERE, diags, camera.timestampSource, false);
                        if (camera.timestampSource.empty())
                            diags.warn(ENTRY_WHERE, "no `timestamp_source`; research 27 open question 5 exists precisely because REALTIME and UNKNOWN cannot be told apart after the fact");

                        const json* INTRINSICS = member(*ENTRY, "intrinsics");
                        if (!INTRINSICS || !INTRINSICS->is_object())
                            diags.error(ENTRY_WHERE, "missing required object `intrinsics` {{fx,fy,cx,cy,distortion[]}}");
                        else {
                            const std::string INTR_WHERE = ENTRY_WHERE + " /intrinsics";
                            wantNumber(*INTRINSICS, "fx", INTR_WHERE, diags, camera.intrinsics.fx);
                            wantNumber(*INTRINSICS, "fy", INTR_WHERE, diags, camera.intrinsics.fy);
                            wantNumber(*INTRINSICS, "cx", INTR_WHERE, diags, camera.intrinsics.cx);
                            wantNumber(*INTRINSICS, "cy", INTR_WHERE, diags, camera.intrinsics.cy);
                            // The sensor region those numbers are stated against.
                            // `pre_correction_active_array` is the one that goes
                            // with un-distortion-corrected intrinsics; the plain
                            // `active_array` is the fallback.
                            std::vector<double> activeArray;
                            for (const char* KEY : {"pre_correction_active_array", "active_array"}) {
                                if (member(*INTRINSICS, KEY) && wantDoubleArray(*INTRINSICS, KEY, 4, INTR_WHERE, diags, activeArray) && activeArray.size() == 4) {
                                    camera.intrinsics.activeArray = {activeArray[0], activeArray[1], activeArray[2], activeArray[3]};
                                    break;
                                }
                            }
                            if (!(camera.intrinsics.fx > 0.0) || !(camera.intrinsics.fy > 0.0))
                                diags.error(INTR_WHERE, "fx/fy must be positive pixel focal lengths, found {}/{}", camera.intrinsics.fx, camera.intrinsics.fy);
                            std::vector<double> distortion;
                            // PINNED: `distortion: null` means "no distortion
                            // coefficients", not "field missing" and not an error.
                            // The Meta cameras pre-undistort, publish nothing, and
                            // the producer forwards that faithfully as a null. An
                            // empty coefficient list is exactly the right model:
                            // the pinhole projection with all terms zero.
                            const json* DISTORTION = member(*INTRINSICS, "distortion");
                            if (DISTORTION && DISTORTION->is_null())
                                camera.intrinsics.distortion.clear();
                            else if (wantDoubleArray(*INTRINSICS, "distortion", 0, INTR_WHERE, diags, distortion)) {
                                if (distortion.size() > 5)
                                    diags.warn(INTR_WHERE, "`distortion` holds {} coefficients; v1 reads the first five as OpenCV's k1,k2,p1,p2,k3", distortion.size());
                                camera.intrinsics.distortion = distortion;
                            }
                        }

                        const json* EXTRINSICS = member(*ENTRY, "extrinsics_head_to_camera");
                        if (!EXTRINSICS)
                            diags.error(ENTRY_WHERE, "missing required object `extrinsics_head_to_camera` {{pos,quat}}");
                        else if (const auto PARSED = parsePose(*EXTRINSICS, ENTRY_WHERE + " /extrinsics_head_to_camera", diags))
                            camera.headToCamera = *PARSED;

                        // THE RAW ANDROID POSE WINS WHENEVER IT IS PRESENT.
                        //
                        // `extrinsics_head_to_camera` is a derived field, and the
                        // derivation that produced the takes recorded so far is
                        // missing a conjugate: Android documents its raw quaternion
                        // as sensor-to-camera (p' = Rp), so a head-to-camera needs
                        // the inverse. Without it the cant is mirrored - cameras
                        // that really point ~10.9 degrees DOWN were stored pointing
                        // ~10.9 degrees UP, a 21.7 degree error, which is exactly
                        // the residue that made the passthrough sit high in the
                        // frame. Recomputing from the raw is how a bundle recorded
                        // by the buggy producer still composites correctly, without
                        // touching the file.
                        //
                        // The other half of the finding is that there is nothing to
                        // wait for: LENS_POSE is head-relative on Quest despite its
                        // GYROSCOPE label, so no imu_to_head constant is needed.
                        // In the producer's shape the raw sits in a top-level map
                        // keyed by camera; in the flattened shapes it may be on the
                        // entry itself. Look in both, so the policy does not depend
                        // on which spelling the header happened to use.
                        const json* RAW = member(*ENTRY, "extrinsics_android_raw");
                        if (!RAW || !RAW->is_object()) {
                            if (const json* RAW_MAP = member(header, "extrinsics_android_raw"); RAW_MAP && RAW_MAP->is_object())
                                RAW = member(*RAW_MAP, RAW_KEY.c_str());
                        }
                        if (RAW && RAW->is_object()) {
                            std::vector<double> translation, rotation;
                            const std::string   RAW_WHERE = ENTRY_WHERE + " /extrinsics_android_raw";
                            const bool          HAVE_T    = wantDoubleArray(*RAW, "lens_pose_translation", 3, RAW_WHERE, diags, translation);
                            const bool          HAVE_R    = wantDoubleArray(*RAW, "lens_pose_rotation", 4, RAW_WHERE, diags, rotation);
                            if (HAVE_T && HAVE_R) {
                                const SQuat RAW_ROTATION{rotation[0], rotation[1], rotation[2], rotation[3]};
                                const SPose STORED = camera.headToCamera;
                                // The Android sensor frame shares the OpenXR head
                                // axes, so the translation carries over untouched.
                                camera.headToCamera.pos = {translation[0], translation[1], translation[2]};
                                camera.headToCamera.rot = headToCameraFromAndroidRaw(RAW_ROTATION);
                                camera.extrinsicsFromRaw = true;

                                const double DISAGREEMENT = angleBetweenDegrees(STORED.rot, camera.headToCamera.rot);
                                camera.extrinsicsDisagreementDegrees = DISAGREEMENT;
                                if (DISAGREEMENT > 0.5)
                                    bundle.loaderNotes.push_back(std::format("camera {}: recomputed head_to_camera from `extrinsics_android_raw`; the stored value disagrees by {:.2f} degrees "
                                                                             "and is a repaired legacy conversion (the mirrored cant - stored optical axis {:+.2f} deg, actual {:+.2f} deg)",
                                                                             camera.key, DISAGREEMENT, opticalAxisPitchDegrees(STORED.rot), opticalAxisPitchDegrees(camera.headToCamera.rot)));
                                else
                                    bundle.loaderNotes.push_back(std::format("camera {}: recomputed head_to_camera from `extrinsics_android_raw`; the stored value agrees to {:.2f} degrees",
                                                                             camera.key, DISAGREEMENT));
                            }
                        }

                        int64_t dimension = 0;
                        if (wantInt(*ENTRY, "width", ENTRY_WHERE, diags, dimension, false))
                            camera.video.width = static_cast<int>(dimension);
                        if (wantInt(*ENTRY, "height", ENTRY_WHERE, diags, dimension, false))
                            camera.video.height = static_cast<int>(dimension);

                        bundle.cameras.push_back(std::move(camera));
                    }

                    // Per-frame records.
                    SCap cap;
                    // The -1 exposure sentinel gets its own budget: it is a
                    // property of the device, so it tends to be on every record,
                    // and it must not crowd out the malformed-record diagnostics.
                    SCap exposureCap;
                    for (size_t i = 1; i < camLines.size(); ++i) {
                        const std::string WHERE = std::format("{}:{}", SIDECAR.filename().string(), camLines[i].number);
                        json              record;
                        try {
                            record = json::parse(camLines[i].text);
                        } catch (const std::exception& e) {
                            if (cap.allow())
                                diags.error(WHERE, "is not valid JSON: {}", e.what());
                            continue;
                        }
                        const json* KEY = member(record, "cam");
                        if (!KEY) {
                            if (cap.allow())
                                diags.error(WHERE, "missing required `cam`");
                            continue;
                        }
                        const auto NORMALIZED = normalizeCameraKey(*KEY);
                        if (!NORMALIZED) {
                            if (cap.allow())
                                diags.error(WHERE, "`cam` value {} is not one of L/R", KEY->dump());
                            continue;
                        }
                        auto target = std::find_if(bundle.cameras.begin(), bundle.cameras.end(), [&](const SCamera& c) { return c.key == NORMALIZED->first; });
                        if (target == bundle.cameras.end()) {
                            if (cap.allow())
                                diags.error(WHERE, "references camera \"{}\", which the calibration header does not declare", NORMALIZED->first);
                            continue;
                        }

                        SCameraFrame frame;
                        if (!wantInt(record, "t_device_ns", WHERE, diags, frame.tDeviceNs))
                            continue;
                        wantInt(record, "exposure_ns", WHERE, diags, frame.exposureNs, false);
                        wantInt(record, "frame", WHERE, diags, frame.frame, false);
                        if (member(record, "t_xr_ns"))
                            frame.hasXrNs = wantInt(record, "t_xr_ns", WHERE, diags, frame.tXrNs, false);
                        // PINNED: -1 is the producer's "the device did not report
                        // an exposure" sentinel, not a malformed value. Mid-exposure
                        // sampling then has nothing to offset by, so the frame is
                        // treated as instantaneous - which is what a camera whose
                        // exposure is unknown is already being assumed to be.
                        if (frame.exposureNs == -1) {
                            frame.exposureNs = 0;
                            if (exposureCap.allow())
                                diags.warn(WHERE, "`exposure_ns` is -1, the device's \"unknown\" sentinel; sampling this frame's pose at its stamp rather than half an exposure later");
                        } else if (frame.exposureNs < 0 && cap.allow())
                            diags.error(WHERE, "`exposure_ns` is negative ({})", frame.exposureNs);
                        if (!target->frames.empty() && frame.tDeviceNs <= target->frames.back().tDeviceNs && cap.allow())
                            diags.error(WHERE, "`t_device_ns` {} does not advance past camera {}'s previous {}", frame.tDeviceNs, target->key, target->frames.back().tDeviceNs);
                        target->frames.push_back(frame);
                    }
                    cap.flush(diags, SIDECAR.filename().string(), "a malformed per-frame record");
                }
            }

            // Match each camera to its video, then map its timestamps to host time.
            const auto VIDEOS = globSuffix(CAMERA_DIR, ".mp4");
            for (auto& camera : bundle.cameras) {
                for (const auto& VIDEO : VIDEOS) {
                    const std::string NAME = VIDEO.stem().string();
                    if (containsCaseInsensitive(NAME, "cam" + camera.key) || containsCaseInsensitive(NAME, camera.eye == 0 ? "left" : "right")) {
                        camera.videoPath = VIDEO.string();
                        break;
                    }
                }
                if (camera.videoPath.empty()) {
                    diags.error("cameras/", "no video matching `*-cam{}.mp4` for the camera the sidecar declares", camera.key);
                    continue;
                }

                if (options.probeMedia) {
                    const int   DECLARED_W = camera.video.width;
                    const int   DECLARED_H = camera.video.height;
                    std::string probeError;
                    if (!probeVideoCached(camera.videoPath, camera.video, probeError, options.probeDepth, options.probeCache)) {
                        diags.error(fs::path(camera.videoPath).filename().string(), "{}", probeError);
                        continue;
                    }
                    if (options.checksum) {
                        std::string digest;
                        if (!checksumVideo(camera.videoPath, 0, digest, probeError))
                            diags.error(fs::path(camera.videoPath).filename().string(), "{}", probeError);
                        else
                            HXC_INFO("{}: {} frames decode clean, md5 {}", fs::path(camera.videoPath).filename().string(), camera.video.ptsNs.size(), digest);
                    }
                    if (DECLARED_W > 0 && (DECLARED_W != camera.video.width || DECLARED_H != camera.video.height))
                        diags.error(fs::path(camera.videoPath).filename().string(), "is {}x{} but the sidecar declares {}x{}", camera.video.width, camera.video.height, DECLARED_W, DECLARED_H);

                    // THE PRINCIPAL POINT IS NOT IN IMAGE COORDINATES until this
                    // runs. Android states intrinsics against the sensor's active
                    // array and then delivers a stream cropped and scaled from it.
                    // On the first real camera take the array is 1280x1280 and the
                    // video is 1280x960, so cy = 638.6 - dead centre of the array -
                    // was being applied to a 960-tall image, where it means 66.5%
                    // down. The compositor therefore believed the camera saw 36.4
                    // degrees above its axis and only 20.3 below, and threw away the
                    // bottom of every frame. Rebasing puts the principal point back
                    // at 49.9% of the image and the field back to a symmetric
                    // 28.9/29.0 degrees.
                    const double BEFORE_CX = camera.intrinsics.cx, BEFORE_CY = camera.intrinsics.cy;
                    if (camera.intrinsics.rebaseToImage(camera.video.width, camera.video.height))
                        bundle.loaderNotes.push_back(std::format("camera {}: intrinsics were stated against a {:.0f}x{:.0f} sensor active array and the video is {}x{}; "
                                                                 "principal point rebased from ({:.1f}, {:.1f}) to ({:.1f}, {:.1f})",
                                                                 camera.key, camera.intrinsics.activeArray[2], camera.intrinsics.activeArray[3], camera.video.width, camera.video.height, BEFORE_CX,
                                                                 BEFORE_CY, camera.intrinsics.cx, camera.intrinsics.cy));
                    if (camera.video.ptsNs.size() != camera.frames.size())
                        diags.error(fs::path(camera.videoPath).filename().string(), "holds {} frames but the sidecar lists {} records for camera {}; the sidecar is authoritative for "
                                                                                    "timestamps, so the two must correspond one-for-one in decode order",
                                    camera.video.ptsNs.size(), camera.frames.size(), camera.key);
                }

                if (camera.intrinsics.cx <= 0.0 || camera.intrinsics.cy <= 0.0)
                    diags.warn(fs::path(camera.videoPath).filename().string(), "principal point ({:.1f}, {:.1f}) is at or outside the top-left corner", camera.intrinsics.cx, camera.intrinsics.cy);

                // Mid-exposure sampling, per research 27 section 3 footnote 1: the
                // sensor timestamp is the start of exposure, so the pose that
                // belongs to the frame is the one half an exposure later. Which
                // stamp that is read from is SCameraFrame::captureDeviceNs's
                // business - XR time when the device reports it.
                camera.hostNs.reserve(camera.frames.size());
                for (const auto& FRAME : camera.frames)
                    camera.hostNs.push_back(bundle.clock.hostFromDevice(FRAME.captureDeviceNs()));

                if (!camera.frames.empty()) {
                    const size_t WITH_XR = static_cast<size_t>(std::count_if(camera.frames.begin(), camera.frames.end(), [](const SCameraFrame& f) { return f.hasXrNs; }));
                    if (WITH_XR == camera.frames.size())
                        bundle.loaderNotes.push_back(std::format("camera {}: timed by `t_xr_ns`, which shares the clock series' domain", camera.key));
                    else if (WITH_XR == 0)
                        bundle.loaderNotes.push_back(std::format("camera {}: timed by `t_device_ns` (domain `{}`); no `t_xr_ns` was reported, so the clock series is assumed to map that domain",
                                                                 camera.key, camera.timestampSource.empty() ? std::string("unstated") : camera.timestampSource));
                    else
                        diags.warn(fs::path(camera.videoPath).filename().string(), "{} of {} frames carry `t_xr_ns` and the rest do not; the take mixes two timing domains", WITH_XR,
                                   camera.frames.size());
                }

                if (!camera.hostNs.empty() && !bundle.telemetryHostNs.empty()) {
                    const bool DISJOINT = camera.hostNs.back() < bundle.firstHostNs() || camera.hostNs.front() > bundle.lastHostNs();
                    if (DISJOINT)
                        diags.error(fs::path(camera.videoPath).filename().string(), "maps to host times [{}, {}], which does not overlap the telemetry span [{}, {}]; check the clock "
                                                                                    "offset sign (device = host + offset)",
                                    camera.hostNs.front(), camera.hostNs.back(), bundle.firstHostNs(), bundle.lastHostNs());
                }
            }

            if (bundle.cameras.empty())
                diags.error("cameras/", "manifest.sources.cameras is true but no usable camera was loaded");
        }

        // ---- audio --------------------------------------------------------------
        const auto loadAudio = [&](const std::string& role, const fs::path& media, const fs::path& sidecar) -> std::optional<SAudioTrack> {
            SAudioTrack track;
            track.role        = role;
            track.path        = media.string();
            track.sidecarPath = sidecar.string();

            std::string text;
            if (!readWholeFile(sidecar, text, readError)) {
                diags.error(sidecar.filename().string(), "{}", readError);
                return std::nullopt;
            }
            json record;
            try {
                record = json::parse(text);
            } catch (const std::exception& e) {
                diags.error(sidecar.filename().string(), "is not valid JSON: {}", e.what());
                return std::nullopt;
            }

            const std::string WHERE     = sidecar.filename().string();
            const bool        HAS_HOST  = member(record, "start_t_host_ns") != nullptr;
            const bool        HAS_DEVICE = member(record, "start_t_device_ns") != nullptr;
            if (HAS_HOST && HAS_DEVICE) {
                diags.error(WHERE, "carries both `start_t_host_ns` and `start_t_device_ns`; a track has one clock domain");
                return std::nullopt;
            }
            if (!HAS_HOST && !HAS_DEVICE) {
                diags.error(WHERE, "needs `start_t_host_ns` (host-side track) or `start_t_device_ns` (device-side track)");
                return std::nullopt;
            }
            track.deviceClock = HAS_DEVICE;
            if (!wantInt(record, HAS_DEVICE ? "start_t_device_ns" : "start_t_host_ns", WHERE, diags, track.startNs))
                return std::nullopt;

            int64_t value = 0;
            if (wantInt(record, "sample_rate_hz", WHERE, diags, value))
                track.sampleRate = static_cast<int>(value);
            if (wantInt(record, "channels", WHERE, diags, value))
                track.channels = static_cast<int>(value);

            track.startHostNs = track.deviceClock ? bundle.clock.hostFromDevice(track.startNs) : track.startNs;

            if (options.probeMedia) {
                SAudioInfo  info;
                std::string probeError;
                if (!probeAudio(track.path, info, probeError)) {
                    diags.error(media.filename().string(), "{}", probeError);
                    return std::nullopt;
                }
                if (track.sampleRate > 0 && info.sampleRate != track.sampleRate)
                    diags.error(media.filename().string(), "decodes at {} Hz but {} says {} Hz", info.sampleRate, sidecar.filename().string(), track.sampleRate);
                if (track.channels > 0 && info.channels != track.channels)
                    diags.error(media.filename().string(), "has {} channel(s) but {} says {}", info.channels, sidecar.filename().string(), track.channels);
                track.sampleRate = info.sampleRate;
                track.channels   = info.channels;
                track.durationNs = info.durationNs;
            }

            if (!bundle.telemetryHostNs.empty()) {
                const int64_t END = track.startHostNs + track.durationNs;
                if (END < bundle.firstHostNs() || track.startHostNs > bundle.lastHostNs())
                    diags.error(media.filename().string(), "covers host times [{}, {}], which does not overlap the telemetry span [{}, {}]", track.startHostNs, END, bundle.firstHostNs(),
                                bundle.lastHostNs());
            }
            return track;
        };

        if (bundle.sources.appAudio) {
            const fs::path MEDIA   = root / "audio" / "app.flac";
            const fs::path SIDECAR = root / "audio" / "app.json";
            if (!fs::exists(MEDIA))
                diags.error("audio/app.flac", "missing, but manifest.sources.app_audio is true");
            else if (!fs::exists(SIDECAR))
                diags.error("audio/app.json", "missing; the start stamp lives there, and without it the track cannot be placed on the timeline");
            else
                bundle.appAudio = loadAudio("app", MEDIA, SIDECAR);
        }

        if (bundle.sources.mic) {
            // PINNED by the first real joined takes: the join drops the device's
            // mic at the take ROOT as `<take>-mic.wav` (pcm_s16le) beside its
            // `-mic.json`, not under audio/ and not as FLAC. Both containers and
            // both locations are accepted - audio/*.flac is what the contract
            // originally wrote and older bundles still carry it - and the prefix
            // is free-form either way.
            std::vector<fs::path> candidates;
            for (const auto& DIRECTORY : {root / "audio", root}) {
                for (const char* SUFFIX : {"mic.flac", "mic.wav"}) {
                    for (const auto& FOUND : globSuffix(DIRECTORY, SUFFIX))
                        candidates.push_back(FOUND);
                }
            }
            if (candidates.empty())
                diags.error("*-mic.wav", "missing, but manifest.sources.mic is true; looked for `*-mic.wav` and `*-mic.flac` at the take root and under audio/");
            else {
                if (candidates.size() > 1)
                    diags.warn("audio/", "{} mic tracks present; using {}", candidates.size(), candidates.front().filename().string());
                fs::path media   = candidates.front();
                fs::path sidecar = media;
                sidecar.replace_extension(".json");
                if (!fs::exists(sidecar))
                    diags.error(sidecar.filename().string(), "missing; a mic track needs its `start_t_device_ns` sidecar");
                else
                    bundle.mic = loadAudio("mic", media, sidecar);
            }
        }

        return bundle;
    }

}
