#include "SynthScene.hpp"

#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

using json = nlohmann::json;

namespace hxc {

    namespace {

        constexpr double TWO_PI = 6.283185307179586;

        json toJson(const SVec3& v) {
            return json::array({v.x, v.y, v.z});
        }
        json toJson(const SQuat& q) {
            return json::array({q.x, q.y, q.z, q.w});
        }
        json toJson(const SPose& p) {
            return json{{"pos", toJson(p.pos)}, {"quat", toJson(p.rot)}};
        }
        json toJson(const SFov& f) {
            return json{{"l", f.l}, {"r", f.r}, {"u", f.u}, {"d", f.d}};
        }
        json toJson(const SCameraIntrinsics& i) {
            return json{{"fx", i.fx}, {"fy", i.fy}, {"cx", i.cx}, {"cy", i.cy}, {"distortion", i.distortion}};
        }

        SVec3 vec3From(const json& node) {
            return {node.at(0).get<double>(), node.at(1).get<double>(), node.at(2).get<double>()};
        }
        SQuat quatFrom(const json& node) {
            return {node.at(0).get<double>(), node.at(1).get<double>(), node.at(2).get<double>(), node.at(3).get<double>()};
        }
        SPose poseFrom(const json& node) {
            return {vec3From(node.at("pos")), quatFrom(node.at("quat"))};
        }
        SFov fovFrom(const json& node) {
            return {node.at("l").get<double>(), node.at("r").get<double>(), node.at("u").get<double>(), node.at("d").get<double>()};
        }
        SCameraIntrinsics intrinsicsFrom(const json& node) {
            SCameraIntrinsics out;
            out.fx         = node.at("fx").get<double>();
            out.fy         = node.at("fy").get<double>();
            out.cx         = node.at("cx").get<double>();
            out.cy         = node.at("cy").get<double>();
            out.distortion = node.at("distortion").get<std::vector<double>>();
            return out;
        }

        json toJson(const SSynthAudio& audio) {
            return json{{"start_ns", audio.startNs}, {"click_t_host_ns", audio.clickHostNs}, {"click_t_track_ns", audio.clickTrackNs}, {"sample_rate_hz", audio.sampleRate},
                        {"click_amplitude", audio.clickAmplitude}};
        }
        SSynthAudio audioFrom(const json& node) {
            SSynthAudio out;
            out.startNs        = node.at("start_ns").get<int64_t>();
            out.clickHostNs    = node.at("click_t_host_ns").get<int64_t>();
            out.clickTrackNs   = node.at("click_t_track_ns").get<int64_t>();
            out.sampleRate     = node.at("sample_rate_hz").get<int>();
            out.clickAmplitude = node.at("click_amplitude").get<double>();
            return out;
        }

    }

    // A slow sweep with a deliberate high-frequency jitter riding on it. The slow
    // terms are what `--framing stabilized` must keep; the fast ones are what it
    // must remove. Every frequency is irrational relative to the frame rates in
    // play so nothing lands on a convenient sample boundary by accident.
    SPose SSynthScene::headAt(int64_t tHostNs) const {
        const double T = static_cast<double>(tHostNs - t0HostNs) * 1e-9;

        SPose pose;
        pose.pos.x = 0.060 * std::sin(TWO_PI * 0.31 * T) + 0.010 * std::sin(TWO_PI * 4.70 * T + 1.1);
        pose.pos.y = 0.030 * std::sin(TWO_PI * 0.47 * T + 0.7) + 0.008 * std::sin(TWO_PI * 6.10 * T);
        pose.pos.z = 0.040 * std::sin(TWO_PI * 0.23 * T + 2.0) + 0.006 * std::sin(TWO_PI * 5.30 * T + 0.4);

        const double YAW   = 0.120 * std::sin(TWO_PI * 0.29 * T) + 0.020 * std::sin(TWO_PI * 5.30 * T + 0.9);
        const double PITCH = 0.060 * std::sin(TWO_PI * 0.19 * T + 1.3) + 0.015 * std::sin(TWO_PI * 6.70 * T);
        const double ROLL  = 0.012 * std::sin(TWO_PI * 0.37 * T + 0.2) + 0.004 * std::sin(TWO_PI * 7.30 * T);
        pose.rot           = SQuat::fromYawPitchRoll(YAW, PITCH, ROLL);
        return pose;
    }

    int64_t SSynthScene::frameHostNs(int frame) const {
        return t0HostNs + static_cast<int64_t>(std::llround(static_cast<double>(frame) * 1e9 / hz));
    }

    std::string SSynthScene::toJson() const {
        json scene;
        scene["version"]     = 1;
        scene["t0_host_ns"]  = t0HostNs;
        scene["hz"]          = hz;
        scene["frames"]      = frames;
        scene["overlay_hz"]  = overlayHz;
        scene["overlay_width"]  = overlayWidth;
        scene["overlay_height"] = overlayHeight;
        scene["overlay_frames"] = overlayFrames;

        scene["wall"] = {
            {"z", wallZ}, {"half_x", wallHalfX}, {"half_y", wallHalfY}, {"checker_cell", checkerCell}, {"markers", json::array()},
        };
        for (const auto& MARKER : wallMarkers) {
            scene["wall"]["markers"].push_back({
                {"name", MARKER.name},
                {"color", json::array({MARKER.color[0], MARKER.color[1], MARKER.color[2]})},
                {"world", hxc::toJson(MARKER.world)},
                {"size", MARKER.size},
            });
        }

        scene["code_patch"] = {
            {"centre", hxc::toJson(codeCentre)}, {"size", codeSize}, {"base", codeBase}, {"step", codeStep}, {"modulus", codeModulus}, {"green", codeGreen}, {"blue", codeBlue},
        };

        scene["overlay_quad"] = {
            {"pose", hxc::toJson(overlayQuad)}, {"width", quadWidth}, {"height", quadHeight}, {"halo", quadHalo}, {"markers", json::array()},
        };
        for (const auto& MARKER : overlayMarkers) {
            scene["overlay_quad"]["markers"].push_back({
                {"name", MARKER.name},
                {"color", json::array({MARKER.color[0], MARKER.color[1], MARKER.color[2]})},
                {"u", MARKER.u},
                {"v", MARKER.v},
                {"size", MARKER.size},
            });
        }

        scene["eyes"] = {{"ipd", ipd}, {"fov", json::array({hxc::toJson(eyeFov[0]), hxc::toJson(eyeFov[1])})}};

        scene["cameras"]     = json::array();
        scene["has_cameras"] = hasCameras;
        for (const auto& CAMERA : cameras) {
            scene["cameras"].push_back({
                {"cam", CAMERA.key},
                {"eye", CAMERA.eye},
                {"head_to_camera", hxc::toJson(CAMERA.headToCamera)},
                {"intrinsics", hxc::toJson(CAMERA.intrinsics)},
                {"width", CAMERA.width},
                {"height", CAMERA.height},
                {"hz", CAMERA.hz},
                {"exposure_ns", CAMERA.exposureNs},
                {"t_device_ns", CAMERA.deviceNs},
            });
        }

        scene["has_app"] = hasApp;
        scene["app"]     = hxc::toJson(app);
        scene["has_mic"] = hasMic;
        scene["mic"]     = hxc::toJson(mic);

        return scene.dump(2);
    }

    std::optional<SSynthScene> SSynthScene::fromJson(const std::string& text, std::string& error) {
        try {
            const json  SCENE = json::parse(text);
            SSynthScene out;
            out.t0HostNs      = SCENE.at("t0_host_ns").get<int64_t>();
            out.hz            = SCENE.at("hz").get<double>();
            out.frames        = SCENE.at("frames").get<int>();
            out.overlayHz     = SCENE.at("overlay_hz").get<double>();
            out.overlayWidth  = SCENE.at("overlay_width").get<int>();
            out.overlayHeight = SCENE.at("overlay_height").get<int>();
            out.overlayFrames = SCENE.at("overlay_frames").get<std::vector<int>>();

            const json& WALL = SCENE.at("wall");
            out.wallZ        = WALL.at("z").get<double>();
            out.wallHalfX    = WALL.at("half_x").get<double>();
            out.wallHalfY    = WALL.at("half_y").get<double>();
            out.checkerCell  = WALL.at("checker_cell").get<double>();
            for (const auto& MARKER : WALL.at("markers")) {
                SSynthWallMarker parsed;
                parsed.name  = MARKER.at("name").get<std::string>();
                parsed.color = {MARKER.at("color").at(0).get<int>(), MARKER.at("color").at(1).get<int>(), MARKER.at("color").at(2).get<int>()};
                parsed.world = vec3From(MARKER.at("world"));
                parsed.size  = MARKER.at("size").get<double>();
                out.wallMarkers.push_back(parsed);
            }

            const json& CODE = SCENE.at("code_patch");
            out.codeCentre   = vec3From(CODE.at("centre"));
            out.codeSize     = CODE.at("size").get<double>();
            out.codeBase     = CODE.at("base").get<int>();
            out.codeStep     = CODE.at("step").get<int>();
            out.codeModulus  = CODE.at("modulus").get<int>();
            out.codeGreen    = CODE.at("green").get<int>();
            out.codeBlue     = CODE.at("blue").get<int>();

            const json& QUAD = SCENE.at("overlay_quad");
            out.overlayQuad  = poseFrom(QUAD.at("pose"));
            out.quadWidth    = QUAD.at("width").get<double>();
            out.quadHeight   = QUAD.at("height").get<double>();
            out.quadHalo     = QUAD.at("halo").get<double>();
            for (const auto& MARKER : QUAD.at("markers")) {
                SSynthOverlayMarker parsed;
                parsed.name  = MARKER.at("name").get<std::string>();
                parsed.color = {MARKER.at("color").at(0).get<int>(), MARKER.at("color").at(1).get<int>(), MARKER.at("color").at(2).get<int>()};
                parsed.u     = MARKER.at("u").get<double>();
                parsed.v     = MARKER.at("v").get<double>();
                parsed.size  = MARKER.at("size").get<double>();
                out.overlayMarkers.push_back(parsed);
            }

            out.ipd       = SCENE.at("eyes").at("ipd").get<double>();
            out.eyeFov[0] = fovFrom(SCENE.at("eyes").at("fov").at(0));
            out.eyeFov[1] = fovFrom(SCENE.at("eyes").at("fov").at(1));

            out.hasCameras = SCENE.at("has_cameras").get<bool>();
            for (const auto& CAMERA : SCENE.at("cameras")) {
                SSynthCamera parsed;
                parsed.key          = CAMERA.at("cam").get<std::string>();
                parsed.eye          = CAMERA.at("eye").get<int>();
                parsed.headToCamera = poseFrom(CAMERA.at("head_to_camera"));
                parsed.intrinsics   = intrinsicsFrom(CAMERA.at("intrinsics"));
                parsed.width        = CAMERA.at("width").get<int>();
                parsed.height       = CAMERA.at("height").get<int>();
                parsed.hz           = CAMERA.at("hz").get<double>();
                parsed.exposureNs   = CAMERA.at("exposure_ns").get<int64_t>();
                parsed.deviceNs     = CAMERA.at("t_device_ns").get<std::vector<int64_t>>();
                out.cameras.push_back(std::move(parsed));
            }

            out.hasApp = SCENE.at("has_app").get<bool>();
            out.app    = audioFrom(SCENE.at("app"));
            out.hasMic = SCENE.at("has_mic").get<bool>();
            out.mic    = audioFrom(SCENE.at("mic"));

            return out;
        } catch (const std::exception& e) {
            error = e.what();
            return std::nullopt;
        }
    }

    std::optional<SSynthScene> SSynthScene::load(const std::filesystem::path& takeRoot, std::string& error) {
        const auto    PATH = takeRoot / RELATIVE_PATH;
        std::ifstream stream(PATH);
        if (!stream) {
            error = PATH.string() + ": cannot be opened";
            return std::nullopt;
        }
        std::ostringstream buffer;
        buffer << stream.rdbuf();
        return fromJson(buffer.str(), error);
    }

}
