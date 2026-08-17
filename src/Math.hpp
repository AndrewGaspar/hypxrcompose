#pragma once

// The pose/projection algebra every other translation unit is measured against.
//
// Conventions, fixed once here because the whole bundle contract depends on them:
//
//   - World / tracking space is OpenXR's: right-handed, +X right, +Y up, -Z forward.
//     Every pose in telemetry.jsonl and every camera extrinsic lives in it.
//   - A pose is (position, rotation) with the rotation taking *local* vectors to
//     world, so world = rot * local + pos.
//   - A quaternion is stored (x, y, z, w) — the OpenXR/JSON order — and composes
//     left-to-right the usual way: (a * b) applied to v is a(b(v)).
//   - An SFov is OpenXR's XrFovf: four half-angles in radians, with `l` and `d`
//     normally negative. The frustum is asymmetric and the image plane is at z = -1.
//   - Camera intrinsics are OpenCV's: +X right, +Y **down**, +Z forward, pixel
//     coordinates with (0,0) at the centre of the top-left pixel's corner — i.e.
//     pixel centres sit at (col + 0.5, row + 0.5). The OpenXR -> OpenCV axis flip
//     (x, -y, -z) is applied inside projectPinhole()/unprojectPinhole(), so callers
//     hand these functions ordinary OpenXR camera-space directions.
//   - Image pixel coordinates are always CPU convention: x to the right, y down
//     from the top row. GL's bottom-up convention is confined to ComposeGL.

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace hxc {

    struct SVec3 {
        double x = 0.0, y = 0.0, z = 0.0;

        constexpr SVec3 operator+(const SVec3& o) const {
            return {x + o.x, y + o.y, z + o.z};
        }
        constexpr SVec3 operator-(const SVec3& o) const {
            return {x - o.x, y - o.y, z - o.z};
        }
        constexpr SVec3 operator*(double s) const {
            return {x * s, y * s, z * s};
        }
        constexpr SVec3 operator-() const {
            return {-x, -y, -z};
        }
        constexpr double dot(const SVec3& o) const {
            return x * o.x + y * o.y + z * o.z;
        }
        constexpr SVec3 cross(const SVec3& o) const {
            return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
        }
        double length() const {
            return std::sqrt(dot(*this));
        }
        SVec3 normalized() const {
            const double L = length();
            return L > 0.0 ? *this * (1.0 / L) : SVec3{0.0, 0.0, -1.0};
        }
        bool finite() const {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }
    };

    struct SQuat {
        double x = 0.0, y = 0.0, z = 0.0, w = 1.0;

        static SQuat identity() {
            return {};
        }
        // Right-handed rotation of `angle` radians about `axis`.
        static SQuat fromAxisAngle(const SVec3& axis, double angle);
        // Intrinsic yaw (about +Y) then pitch (about +X) then roll (about -Z),
        // which is the order that reads naturally for a head pose.
        static SQuat fromYawPitchRoll(double yaw, double pitch, double roll);

        double norm() const {
            return std::sqrt(x * x + y * y + z * z + w * w);
        }
        SQuat  normalized() const;
        SQuat  conjugate() const {
            return {-x, -y, -z, w};
        }
        // Unit quaternions only, which is all this program ever produces.
        SQuat inverse() const {
            return conjugate();
        }
        SQuat  operator*(const SQuat& o) const;
        SVec3  rotate(const SVec3& v) const;
        // Rotation matrix in column-major order (m[col][row]) ready for glUniformMatrix3fv.
        std::array<double, 9> toMat3ColumnMajor() const;

        // Tangent-space log/exp about the identity: log() returns the rotation
        // vector (axis * angle), exp() inverts it. Used by the pose smoother.
        SVec3       log() const;
        static SQuat exp(const SVec3& rotationVector);

        bool  finite() const {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
        }
    };

    // Shortest-arc interpolation; `q1` is sign-flipped when the dot product is
    // negative so the path never takes the long way round.
    SQuat slerp(const SQuat& q0, const SQuat& q1, double t);

    struct SPose {
        SVec3 pos;
        SQuat rot;

        SVec3 pointToWorld(const SVec3& local) const {
            return rot.rotate(local) + pos;
        }
        SVec3 pointToLocal(const SVec3& world) const {
            return rot.inverse().rotate(world - pos);
        }
        SVec3 dirToWorld(const SVec3& local) const {
            return rot.rotate(local);
        }
        SVec3 dirToLocal(const SVec3& world) const {
            return rot.inverse().rotate(world);
        }
        SPose inverse() const {
            const SQuat INV = rot.inverse();
            return {INV.rotate(-pos), INV};
        }
        // this ∘ child: the child expressed in this pose's parent frame.
        SPose compose(const SPose& child) const {
            return {pointToWorld(child.pos), rot * child.rot};
        }
        bool finite() const {
            return pos.finite() && rot.finite();
        }
    };

    struct SFov {
        double l = -0.7, r = 0.7, u = 0.7, d = -0.7;

        // Tangents in the order the shader wants them.
        std::array<double, 4> tangents() const {
            return {std::tan(l), std::tan(r), std::tan(u), std::tan(d)};
        }
        bool sane() const;
    };

    // A direction in eye space (z negative, unnormalized) for the centre of pixel
    // (px, py) of a `width` x `height` image rendered with this frustum.
    SVec3 fovRay(const SFov& fov, double px, double py, int width, int height);

    // Inverse of fovRay(). Returns false when the direction is at or behind the
    // eye plane; the pixel may land outside the image, which the caller checks.
    bool fovProject(const SFov& fov, const SVec3& dirEye, int width, int height, double& px, double& py);

    struct SCameraIntrinsics {
        double              fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
        // OpenCV order: k1, k2, p1, p2, k3. Shorter vectors are zero-extended.
        std::vector<double> distortion;

        std::array<double, 5> distortion5() const;
        // Horizontal/vertical field of view implied by fx/fy and the image size,
        // ignoring distortion. Diagnostics only.
        double                hfovDegrees(int width) const;
    };

    // Projects an OpenXR-convention camera-space direction to a pixel. Returns
    // false when the point is behind the lens.
    bool projectPinhole(const SCameraIntrinsics& intr, const SVec3& dirCamera, double& px, double& py);

    // Inverse of projectPinhole(): pixel -> unnormalized OpenXR-convention
    // direction. The distortion inverse is a fixed-point iteration; it converges
    // in a handful of steps for the mild coefficients real headset cameras carry.
    SVec3 unprojectPinhole(const SCameraIntrinsics& intr, double px, double py);

    // The v1 reprojection kernel, shared by the CPU predictors and mirrored in
    // GLSL by ComposeGL. `depth` is the assumed distance to the content along the
    // output ray; an infinite depth degenerates to a pure rotation, which is what
    // "no parallax correction" means.
    //
    // Returns the world-space point the output pixel is looking at.
    SVec3 assumedDepthPoint(const SPose& outputCamera, const SVec3& dirWorld, double depth);

    inline bool depthIsInfinite(double depth) {
        return !(depth > 0.0) || !std::isfinite(depth);
    }

    // Convenience wrappers used by the tests' predictors. `depth` semantics as above.
    bool reprojectToCamera(const SPose& outputCamera, const SVec3& dirWorld, double depth, const SPose& cameraPose, const SCameraIntrinsics& intr, double& px, double& py);
    bool reprojectToFov(const SPose& outputCamera, const SVec3& dirWorld, double depth, const SPose& sourcePose, const SFov& fov, int width, int height, double& px, double& py);

    // Formatting helpers for diagnostics.
    std::string toString(const SVec3& v);
    std::string toString(const SQuat& q);

}
