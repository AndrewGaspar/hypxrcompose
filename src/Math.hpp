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

    // The part of `q` that turns about `axis` - the "twist" of a swing-twist
    // decomposition, q = swing * twist. For a camera whose local axis is its
    // optical axis, this is exactly the roll: the rest of the rotation is where
    // the lens is pointing, and the twist is how the image is turned about that
    // direction. Returns identity when the twist is undefined (a 180-degree
    // swing), which is the harmless answer.
    SQuat twistAbout(const SQuat& q, const SVec3& axis);

    // The head-to-camera rotation implied by Android's raw LENS_POSE_ROTATION.
    //
    // Two facts, both established from Meta's shipping code rather than guessed:
    //
    //  - LENS_POSE on Quest is HEAD-relative in spite of its GYROSCOPE label.
    //    `LENS_POSE_REFERENCE` reuses that enumerant, and Meta's own samples
    //    compose the pose directly onto the Head node with nothing but a 180
    //    degree X flip. The native docs say "relative to the center of the HMD".
    //    So there is no IMU-to-head constant to wait for.
    //  - Android documents the raw quaternion as SENSOR TO CAMERA (p' = Rp), so
    //    getting a head-to-camera out of it needs the conjugate. Omitting it is
    //    what mirrors the cant: a camera that really points 10.9 degrees DOWN is
    //    stored as pointing 10.9 degrees UP, a 21.7 degree error.
    //
    // The translation needs no conversion at all - the Android sensor frame
    // shares the OpenXR head axes - so only the rotation is computed here.
    SQuat headToCameraFromAndroidRaw(const SQuat& rawSensorToCamera);

    // Undoes the mirrored conversion above, for a bundle that stored the buggy
    // value and carries no raw to recompute from: Rx(180) * q^-1 * Rx(180),
    // which is the same rotation as negating x. Verified against both cameras of
    // the reference take.
    SQuat repairMirroredHeadToCamera(const SQuat& stored);

    // Inverse of headToCameraFromAndroidRaw: the raw sensor-to-camera quaternion
    // a device would have reported for a given head-to-camera rotation. The
    // synthetic bundle writes this so a real header's round trip is exercised.
    SQuat androidRawFromHeadToCamera(const SQuat& headToCamera);

    // Shortest angle between two rotations, in degrees. Diagnostics.
    double angleBetweenDegrees(const SQuat& a, const SQuat& b);
    // Elevation of the -Z optical axis after `q`, in degrees; positive is up.
    double opticalAxisPitchDegrees(const SQuat& q);

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

        // The frustum's extent in tan space, horizontal and vertical. This, not
        // the angles, is what a pinhole projection is linear in - so the ratio
        // below is the aspect the frustum actually *has*, and it is generally
        // not the pixel aspect of the buffer a runtime rendered it into.
        double tanWidth() const {
            return std::tan(r) - std::tan(l);
        }
        double tanHeight() const {
            return std::tan(u) - std::tan(d);
        }
        // Width over height in tan space. A frustum whose angular aspect differs
        // from the aspect of the image it is mapped onto renders squares as
        // rectangles, by exactly that ratio.
        double angularAspect() const {
            return tanWidth() / tanHeight();
        }
        // Where the optical axis (the direction with zero tangent) falls in the
        // image, as a fraction of width from the left and of height from the top.
        // An asymmetric frustum does not put it at the centre, and a compositor
        // that assumes it does shifts the whole picture.
        double opticalCentreU() const {
            return -std::tan(l) / tanWidth();
        }
        double opticalCentreV() const {
            return std::tan(u) / tanHeight();
        }
    };

    // The tan-space size of one output pixel needed to show all of `fov` on a
    // `width` x `height` pane without cropping and with angularly square pixels.
    double angularPixelForPane(const SFov& fov, int width, int height);

    // `fov` widened - never narrowed - so that it exactly fills a `width` x
    // `height` pane at `angularPixel` tan units per pixel.
    //
    // This is how an output camera is built from a recorded one. A recorded eye
    // frustum is asymmetric and its angular aspect is whatever the runtime chose;
    // the pane the user asked for has its own aspect. Mapping the first linearly
    // onto the second - which is what a compositor does if it just hands the
    // recorded fov to the shader - stretches the picture by the ratio between
    // them and slides the optical axis off where it belongs. Padding instead
    // keeps every recorded pixel, keeps the optical axis pointing where it
    // pointed, and makes output pixels square in angle, so a square in the world
    // comes out square on screen.
    //
    // Pass one `angularPixel` for every pane of a stereo pair: two eyes fitted
    // independently would come out at two different scales, which is a broken
    // stereo pair. 0 derives it from this frustum alone.
    SFov   fitFovToPane(const SFov& fov, int width, int height, double angularPixel = 0.0);

    // The largest frustum symmetric about forward that fits inside `fov`. Every
    // direction it covers, `fov` covered too - so symmetrizing only ever throws
    // periphery away, never invents any.
    SFov   symmetrizeFov(const SFov& fov);

    // The part both frusta see.
    SFov   intersectFov(const SFov& a, const SFov& b);

    // `fov` cropped - never padded - to exactly fill a `width` x `height` pane
    // with angularly square pixels, keeping it centred on forward. The opposite
    // trade to fitFovToPane: that one keeps all the field and pads the frame,
    // this one fills the frame and drops the periphery that does not fit.
    SFov   cropFovToPane(const SFov& fov, int width, int height);

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
        // The sensor region fx/fy/cx/cy are expressed in, when that is not the
        // delivered image. Android states intrinsics against the sensor's ACTIVE
        // ARRAY and then hands out a stream cropped and scaled from it, so the
        // two frames differ and the numbers cannot be used as they stand. Zero
        // width or height means "no active array declared", i.e. the intrinsics
        // are already in image coordinates.
        std::array<double, 4> activeArray{}; // x, y, w, h

        std::array<double, 5> distortion5() const;
        // Re-expresses fx/fy/cx/cy in the coordinates of a `width` x `height`
        // image delivered from `activeArray`, following Android's own pipeline:
        // the active array is cropped, centred, to the output's aspect, and then
        // scaled to the output's size. A no-op when no active array was declared
        // or when it already matches the image. Returns true when it changed
        // something.
        bool                  rebaseToImage(int width, int height);
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

    // The sRGB transfer function, on 0..1 values. Compositing is a *linear-light*
    // operation: blending encoded values is wrong at any partial alpha, and
    // visibly so - a 50% blend done in encoded space lands roughly a third of the
    // way from where it belongs. The overlay tap premultiplies in linear light and
    // encodes afterwards, so a stored byte is srgb_encode(color_linear * alpha),
    // and the compositor has to undo the encode before it can do anything with it.
    //
    // Alpha itself is never encoded; only the colour channels are.
    double srgbToLinear(double encoded);
    double linearToSrgb(double linear);

    // Formatting helpers for diagnostics.
    std::string toString(const SVec3& v);
    std::string toString(const SQuat& q);

}
