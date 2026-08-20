#include "Math.hpp"

#include <format>

namespace hxc {

    SQuat SQuat::fromAxisAngle(const SVec3& axis, double angle) {
        const SVec3  UNIT = axis.normalized();
        const double HALF = angle * 0.5;
        const double S    = std::sin(HALF);
        return {UNIT.x * S, UNIT.y * S, UNIT.z * S, std::cos(HALF)};
    }

    SQuat SQuat::fromYawPitchRoll(double yaw, double pitch, double roll) {
        const SQuat Y = fromAxisAngle({0.0, 1.0, 0.0}, yaw);
        const SQuat P = fromAxisAngle({1.0, 0.0, 0.0}, pitch);
        const SQuat R = fromAxisAngle({0.0, 0.0, -1.0}, roll);
        return (Y * P * R).normalized();
    }

    SQuat SQuat::normalized() const {
        const double N = norm();
        if (!(N > 0.0))
            return identity();
        return {x / N, y / N, z / N, w / N};
    }

    SQuat SQuat::operator*(const SQuat& o) const {
        return {
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w,
            w * o.w - x * o.x - y * o.y - z * o.z,
        };
    }

    SVec3 SQuat::rotate(const SVec3& v) const {
        // v + 2w(q x v) + 2(q x (q x v)) — the standard expansion, which costs two
        // cross products instead of a quaternion sandwich.
        const SVec3 Q{x, y, z};
        const SVec3 T = Q.cross(v) * 2.0;
        return v + T * w + Q.cross(T);
    }

    std::array<double, 9> SQuat::toMat3ColumnMajor() const {
        const double XX = x * x, YY = y * y, ZZ = z * z;
        const double XY = x * y, XZ = x * z, YZ = y * z;
        const double WX = w * x, WY = w * y, WZ = w * z;

        // Column 0 is the image of the local +X axis, and so on.
        return {
            1.0 - 2.0 * (YY + ZZ), 2.0 * (XY + WZ),       2.0 * (XZ - WY),      //
            2.0 * (XY - WZ),       1.0 - 2.0 * (XX + ZZ), 2.0 * (YZ + WX),      //
            2.0 * (XZ + WY),       2.0 * (YZ - WX),       1.0 - 2.0 * (XX + YY),
        };
    }

    SVec3 SQuat::log() const {
        const SQuat  Q       = normalized();
        // Both q and -q name the same rotation; the positive-w representative is the
        // one whose log is the short way round.
        const double W       = Q.w < 0.0 ? -Q.w : Q.w;
        const double SIGN    = Q.w < 0.0 ? -1.0 : 1.0;
        const SVec3  AXIS    = SVec3{Q.x, Q.y, Q.z} * SIGN;
        const double SINHALF = AXIS.length();
        if (SINHALF < 1e-12)
            return AXIS * 2.0; // small-angle limit of 2*atan2(s, w)/s
        const double ANGLE = 2.0 * std::atan2(SINHALF, W);
        return AXIS * (ANGLE / SINHALF);
    }

    SQuat SQuat::exp(const SVec3& rotationVector) {
        const double ANGLE = rotationVector.length();
        if (ANGLE < 1e-12)
            return {rotationVector.x * 0.5, rotationVector.y * 0.5, rotationVector.z * 0.5, 1.0};
        return fromAxisAngle(rotationVector, ANGLE);
    }

    SQuat slerp(const SQuat& q0, const SQuat& q1, double t) {
        SQuat        a   = q0.normalized();
        SQuat        b   = q1.normalized();
        double       dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        if (dot < 0.0) {
            b   = {-b.x, -b.y, -b.z, -b.w};
            dot = -dot;
        }

        if (dot > 0.9995) {
            const SQuat LERPED{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t};
            return LERPED.normalized();
        }

        const double THETA = std::acos(dot);
        const double S     = std::sin(THETA);
        const double W0    = std::sin((1.0 - t) * THETA) / S;
        const double W1    = std::sin(t * THETA) / S;
        return SQuat{a.x * W0 + b.x * W1, a.y * W0 + b.y * W1, a.z * W0 + b.z * W1, a.w * W0 + b.w * W1}.normalized();
    }

    bool SFov::sane() const {
        if (!std::isfinite(l) || !std::isfinite(r) || !std::isfinite(u) || !std::isfinite(d))
            return false;
        if (!(r > l) || !(u > d))
            return false;
        // A half-angle at or past 90 degrees would put the frustum plane at infinity.
        const double LIMIT = 1.55;
        return std::abs(l) < LIMIT && std::abs(r) < LIMIT && std::abs(u) < LIMIT && std::abs(d) < LIMIT;
    }

    double angularPixelForPane(const SFov& fov, int width, int height) {
        if (width <= 0 || height <= 0)
            return 0.0;
        // The larger of the two demands wins, so the axis that needs more room
        // per pixel sets the scale and the other one gets padded.
        return std::max(fov.tanWidth() / static_cast<double>(width), fov.tanHeight() / static_cast<double>(height));
    }

    SFov fitFovToPane(const SFov& fov, int width, int height, double angularPixel) {
        if (width <= 0 || height <= 0)
            return fov;

        const double PIXEL = angularPixel > 0.0 ? angularPixel : angularPixelForPane(fov, width, height);
        if (!(PIXEL > 0.0))
            return fov;

        const auto   T    = fov.tangents();
        // Half the shortfall on each side, so the padding is symmetric about the
        // recorded frustum and the optical axis keeps pointing where it pointed.
        const double PADX = std::max(0.0, PIXEL * static_cast<double>(width) - fov.tanWidth()) * 0.5;
        const double PADY = std::max(0.0, PIXEL * static_cast<double>(height) - fov.tanHeight()) * 0.5;

        SFov out;
        out.l = std::atan(T[0] - PADX);
        out.r = std::atan(T[1] + PADX);
        out.u = std::atan(T[2] + PADY);
        out.d = std::atan(T[3] - PADY);
        return out;
    }

    SFov symmetrizeFov(const SFov& fov) {
        const auto   T    = fov.tangents();
        // The smaller half-extent on each axis wins, so the result is contained
        // in the original on all four sides.
        const double HALF_X = std::min(std::abs(T[0]), std::abs(T[1]));
        const double HALF_Y = std::min(std::abs(T[2]), std::abs(T[3]));
        return {-std::atan(HALF_X), std::atan(HALF_X), std::atan(HALF_Y), -std::atan(HALF_Y)};
    }

    SFov intersectFov(const SFov& a, const SFov& b) {
        return {std::max(a.l, b.l), std::min(a.r, b.r), std::min(a.u, b.u), std::max(a.d, b.d)};
    }

    SFov cropFovToPane(const SFov& fov, int width, int height) {
        if (width <= 0 || height <= 0)
            return fov;
        // The *smaller* demand wins here, which is what makes this a crop: the
        // axis with field to spare gives some up rather than the other axis
        // gaining any.
        const double PIXEL = std::min(fov.tanWidth() / static_cast<double>(width), fov.tanHeight() / static_cast<double>(height));
        if (!(PIXEL > 0.0))
            return fov;
        const double HALF_X = PIXEL * static_cast<double>(width) * 0.5;
        const double HALF_Y = PIXEL * static_cast<double>(height) * 0.5;
        return {-std::atan(HALF_X), std::atan(HALF_X), std::atan(HALF_Y), -std::atan(HALF_Y)};
    }

    SVec3 fovRay(const SFov& fov, double px, double py, int width, int height) {
        const auto   T = fov.tangents();
        const double U = (px + 0.5) / static_cast<double>(width);
        const double V = (py + 0.5) / static_cast<double>(height);
        // Row 0 is the top of the image, which is the `u` (upper) edge of the frustum.
        return {T[0] + U * (T[1] - T[0]), T[2] + V * (T[3] - T[2]), -1.0};
    }

    bool fovProject(const SFov& fov, const SVec3& dirEye, int width, int height, double& px, double& py) {
        if (!(dirEye.z < 0.0))
            return false;

        const auto   T    = fov.tangents();
        const double TANX = dirEye.x / -dirEye.z;
        const double TANY = dirEye.y / -dirEye.z;
        px                = ((TANX - T[0]) / (T[1] - T[0])) * static_cast<double>(width) - 0.5;
        py                = ((TANY - T[2]) / (T[3] - T[2])) * static_cast<double>(height) - 0.5;
        return std::isfinite(px) && std::isfinite(py);
    }

    namespace {
        // A half turn about +X: the flip between Android's camera convention and
        // OpenXR's, where a camera looks down -Z.
        const SQuat X_HALF_TURN{1.0, 0.0, 0.0, 0.0};
    }

    SQuat headToCameraFromAndroidRaw(const SQuat& rawSensorToCamera) {
        return (rawSensorToCamera.conjugate() * X_HALF_TURN).normalized();
    }

    SQuat androidRawFromHeadToCamera(const SQuat& headToCamera) {
        // A half turn is its own inverse, so the same flip undoes it.
        return (headToCamera * X_HALF_TURN).conjugate().normalized();
    }

    SQuat repairMirroredHeadToCamera(const SQuat& stored) {
        return (X_HALF_TURN * stored.inverse() * X_HALF_TURN).normalized();
    }

    double angleBetweenDegrees(const SQuat& a, const SQuat& b) {
        const double DOT = std::min(1.0, std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w));
        return 2.0 * std::acos(DOT) * 180.0 / M_PI;
    }

    double opticalAxisPitchDegrees(const SQuat& q) {
        const SVec3 AXIS = q.rotate({0.0, 0.0, -1.0});
        return std::asin(std::clamp(AXIS.y, -1.0, 1.0)) * 180.0 / M_PI;
    }

    SQuat twistAbout(const SQuat& q, const SVec3& axis) {
        const SVec3  A         = axis.normalized();
        const double PROJECTED = q.x * A.x + q.y * A.y + q.z * A.z;
        SQuat        twist{A.x * PROJECTED, A.y * PROJECTED, A.z * PROJECTED, q.w};
        if (twist.norm() < 1e-9)
            return SQuat::identity();
        twist = twist.normalized();
        // Keep the shortest turn, so a roll of -1 degree does not come back as 359.
        return twist.w < 0.0 ? SQuat{-twist.x, -twist.y, -twist.z, -twist.w} : twist;
    }

    bool SCameraIntrinsics::rebaseToImage(int width, int height) {
        const double AW = activeArray[2], AH = activeArray[3];
        if (!(AW > 0.0) || !(AH > 0.0) || width <= 0 || height <= 0)
            return false;
        const double EW = static_cast<double>(width), EH = static_cast<double>(height);
        if (std::abs(AW - EW) < 0.5 && std::abs(AH - EH) < 0.5 && std::abs(activeArray[0]) < 0.5 && std::abs(activeArray[1]) < 0.5)
            return false;

        // Android crops the active array to the output's aspect ratio about its
        // centre, then scales that region onto the output. Doing the same here
        // is what turns "intrinsics of the sensor" into "intrinsics of this
        // video".
        double cropW = AW, cropH = AH;
        if (AW / AH > EW / EH)
            cropW = AH * EW / EH;
        else
            cropH = AW * EH / EW;

        const double OFFSET_X = activeArray[0] + (AW - cropW) * 0.5;
        const double OFFSET_Y = activeArray[1] + (AH - cropH) * 0.5;
        const double SCALE    = EW / cropW;

        cx = (cx - OFFSET_X) * SCALE;
        cy = (cy - OFFSET_Y) * SCALE;
        fx *= SCALE;
        fy *= SCALE;
        return true;
    }

    std::array<double, 5> SCameraIntrinsics::distortion5() const {
        std::array<double, 5> out{0.0, 0.0, 0.0, 0.0, 0.0};
        for (size_t i = 0; i < out.size() && i < distortion.size(); ++i)
            out[i] = distortion[i];
        return out;
    }

    double SCameraIntrinsics::hfovDegrees(int width) const {
        if (!(fx > 0.0))
            return 0.0;
        return 2.0 * std::atan(0.5 * static_cast<double>(width) / fx) * 180.0 / M_PI;
    }

    namespace {
        // Brown-Conrady forward model on normalized image coordinates.
        void distort(const std::array<double, 5>& K, double xn, double yn, double& xd, double& yd) {
            const double R2     = xn * xn + yn * yn;
            const double RADIAL = 1.0 + K[0] * R2 + K[1] * R2 * R2 + K[4] * R2 * R2 * R2;
            xd                  = xn * RADIAL + 2.0 * K[2] * xn * yn + K[3] * (R2 + 2.0 * xn * xn);
            yd                  = yn * RADIAL + K[2] * (R2 + 2.0 * yn * yn) + 2.0 * K[3] * xn * yn;
        }
    }

    bool projectPinhole(const SCameraIntrinsics& intr, const SVec3& dirCamera, double& px, double& py) {
        // OpenXR camera space (-Z forward, +Y up) into OpenCV's (+Z forward, +Y down).
        const double ZCV = -dirCamera.z;
        if (!(ZCV > 1e-9))
            return false;

        const double XN = dirCamera.x / ZCV;
        const double YN = -dirCamera.y / ZCV;

        double       xd = 0.0, yd = 0.0;
        distort(intr.distortion5(), XN, YN, xd, yd);

        px = intr.fx * xd + intr.cx;
        py = intr.fy * yd + intr.cy;
        return std::isfinite(px) && std::isfinite(py);
    }

    SVec3 unprojectPinhole(const SCameraIntrinsics& intr, double px, double py) {
        const auto   K  = intr.distortion5();
        const double XD = (px - intr.cx) / intr.fx;
        const double YD = (py - intr.cy) / intr.fy;

        // Fixed-point inverse: repeatedly ask "what undistorted point would the
        // forward model have sent here?". Twenty iterations is far past convergence
        // for headset-class coefficients and costs nothing offline.
        double xn = XD, yn = YD;
        for (int i = 0; i < 20; ++i) {
            double fx = 0.0, fy = 0.0;
            distort(K, xn, yn, fx, fy);
            const double EX = fx - XD;
            const double EY = fy - YD;
            xn -= EX;
            yn -= EY;
            if (std::abs(EX) < 1e-13 && std::abs(EY) < 1e-13)
                break;
        }

        return {xn, -yn, -1.0};
    }

    SVec3 assumedDepthPoint(const SPose& outputCamera, const SVec3& dirWorld, double depth) {
        if (depthIsInfinite(depth))
            return dirWorld; // caller must treat the result as a direction
        return outputCamera.pos + dirWorld.normalized() * depth;
    }

    bool reprojectToCamera(const SPose& outputCamera, const SVec3& dirWorld, double depth, const SPose& cameraPose, const SCameraIntrinsics& intr, double& px, double& py) {
        SVec3 local;
        if (depthIsInfinite(depth))
            local = cameraPose.dirToLocal(dirWorld);
        else
            local = cameraPose.pointToLocal(assumedDepthPoint(outputCamera, dirWorld, depth));
        return projectPinhole(intr, local, px, py);
    }

    bool reprojectToFov(const SPose& outputCamera, const SVec3& dirWorld, double depth, const SPose& sourcePose, const SFov& fov, int width, int height, double& px, double& py) {
        SVec3 local;
        if (depthIsInfinite(depth))
            local = sourcePose.dirToLocal(dirWorld);
        else
            local = sourcePose.pointToLocal(assumedDepthPoint(outputCamera, dirWorld, depth));
        return fovProject(fov, local, width, height, px, py);
    }

    double srgbToLinear(double encoded) {
        if (encoded <= 0.0)
            return 0.0;
        if (encoded >= 1.0)
            return 1.0;
        return encoded <= 0.04045 ? encoded / 12.92 : std::pow((encoded + 0.055) / 1.055, 2.4);
    }

    double linearToSrgb(double linear) {
        if (linear <= 0.0)
            return 0.0;
        if (linear >= 1.0)
            return 1.0;
        return linear <= 0.0031308 ? linear * 12.92 : 1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
    }

    std::string toString(const SVec3& v) {
        return std::format("({:.6f}, {:.6f}, {:.6f})", v.x, v.y, v.z);
    }

    std::string toString(const SQuat& q) {
        return std::format("({:.6f}, {:.6f}, {:.6f}, {:.6f})", q.x, q.y, q.z, q.w);
    }

}
