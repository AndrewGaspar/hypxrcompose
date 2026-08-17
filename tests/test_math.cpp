// Pose algebra and projection models, checked against hand-computed cases rather
// than against themselves wherever a closed form exists.

#include "Math.hpp"

#include <gtest/gtest.h>

using namespace hxc;

namespace {
    constexpr double EPS = 1e-12;

    void expectVec(const SVec3& got, const SVec3& want, double tolerance) {
        EXPECT_NEAR(got.x, want.x, tolerance) << "x";
        EXPECT_NEAR(got.y, want.y, tolerance) << "y";
        EXPECT_NEAR(got.z, want.z, tolerance) << "z";
    }
}

TEST(Quaternion, QuarterTurnAboutYSendsForwardToLeft) {
    // A right-handed quarter turn about +Y takes -Z (forward) to -X (left).
    const SQuat Q = SQuat::fromAxisAngle({0, 1, 0}, M_PI / 2);
    EXPECT_NEAR(Q.x, 0.0, EPS);
    EXPECT_NEAR(Q.y, std::sqrt(0.5), 1e-15);
    EXPECT_NEAR(Q.z, 0.0, EPS);
    EXPECT_NEAR(Q.w, std::sqrt(0.5), 1e-15);
    expectVec(Q.rotate({0, 0, -1}), {-1, 0, 0}, 1e-15);
    expectVec(Q.rotate({1, 0, 0}), {0, 0, -1}, 1e-15);
}

TEST(Quaternion, CompositionIsLeftToRightAndMatchesTheHandComputedProduct) {
    // (0,0,0,1)-normalized half turns: about X then about Y. Hand product:
    //   qx = (1,0,0,0), qy = (0,1,0,0)  ->  qy*qx = (0,0,-1,0)? Check by rotation.
    const SQuat QX = SQuat::fromAxisAngle({1, 0, 0}, M_PI);
    const SQuat QY = SQuat::fromAxisAngle({0, 1, 0}, M_PI);
    const SQuat QZ = QY * QX;

    // Rotating pi about X then pi about Y is a pi rotation about Z.
    expectVec(QZ.rotate({1, 0, 0}), {-1, 0, 0}, 1e-14);
    expectVec(QZ.rotate({0, 1, 0}), {0, -1, 0}, 1e-14);
    expectVec(QZ.rotate({0, 0, 1}), {0, 0, 1}, 1e-14);

    // And the composition really is a(b(v)), not b(a(v)).
    const SVec3 V{0.3, -0.7, 0.2};
    expectVec(QZ.rotate(V), QY.rotate(QX.rotate(V)), 1e-14);
}

TEST(Quaternion, InverseUndoesRotationForArbitraryAxes) {
    const SQuat Q = SQuat::fromAxisAngle({0.3, -0.9, 0.4}, 1.234);
    const SVec3 V{1.5, -0.25, 3.0};
    expectVec(Q.inverse().rotate(Q.rotate(V)), V, 1e-14);
    expectVec((Q * Q.inverse()).rotate(V), V, 1e-14);
}

TEST(Quaternion, MatrixColumnsAreTheImagesOfTheLocalAxes) {
    const SQuat Q = SQuat::fromAxisAngle({0.2, 0.5, -0.84}, 0.77);
    const auto  M = Q.toMat3ColumnMajor();
    const SVec3 X = Q.rotate({1, 0, 0});
    const SVec3 Y = Q.rotate({0, 1, 0});
    const SVec3 Z = Q.rotate({0, 0, 1});
    EXPECT_NEAR(M[0], X.x, 1e-14);
    EXPECT_NEAR(M[1], X.y, 1e-14);
    EXPECT_NEAR(M[2], X.z, 1e-14);
    EXPECT_NEAR(M[3], Y.x, 1e-14);
    EXPECT_NEAR(M[4], Y.y, 1e-14);
    EXPECT_NEAR(M[5], Y.z, 1e-14);
    EXPECT_NEAR(M[6], Z.x, 1e-14);
    EXPECT_NEAR(M[7], Z.y, 1e-14);
    EXPECT_NEAR(M[8], Z.z, 1e-14);
}

TEST(Quaternion, LogAndExpRoundTripAndTakeTheShortWay) {
    const SQuat Q = SQuat::fromAxisAngle({0, 0, 1}, 0.6);
    const SVec3 L = Q.log();
    EXPECT_NEAR(L.z, 0.6, 1e-14);
    EXPECT_NEAR(SQuat::exp(L).z, Q.z, 1e-14);

    // The negated quaternion names the same rotation and must log identically.
    const SQuat NEGATED{-Q.x, -Q.y, -Q.z, -Q.w};
    expectVec(NEGATED.log(), L, 1e-14);
}

TEST(Slerp, EndpointsAndMidpointOfAKnownArc) {
    const SQuat A = SQuat::identity();
    const SQuat B = SQuat::fromAxisAngle({0, 1, 0}, 1.0);
    EXPECT_NEAR(slerp(A, B, 0.0).w, A.w, 1e-15);
    EXPECT_NEAR(slerp(A, B, 1.0).y, B.y, 1e-15);

    const SQuat HALF = slerp(A, B, 0.5);
    const SQuat WANT = SQuat::fromAxisAngle({0, 1, 0}, 0.5);
    EXPECT_NEAR(HALF.y, WANT.y, 1e-14);
    EXPECT_NEAR(HALF.w, WANT.w, 1e-14);
}

TEST(Pose, ComposeAndInverseAgreeWithExplicitTransforms) {
    const SPose PARENT{{1.0, 2.0, -3.0}, SQuat::fromAxisAngle({0, 1, 0}, 0.4)};
    const SPose CHILD{{0.1, -0.2, 0.3}, SQuat::fromAxisAngle({1, 0, 0}, -0.2)};
    const SPose COMPOSED = PARENT.compose(CHILD);

    const SVec3 LOCAL{0.5, 0.25, -0.75};
    expectVec(COMPOSED.pointToWorld(LOCAL), PARENT.pointToWorld(CHILD.pointToWorld(LOCAL)), 1e-14);
    expectVec(COMPOSED.inverse().pointToWorld(COMPOSED.pointToWorld(LOCAL)), LOCAL, 1e-14);
    expectVec(PARENT.pointToLocal(PARENT.pointToWorld(LOCAL)), LOCAL, 1e-14);
}

TEST(Fov, RayAndProjectAreInversesAcrossAnAsymmetricFrustum) {
    const SFov FOV{-0.95, 0.86, 0.75, -0.78};
    for (int y : {0, 137, 479}) {
        for (int x : {0, 311, 639}) {
            const SVec3 RAY = fovRay(FOV, x, y, 640, 480);
            double      px = 0.0, py = 0.0;
            ASSERT_TRUE(fovProject(FOV, RAY, 640, 480, px, py));
            EXPECT_NEAR(px, x, 1e-9);
            EXPECT_NEAR(py, y, 1e-9);
        }
    }
}

TEST(Fov, TheFrustumEdgesLandOnTheImageEdges) {
    const SFov FOV{-0.95, 0.86, 0.75, -0.78};
    double     px = 0.0, py = 0.0;

    // A ray exactly along the left/upper edge maps to pixel coordinate -0.5, which
    // is the outer boundary of the first pixel.
    ASSERT_TRUE(fovProject(FOV, {std::tan(FOV.l), std::tan(FOV.u), -1.0}, 640, 480, px, py));
    EXPECT_NEAR(px, -0.5, 1e-12);
    EXPECT_NEAR(py, -0.5, 1e-12);

    ASSERT_TRUE(fovProject(FOV, {std::tan(FOV.r), std::tan(FOV.d), -1.0}, 640, 480, px, py));
    EXPECT_NEAR(px, 639.5, 1e-9);
    EXPECT_NEAR(py, 479.5, 1e-9);
}

TEST(Fov, RaysBehindTheEyeAreRejected) {
    const SFov FOV{-0.95, 0.86, 0.75, -0.78};
    double     px = 0.0, py = 0.0;
    EXPECT_FALSE(fovProject(FOV, {0.0, 0.0, 1.0}, 640, 480, px, py));
    EXPECT_FALSE(fovProject(FOV, {0.0, 0.0, 0.0}, 640, 480, px, py));
}

TEST(Pinhole, PrincipalPointIsWhereTheOpticalAxisLands) {
    SCameraIntrinsics intrinsics;
    intrinsics.fx         = 190.0;
    intrinsics.fy         = 190.0;
    intrinsics.cx         = 323.5;
    intrinsics.cy         = 237.5;
    intrinsics.distortion = {-0.06, 0.008, 0.0009, -0.0006, 0.0};

    double px = 0.0, py = 0.0;
    // Straight ahead in OpenXR camera axes is -Z.
    ASSERT_TRUE(projectPinhole(intrinsics, {0.0, 0.0, -1.0}, px, py));
    EXPECT_NEAR(px, 323.5, 1e-12);
    EXPECT_NEAR(py, 237.5, 1e-12);
}

TEST(Pinhole, TheOpenXrToOpenCvAxisFlipPutsUpTowardTheTopRow) {
    SCameraIntrinsics intrinsics;
    intrinsics.fx = intrinsics.fy = 200.0;
    intrinsics.cx                 = 320.0;
    intrinsics.cy                 = 240.0;

    double px = 0.0, py = 0.0;
    // +Y is up in OpenXR; the top of an image is row 0, so up must decrease py.
    ASSERT_TRUE(projectPinhole(intrinsics, {0.0, 0.5, -1.0}, px, py));
    EXPECT_LT(py, 240.0);
    // +X is right in both conventions.
    ASSERT_TRUE(projectPinhole(intrinsics, {0.5, 0.0, -1.0}, px, py));
    EXPECT_GT(px, 320.0);
}

TEST(Pinhole, DistortionMatchesTheHandEvaluatedBrownConradyModel) {
    SCameraIntrinsics intrinsics;
    intrinsics.fx = intrinsics.fy = 100.0;
    intrinsics.cx                 = 0.0;
    intrinsics.cy                 = 0.0;
    intrinsics.distortion         = {-0.1, 0.02, 0.003, 0.004, 0.001};

    // Normalized coordinates (0.3, -0.4): r2 = 0.25.
    //   radial = 1 - 0.1*0.25 + 0.02*0.0625 + 0.001*0.015625 = 0.97627(id)
    //   xd = 0.3*radial + 2*p1*x*y + p2*(r2 + 2x^2)
    //      = 0.29288125 + 2*0.003*(-0.12) + 0.004*(0.25 + 0.18)
    //   yd = -0.4*radial + p1*(r2 + 2y^2) + 2*p2*x*y
    //      = -0.3905083... + 0.003*(0.25 + 0.32) + 2*0.004*(-0.12)
    const double RADIAL = 1.0 - 0.1 * 0.25 + 0.02 * 0.0625 + 0.001 * 0.015625;
    const double WANT_X = 0.3 * RADIAL + 2.0 * 0.003 * (0.3 * -0.4) + 0.004 * (0.25 + 2.0 * 0.09);
    const double WANT_Y = -0.4 * RADIAL + 0.003 * (0.25 + 2.0 * 0.16) + 2.0 * 0.004 * (0.3 * -0.4);

    double px = 0.0, py = 0.0;
    // OpenXR direction (0.3, +0.4, -1) has OpenCV normalized coords (0.3, -0.4).
    ASSERT_TRUE(projectPinhole(intrinsics, {0.3, 0.4, -1.0}, px, py));
    EXPECT_NEAR(px, 100.0 * WANT_X, 1e-12);
    EXPECT_NEAR(py, 100.0 * WANT_Y, 1e-12);
}

TEST(Pinhole, UnprojectInvertsProjectAcrossTheWholeFrame) {
    SCameraIntrinsics intrinsics;
    intrinsics.fx = intrinsics.fy = 190.0;
    intrinsics.cx                 = 323.5;
    intrinsics.cy                 = 237.5;
    intrinsics.distortion         = {-0.06, 0.008, 0.0009, -0.0006, 0.0};

    double worst = 0.0;
    for (int y = 0; y < 480; y += 17) {
        for (int x = 0; x < 640; x += 19) {
            const SVec3 RAY = unprojectPinhole(intrinsics, x + 0.5, y + 0.5);
            double      px = 0.0, py = 0.0;
            ASSERT_TRUE(projectPinhole(intrinsics, RAY, px, py)) << x << "," << y;
            worst = std::max(worst, std::max(std::abs(px - (x + 0.5)), std::abs(py - (y + 0.5))));
        }
    }
    // The fixed-point inverse converges far past the precision the compositor
    // needs; anything above a thousandth of a pixel would be a real defect.
    EXPECT_LT(worst, 1e-3);
}

TEST(Pinhole, ZeroDistortionIsTheIdentityOnNormalizedCoordinates) {
    SCameraIntrinsics intrinsics;
    intrinsics.fx = intrinsics.fy = 250.0;
    intrinsics.cx                 = 100.0;
    intrinsics.cy                 = 50.0;

    double px = 0.0, py = 0.0;
    ASSERT_TRUE(projectPinhole(intrinsics, {0.4, -0.2, -2.0}, px, py));
    EXPECT_NEAR(px, 250.0 * 0.2 + 100.0, 1e-12);
    EXPECT_NEAR(py, 250.0 * 0.1 + 50.0, 1e-12);
}

TEST(Reprojection, AnInfiniteDepthIsExactlyARotation) {
    const SPose OUTPUT{{0.1, 0.2, 0.3}, SQuat::fromAxisAngle({0, 1, 0}, 0.2)};
    const SPose SOURCE{{-5.0, 3.0, 1.0}, SQuat::fromAxisAngle({1, 0, 0}, -0.1)};
    const SFov  FOV{-0.9, 0.9, 0.8, -0.8};
    const SVec3 DIRECTION = SVec3{0.2, -0.1, -1.0}.normalized();

    double px = 0.0, py = 0.0;
    ASSERT_TRUE(reprojectToFov(OUTPUT, DIRECTION, std::numeric_limits<double>::infinity(), SOURCE, FOV, 640, 480, px, py));

    // The source's position must not matter at all when the depth is infinite.
    const SPose MOVED{{100.0, -60.0, 12.0}, SOURCE.rot};
    double      px2 = 0.0, py2 = 0.0;
    ASSERT_TRUE(reprojectToFov(OUTPUT, DIRECTION, std::numeric_limits<double>::infinity(), MOVED, FOV, 640, 480, px2, py2));
    EXPECT_NEAR(px, px2, 1e-12);
    EXPECT_NEAR(py, py2, 1e-12);
}

TEST(Reprojection, AFiniteDepthMovesWithTheSourcePositionByTheParallaxAmount) {
    const SPose OUTPUT{{0.0, 0.0, 0.0}, SQuat::identity()};
    const SFov  FOV{-0.9, 0.9, 0.8, -0.8};
    const SVec3 FORWARD{0.0, 0.0, -1.0};

    // A source shifted 10 cm to the right sees a point 2 m ahead shifted left by
    // atan(0.1/2) = 0.05 rad worth of tangent.
    const SPose SOURCE{{0.1, 0.0, 0.0}, SQuat::identity()};
    double      px = 0.0, py = 0.0;
    ASSERT_TRUE(reprojectToFov(OUTPUT, FORWARD, 2.0, SOURCE, FOV, 640, 480, px, py));

    const double TAN_SPAN  = std::tan(0.9) * 2.0;
    const double WANT_TAN  = -0.05;
    const double WANT_PIXEL = ((WANT_TAN - std::tan(-0.9)) / TAN_SPAN) * 640.0 - 0.5;
    EXPECT_NEAR(px, WANT_PIXEL, 1e-9);
}

TEST(Fov, RejectsFrustaThatAreNotUsable) {
    EXPECT_TRUE((SFov{-0.9, 0.9, 0.8, -0.8}).sane());
    EXPECT_FALSE((SFov{0.9, -0.9, 0.8, -0.8}).sane());  // l >= r
    EXPECT_FALSE((SFov{-0.9, 0.9, -0.8, 0.8}).sane());  // d >= u
    EXPECT_FALSE((SFov{-1.6, 0.9, 0.8, -0.8}).sane());  // past 90 degrees
    EXPECT_FALSE((SFov{std::nan(""), 0.9, 0.8, -0.8}).sane());
}

TEST(Srgb, KnownPointsOnTheTransferCurve) {
    EXPECT_NEAR(srgbToLinear(0.0), 0.0, 1e-15);
    EXPECT_NEAR(srgbToLinear(1.0), 1.0, 1e-15);
    // The linear segment below the 0.04045 knee has slope 1/12.92.
    EXPECT_NEAR(srgbToLinear(0.04), 0.04 / 12.92, 1e-15);
    // Mid grey: sRGB 0.5 is famously about 21.4% of the light.
    EXPECT_NEAR(srgbToLinear(0.5), 0.21404, 1e-5);
    EXPECT_NEAR(linearToSrgb(0.21404), 0.5, 1e-5);
    EXPECT_NEAR(linearToSrgb(0.0031308 / 2.0), 0.0031308 / 2.0 * 12.92, 1e-15);
}

TEST(Srgb, RoundTripsEveryByteValueExactly) {
    // The compositor decodes, blends, and encodes. An opaque pixel must survive
    // that unchanged, or every frame would drift on its own transfer curve.
    for (int byte = 0; byte <= 255; ++byte) {
        const double ENCODED = byte / 255.0;
        const double BACK    = linearToSrgb(srgbToLinear(ENCODED));
        EXPECT_EQ(static_cast<int>(std::lround(BACK * 255.0)), byte) << "byte " << byte;
    }
}

TEST(Srgb, IsClampedRatherThanUndefinedOutsideTheUnitRange) {
    EXPECT_EQ(srgbToLinear(-0.5), 0.0);
    EXPECT_EQ(srgbToLinear(1.5), 1.0);
    EXPECT_EQ(linearToSrgb(-0.5), 0.0);
    EXPECT_EQ(linearToSrgb(1.5), 1.0);
}

TEST(Srgb, BlendingInEncodedSpaceIsVisiblyWrongWhichIsWhyItIsNotDone) {
    // Half of sRGB 1.0 over black: linear light says 0.5 of the light, which
    // encodes to about 0.7354 - not 0.5. The gap is the artifact the compositor
    // exists to avoid.
    const double CORRECT = linearToSrgb(srgbToLinear(1.0) * 0.5 + srgbToLinear(0.0) * 0.5);
    EXPECT_NEAR(CORRECT, 0.7354, 1e-3);
    EXPECT_GT(std::abs(CORRECT - 0.5) * 255.0, 55.0);
}
