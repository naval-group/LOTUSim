/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_common/common.hpp"

#include <gz/math/Rand.hh>
#include <gtest/gtest.h>
#include <cmath>

using namespace lotusim::common;

namespace {

/**
 * @brief Generate a random 3D vector with each component uniformly distributed in [min, max].
 */
gz::math::Vector3d RandomVector3d(double min = -10.0, double max = 10.0)
{
    return gz::math::Vector3d(
        gz::math::Rand::DblUniform(min, max),
        gz::math::Rand::DblUniform(min, max),
        gz::math::Rand::DblUniform(min, max));
}

/**
 * @brief Returns a random unit quaternion (uniformly distributed on SO(3)).
 * Uses Shoemake's method for a proper uniform random rotation,
 * rather than naive random roll/pitch/yaw (which is NOT uniform).
 */
gz::math::Quaterniond RandomQuaternion()
{
    const double u1 = gz::math::Rand::DblUniform(0.0, 1.0);
    const double u2 = gz::math::Rand::DblUniform(0.0, 2.0 * GZ_PI);
    const double u3 = gz::math::Rand::DblUniform(0.0, 2.0 * GZ_PI);

    const double sqrt1MinusU1 = std::sqrt(1.0 - u1);
    const double sqrtU1 = std::sqrt(u1);

    const double sign = (sqrt1MinusU1 * std::sin(u2)) < 0.0 ? -1.0 : +1.0;
    gz::math::Quaterniond res(
        sign * sqrt1MinusU1 * std::sin(u2),   // w
        sign * sqrt1MinusU1 * std::cos(u2),   // x
        sign * sqrtU1 * std::sin(u3),         // y
        sign * sqrtU1 * std::cos(u3));        // z
    res.Normalize();
    return res;
}

/**
 * @brief Compare quaternions up to sign, since q and -q represent the same rotation.
 */
void EXPECT_QUATERNION_NEAR(const gz::math::Quaterniond& actual, const gz::math::Quaterniond& expected, double tol = 1e-12)
{
    if (actual.Dot(expected) < 0.0) {
        EXPECT_NEAR(-expected.W(), actual.W(), tol);
        EXPECT_NEAR(-expected.X(), actual.X(), tol);
        EXPECT_NEAR(-expected.Y(), actual.Y(), tol);
        EXPECT_NEAR(-expected.Z(), actual.Z(), tol);
        return;
    }
    EXPECT_NEAR(expected.W(), actual.W(), tol);
    EXPECT_NEAR(expected.X(), actual.X(), tol);
    EXPECT_NEAR(expected.Y(), actual.Y(), tol);
    EXPECT_NEAR(expected.Z(), actual.Z(), tol);
}

void EXPECT_VECTOR3_NEAR(const gz::math::Vector3d& actual, const gz::math::Vector3d& expected, double tol = 1e-12)
{
    EXPECT_NEAR(expected.X(), actual.X(), tol);
    EXPECT_NEAR(expected.Y(), actual.Y(), tol);
    EXPECT_NEAR(expected.Z(), actual.Z(), tol);
}

// A self-inverse, handedness-flipping axis swap (X<->Y), det = -1. Stands in
// for the ENU<->NED-style conversions these helpers exist to serve, without
// pulling any convention-specific knowledge into this generic-utility test.
const gz::math::Matrix3d kSwapXY(0, 1, 0,  1, 0, 0,  0, 0, 1);

// A proper rotation (90 deg about Z), det = +1: exercises the "no handedness
// flip" branch of pseudoVecBodyChangeFrame.
const gz::math::Matrix3d kRotateZ90(0, -1, 0,  1, 0, 0,  0, 0, 1);

}  // namespace

TEST(QuatChangeFrame, IdentityMatricesReturnSameQuaternion)
{
    const gz::math::Quaterniond q = RandomQuaternion();
    const gz::math::Quaterniond result =
        quatChangeFrame(q, gz::math::Matrix3d::Identity, gz::math::Matrix3d::Identity);
    EXPECT_QUATERNION_NEAR(result, q);
}

TEST(QuatChangeFrame, SelfInverseMatricesRoundTrip)
{
    const gz::math::Quaterniond q = RandomQuaternion();
    const gz::math::Quaterniond once = quatChangeFrame(q, kSwapXY, kSwapXY);
    const gz::math::Quaterniond twice = quatChangeFrame(once, kSwapXY, kSwapXY);
    EXPECT_QUATERNION_NEAR(twice, q);
}

TEST(PoseChangeFrame, AppliesPositionAndAttitudeIndependently)
{
    const gz::math::Pose3d pose(RandomVector3d(), RandomQuaternion());
    const gz::math::Pose3d converted = poseChangeFrame(pose, kSwapXY, kSwapXY);
    EXPECT_VECTOR3_NEAR(converted.Pos(), kSwapXY * pose.Pos());
    EXPECT_QUATERNION_NEAR(converted.Rot(), quatChangeFrame(pose.Rot(), kSwapXY, kSwapXY));
}

TEST(PoseChangeFrame, SelfInverseMatricesRoundTrip)
{
    const gz::math::Pose3d pose(RandomVector3d(), RandomQuaternion());
    const gz::math::Pose3d once = poseChangeFrame(pose, kSwapXY, kSwapXY);
    const gz::math::Pose3d twice = poseChangeFrame(once, kSwapXY, kSwapXY);
    EXPECT_VECTOR3_NEAR(twice.Pos(), pose.Pos());
    EXPECT_QUATERNION_NEAR(twice.Rot(), pose.Rot());
}

TEST(VecBodyChangeFrame, AppliesMatrixDirectly)
{
    const gz::math::Vector3d v = RandomVector3d();
    EXPECT_VECTOR3_NEAR(vecBodyChangeFrame(v, kSwapXY), kSwapXY * v);
}

TEST(PseudoVecBodyChangeFrame, ProperRotationMatchesOrdinaryRelabeling)
{
    // det(kRotateZ90) == +1: no handedness flip, so the pseudovector helper
    // and the ordinary vector helper must agree.
    const gz::math::Vector3d v = RandomVector3d();
    EXPECT_VECTOR3_NEAR(
        pseudoVecBodyChangeFrame(v, kRotateZ90),
        vecBodyChangeFrame(v, kRotateZ90));
}

TEST(PseudoVecBodyChangeFrame, ImproperReflectionFlipsSign)
{
    // det(kSwapXY) == -1: the pseudovector helper picks up an extra global
    // sign flip relative to the ordinary vector helper.
    const gz::math::Vector3d v = RandomVector3d();
    EXPECT_VECTOR3_NEAR(
        pseudoVecBodyChangeFrame(v, kSwapXY),
        -vecBodyChangeFrame(v, kSwapXY));
}

TEST(WorldVelToTargetBody, IdentityAttitudeAndBodyMatrixIsNoOp)
{
    const gz::math::Vector3d v = RandomVector3d();
    EXPECT_VECTOR3_NEAR(
        worldVelToTargetBody(
            v, gz::math::Quaterniond::Identity, gz::math::Matrix3d::Identity,
            /*isPseudoVector=*/false),
        v);
}

TEST(WorldVelToTargetBody, UndoesAttitudeThenRelabels)
{
    const gz::math::Vector3d v = RandomVector3d();
    const gz::math::Quaterniond q = RandomQuaternion();
    const gz::math::Vector3d expected = kSwapXY * q.RotateVectorReverse(v);
    EXPECT_VECTOR3_NEAR(
        worldVelToTargetBody(v, q, kSwapXY, /*isPseudoVector=*/false), expected);
}

TEST(TargetBodyVelToWorld, IsInverseOfWorldVelToTargetBody)
{
    const gz::math::Vector3d v = RandomVector3d();
    const gz::math::Quaterniond q = RandomQuaternion();
    const gz::math::Vector3d body =
        worldVelToTargetBody(v, q, kSwapXY, /*isPseudoVector=*/false);
    const gz::math::Vector3d back =
        targetBodyVelToWorld(body, q, kSwapXY, /*isPseudoVector=*/false);
    EXPECT_VECTOR3_NEAR(back, v);
}

TEST(TargetBodyVelToWorld, IsInverseOfWorldVelToTargetBodyForPseudoVector)
{
    const gz::math::Vector3d v = RandomVector3d();
    const gz::math::Quaterniond q = RandomQuaternion();
    const gz::math::Vector3d body =
        worldVelToTargetBody(v, q, kSwapXY, /*isPseudoVector=*/true);
    const gz::math::Vector3d back =
        targetBodyVelToWorld(body, q, kSwapXY, /*isPseudoVector=*/true);
    EXPECT_VECTOR3_NEAR(back, v);
}

// Concrete regression case: the ENU<->NED / FLU<->FRD conversion that
// physics_engine_interface's VesselInformation::to_xdyn used to compute
// inline (see physics_engine_interface's test_vessel_information.cpp,
// ConvertsToXdyn) before quatChangeFrame, poseChangeFrame and
// worldVelToTargetBody moved here. Pins the exact numbers so the move
// didn't change behavior.
TEST(FrameChangeRegression, MatchesPriorEnuNedXdynConversion)
{
    const gz::math::Matrix3d worldEnuNed(0, 1, 0,  1, 0, 0,  0, 0, -1);
    const gz::math::Matrix3d bodyFluFrd(1, 0, 0,  0, -1, 0,  0, 0, -1);

    const gz::math::Pose3d pose(
        gz::math::Vector3d(10.0, 20.0, 30.0), gz::math::Quaterniond::Identity);
    const gz::math::Vector3d lin_vel(1.0, 2.0, 3.0);
    const gz::math::Vector3d ang_vel(4.0, 5.0, 6.0);

    const gz::math::Pose3d converted_pose = poseChangeFrame(pose, worldEnuNed, bodyFluFrd);
    const gz::math::Vector3d converted_lin_vel =
        worldVelToTargetBody(lin_vel, pose.Rot(), bodyFluFrd, /*isPseudoVector=*/false);
    const gz::math::Vector3d converted_ang_vel =
        worldVelToTargetBody(ang_vel, pose.Rot(), bodyFluFrd, /*isPseudoVector=*/true);

    EXPECT_VECTOR3_NEAR(converted_pose.Pos(), gz::math::Vector3d(20.0, 10.0, -30.0));
    const gz::math::Quaterniond expected_quaternion =
        gz::math::Quaterniond(0, 1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), 0) *
        gz::math::Quaterniond(0, 1, 0, 0);
    EXPECT_QUATERNION_NEAR(converted_pose.Rot(), -expected_quaternion, 1e-12);
    EXPECT_VECTOR3_NEAR(converted_lin_vel, gz::math::Vector3d(1.0, -2.0, -3.0));
    EXPECT_VECTOR3_NEAR(converted_ang_vel, gz::math::Vector3d(4.0, -5.0, -6.0));
}
