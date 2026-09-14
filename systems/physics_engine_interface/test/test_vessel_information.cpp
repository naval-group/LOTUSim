/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "physics_engine_interface/vessel_information.hpp"

#include <gz/math/Rand.hh>
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>

using namespace lotusim::gazebo;

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

    const double sign = (sqrt1MinusU1 * std::sin(u2))<0.0?-1.0:+1.0;
    gz::math::Quaterniond res(
        sign * sqrt1MinusU1 * std::sin(u2),   // w
        sign * sqrt1MinusU1 * std::cos(u2),   // x
        sign * sqrtU1 * std::sin(u3),         // y
        sign * sqrtU1 * std::cos(u3));        // z
    res.Normalize();
    return res;
}

class VesselInformationTest : public ::testing::Test {
protected:
    VesselInformation vessel;

    void SetUp() override
    {
        vessel.time = 12.5;
        vessel.convention = Convention::GAZEBO;
        vessel.pose = gz::math::Pose3d(
            gz::math::Vector3d(10.0, 20.0, 30.0),
            gz::math::Quaterniond::Identity);
        vessel.lin_vel = gz::math::Vector3d(1.0, 2.0, 3.0);
        vessel.ang_vel = gz::math::Vector3d(4.0, 5.0, 6.0);
    }
};

void EXPECT_QUATERNION_NEAR(const gz::math::Quaterniond& actual, const gz::math::Quaterniond& expected, double tol=1e-12);
void EXPECT_QUATERNION_NEAR(const gz::math::Quaterniond& actual, const gz::math::Quaterniond& expected, double tol)
{
    if (actual.Dot(expected) < 0.0)
    {
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

TEST_F(VesselInformationTest, HasExpectedDefaultValues)
{
    const VesselInformation default_vessel;
    EXPECT_EQ(default_vessel.convention, Convention::GAZEBO);
    EXPECT_DOUBLE_EQ(default_vessel.time, 0.0);
    EXPECT_EQ(default_vessel.pose, gz::math::Pose3d());
    EXPECT_EQ(default_vessel.pose.Rot(), gz::math::Quaterniond::Identity);
    EXPECT_EQ(default_vessel.lin_vel, gz::math::Vector3d::Zero);
    EXPECT_EQ(default_vessel.ang_vel, gz::math::Vector3d::Zero);
}

TEST_F(VesselInformationTest, ConvertsToXdyn)
{
    const VesselInformation converted = vessel.to_xdyn();
    EXPECT_EQ(converted.convention, Convention::NED_FRD);
    EXPECT_DOUBLE_EQ(converted.time, vessel.time);
    EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(20.0, 10.0, -30.0));
    const gz::math::Quaterniond expected_quaternion =
        gz::math::Quaterniond(0, 1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), 0) *
        gz::math::Quaterniond(0, 1, 0, 0);
    EXPECT_QUATERNION_NEAR(converted.pose.Rot(), -expected_quaternion, 1e-12);
    EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(1.0, -2.0, -3.0));
    EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(4.0, -5.0, -6.0));
}

TEST_F(VesselInformationTest, ConvertsToUnity)
{
    const VesselInformation converted = vessel.to_unity();
    EXPECT_EQ(converted.convention, Convention::EUN_FUL);
    EXPECT_DOUBLE_EQ(converted.time, vessel.time);
    EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(10.0, 30.0, 20.0));
    EXPECT_EQ(converted.pose.Rot(), gz::math::Quaterniond::Identity);
    EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(1.0, 3.0, 2.0));
    EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-4.0, -6.0, -5.0));
}

TEST_F(VesselInformationTest, ConvertsToUnreal)
{
    const VesselInformation converted = vessel.to_unreal();
    EXPECT_EQ(converted.convention, Convention::NEU_FRU);
    EXPECT_DOUBLE_EQ(converted.time, vessel.time);
    EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(20.0, 10.0, 30.0));
    EXPECT_EQ(converted.pose.Rot(),
        gz::math::Quaterniond(std::sqrt(0.5), 0, 0, std::sqrt(0.5)));
    EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(1.0, -2.0, 3.0));
    EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-4.0, 5.0, -6.0));
}

TEST_F(VesselInformationTest, ConvertsFromXdyn)
{
    const VesselInformation converted = VesselInformation::from_xdyn(
        gz::math::Vector3d(20.0, 10.0, -30.0),
        gz::math::Quaterniond::Identity,
        gz::math::Vector3d(1.0, -2.0, -3.0),
        gz::math::Vector3d(4.0, -5.0, -6.0));

    const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(0,1.0/sqrt(2.0),1.0/sqrt(2.0),0) * gz::math::Quaterniond(0,1,0,0);
    EXPECT_EQ(converted.convention, Convention::GAZEBO);
    EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(10.0, 20.0, 30.0));
    EXPECT_QUATERNION_NEAR(converted.pose.Rot(), -expected_quaternion, 1e-12);
    EXPECT_EQ(converted.lin_vel, expected_quaternion.RotateVector(gz::math::Vector3d(1.0, 2.0, 3.0)));
    EXPECT_EQ(converted.ang_vel, expected_quaternion.RotateVector(gz::math::Vector3d(4.0, 5.0, 6.0)));
}

TEST_F(VesselInformationTest, ConvertsFromXdynRandom)
{
    const gz::math::Vector3d ned_xyz = RandomVector3d();
    const gz::math::Quaterniond ned_quat = RandomQuaternion();
    const gz::math::Vector3d ned_uvw = RandomVector3d();
    const gz::math::Vector3d ned_pqr = RandomVector3d();
    const VesselInformation converted = VesselInformation::from_xdyn(
        ned_xyz,
        ned_quat,
        ned_uvw,
        ned_pqr);

    gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(0,1.0/sqrt(2.0),1.0/sqrt(2.0),0) * ned_quat *gz::math::Quaterniond(0,1,0,0);
    EXPECT_EQ(converted.convention, Convention::GAZEBO);
    EXPECT_EQ(converted.pose.Pos(),
            gz::math::Vector3d(ned_xyz.Y(), ned_xyz.X(), -ned_xyz.Z()));
    EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
    EXPECT_EQ(converted.lin_vel,
            expected_quaternion.RotateVector(
                gz::math::Vector3d(ned_uvw.X(), -ned_uvw.Y(), -ned_uvw.Z())));
    EXPECT_EQ(converted.ang_vel,
            expected_quaternion.RotateVector(
            gz::math::Vector3d(ned_pqr.X(), -ned_pqr.Y(), -ned_pqr.Z())));
}

TEST_F(VesselInformationTest, FromXdynThenToXdynIsIdentity)
{
    const gz::math::Vector3d ned_xyz(20.0, -10.0, 30.0);
    gz::math::Quaterniond ned_quat(0.8, 0.2, -0.3, 0.4);
    ned_quat.Normalize();
    const gz::math::Vector3d ned_uvw(1.5, -2.5, 3.5);
    const gz::math::Vector3d ned_pqr(-4.5, 5.5, -6.5);

    const VesselInformation gazebo = VesselInformation::from_xdyn(
        ned_xyz, ned_quat, ned_uvw, ned_pqr);
    const VesselInformation round_trip = gazebo.to_xdyn();

    EXPECT_EQ(round_trip.convention, Convention::NED_FRD);
    EXPECT_NEAR(round_trip.pose.Pos().X(), ned_xyz.X(), 1e-12);
    EXPECT_NEAR(round_trip.pose.Pos().Y(), ned_xyz.Y(), 1e-12);
    EXPECT_NEAR(round_trip.pose.Pos().Z(), ned_xyz.Z(), 1e-12);
    EXPECT_NEAR(round_trip.pose.Rot().W(), ned_quat.W(), 1e-12);
    EXPECT_NEAR(round_trip.pose.Rot().X(), ned_quat.X(), 1e-12);
    EXPECT_NEAR(round_trip.pose.Rot().Y(), ned_quat.Y(), 1e-12);
    EXPECT_NEAR(round_trip.pose.Rot().Z(), ned_quat.Z(), 1e-12);
    EXPECT_NEAR(round_trip.lin_vel.X(), ned_uvw.X(), 1e-12);
    EXPECT_NEAR(round_trip.lin_vel.Y(), ned_uvw.Y(), 1e-12);
    EXPECT_NEAR(round_trip.lin_vel.Z(), ned_uvw.Z(), 1e-12);
    EXPECT_NEAR(round_trip.ang_vel.X(), ned_pqr.X(), 1e-12);
    EXPECT_NEAR(round_trip.ang_vel.Y(), ned_pqr.Y(), 1e-12);
    EXPECT_NEAR(round_trip.ang_vel.Z(), ned_pqr.Z(), 1e-12);
}

TEST_F(VesselInformationTest, ConvertsToXdynRandom)
{
    const gz::math::Vector3d xyz = RandomVector3d();
    const gz::math::Quaterniond quat = RandomQuaternion();
    const gz::math::Vector3d uvw = RandomVector3d();
    const gz::math::Vector3d pqr = RandomVector3d();
    const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
    const VesselInformation converted = state.to_xdyn();
    const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(0,1.0/sqrt(2.0),1.0/sqrt(2.0),0) * quat * gz::math::Quaterniond(0,1,0,0);
    EXPECT_EQ(converted.convention, Convention::NED_FRD);
    EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.Y(), xyz.X(), -xyz.Z()));
    EXPECT_QUATERNION_NEAR(converted.pose.Rot(), -expected_quaternion, 1e-12);
    const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
    EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), -body_uvw.Y(), -body_uvw.Z()));
    const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
    EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(body_pqr.X(), -body_pqr.Y(), -body_pqr.Z()));
}

TEST_F(VesselInformationTest, ConvertsToXdynRandom2)
{
    for (size_t i=0;i<10000;++i)
    {
        const gz::math::Vector3d xyz = RandomVector3d();
        const gz::math::Quaterniond quat = RandomQuaternion();
        const gz::math::Vector3d uvw = RandomVector3d();
        const gz::math::Vector3d pqr = RandomVector3d();
        const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
        const VesselInformation converted = state.to_xdyn();
        const double sign = (quat.W()+quat.Z())/sqrt(2.0)<0.0?-1.0:+1.0;
        gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(
            sign * (quat.W() + quat.Z()) / sqrt(2.0),
            sign * (quat.X() + quat.Y()) / sqrt(2.0),
            sign * (quat.X() - quat.Y()) / sqrt(2.0),
            sign * (quat.W() - quat.Z()) / sqrt(2.0));
        expected_quaternion.Normalize();
        EXPECT_EQ(converted.convention, Convention::NED_FRD);
        EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.Y(), xyz.X(), -xyz.Z()));
        EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
        const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
        EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), -body_uvw.Y(), -body_uvw.Z()));
        const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
        EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(body_pqr.X(), -body_pqr.Y(), -body_pqr.Z()));
    }
}

TEST_F(VesselInformationTest, ConvertsToXdynRandom2_Inplace)
{
    for (size_t i=0;i<10000;++i)
    {
        const gz::math::Vector3d xyz = RandomVector3d();
        const gz::math::Quaterniond quat = RandomQuaternion();
        const gz::math::Vector3d uvw = RandomVector3d();
        const gz::math::Vector3d pqr = RandomVector3d();
        const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
        VesselInformation converted(state);
        converted.convert_to_xdyn();
        const double sign = (quat.W()+quat.Z())/sqrt(2.0)<0.0?-1.0:+1.0;
        gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(
            sign * (quat.W() + quat.Z()) / sqrt(2.0),
            sign * (quat.X() + quat.Y()) / sqrt(2.0),
            sign * (quat.X() - quat.Y()) / sqrt(2.0),
            sign * (quat.W() - quat.Z()) / sqrt(2.0));
        expected_quaternion.Normalize();
        EXPECT_EQ(converted.convention, Convention::NED_FRD);
        EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.Y(), xyz.X(), -xyz.Z()));
        EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
        const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
        EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), -body_uvw.Y(), -body_uvw.Z()));
        const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
        EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(body_pqr.X(), -body_pqr.Y(), -body_pqr.Z()));
    }
}

TEST_F(VesselInformationTest, ConvertsToUnityRandom)
{
    const gz::math::Vector3d xyz = RandomVector3d();
    const gz::math::Quaterniond quat = RandomQuaternion();
    const gz::math::Vector3d uvw = RandomVector3d();
    const gz::math::Vector3d pqr = RandomVector3d();
    const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
    const VesselInformation converted = state.to_unity();
    const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(quat.W(), -quat.X(), -quat.Z(), -quat.Y());
    EXPECT_EQ(converted.convention, Convention::EUN_FUL);
    EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.X(), xyz.Z(), xyz.Y()));
    EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
    const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
    EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), body_uvw.Z(), body_uvw.Y()));
    const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
    EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-body_pqr.X(), -body_pqr.Z(), -body_pqr.Y()));
}

TEST_F(VesselInformationTest, ConvertsToUnityRandom2)
{
    for (size_t i=0;i<10000;++i)
    {
        const gz::math::Vector3d xyz = RandomVector3d();
        const gz::math::Quaterniond quat = RandomQuaternion();
        const gz::math::Vector3d uvw = RandomVector3d();
        const gz::math::Vector3d pqr = RandomVector3d();
        const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
        const VesselInformation converted = state.to_unity();
        const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(quat.W(), -quat.X(), -quat.Z(), -quat.Y());
        EXPECT_EQ(converted.convention, Convention::EUN_FUL);
        EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.X(), xyz.Z(), xyz.Y()));
        EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
        const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
        EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), body_uvw.Z(), body_uvw.Y()));
        const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
        EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-body_pqr.X(), -body_pqr.Z(), -body_pqr.Y()));
    }
}

TEST_F(VesselInformationTest, ConvertsToUnityRandom2_Inplace)
{
    for (size_t i=0;i<10000;++i)
    {
        const gz::math::Vector3d xyz = RandomVector3d();
        const gz::math::Quaterniond quat = RandomQuaternion();
        const gz::math::Vector3d uvw = RandomVector3d();
        const gz::math::Vector3d pqr = RandomVector3d();
        const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
        VesselInformation converted(state);
        converted.convert_to_unity();
        const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(quat.W(), -quat.X(), -quat.Z(), -quat.Y());
        EXPECT_EQ(converted.convention, Convention::EUN_FUL);
        EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.X(), xyz.Z(), xyz.Y()));
        EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
        const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
        EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), body_uvw.Z(), body_uvw.Y()));
        const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
        EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-body_pqr.X(), -body_pqr.Z(), -body_pqr.Y()));
    }
}

TEST_F(VesselInformationTest, ConvertsToUnRealRandom)
{
    const gz::math::Vector3d xyz = RandomVector3d();
    const gz::math::Quaterniond quat = RandomQuaternion();
    const gz::math::Vector3d uvw = RandomVector3d();
    const gz::math::Vector3d pqr = RandomVector3d();
    const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
    const VesselInformation converted = state.to_unreal();

    EXPECT_EQ(converted.convention, Convention::NEU_FRU);
    EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.Y(), xyz.X(), xyz.Z()));

    // NEU_FRU has no quaternion-constant shortcut: the world relabeling
    // (X<->Y swap) and body relabeling (Left<->Right flip) are different
    // matrices, and each is individually improper (det = -1), so there's
    // no single quaternion to hand-multiply on either side. The expected
    // value has to go through the rotation matrix, same as quatChangeFrame.
    const gz::math::Matrix3d worldC(0, 1, 0,  1, 0, 0,  0, 0, 1);   // ENU <-> NEU
    const gz::math::Matrix3d bodyC(1, 0, 0,  0, -1, 0,  0, 0, 1);   // FLU <-> FRU
    const gz::math::Matrix3d R(quat);
    gz::math::Quaterniond expected_quaternion(worldC.Inverse() * R * bodyC);
    expected_quaternion.Normalize();
    EXPECT_EQ(converted.pose.Rot(), expected_quaternion);

    // Linear velocity: ordinary vector, just relabeled (Y flips sign).
    const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
    EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), -body_uvw.Y(), body_uvw.Z()));

    // Angular velocity: pseudovector, so it picks up an EXTRA det(bodyC) = -1
    // global sign flip on top of the relabeling -- all three components
    // flip sign, they don't get rearranged.
    const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
    EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-body_pqr.X(), body_pqr.Y(), -body_pqr.Z()));
}

TEST_F(VesselInformationTest, ConvertsToUnRealRandom2)
{
    for (size_t i=0;i<10000;++i)
    {
        const gz::math::Vector3d xyz = RandomVector3d();
        const gz::math::Quaterniond quat = RandomQuaternion();
        const gz::math::Vector3d uvw = RandomVector3d();
        const gz::math::Vector3d pqr = RandomVector3d();
        const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
        const VesselInformation converted = state.to_unreal();
        const double sign = (quat.W()+quat.Z())/sqrt(2.0)<0.0?-1.0:+1.0;
        gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(
            +sign * (quat.W() + quat.Z()) / sqrt(2.0),
            -sign * (quat.X() + quat.Y()) / sqrt(2.0),
            -sign * (quat.X() - quat.Y()) / sqrt(2.0),
            +sign * (quat.W() - quat.Z()) / sqrt(2.0));
        expected_quaternion.Normalize();
        EXPECT_EQ(converted.convention, Convention::NEU_FRU);
        EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.Y(), xyz.X(), xyz.Z()));
        EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
        const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
        EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), -body_uvw.Y(), body_uvw.Z()));
        const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
        EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-body_pqr.X(), +body_pqr.Y(), -body_pqr.Z()));
    }
}

TEST_F(VesselInformationTest, RejectsConversionsFromOtherConventions)
{
    vessel.convention = Convention::NED_FRD;

    EXPECT_THROW(vessel.to_xdyn(), std::runtime_error);
    EXPECT_THROW(vessel.to_unity(), std::runtime_error);
    EXPECT_THROW(vessel.to_unreal(), std::runtime_error);
}

TEST_F(VesselInformationTest, ConvertsToUnRealRandom2_Inplace)
{
    for (size_t i=0;i<10000;++i)
    {
        const gz::math::Vector3d xyz = RandomVector3d();
        const gz::math::Quaterniond quat = RandomQuaternion();
        const gz::math::Vector3d uvw = RandomVector3d();
        const gz::math::Vector3d pqr = RandomVector3d();
        const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
        VesselInformation converted(state);
        converted.convert_to_unreal();
        const double sign = (quat.W()+quat.Z())/sqrt(2.0)<0.0?-1.0:+1.0;
        gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(
            +sign * (quat.W() + quat.Z()) / sqrt(2.0),
            -sign * (quat.X() + quat.Y()) / sqrt(2.0),
            -sign * (quat.X() - quat.Y()) / sqrt(2.0),
            +sign * (quat.W() - quat.Z()) / sqrt(2.0));
        expected_quaternion.Normalize();
        EXPECT_EQ(converted.convention, Convention::NEU_FRU);
        EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(xyz.Y(), xyz.X(), xyz.Z()));
        EXPECT_QUATERNION_NEAR(converted.pose.Rot(), expected_quaternion, 1e-12);
        const gz::math::Vector3d body_uvw = quat.RotateVectorReverse(uvw);
        EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(body_uvw.X(), -body_uvw.Y(), body_uvw.Z()));
        const gz::math::Vector3d body_pqr = quat.RotateVectorReverse(pqr);
        EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-body_pqr.X(), +body_pqr.Y(), -body_pqr.Z()));
    }
}