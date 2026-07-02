/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include <gtest/gtest.h>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <cmath>

// These free functions are defined in physics_interface_plugin
// (src/xdyn_websocket.cpp) and declared in xdyn_websocket.hpp. They are
// forward-declared here so the test does not pull in the plugin's heavy
// websocketpp / nlohmann-json includes; the test links the library to resolve
// them at link time.
namespace lotusim::gazebo {
gz::math::Quaterniond quatNedToEnu(const gz::math::Quaterniond& q_ned);
gz::math::Quaterniond quatEnuToNed(const gz::math::Quaterniond& q_enu);
gz::math::Vector3d vecNedToEnu(const gz::math::Vector3d& v_ned);
gz::math::Vector3d vecEnuToNed(const gz::math::Vector3d& v_enu);
}  // namespace lotusim::gazebo

using gz::math::Quaterniond;
using gz::math::Vector3d;
using lotusim::gazebo::quatEnuToNed;
using lotusim::gazebo::quatNedToEnu;
using lotusim::gazebo::vecEnuToNed;
using lotusim::gazebo::vecNedToEnu;

namespace {
constexpr double kTol = 1e-9;

const Quaterniond kIdentity(1.0, 0.0, 0.0, 0.0);

void expectQuatNear(const Quaterniond& a, const Quaterniond& b, double tol = kTol)
{
    EXPECT_NEAR(a.W(), b.W(), tol);
    EXPECT_NEAR(a.X(), b.X(), tol);
    EXPECT_NEAR(a.Y(), b.Y(), tol);
    EXPECT_NEAR(a.Z(), b.Z(), tol);
}
}  // namespace

// NED (X=North, Y=East, Z=Down) -> ENU (X=East, Y=North, Z=Up):
// swap X/Y, flip Z.
TEST(NedEnuConversions, VecNedToEnuKnownValue)
{
    const Vector3d enu = vecNedToEnu({1.0, 2.0, 3.0});
    EXPECT_NEAR(enu.X(), 2.0, kTol);
    EXPECT_NEAR(enu.Y(), 1.0, kTol);
    EXPECT_NEAR(enu.Z(), -3.0, kTol);
}

// The axis permutation is its own inverse: NED->ENU->NED is the identity.
// vecNedToEnu and vecEnuToNed share the same body on purpose (the swap is an
// involution) — this guards that from being "fixed" into a real bug.
TEST(NedEnuConversions, VecRoundTripIsIdentity)
{
    const Vector3d v{1.0, 2.0, 3.0};
    const Vector3d back = vecEnuToNed(vecNedToEnu(v));
    EXPECT_NEAR(back.X(), v.X(), kTol);
    EXPECT_NEAR(back.Y(), v.Y(), kTol);
    EXPECT_NEAR(back.Z(), v.Z(), kTol);
}

// An orientation converted NED->ENU->NED must come back unchanged.
TEST(NedEnuConversions, QuatRoundTripIsIdentity)
{
    const double d2r = M_PI / 180.0;
    const Quaterniond q(12.0 * d2r, -8.0 * d2r, 30.0 * d2r);  // roll, pitch, yaw
    expectQuatNear(q, quatEnuToNed(quatNedToEnu(q)));
}

// A level orientation stays level.
TEST(NedEnuConversions, IdentityMapsToIdentity)
{
    expectQuatNear(kIdentity, quatNedToEnu(kIdentity));
}

// Heading sign flips between frames: a +90 deg NED heading (clockwise from
// North, about Down) is a -90 deg ENU yaw (counter-clockwise from East, about
// Up). This is exactly the class of axis/handedness error a broken quaternion
// mapping produces (cf. the xdyn qj/qk fix).
TEST(NedEnuConversions, NedHeadingBecomesOppositeEnuYaw)
{
    const double yaw_ned = 90.0 * M_PI / 180.0;
    const Quaterniond q_ned(0.0, 0.0, yaw_ned);
    const Vector3d euler = quatNedToEnu(q_ned).Euler();  // roll, pitch, yaw
    EXPECT_NEAR(euler.X(), 0.0, 1e-6);
    EXPECT_NEAR(euler.Y(), 0.0, 1e-6);
    EXPECT_NEAR(euler.Z(), -yaw_ned, 1e-6);
}
