/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include <gtest/gtest.h>
#include "lotusim_common/logger.hpp"
#include <gz/sim/EntityComponentManager.hh>
#include "power_subsystem/sensor_power_consumer.hpp"

namespace lotusim::gazebo
{

// ──────────────────────────────────────────────────────────────    
//  helpers 
// ──────────────────────────────────────────────────────────────

static gz::sim::EntityComponentManager g_ecm;  // shared across all tests

static std::unique_ptr<SensorPowerConsumer> makeSensor(
    const std::string& name = "ais",
    float nominal_w = 10.0f,
    int priority = 3)
{
    return std::make_unique<SensorPowerConsumer>(
        name, nominal_w, priority, nullptr, nullptr, g_ecm);
}

// ──────────────────────────────────────────────────────────────
//  fixture 
// ──────────────────────────────────────────────────────────────

class SensorPowerConsumerTest : public ::testing::Test
{
protected:
    std::unique_ptr<SensorPowerConsumer> sensor = makeSensor();
};

// ──────────────────────────────────────────────────────────────
//  construction 
// ──────────────────────────────────────────────────────────────

TEST_F(SensorPowerConsumerTest, InitialStateIsActive)
{
    EXPECT_TRUE(sensor->isActive());
}

TEST_F(SensorPowerConsumerTest, NameIsPreserved)
{
    EXPECT_EQ(sensor->name(), "ais");
}

TEST_F(SensorPowerConsumerTest, PriorityIsPreserved)
{
    EXPECT_EQ(sensor->priority(), 3);
}

TEST_F(SensorPowerConsumerTest, NominalPowerIsPreserved)
{
    EXPECT_FLOAT_EQ(sensor->nominalPowerW(), 10.0f);
}

// ──────────────────────────────────────────────────────────────
//  drawnCurrent 
// ──────────────────────────────────────────────────────────────

TEST_F(SensorPowerConsumerTest, DrawnCurrentIsNominalDividedByVoltage)
{
    sensor->receiveVoltage(48.0f);
    EXPECT_NEAR(sensor->drawnCurrent(), 10.0f / 48.0f, 1e-5f);
}

TEST_F(SensorPowerConsumerTest, DrawnCurrentZeroWhenVoltageIsZero)
{
    sensor->receiveVoltage(0.0f);
    EXPECT_FLOAT_EQ(sensor->drawnCurrent(), 0.0f);
}

TEST_F(SensorPowerConsumerTest, DrawnCurrentZeroWhenInactive)
{
    sensor->receiveVoltage(48.0f);
    sensor->deactivate();
    EXPECT_FLOAT_EQ(sensor->drawnCurrent(), 0.0f);
}

TEST_F(SensorPowerConsumerTest, DrawnCurrentScalesWithVoltage)
{
    sensor->receiveVoltage(24.0f);
    const float i_24v = sensor->drawnCurrent();
    sensor->receiveVoltage(48.0f);
    const float i_48v = sensor->drawnCurrent();
    EXPECT_GT(i_24v, i_48v);
    EXPECT_NEAR(i_24v, 10.0f / 24.0f, 1e-5f);
    EXPECT_NEAR(i_48v, 10.0f / 48.0f, 1e-5f);
}

TEST_F(SensorPowerConsumerTest, DrawnCurrentDifferentNominalPower)
{
    auto s5w  = makeSensor("s5w",  5.0f);
    auto s20w = makeSensor("s20w", 20.0f);
    s5w->receiveVoltage(48.0f);
    s20w->receiveVoltage(48.0f);
    EXPECT_NEAR(s5w->drawnCurrent(),  5.0f / 48.0f, 1e-5f);
    EXPECT_NEAR(s20w->drawnCurrent(), 20.0f / 48.0f, 1e-5f);
}

// ──────────────────────────────────────────────────────────────
//  deactivate / reactivate 
// ──────────────────────────────────────────────────────────────

TEST_F(SensorPowerConsumerTest, DeactivateStopsCurrentDraw)
{
    sensor->receiveVoltage(48.0f);
    sensor->deactivate();
    EXPECT_FALSE(sensor->isActive());
    EXPECT_FLOAT_EQ(sensor->drawnCurrent(), 0.0f);
}

TEST_F(SensorPowerConsumerTest, ReactivateRestoresCurrentDraw)
{
    sensor->receiveVoltage(48.0f);
    sensor->deactivate();
    sensor->reactivate();
    EXPECT_TRUE(sensor->isActive());
    EXPECT_NEAR(sensor->drawnCurrent(), 10.0f / 48.0f, 1e-5f);
}

TEST_F(SensorPowerConsumerTest, MultipleDeactivateCallsAreIdempotent)
{
    sensor->deactivate();
    sensor->deactivate();
    EXPECT_FALSE(sensor->isActive());
}

TEST_F(SensorPowerConsumerTest, MultipleReactivateCallsAreIdempotent)
{
    sensor->deactivate();
    sensor->reactivate();
    sensor->reactivate();
    EXPECT_TRUE(sensor->isActive());
}

// ──────────────────────────────────────────────────────────────
//  eachNew / eachDelete 
// ──────────────────────────────────────────────────────────────

TEST_F(SensorPowerConsumerTest, EachNewActivatesSensor)
{
    sensor->deactivate();
    EXPECT_FALSE(sensor->isActive());
    sensor->eachNew();
    EXPECT_TRUE(sensor->isActive());
}

TEST_F(SensorPowerConsumerTest, EachDeleteDeactivatesSensor)
{
    sensor->receiveVoltage(48.0f);
    EXPECT_TRUE(sensor->isActive());
    sensor->eachDelete();
    EXPECT_FALSE(sensor->isActive());
    EXPECT_FLOAT_EQ(sensor->drawnCurrent(), 0.0f);
}

TEST_F(SensorPowerConsumerTest, EachNewThenEachDeleteCycle)
{
    sensor->eachDelete();
    EXPECT_FALSE(sensor->isActive());
    sensor->eachNew();
    EXPECT_TRUE(sensor->isActive());
    sensor->eachDelete();
    EXPECT_FALSE(sensor->isActive());
}

// ──────────────────────────────────────────────────────────────
//  priority 
// ──────────────────────────────────────────────────────────────

TEST_F(SensorPowerConsumerTest, PrioritySafetyLevel1NeverShed)
{
    auto critical = makeSensor("nav", 5.0f, 1);
    EXPECT_EQ(critical->priority(), 1);
}

TEST_F(SensorPowerConsumerTest, PriorityNonEssentialLevel4ShedFirst)
{
    auto nonessential = makeSensor("deck_light", 2.0f, 4);
    EXPECT_EQ(nonessential->priority(), 4);
}

TEST_F(SensorPowerConsumerTest, HigherPriorityNumberShedFirst)
{
    auto critical = makeSensor("nav", 5.0f, 1);
    auto nonessential = makeSensor("deck_light", 2.0f, 4);
    EXPECT_LT(critical->priority(), nonessential->priority());
}

// ──────────────────────────────────────────────────────────────
//  multiple vessel isolation 
// ──────────────────────────────────────────────────────────────

TEST_F(SensorPowerConsumerTest, TwoVesselSensorsAreIndependent)
{
    auto v1_ais = makeSensor("dtmb_0_ais", 5.0f, 3);
    auto v2_ais = makeSensor("dtmb_1_ais", 5.0f, 3);

    v1_ais->receiveVoltage(48.0f);
    v2_ais->receiveVoltage(48.0f);

    v1_ais->deactivate();

    EXPECT_FALSE(v1_ais->isActive());
    EXPECT_TRUE(v2_ais->isActive());      
    EXPECT_FLOAT_EQ(v1_ais->drawnCurrent(), 0.0f);
    EXPECT_NEAR(v2_ais->drawnCurrent(), 5.0f / 48.0f, 1e-5f);
}

TEST_F(SensorPowerConsumerTest, TwoVesselsDifferentVoltages)
{
    auto v1 = makeSensor("v1_sensor", 10.0f);
    auto v2 = makeSensor("v2_sensor", 10.0f);

    v1->receiveVoltage(48.0f);
    v2->receiveVoltage(24.0f);

    EXPECT_NEAR(v1->drawnCurrent(), 10.0f / 48.0f, 1e-5f);
    EXPECT_NEAR(v2->drawnCurrent(), 10.0f / 24.0f, 1e-5f);
}

TEST_F(SensorPowerConsumerTest, ThreeVesselsSimultaneously)
{
    auto s1 = makeSensor("dtmb_0_ais", 5.0f, 3);
    auto s2 = makeSensor("dtmb_1_radar", 50.0f, 2);
    auto s3 = makeSensor("lrauv_0_sonar", 25.0f, 3);

    s1->receiveVoltage(48.0f);
    s2->receiveVoltage(48.0f);
    s3->receiveVoltage(36.0f);

    EXPECT_NEAR(s1->drawnCurrent(), 5.0f / 48.0f, 1e-5f);
    EXPECT_NEAR(s2->drawnCurrent(), 50.0f / 48.0f, 1e-5f);
    EXPECT_NEAR(s3->drawnCurrent(), 25.0f / 36.0f, 1e-5f);

    s3->deactivate();
    EXPECT_FLOAT_EQ(s3->drawnCurrent(), 0.0f);
    EXPECT_NEAR(s1->drawnCurrent(), 5.0f / 48.0f, 1e-5f);
    EXPECT_NEAR(s2->drawnCurrent(), 50.0f / 48.0f, 1e-5f);
}

// ──────────────────────────────────────────────────────────────
//  update is a no-op 
// ──────────────────────────────────────────────────────────────

TEST_F(SensorPowerConsumerTest, UpdateIsNoOp)
{
    sensor->receiveVoltage(48.0f);
    const float before = sensor->drawnCurrent();
    sensor->update(g_ecm);
    EXPECT_FLOAT_EQ(sensor->drawnCurrent(), before);
}

} // namespace lotusim::gazebo

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}