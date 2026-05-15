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
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

// test the logic directly without Gazebo or ROS2
// SDF and node construction are bypassed to test SimpleBattery and Battery

#include "power_subsystem/battery.hpp"
#include "power_subsystem/power_provider.hpp"

namespace lotusim::gazebo
{

// ──────────────────────────────────────────────────────────────
// Mirrors SimpleBattery but accepts plain constructor args 
// ──────────────────────────────────────────────────────────────

class TestBattery : public Battery
{
public:
    float t_voltage_nominal{48.0f};
    float t_remainingAh{100.0f};
    float t_voltage{48.0f};

    TestBattery(
        const std::string& name,
        float capacity_ah,
        float initial_soc,
        float voltage_min,
        float voltage_nominal)
        : Battery(name, nullptr, capacity_ah, initial_soc, voltage_min)
    {
        t_voltage_nominal = voltage_nominal;
        t_remainingAh = capacity_ah * initial_soc;
        t_voltage = voltage_min + initial_soc * (voltage_nominal - voltage_min);
    }

    // ──────────────────────────────────────────────────────────────
    // Battery interface -> same logic as SimpleBattery
    // ──────────────────────────────────────────────────────────────
    void receiveLoad(float currentA, float dt) override
    {
        t_remainingAh -= currentA * dt;
        t_remainingAh  = std::max(t_remainingAh, 0.0f);
        updateVoltage();
    }
 
    void receiveCharge(float currentA, float dt) override
    {
        t_remainingAh += currentA * dt;
        t_remainingAh  = std::min(t_remainingAh, m_capacityAh);
        updateVoltage();
    }
 
    float voltage() const override { return t_voltage; }
 
    float getStateOfCharge() const override
    {
        if (m_capacityAh <= 0.0f) { return 0.0f; }
        return std::clamp(t_remainingAh / m_capacityAh, 0.0f, 1.0f);
    }
 
    void updateVoltage() override
    {
        t_voltage = m_voltageMin
                  + getStateOfCharge() * (t_voltage_nominal - m_voltageMin);
    }

    float availablePowerW() const override
    {
        return voltage() * t_remainingAh;
    }
 
    // test helpers
    float remainingAh() const { return t_remainingAh; }
    float voltageNominal() const { return t_voltage_nominal; }
};

// ──────────────────────────────────────────────────────────────
// Helper 
// ──────────────────────────────────────────────────────────────

static std::unique_ptr<TestBattery> makeBattery(
    const std::string& name = "test_battery",
    float capacity_ah = 100.0f,
    float initial_soc = 1.0f,
    float voltage_min = 36.0f,
    float voltage_nominal = 48.0f)
{
    return std::make_unique<TestBattery>(
        name, capacity_ah, initial_soc, voltage_min, voltage_nominal);
}

// ──────────────────────────────────────────────────────────────
// Fixture
// ──────────────────────────────────────────────────────────────
 
class SimpleBatteryTest : public ::testing::Test
{
protected:
    // default fully-charged 100 Ah battery
    std::unique_ptr<TestBattery> bat = makeBattery();
};

// ──────────────────────────────────────────────────────────────
// Construction tests
// ──────────────────────────────────────────────────────────────

TEST_F(SimpleBatteryTest, FullChargeInitialState)
{
    EXPECT_FLOAT_EQ(bat->getStateOfCharge(), 1.0f);
    EXPECT_FLOAT_EQ(bat->voltage(), 48.0f);
    EXPECT_FALSE(bat->isDepleted());
    EXPECT_EQ(bat->powerLevel(), PowerLevel::NORMAL);
    EXPECT_EQ(bat->name(), "test_battery");
}

TEST_F(SimpleBatteryTest, HalfChargeInitialState)
{
    auto half = makeBattery("half", 100.0f, 0.5f, 36.0f, 48.0f);
    // voltage should be midpoint: 36 + 0.5*(48-36) = 42V
    EXPECT_NEAR(half->getStateOfCharge(), 0.5f, 1e-5f);
    EXPECT_NEAR(half->voltage(), 42.0f, 1e-4f);
    EXPECT_FALSE(half->isDepleted());
}

TEST_F(SimpleBatteryTest, EmptyInitialState)
{
    auto empty = makeBattery("empty", 100.0f, 0.0f, 36.0f, 48.0f);
    EXPECT_NEAR(empty->getStateOfCharge(), 0.0f, 1e-5f);
    EXPECT_NEAR(empty->voltage(), 36.0f, 1e-4f);
    EXPECT_TRUE(empty->isDepleted());
    EXPECT_EQ(empty->powerLevel(), PowerLevel::DEPLETED);
}

TEST_F(SimpleBatteryTest, NameIsPreserved)
{
    auto named = makeBattery("frigate1_battery");
    EXPECT_EQ(named->name(), "frigate1_battery");
}

TEST_F(SimpleBatteryTest, ZeroCapacityReturnsZeroSoc)
{
    TestBattery zero("zero", 0.0f, 1.0f, 36.0f, 48.0f);
    EXPECT_FLOAT_EQ(zero.getStateOfCharge(), 0.0f);
}

// ──────────────────────────────────────────────────────────────
// receiveLoad tests
// ──────────────────────────────────────────────────────────────

TEST_F(SimpleBatteryTest, ReceiveLoadDrainsCharge)
{
    // draw 10A for 1s : should consume 10 Ah but dt is in seconds
    // model uses Ah directly with dt in seconds:
    // remainingAh -= currentA * dt  (simplified, not hour-correct)
    const float before = bat->remainingAh();
    bat->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(bat->remainingAh(), before);
    EXPECT_LT(bat->getStateOfCharge(), 1.0f);
    EXPECT_LT(bat->voltage(), 48.0f);
}

TEST_F(SimpleBatteryTest, ReceiveLoadReducesVoltage)
{
    bat->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(bat->voltage(), 48.0f);
    EXPECT_GT(bat->voltage(), 36.0f); 
}

TEST_F(SimpleBatteryTest, ReceiveLoadDoesNotGoBelowZeroAh)
{
    // draw more than capacity
    bat->receiveLoad(1000.0f, 10000.0f);
    EXPECT_GE(bat->remainingAh(), 0.0f);
    EXPECT_GE(bat->getStateOfCharge(), 0.0f);
}

TEST_F(SimpleBatteryTest, ReceiveLoadDoesNotGoBelowVoltageMin)
{
    bat->receiveLoad(1000.0f, 10000.0f);
    EXPECT_GE(bat->voltage(), 36.0f);
}

TEST_F(SimpleBatteryTest, ReceiveLoadZeroCurrent)
{
    const float soc_before = bat->getStateOfCharge();
    const float voltage_before = bat->voltage();
    bat->receiveLoad(0.0f, 1.0f);
    EXPECT_FLOAT_EQ(bat->getStateOfCharge(), soc_before);
    EXPECT_FLOAT_EQ(bat->voltage(), voltage_before);
}

TEST_F(SimpleBatteryTest, ReceiveLoadZeroDt)
{
    const float soc_before = bat->getStateOfCharge();
    bat->receiveLoad(100.0f, 0.0f);
    EXPECT_FLOAT_EQ(bat->getStateOfCharge(), soc_before);
}

TEST_F(SimpleBatteryTest, ReceiveLoadProportionalDrain)
{
    // two loads of dt=0.5 should be one load of dt=1.0
    auto bat1 = makeBattery("b1");
    auto bat2 = makeBattery("b2");
 
    bat1->receiveLoad(10.0f, 1.0f);
    bat2->receiveLoad(10.0f, 0.5f);
    bat2->receiveLoad(10.0f, 0.5f);
 
    EXPECT_NEAR(bat1->getStateOfCharge(), bat2->getStateOfCharge(), 1e-5f);
    EXPECT_NEAR(bat1->voltage(), bat2->voltage(), 1e-4f);
}

// ──────────────────────────────────────────────────────────────
// receiveCharge tests
// ──────────────────────────────────────────────────────────────

TEST_F(SimpleBatteryTest, ReceiveChargeIncreasesCharge)
{
    // start half charged
    auto half = makeBattery("half", 100.0f, 0.5f, 36.0f, 48.0f);
    const float soc_before = half->getStateOfCharge();
    half->receiveCharge(10.0f, 1.0f);
    EXPECT_GT(half->getStateOfCharge(), soc_before);
    EXPECT_GT(half->voltage(), 42.0f);
}
 
TEST_F(SimpleBatteryTest, ReceiveChargeDoesNotExceedCapacity)
{
    // already full -> charging should not exceed capacity
    bat->receiveCharge(1000.0f, 1000.0f);
    EXPECT_LE(bat->remainingAh(), 100.0f);
    EXPECT_LE(bat->getStateOfCharge(), 1.0f);
    EXPECT_LE(bat->voltage(), 48.0f);
}
 
TEST_F(SimpleBatteryTest, ReceiveChargeDoesNotExceedVoltageNominal)
{
    bat->receiveCharge(1000.0f, 1000.0f);
    EXPECT_LE(bat->voltage(), 48.0f);
}
 
TEST_F(SimpleBatteryTest, ReceiveChargeFromEmptyToFull)
{
    auto empty = makeBattery("empty", 100.0f, 0.0f, 36.0f, 48.0f);
    EXPECT_TRUE(empty->isDepleted());
 
    // charge enough to fill
    empty->receiveCharge(100.0f, 1.0f);  // +100 Ah
    EXPECT_NEAR(empty->getStateOfCharge(), 1.0f, 1e-5f);
    EXPECT_NEAR(empty->voltage(), 48.0f, 1e-4f);
    EXPECT_FALSE(empty->isDepleted());
}
 
TEST_F(SimpleBatteryTest, ReceiveChargeZeroCurrent)
{
    auto half = makeBattery("half", 100.0f, 0.5f, 36.0f, 48.0f);
    const float soc_before = half->getStateOfCharge();
    half->receiveCharge(0.0f, 1.0f);
    EXPECT_FLOAT_EQ(half->getStateOfCharge(), soc_before);
}
 
TEST_F(SimpleBatteryTest, CanReceiveChargeIsTrue)
{
    EXPECT_TRUE(bat->canReceiveCharge());
}

// ──────────────────────────────────────────────────────────────
// PowerLevel transitions
// ──────────────────────────────────────────────────────────────

TEST_F(SimpleBatteryTest, PowerLevelNormalWhenFull)
{
    EXPECT_EQ(bat->powerLevel(), PowerLevel::NORMAL);
}
 
TEST_F(SimpleBatteryTest, PowerLevelDepleted)
{
    auto empty = makeBattery("empty", 100.0f, 0.0f, 36.0f, 48.0f);
    EXPECT_EQ(empty->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(empty->isDepleted());
}
 
TEST_F(SimpleBatteryTest, PowerLevelCritical)
{
    // voltage just above voltage_min (36V) : critical when ≤ 36*1.05 = 37.8V
    // SOC to reach ~37.0V: soc = (37.0 - 36.0) / (48.0 - 36.0) ≈ 0.083
    auto crit = makeBattery("crit", 100.0f, 0.04f, 36.0f, 48.0f);
    EXPECT_EQ(crit->powerLevel(), PowerLevel::CRITICAL);
    EXPECT_FALSE(crit->isDepleted());
}
 
TEST_F(SimpleBatteryTest, PowerLevelWarn)
{
    // voltage between 36*1.05=37.8V and 36*1.15=41.4V
    auto warn = makeBattery("warn", 100.0f, 0.25f, 36.0f, 48.0f);
    EXPECT_EQ(warn->powerLevel(), PowerLevel::WARN);
}
 
TEST_F(SimpleBatteryTest, PowerLevelTransitionsOnDischarge)
{
    // start NORMAL, drain to WARN then CRITICAL then DEPLETED
    auto bat2 = makeBattery("drain", 100.0f, 1.0f, 36.0f, 48.0f);
    EXPECT_EQ(bat2->powerLevel(), PowerLevel::NORMAL);
 
    // drain to WARN range: voltage between 37.8V and 41.4V
    bat2->receiveLoad(75.0f, 1.0f);  // remove 75 Ah -> 25 Ah remaining -> soc=0.25
    EXPECT_EQ(bat2->powerLevel(), PowerLevel::WARN);
 
    // drain to CRITICAL: soc ~0.04 -> voltage = 36.48V
    bat2->receiveLoad(21.0f, 1.0f);  // 4 Ah remaining
    EXPECT_EQ(bat2->powerLevel(), PowerLevel::CRITICAL);
 
    // drain to DEPLETED
    bat2->receiveLoad(10.0f, 1.0f);
    EXPECT_EQ(bat2->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(bat2->isDepleted());
}

// ──────────────────────────────────────────────────────────────
// SOC and voltage consistency tests
// ────────────────────────────────────────────────────────────── 

TEST_F(SimpleBatteryTest, VoltageConsistentWithSoc)
{
    // voltage = voltageMin + soc * (voltageNominal - voltageMin)
    auto b = makeBattery("b", 100.0f, 0.75f, 36.0f, 48.0f);
    const float expected_v = 36.0f + 0.75f * (48.0f - 36.0f);  // 45V
    EXPECT_NEAR(b->voltage(), expected_v, 1e-4f);
}
 
TEST_F(SimpleBatteryTest, SocConsistentAfterLoadAndCharge)
{
    auto b = makeBattery("b", 100.0f, 1.0f, 36.0f, 48.0f);
    b->receiveLoad(20.0f, 1.0f);   // -20 Ah → 80 Ah
    b->receiveCharge(10.0f, 1.0f); // +10 Ah → 90 Ah
    EXPECT_NEAR(b->getStateOfCharge(), 0.9f, 1e-5f);
    EXPECT_NEAR(b->voltage(), 36.0f + 0.9f * 12.0f, 1e-4f);
}
 
TEST_F(SimpleBatteryTest, AvailablePowerW)
{
    // availablePowerW = voltage * remainingAh
    // full battery: 48V * 100Ah = 4800W
    EXPECT_NEAR(bat->availablePowerW(), 48.0f * 100.0f, 1e-2f);
}
 
TEST_F(SimpleBatteryTest, AvailablePowerWDecreaseOnDischarge)
{
    const float before = bat->availablePowerW();
    bat->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(bat->availablePowerW(), before);
}

// ──────────────────────────────────────────────────────────────
// Multi-vessel isolation tests
// ──────────────────────────────────────────────────────────────

TEST_F(SimpleBatteryTest, MultipleBatteriesAreIndependent)
{
    auto vessel1_bat = makeBattery("vessel1_main", 100.0f, 1.0f, 36.0f, 48.0f);
    auto vessel2_bat = makeBattery("vessel2_main", 100.0f, 1.0f, 36.0f, 48.0f);
 
    // drain vessel1 battery
    vessel1_bat->receiveLoad(50.0f, 1.0f);
 
    // vessel2 battery should be unaffected
    EXPECT_NEAR(vessel2_bat->getStateOfCharge(), 1.0f, 1e-5f);
    EXPECT_NEAR(vessel2_bat->voltage(), 48.0f, 1e-4f);
    EXPECT_EQ(vessel2_bat->powerLevel(), PowerLevel::NORMAL);
}
 
TEST_F(SimpleBatteryTest, MultipleBatteriesDifferentCapacities)
{
    auto small = makeBattery("small", 50.0f,  1.0f, 36.0f, 48.0f);
    auto large = makeBattery("large", 200.0f, 1.0f, 36.0f, 48.0f);
 
    // same load drains small faster
    small->receiveLoad(25.0f, 1.0f);
    large->receiveLoad(25.0f, 1.0f);
 
    EXPECT_LT(small->getStateOfCharge(), large->getStateOfCharge());
}
 
TEST_F(SimpleBatteryTest, MultipleBatteriesDifferentInitialSoc)
{
    auto full  = makeBattery("full",  100.0f, 1.0f,  36.0f, 48.0f);
    auto half  = makeBattery("half",  100.0f, 0.5f,  36.0f, 48.0f);
    auto empty = makeBattery("empty", 100.0f, 0.0f,  36.0f, 48.0f);
 
    EXPECT_GT(full->voltage(), half->voltage());
    EXPECT_GT(half->voltage(), empty->voltage());
    EXPECT_FALSE(full->isDepleted());
    EXPECT_FALSE(half->isDepleted());
    EXPECT_TRUE(empty->isDepleted());
}
 
TEST_F(SimpleBatteryTest, MultipleBatteriesChargeIndependently)
{
    auto v1 = makeBattery("v1", 100.0f, 0.5f, 36.0f, 48.0f);
    auto v2 = makeBattery("v2", 100.0f, 0.5f, 36.0f, 48.0f);
 
    // only charge v1
    v1->receiveCharge(20.0f, 1.0f);
 
    EXPECT_GT(v1->getStateOfCharge(), v2->getStateOfCharge());
    EXPECT_NEAR(v2->getStateOfCharge(), 0.5f, 1e-5f);  // v2 unchanged
}
 
TEST_F(SimpleBatteryTest, ThreeVesselsSimultaneously)
{
    auto b1 = makeBattery("dtmb_0_battery",  100.0f, 1.0f,  36.0f, 48.0f);
    auto b2 = makeBattery("dtmb_1_battery",  100.0f, 0.75f, 36.0f, 48.0f);
    auto b3 = makeBattery("lrauv_0_battery", 50.0f,  0.5f,  24.0f, 36.0f);
 
    // simulate a few ticks
    for (int i = 0; i < 10; ++i) {
        b1->receiveLoad(5.0f, 0.1f);
        b2->receiveLoad(8.0f, 0.1f);
        b3->receiveCharge(3.0f, 0.1f);
    }
 
    // b1 and b2 drained, b3 charged
    EXPECT_LT(b1->getStateOfCharge(), 1.0f);
    EXPECT_LT(b2->getStateOfCharge(), 0.75f);
    EXPECT_GT(b3->getStateOfCharge(), 0.5f);
 
    // all voltages within valid range
    EXPECT_GE(b1->voltage(), 36.0f);
    EXPECT_GE(b2->voltage(), 36.0f);
    EXPECT_GE(b3->voltage(), 24.0f);
    EXPECT_LE(b3->voltage(), 36.0f);
}

// ──────────────────────────────────────────────────────────────
// Tests at extremes
// ──────────────────────────────────────────────────────────────

TEST_F(SimpleBatteryTest, VerySmallDt)
{
    const float soc_before = bat->getStateOfCharge();
    bat->receiveLoad(100.0f, 1e-9f);
    // should change slightly but not crash
    EXPECT_LE(bat->getStateOfCharge(), soc_before);
    EXPECT_GE(bat->voltage(), 36.0f);
}
 
TEST_F(SimpleBatteryTest, LargeCurrentSmallDt)
{
    // 1000A for 0.001s = 1 Ah drawn
    bat->receiveLoad(1000.0f, 0.001f);
    EXPECT_NEAR(bat->remainingAh(), 99.0f, 1e-3f);
}
 
TEST_F(SimpleBatteryTest, UpdateVoltageConsistency)
{
    // manually drain then call updateVoltage
    bat->receiveLoad(50.0f, 1.0f);
    const float v1 = bat->voltage();
    bat->updateVoltage();
    const float v2 = bat->voltage();
    EXPECT_FLOAT_EQ(v1, v2);  // same
}
} // namespace lotusim::gazebo

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}