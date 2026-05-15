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
// SDF and node construction are bypassed to test RpmGenerator and Generator

#include "power_subsystem/generator.hpp"
#include "power_subsystem/power_provider.hpp"
#include "power_subsystem/fuel_properties.hpp"

namespace lotusim::gazebo
{
// ──────────────────────────────────────────────────────────────
// Mirrors RpmGenerator but accepts plain constructor args 
// ──────────────────────────────────────────────────────────────
class TestRPMGenerator : public Generator
{
public:
    TestRPMGenerator(
        const std::string& name,
        float fuel_level_start,
        float fuel_capacity,
        float rated_output_w,
        float efficiency,
        float voltage_nominal,
        const std::string& fuel_type,
        float rated_rpm)
        : Generator(name, nullptr, fuel_level_start, 
            fuel_capacity, rated_output_w, efficiency, 
            voltage_nominal, fuel_type)
        , m_rated_rpm(rated_rpm)
        , m_current_rpm(0.0f)
        {}
    
    // ── test control ─────────────────────────────────────────────────
    void setRpm(float rpm) { m_current_rpm = rpm; }
    float currentRpm() const { return m_current_rpm; }
    float fuelLevel() const { return m_fuel_level; }
    float fuelCapacity() const { return m_fuel_capacity; }
    float ratedRpm() const { return m_rated_rpm; }

    // ──────────────────────────────────────────────────────────────
    // Generator interface -> same logic as RpmGenerator
    // ──────────────────────────────────────────────────────────────

    float availablePowerW() const override
    {
        if (isDepleted() || m_current_rpm <= 0.0f) { return 0.0f; }
        const float ratio = m_current_rpm / m_rated_rpm;
        return m_rated_output_W * ratio * ratio * ratio;
    }
 
    void receiveLoad(float /*currentA*/, float dt) override
    {
        if (m_current_rpm <= 0.0f || isDepleted()) { return; }
 
        const float ratio = m_current_rpm / m_rated_rpm;
        const float power_w = m_rated_output_W * ratio * ratio * ratio;
        const float density = getFuelEnergyDensity(m_fuel_type);
        const float consumed = (power_w * dt) / (m_efficiency * density);
 
        m_fuel_level = std::max(0.0f, m_fuel_level - consumed);
    }

private:
    float m_rated_rpm{1000.0f};
    float m_current_rpm{0.0f};
};

// ──────────────────────────────────────────────────────────────
// Helper 
// ──────────────────────────────────────────────────────────────

static std::unique_ptr<TestRPMGenerator> makeGenerator(
    const std::string& name = "test_rpm_generator",
    float fuel_level_start = 100.0f,
    float fuel_capacity = 200.0f,
    float rated_output_w = 5000.0f,
    float efficiency = 0.35f,
    float voltage_nominal = 48.0f,
    const std::string& fuel_type = "diesel",
    float rated_rpm = 1000.0f)
{
    return std::make_unique<TestRPMGenerator>(
        name, fuel_level_start, fuel_capacity, rated_output_w, efficiency, 
            voltage_nominal, fuel_type, rated_rpm);
}

// ──────────────────────────────────────────────────────────────
// Fixture
// ──────────────────────────────────────────────────────────────
 
class RpmGeneratorTest : public ::testing::Test
{
protected:
    std::unique_ptr<TestRPMGenerator> gen = makeGenerator();
};

// ──────────────────────────────────────────────────────────────
// Construction tests
// ──────────────────────────────────────────────────────────────

TEST_F(RpmGeneratorTest, InitialRpmIsZero)
{
    EXPECT_FLOAT_EQ(gen->currentRpm(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, InitialFuelLevelMatchesStart)
{
    EXPECT_FLOAT_EQ(gen->fuelLevel(), 100.0f);
}
 
TEST_F(RpmGeneratorTest, InitialFuelRatio)
{
    EXPECT_NEAR(gen->fuelRatio(), 0.5f, 1e-5f);
}
 
TEST_F(RpmGeneratorTest, InitialPowerLevelWarn)
{
    EXPECT_EQ(gen->powerLevel(), PowerLevel::NORMAL);
}
 
TEST_F(RpmGeneratorTest, NameIsPreserved)
{
    EXPECT_EQ(gen->name(), "test_rpm_generator");
}
 
TEST_F(RpmGeneratorTest, CannotReceiveCharge)
{
    EXPECT_FALSE(gen->canReceiveCharge());
}
 
TEST_F(RpmGeneratorTest, VoltageNominalWhenNotDepleted)
{
    EXPECT_FLOAT_EQ(gen->voltage(), 48.0f);
}
 
TEST_F(RpmGeneratorTest, VoltageZeroWhenDepleted)
{
    // drain all fuel
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    EXPECT_FLOAT_EQ(empty->voltage(), 0.0f);
}
 
// ──────────────────────────────────────────────────────────────
// availablePowerW -> cubic law tests
// ────────────────────────────────────────────────────────────── 

TEST_F(RpmGeneratorTest, ZeroRpmZeroPower)
{
    gen->setRpm(0.0f);
    EXPECT_FLOAT_EQ(gen->availablePowerW(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, RatedRpmRatedPower)
{
    gen->setRpm(1000.0f);  // rated_rpm
    EXPECT_NEAR(gen->availablePowerW(), 5000.0f, 1e-2f);
}
 
TEST_F(RpmGeneratorTest, HalfRpmEighthPower)
{
    gen->setRpm(500.0f);
    EXPECT_NEAR(gen->availablePowerW(), 5000.0f * 0.125f, 1e-2f);
}
 
TEST_F(RpmGeneratorTest, DoubleRpmEightTimesPower)
{
    gen->setRpm(2000.0f);
    EXPECT_NEAR(gen->availablePowerW(), 40000.0f, 1e-1f);
}
 
TEST_F(RpmGeneratorTest, QuarterRpmSixteenthPower)
{
    gen->setRpm(250.0f);
    EXPECT_NEAR(gen->availablePowerW(), 5000.0f * 0.015625f, 1e-3f);
}
 
TEST_F(RpmGeneratorTest, NegativeRpmZeroPower)
{
    gen->setRpm(-100.0f);
    EXPECT_FLOAT_EQ(gen->availablePowerW(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, ZeroPowerWhenDepleted)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    empty->setRpm(1000.0f);
    EXPECT_FLOAT_EQ(empty->availablePowerW(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, CubicLawContinuous)
{
    for (float ratio : {0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.0f}) {
        gen->setRpm(ratio * 1000.0f);
        const float expected = 5000.0f * ratio * ratio * ratio;
        EXPECT_NEAR(gen->availablePowerW(), expected, 1e-2f)
            << "Failed at rpm ratio=" << ratio;
    }
}
 
// ──────────────────────────────────────────────────────────────
// receiveLoad tests
// ──────────────────────────────────────────────────────────────

TEST_F(RpmGeneratorTest, ZeroRpmNoFuelConsumed)
{
    gen->setRpm(0.0f);
    const float before = gen->fuelLevel();
    gen->receiveLoad(0.0f, 1.0f);
    EXPECT_FLOAT_EQ(gen->fuelLevel(), before);
}
 
TEST_F(RpmGeneratorTest, DepletedNoFuelConsumed)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    empty->setRpm(1000.0f);
    empty->receiveLoad(0.0f, 1.0f);
    EXPECT_FLOAT_EQ(empty->fuelLevel(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, FuelDecreasesOnLoad)
{
    gen->setRpm(1000.0f);
    const float before = gen->fuelLevel();
    gen->receiveLoad(0.0f, 1.0f);
    EXPECT_LT(gen->fuelLevel(), before);
}
 
TEST_F(RpmGeneratorTest, ZeroDtNoFuelConsumed)
{
    gen->setRpm(1000.0f);
    const float before = gen->fuelLevel();
    gen->receiveLoad(0.0f, 0.0f);
    EXPECT_FLOAT_EQ(gen->fuelLevel(), before);
}
 
TEST_F(RpmGeneratorTest, FuelConsumptionProportionalToDt)
{
    auto g1 = makeGenerator();
    auto g2 = makeGenerator();
    g1->setRpm(1000.0f);
    g2->setRpm(1000.0f);
 
    // one tick of dt=1.0 vs two ticks of dt=0.5
    g1->receiveLoad(0.0f, 1.0f);
    g2->receiveLoad(0.0f, 0.5f);
    g2->receiveLoad(0.0f, 0.5f);
 
    EXPECT_NEAR(g1->fuelLevel(), g2->fuelLevel(), 1e-5f);
}
 
TEST_F(RpmGeneratorTest, HigherRpmMoreFuelConsumed)
{
    auto slow = makeGenerator();
    auto fast = makeGenerator();
    slow->setRpm(500.0f);
    fast->setRpm(1000.0f);
 
    slow->receiveLoad(0.0f, 1.0f);
    fast->receiveLoad(0.0f, 1.0f);
 
    EXPECT_LT(fast->fuelLevel(), slow->fuelLevel());
}
 
TEST_F(RpmGeneratorTest, FuelDoesNotGoBelowZero)
{
    gen->setRpm(1000.0f);
    // run for a very long time
    for (int i = 0; i < 10000; ++i) {
        gen->receiveLoad(0.0f, 100.0f);
    }
    EXPECT_GE(gen->fuelLevel(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, CurrentAIsIgnored)
{
    // receiveLoad ignores currentA -> power is RPM-driven
    auto g1 = makeGenerator();
    auto g2 = makeGenerator();
    g1->setRpm(1000.0f);
    g2->setRpm(1000.0f);
 
    g1->receiveLoad(0.0f,    1.0f);
    g2->receiveLoad(1000.0f, 1.0f);  // large current, same result
 
    EXPECT_NEAR(g1->fuelLevel(), g2->fuelLevel(), 1e-5f);
}
 
TEST_F(RpmGeneratorTest, FuelConsumptionMatchesMathModel)
{
    const float rpm       = 1000.0f;
    const float dt        = 1.0f;
    const float ratio     = rpm / 1000.0f;
    const float power_w   = 5000.0f * ratio * ratio * ratio;
    const float density   = getFuelEnergyDensity("diesel");
    const float expected  = (power_w * dt) / (0.35f * density);
 
    gen->setRpm(rpm);
    gen->receiveLoad(0.0f, dt);
 
    EXPECT_NEAR(gen->fuelLevel(), 100.0f - expected, 1e-6f);
}
 
// ──────────────────────────────────────────────────────────────
// PowerLevel threshold tests
// ──────────────────────────────────────────────────────────────
 
TEST_F(RpmGeneratorTest, PowerLevelNormal)
{
    EXPECT_EQ(gen->powerLevel(), PowerLevel::NORMAL);
}
 
TEST_F(RpmGeneratorTest, PowerLevelWarn)
{
    auto g = makeGenerator("w", 30.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::WARN);
}
 
TEST_F(RpmGeneratorTest, PowerLevelCritical)
{
    auto g = makeGenerator("c", 15.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::CRITICAL);
}
 
TEST_F(RpmGeneratorTest, PowerLevelDepleted)
{
    auto g = makeGenerator("d", 5.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(g->isDepleted());
}
 
TEST_F(RpmGeneratorTest, PowerLevelTransitionsOnFuelDrain)
{
    auto g = makeGenerator("drain", 200.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::NORMAL);
    g->setRpm(1000.0f);
 
    while (g->fuelRatio() > 0.25f) {
        g->receiveLoad(0.0f, 10.0f);
    }
    EXPECT_EQ(g->powerLevel(), PowerLevel::WARN);
 
    // drain until critical
    while (g->fuelRatio() > 0.10f) {
        g->receiveLoad(0.0f, 10.0f);
    }
    EXPECT_EQ(g->powerLevel(), PowerLevel::CRITICAL);
 
    // drain until depleted
    while (g->fuelRatio() > 0.05f) {
        g->receiveLoad(0.0f, 10.0f);
    }
    EXPECT_EQ(g->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(g->isDepleted());
}
 
TEST_F(RpmGeneratorTest, IsDepletedFalseWhenFuelled)
{
    EXPECT_FALSE(gen->isDepleted());
}
 
TEST_F(RpmGeneratorTest, IsDepletedTrueAtZeroFuel)
{
    auto g = makeGenerator("d", 0.0f, 200.0f);
    EXPECT_TRUE(g->isDepleted());
}
 
// ──────────────────────────────────────────────────────────────
// getStateOfCharge tests
// ──────────────────────────────────────────────────────────────

TEST_F(RpmGeneratorTest, StateOfChargeMatchesFuelRatio)
{
    EXPECT_NEAR(gen->getStateOfCharge(), gen->fuelRatio(), 1e-5f);
}
 
TEST_F(RpmGeneratorTest, StateOfChargeFullTank)
{
    auto full = makeGenerator("f", 200.0f, 200.0f);
    EXPECT_NEAR(full->getStateOfCharge(), 1.0f, 1e-5f);
}
 
TEST_F(RpmGeneratorTest, StateOfChargeEmptyTank)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    EXPECT_NEAR(empty->getStateOfCharge(), 0.0f, 1e-5f);
}
 
TEST_F(RpmGeneratorTest, StateOfChargeZeroCapacity)
{
    TestRPMGenerator zero("z", 0.0f, 0.0f, 5000.0f, 0.35f, 48.0f, "diesel", 1000.0f);
    EXPECT_FLOAT_EQ(zero.getStateOfCharge(), 0.0f);
}
 
// ──────────────────────────────────────────────────────────────
// surplusChargingCurrent tests
// ──────────────────────────────────────────────────────────────

TEST_F(RpmGeneratorTest, SurplusZeroWhenDepleted)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    empty->setRpm(1000.0f);
    EXPECT_FLOAT_EQ(empty->surplusChargingCurrent(10.0f), 0.0f);
}
 
TEST_F(RpmGeneratorTest, SurplusZeroWhenRpmZero)
{
    gen->setRpm(0.0f);
    EXPECT_FLOAT_EQ(gen->surplusChargingCurrent(0.0f), 0.0f);
}
 
TEST_F(RpmGeneratorTest, SurplusPositiveWhenSupplyExceedsDemand)
{
    gen->setRpm(1000.0f);
    const float surplus = gen->surplusChargingCurrent(10.0f);
    EXPECT_GT(surplus, 0.0f);
}
 
TEST_F(RpmGeneratorTest, SurplusZeroWhenDemandExceedsSupply)
{
    // very low RPM -> low power, high demand
    gen->setRpm(10.0f);  
    const float surplus = gen->surplusChargingCurrent(1000.0f);
    EXPECT_FLOAT_EQ(surplus, 0.0f);
}
 
TEST_F(RpmGeneratorTest, SurplusCalculationCorrect)
{
    gen->setRpm(1000.0f);
    const float surplus = gen->surplusChargingCurrent(10.0f);
    EXPECT_NEAR(surplus, (5000.0f - 10.0f * 48.0f) / 48.0f, 1e-2f);
}
 
// ──────────────────────────────────────────────────────────────
// Fuel type tests
// ──────────────────────────────────────────────────────────────
 
TEST_F(RpmGeneratorTest, DifferentFuelTypesConsumeDifferently)
{
    // hydrogen has lower energy density than diesel -> more consumed
    auto diesel   = makeGenerator("diesel_gen",   100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "diesel",   1000.0f);
    auto hydrogen = makeGenerator("hydrogen_gen", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "hydrogen", 1000.0f);
 
    diesel->setRpm(1000.0f);
    hydrogen->setRpm(1000.0f); 
    diesel->receiveLoad(0.0f, 1.0f);
    hydrogen->receiveLoad(0.0f, 1.0f);
 
    EXPECT_LT(hydrogen->fuelLevel(), diesel->fuelLevel());
}
 
TEST_F(RpmGeneratorTest, AllSupportedFuelTypesWork)
{
    for (const auto& fuel : {"diesel", "gasoline", "lng", "hydrogen", "methanol"}) {
        auto g = makeGenerator("g", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, fuel, 1000.0f);
        g->setRpm(1000.0f);
        EXPECT_NO_THROW(g->receiveLoad(0.0f, 1.0f))
            << "Failed for fuel type: " << fuel;
        EXPECT_GE(g->fuelLevel(), 0.0f)
            << "Negative fuel for type: " << fuel;
    }
}
 
TEST_F(RpmGeneratorTest, UnknownFuelTypeThrows)
{
    auto bad = makeGenerator("wrong", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "fusion_power", 1000.0f);
    bad->setRpm(1000.0f);
    EXPECT_THROW(bad->receiveLoad(0.0f, 1.0f), std::invalid_argument);
}
 
// ──────────────────────────────────────────────────────────────
// Multiple vessel tests
// ──────────────────────────────────────────────────────────────

TEST_F(RpmGeneratorTest, TwoVesselsAreIndependent)
{
    auto v1 = makeGenerator("dtmb_0_gen");
    auto v2 = makeGenerator("dtmb_1_gen");
 
    v1->setRpm(1000.0f);
    v2->setRpm(0.0f);  // engine off
    v1->receiveLoad(0.0f, 1.0f);
    v2->receiveLoad(0.0f, 1.0f);
 
    EXPECT_LT(v1->fuelLevel(), 100.0f);  // v1 consumed fuel
    EXPECT_FLOAT_EQ(v2->fuelLevel(), 100.0f);  // v2 unchanged
}
 
TEST_F(RpmGeneratorTest, ThreeVesselsDifferentRpm)
{
    auto v1 = makeGenerator("v1", 200.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "diesel", 1000.0f);
    auto v2 = makeGenerator("v2", 200.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "diesel", 1000.0f);
    auto v3 = makeGenerator("v3", 200.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "diesel", 1000.0f);
 
    v1->setRpm(1000.0f);  // full power
    v2->setRpm(500.0f);   // half RPM 
    v3->setRpm(0.0f);     // off
 
    for (int i = 0; i < 10; ++i) {
        v1->receiveLoad(0.0f, 1.0f);
        v2->receiveLoad(0.0f, 1.0f);
        v3->receiveLoad(0.0f, 1.0f);
    }
 
    // v1 consumed most, v2 less, v3 none
    EXPECT_LT(v1->fuelLevel(), v2->fuelLevel());
    EXPECT_LT(v2->fuelLevel(), v3->fuelLevel());
    EXPECT_FLOAT_EQ(v3->fuelLevel(), 200.0f);
}
 
TEST_F(RpmGeneratorTest, IndependentDepletionPerVessel)
{
    auto v1 = makeGenerator("v1", 5.0f, 200.0f); 
    auto v2 = makeGenerator("v2", 200.0f, 200.0f);
 
    v1->setRpm(1000.0f);
    v2->setRpm(1000.0f);

    // drain v1 to depletion
    for (int i = 0; i < 10000; ++i) {
        v1->receiveLoad(0.0f, 100.0f);
    }
 
    EXPECT_TRUE(v1->isDepleted());
    EXPECT_FALSE(v2->isDepleted());  // v2 unaffected
}
 
// ──────────────────────────────────────────────────────────────
// availablePowerW after fuel drain
// ──────────────────────────────────────────────────────────────

TEST_F(RpmGeneratorTest, AvailablePowerZeroAfterDepletion)
{
    gen->setRpm(1000.0f);
    for (int i = 0; i < 10000; ++i) {
        gen->receiveLoad(0.0f, 100.0f);
    }
    EXPECT_FLOAT_EQ(gen->availablePowerW(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, AvailablePowerW_GetStateOfCharge_Consistent)
{
    gen->setRpm(1000.0f);
    EXPECT_NEAR(gen->getStateOfCharge(), gen->fuelRatio(), 1e-5f);
    gen->receiveLoad(0.0f, 1.0f);
    EXPECT_NEAR(gen->getStateOfCharge(), gen->fuelRatio(), 1e-5f);
}
 
// ──────────────────────────────────────────────────────────────
// Tests at extremes
// ──────────────────────────────────────────────────────────────

TEST_F(RpmGeneratorTest, VerySmallDt)
{
    gen->setRpm(1000.0f);
    const float before = gen->fuelLevel();
    gen->receiveLoad(0.0f, 1e-9f);
    EXPECT_LE(gen->fuelLevel(), before);
    EXPECT_GE(gen->fuelLevel(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, VeryHighRpm)
{
    gen->setRpm(1e6f);
    // should not crash, power output will be huge but fuel clamps at 0
    EXPECT_NO_THROW(gen->receiveLoad(0.0f, 1.0f));
    EXPECT_GE(gen->fuelLevel(), 0.0f);
}
 
TEST_F(RpmGeneratorTest, RpmExactlyAtRatedProducesExactRatedPower)
{
    gen->setRpm(gen->ratedRpm());
    EXPECT_NEAR(gen->availablePowerW(), 5000.0f, 1e-3f);
}
 
TEST_F(RpmGeneratorTest, FuelRatioConsistentAfterMultipleTicks)
{
    gen->setRpm(750.0f);
    for (int i = 0; i < 100; ++i) {
        gen->receiveLoad(0.0f, 0.1f);
        const float ratio = gen->fuelRatio();
        EXPECT_GE(ratio, 0.0f);
        EXPECT_LE(ratio, 1.0f);
    }
}
 
} // namespace lotusim::gazebo
 
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}