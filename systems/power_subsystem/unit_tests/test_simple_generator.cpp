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
// SDF and node construction are bypassed to test SimpleGenerator and Generator

#include "power_subsystem/generator.hpp"
#include "power_subsystem/power_provider.hpp"
#include "power_subsystem/fuel_properties.hpp"

namespace lotusim::gazebo
{
// ──────────────────────────────────────────────────────────────
// Mirrors SimpleGenerator but accepts plain constructor args 
// ──────────────────────────────────────────────────────────────
class TestSimpleGenerator : public Generator
{
public:
    TestSimpleGenerator(
        const std::string& name,
        float fuel_level_start,
        float fuel_capacity,
        float rated_output_w,
        float efficiency,
        float voltage_nominal,
        std::string fuel_type)
        : Generator(name, nullptr, fuel_level_start, 
            fuel_capacity, rated_output_w, efficiency, 
            voltage_nominal, fuel_type){}
    
    // ──────────────────────────────────────────────────────────────
    // Generator interface -> same logic as SimpleGenerator
    // ──────────────────────────────────────────────────────────────

    void receiveLoad(float currentA, float dt) override {
        if (isDepleted()){
            return;
        }

        const float powerW = currentA * m_voltage_nominal;
        const float density = getFuelEnergyDensity(m_fuel_type);
        const float consumed = (powerW * dt) / (m_efficiency * density);

        m_fuel_level = std::max(0.0f, m_fuel_level - consumed);
    }
    float fuelLevel() const { return m_fuel_level; }
    float fuelCapacity() const { return m_fuel_capacity; }
    float ratedOutputW() const { return m_rated_output_W; }
    float efficiency() const { return m_efficiency; }
    float voltageNominal() const { return m_voltage_nominal; }
};

// ──────────────────────────────────────────────────────────────
// Helper 
// ──────────────────────────────────────────────────────────────

static std::unique_ptr<TestSimpleGenerator> makeGenerator(
    const std::string& name = "test_generator",
    float fuel_level_start = 100.0f,
    float fuel_capacity = 200.0f,
    float rated_output_w = 5000.0f,
    float efficiency = 0.35f,
    float voltage_nominal = 48.0f,
    const std::string& fuel = "diesel")
{
    return std::make_unique<TestSimpleGenerator>(
        name, fuel_level_start, fuel_capacity, rated_output_w, 
        efficiency, voltage_nominal, fuel);
}

// ──────────────────────────────────────────────────────────────
// Fixture
// ──────────────────────────────────────────────────────────────
 
class SimpleGeneratorTest : public ::testing::Test
{
protected:
    std::unique_ptr<TestSimpleGenerator> gen = makeGenerator();
};

// ──────────────────────────────────────────────────────────────
// Construction tests
// ──────────────────────────────────────────────────────────────
TEST_F(SimpleGeneratorTest, InitialFuelLevelMatchesStart)
{
    EXPECT_FLOAT_EQ(gen->fuelLevel(), 100.0f);
}
 
TEST_F(SimpleGeneratorTest, InitialFuelCapacity)
{
    EXPECT_FLOAT_EQ(gen->fuelCapacity(), 200.0f);
}
 
TEST_F(SimpleGeneratorTest, InitialFuelRatio)
{
    EXPECT_NEAR(gen->fuelRatio(), 0.5f, 1e-5f);
}
 
TEST_F(SimpleGeneratorTest, InitialPowerLevelNormal)
{
    // fuelRatio = 0.5 is NORMAL (> 0.25)
    EXPECT_EQ(gen->powerLevel(), PowerLevel::NORMAL);
}
 
TEST_F(SimpleGeneratorTest, NameIsPreserved)
{
    EXPECT_EQ(gen->name(), "test_generator");
}
 
TEST_F(SimpleGeneratorTest, CannotReceiveCharge)
{
    EXPECT_FALSE(gen->canReceiveCharge());
}
 
TEST_F(SimpleGeneratorTest, VoltageNominalWhenNotDepleted)
{
    EXPECT_FLOAT_EQ(gen->voltage(), 48.0f);
}
 
TEST_F(SimpleGeneratorTest, VoltageZeroWhenDepleted)
{
    // fuelRatio = 0/200 = 0 is DEPLETED (≤ 0.05)
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    EXPECT_FLOAT_EQ(empty->voltage(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, FullTankInitialState)
{
    auto full = makeGenerator("full", 200.0f, 200.0f);
    EXPECT_NEAR(full->fuelRatio(), 1.0f, 1e-5f);
    EXPECT_EQ(full->powerLevel(), PowerLevel::NORMAL);
    EXPECT_FALSE(full->isDepleted());
}
 
TEST_F(SimpleGeneratorTest, EmptyTankInitialState)
{
    auto empty = makeGenerator("empty", 0.0f, 200.0f);
    EXPECT_NEAR(empty->fuelRatio(), 0.0f, 1e-5f);
    EXPECT_EQ(empty->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(empty->isDepleted());
}

// ──────────────────────────────────────────────────────────────
// receiveLoad tests
// ──────────────────────────────────────────────────────────────
TEST_F(SimpleGeneratorTest, ZeroCurrentNoFuelConsumed)
{
    const float before = gen->fuelLevel();
    gen->receiveLoad(0.0f, 1.0f);
    EXPECT_FLOAT_EQ(gen->fuelLevel(), before);
}
 
TEST_F(SimpleGeneratorTest, ZeroDtNoFuelConsumed)
{
    const float before = gen->fuelLevel();
    gen->receiveLoad(100.0f, 0.0f);
    EXPECT_FLOAT_EQ(gen->fuelLevel(), before);
}
 
TEST_F(SimpleGeneratorTest, DepletedNoFuelConsumed)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    empty->receiveLoad(100.0f, 1.0f);
    EXPECT_FLOAT_EQ(empty->fuelLevel(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, FuelDecreasesOnLoad)
{
    const float before = gen->fuelLevel();
    gen->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(gen->fuelLevel(), before);
}
 
TEST_F(SimpleGeneratorTest, FuelDoesNotGoBelowZero)
{
    gen->receiveLoad(1e6f, 1e6f);
    EXPECT_GE(gen->fuelLevel(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, FuelConsumptionProportionalToCurrent)
{
    auto g1 = makeGenerator();
    auto g2 = makeGenerator();
 
    g1->receiveLoad(10.0f, 1.0f);
    g2->receiveLoad(20.0f, 1.0f);
 
    // g2 draws twice the current -> twice the fuel consumed
    const float consumed1 = 100.0f - g1->fuelLevel();
    const float consumed2 = 100.0f - g2->fuelLevel();
    EXPECT_NEAR(consumed2, 2.0f * consumed1, 1e-5f);
}
 
TEST_F(SimpleGeneratorTest, FuelConsumptionProportionalToDt)
{
    auto g1 = makeGenerator();
    auto g2 = makeGenerator();
 
    // one tick of dt=1.0 vs two ticks of dt=0.5
    g1->receiveLoad(10.0f, 1.0f);
    g2->receiveLoad(10.0f, 0.5f);
    g2->receiveLoad(10.0f, 0.5f);
 
    EXPECT_NEAR(g1->fuelLevel(), g2->fuelLevel(), 1e-5f);
}
 
TEST_F(SimpleGeneratorTest, FuelConsumptionMatchesMathModel)
{
    // verify formulas power_w and consumed
    const float currentA  = 10.0f;
    const float dt = 1.0f;
    const float power_w = currentA * 48.0f;
    const float density = getFuelEnergyDensity("diesel");
    const float expected = (power_w * dt) / (0.35f * density);
 
    gen->receiveLoad(currentA, dt);
    EXPECT_NEAR(gen->fuelLevel(), 100.0f - expected, 1e-6f);
}
 
TEST_F(SimpleGeneratorTest, HigherEfficiencyLessFuelConsumed)
{
    auto low_eff  = makeGenerator("low", 100.0f, 200.0f, 5000.0f, 0.20f);
    auto high_eff = makeGenerator("high", 100.0f, 200.0f, 5000.0f, 0.60f);
 
    low_eff->receiveLoad(10.0f, 1.0f);
    high_eff->receiveLoad(10.0f, 1.0f);
 
    EXPECT_GT(high_eff->fuelLevel(), low_eff->fuelLevel());
}
 
TEST_F(SimpleGeneratorTest, HigherVoltageMoreFuelConsumed)
{
    auto low_v  = makeGenerator("low_v", 100.0f, 200.0f, 5000.0f, 0.35f, 24.0f);
    auto high_v = makeGenerator("high_v", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f);
 
    low_v->receiveLoad(10.0f, 1.0f);
    high_v->receiveLoad(10.0f, 1.0f);
 
    EXPECT_GT(low_v->fuelLevel(), high_v->fuelLevel());
}
 
TEST_F(SimpleGeneratorTest, LargeCurrentSmallDt)
{
    // 1000A for 0.001s
    const float power_w = 1000.0f * 48.0f;
    const float dt = 0.001f;
    const float density = getFuelEnergyDensity("diesel");
    const float expected = (power_w * dt) / (0.35f * density);
 
    gen->receiveLoad(1000.0f, 0.001f);
    EXPECT_NEAR(gen->fuelLevel(), 100.0f - expected, 1e-6f);
}
 
TEST_F(SimpleGeneratorTest, FuelRatioConsistentAfterMultipleTicks)
{
    for (int i = 0; i < 100; ++i) {
        gen->receiveLoad(5.0f, 0.1f);
        const float ratio = gen->fuelRatio();
        EXPECT_GE(ratio, 0.0f);
        EXPECT_LE(ratio, 1.0f);
    }
}

// ──────────────────────────────────────────────────────────────
// PowerLevel threshold tests
// ──────────────────────────────────────────────────────────────
 TEST_F(SimpleGeneratorTest, PowerLevelNormal)
{
    EXPECT_EQ(gen->powerLevel(), PowerLevel::NORMAL);
}
 
TEST_F(SimpleGeneratorTest, PowerLevelWarn)
{
    auto g = makeGenerator("w", 30.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::WARN);
}
 
TEST_F(SimpleGeneratorTest, PowerLevelCritical)
{
    auto g = makeGenerator("c", 15.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::CRITICAL);
}
 
TEST_F(SimpleGeneratorTest, PowerLevelDepleted)
{
    auto g = makeGenerator("d", 5.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(g->isDepleted());
}
 
TEST_F(SimpleGeneratorTest, PowerLevelBoundaryNormalToWarn)
{
    auto g = makeGenerator("b", 50.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::WARN);
}
 
TEST_F(SimpleGeneratorTest, PowerLevelBoundaryWarnToCritical)
{
    auto g = makeGenerator("b", 20.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::CRITICAL);
}
 
TEST_F(SimpleGeneratorTest, PowerLevelBoundaryCriticalToDepleted)
{
    auto g = makeGenerator("b", 10.0f, 200.0f); 
    EXPECT_EQ(g->powerLevel(), PowerLevel::DEPLETED);
}
 
TEST_F(SimpleGeneratorTest, PowerLevelTransitionsOnDrain)
{
    auto g = makeGenerator("drain", 200.0f, 200.0f);
    EXPECT_EQ(g->powerLevel(), PowerLevel::NORMAL);
 
    while (g->fuelRatio() > 0.25f) { g->receiveLoad(100.0f, 1.0f); }
    EXPECT_EQ(g->powerLevel(), PowerLevel::WARN);
 
    while (g->fuelRatio() > 0.10f) { g->receiveLoad(100.0f, 1.0f); }
    EXPECT_EQ(g->powerLevel(), PowerLevel::CRITICAL);
 
    while (g->fuelRatio() > 0.05f) { g->receiveLoad(100.0f, 1.0f); }
    EXPECT_EQ(g->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(g->isDepleted());
}

// ──────────────────────────────────────────────────────────────
// getStateOfCharge tests
// ──────────────────────────────────────────────────────────────
TEST_F(SimpleGeneratorTest, StateOfChargeMatchesFuelRatio)
{
    EXPECT_NEAR(gen->getStateOfCharge(), gen->fuelRatio(), 1e-5f);
}
 
TEST_F(SimpleGeneratorTest, StateOfChargeFullTank)
{
    auto full = makeGenerator("f", 200.0f, 200.0f);
    EXPECT_NEAR(full->getStateOfCharge(), 1.0f, 1e-5f);
}
 
TEST_F(SimpleGeneratorTest, StateOfChargeEmptyTank)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    EXPECT_NEAR(empty->getStateOfCharge(), 0.0f, 1e-5f);
}
 
TEST_F(SimpleGeneratorTest, StateOfChargeZeroCapacity)
{
    TestSimpleGenerator zero("z", 0.0f, 0.0f, 5000.0f, 0.35f, 48.0f, "diesel");
    EXPECT_FLOAT_EQ(zero.getStateOfCharge(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, StateOfChargeDecreasesOnLoad)
{
    const float before = gen->getStateOfCharge();
    gen->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(gen->getStateOfCharge(), before);
}

// ──────────────────────────────────────────────────────────────
// surplusChargingCurrent tests
// ────────────────────────────────────────────────────────────── 
TEST_F(SimpleGeneratorTest, SurplusZeroWhenDepleted)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    EXPECT_FLOAT_EQ(empty->surplusChargingCurrent(10.0f), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, SurplusPositiveWhenSupplyExceedsDemand)
{
    EXPECT_GT(gen->surplusChargingCurrent(10.0f), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, SurplusZeroWhenDemandExceedsSupply)
{
    EXPECT_FLOAT_EQ(gen->surplusChargingCurrent(1000.0f), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, SurplusCalculationCorrect)
{
    const float available = gen->availablePowerW();
    const float demand_w = 10.0f * 48.0f;
    const float expected = (available - demand_w) / 48.0f;
    EXPECT_NEAR(gen->surplusChargingCurrent(10.0f), expected, 1e-3f);
}
 
TEST_F(SimpleGeneratorTest, SurplusZeroAtZeroDemandFullyDepleted)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    EXPECT_FLOAT_EQ(empty->surplusChargingCurrent(0.0f), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, SurplusDecreasesAsFuelDrains)
{
    const float surplus_before = gen->surplusChargingCurrent(5.0f);
    gen->receiveLoad(10.0f, 1.0f);
    const float surplus_after  = gen->surplusChargingCurrent(5.0f);
    EXPECT_LT(surplus_after, surplus_before);
}

// ──────────────────────────────────────────────────────────────
// Fuel types tests
// ──────────────────────────────────────────────────────────────
TEST_F(SimpleGeneratorTest, AllSupportedFuelTypesWork)
{
    for (const auto& fuel : {"diesel", "gasoline", "lng", "hydrogen", "methanol"}) {
        auto g = makeGenerator("g", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, fuel);
        EXPECT_NO_THROW(g->receiveLoad(10.0f, 1.0f))
            << "Failed for fuel type: " << fuel;
        EXPECT_GE(g->fuelLevel(), 0.0f)
            << "Negative fuel for type: " << fuel;
    }
}
 
TEST_F(SimpleGeneratorTest, UnknownFuelTypeThrows)
{
    auto bad = makeGenerator("bad", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "fusion_power");
    EXPECT_THROW(bad->receiveLoad(10.0f, 1.0f), std::invalid_argument);
}
 
TEST_F(SimpleGeneratorTest, DifferentFuelTypesConsumeDifferently)
{
    auto diesel = makeGenerator("d", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "diesel");
    auto hydrogen = makeGenerator("h", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "hydrogen");
 
    diesel->receiveLoad(10.0f, 1.0f);
    hydrogen->receiveLoad(10.0f, 1.0f);
 
    EXPECT_LT(hydrogen->fuelLevel(), diesel->fuelLevel());
}
 
TEST_F(SimpleGeneratorTest, MethanolConsumesMoreThanDiesel)
{
    auto diesel = makeGenerator("d", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "diesel");
    auto methanol = makeGenerator("m", 100.0f, 200.0f, 5000.0f, 0.35f, 48.0f, "methanol");
 
    diesel->receiveLoad(10.0f, 1.0f);
    methanol->receiveLoad(10.0f, 1.0f);
 
    EXPECT_LT(methanol->fuelLevel(), diesel->fuelLevel());
}

// ──────────────────────────────────────────────────────────────
// multiple vessel isolation tests
// ──────────────────────────────────────────────────────────────
TEST_F(SimpleGeneratorTest, TwoVesselsAreIndependent)
{
    auto v1 = makeGenerator("dtmb_0_gen");
    auto v2 = makeGenerator("dtmb_1_gen");
 
    v1->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(v1->fuelLevel(), 100.0f);
    EXPECT_FLOAT_EQ(v2->fuelLevel(), 100.0f);
}
 
TEST_F(SimpleGeneratorTest, TwoVesselsDifferentEfficiency)
{
    auto v1 = makeGenerator("v1", 100.0f, 200.0f, 5000.0f, 0.20f);  
    auto v2 = makeGenerator("v2", 100.0f, 200.0f, 5000.0f, 0.60f); 
 
    v1->receiveLoad(10.0f, 1.0f);
    v2->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(v1->fuelLevel(), v2->fuelLevel());
}
 
TEST_F(SimpleGeneratorTest, ThreeVesselsSimultaneously)
{
    auto v1 = makeGenerator("v1", 200.0f, 200.0f);
    auto v2 = makeGenerator("v2", 200.0f, 200.0f);
    auto v3 = makeGenerator("v3", 200.0f, 200.0f);
 
    for (int i = 0; i < 10; ++i) {
        v1->receiveLoad(10.0f, 1.0f);  
        v2->receiveLoad(50.0f, 1.0f);   
        v3->receiveLoad(0.0f, 1.0f); 
    }
    EXPECT_LT(v2->fuelLevel(), v1->fuelLevel());  // v2 used more
    EXPECT_LT(v1->fuelLevel(), v3->fuelLevel());  // v1 used more than v3
    EXPECT_FLOAT_EQ(v3->fuelLevel(), 200.0f);     // v3 unchanged
}
 
TEST_F(SimpleGeneratorTest, IndependentDepletionPerVessel)
{
    auto v1 = makeGenerator("v1", 5.0f, 200.0f);
    auto v2 = makeGenerator("v2", 200.0f, 200.0f);
 
    for (int i = 0; i < 10000; ++i) {
        v1->receiveLoad(100.0f, 100.0f);
    }
    EXPECT_TRUE(v1->isDepleted());
    EXPECT_FALSE(v2->isDepleted());  // v2 unaffected
}

// ──────────────────────────────────────────────────────────────
// Extreme cases tests
// ──────────────────────────────────────────────────────────────
TEST_F(SimpleGeneratorTest, VerySmallDt)
{
    const float before = gen->fuelLevel();
    gen->receiveLoad(100.0f, 1e-9f);
    EXPECT_LE(gen->fuelLevel(), before);
    EXPECT_GE(gen->fuelLevel(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, VeryLargeCurrent)
{
    EXPECT_NO_THROW(gen->receiveLoad(1e9f, 1.0f));
    EXPECT_GE(gen->fuelLevel(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, ZeroCapacityGenerator)
{
    TestSimpleGenerator zero("z", 0.0f, 0.0f, 5000.0f, 0.35f, 48.0f, "diesel");
    EXPECT_FLOAT_EQ(zero.getStateOfCharge(), 0.0f);
    EXPECT_TRUE(zero.isDepleted());
    EXPECT_FLOAT_EQ(zero.availablePowerW(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, ReceiveLoadIdempotentWhenDepleted)
{
    auto empty = makeGenerator("e", 0.0f, 200.0f);
    empty->receiveLoad(100.0f, 1.0f);
    empty->receiveLoad(100.0f, 1.0f);
    EXPECT_FLOAT_EQ(empty->fuelLevel(), 0.0f);
}
 
TEST_F(SimpleGeneratorTest, FuelLevelNeverExceedsCapacity)
{
    gen->receiveLoad(0.0f, 1.0f);
    EXPECT_LE(gen->fuelLevel(), gen->fuelCapacity());
}
} //namespace lotusim::gazebo

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}