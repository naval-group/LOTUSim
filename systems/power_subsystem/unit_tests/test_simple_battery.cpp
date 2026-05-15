#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <algorithm>

// test the logic directly without Gazebo or ROS2
// SDF and node construction are bypassed to test SimpleBattery and Battery

#include "power_subsystem/battery.hpp"
#include "power_subsystem/power_provider.hpp"

namespace lotusim::gazebo
{

// ──────────────────────────────────────────────────────────────
// Mirrors SimpleBattery but accepts plain constructor args so
// tests do not need sdf::ElementPtr or rclcpp::Node
// ──────────────────────────────────────────────────────────────

class TestBattery : public Battery
{
public:
    TestBattery(
        std::string name,
        float capacityAh,
        float initialSoc,
        float voltageMin,
        float voltageNominal)
        : Battery(std::move(name), nullptr, capacityAh, initialSoc, voltageMin)
        , m_voltage_nominal(voltageNominal)
    {
        m_voltage = voltageNominal;
    }

    void receiveLoad(float currentA, float dt) override
    {
        m_voltage -= (currentA * dt) / m_capacityAh;
        m_voltage  = std::max(m_voltage, m_voltageMin);
    }

    float voltage() const override { return m_voltage; }

    void receiveCharge(float currentA, float dt) override
    {
        m_voltage += (currentA * dt) / m_capacityAh;
        m_voltage  = std::min(m_voltage, m_voltage_nominal);
    }

    float getStateOfCharge() const override
    {
        if (m_voltage_nominal <= m_voltageMin) { return 0.0f; }
        const float soc = (m_voltage - m_voltageMin)
                        / (m_voltage_nominal - m_voltageMin);
        return std::clamp(soc, 0.0f, 1.0f);
    }

    // availablePowerW uses voltage() which is virtual 
    float availablePowerW() const override
    {
        return voltage() * getStateOfCharge() * m_capacityAh;
    }

private:
    float m_voltage{48.0f};
    float m_voltage_nominal{48.0f};
};

// ============================================================
// Helper — create a standard test battery
// voltage_min=36, voltage_nominal=48, capacity=100Ah
// PowerLevel thresholds:
//   NORMAL   : v > 41.4  (36 * 1.15)
//   WARN     : 37.8 < v <= 41.4  (36 * 1.05 .. 36 * 1.15)
//   CRITICAL : 36.0 < v <= 37.8
//   DEPLETED : v <= 36.0
// ============================================================

static std::unique_ptr<TestBattery> makeBattery(
    const std::string& name = "test_battery",
    float capacityAh       = 100.0f,
    float initialSoc       = 1.0f,
    float voltageMin       = 36.0f,
    float voltageNominal   = 48.0f)
{
    return std::make_unique<TestBattery>(
        name, capacityAh, initialSoc, voltageMin, voltageNominal);
}

// ============================================================
// Construction and initial state
// ============================================================

TEST(SimpleBatteryTest, InitialVoltageIsNominal)
{
    auto b = makeBattery();
    EXPECT_FLOAT_EQ(b->voltage(), 48.0f);
}

TEST(SimpleBatteryTest, InitialSocIsOne)
{
    auto b = makeBattery();
    EXPECT_NEAR(b->getStateOfCharge(), 1.0f, 1e-4f);
}

TEST(SimpleBatteryTest, InitialLevelIsNormal)
{
    auto b = makeBattery();
    EXPECT_EQ(b->powerLevel(), PowerLevel::NORMAL);
}

TEST(SimpleBatteryTest, InitialIsNotDepleted)
{
    auto b = makeBattery();
    EXPECT_FALSE(b->isDepleted());
}

TEST(SimpleBatteryTest, CanReceiveCharge)
{
    auto b = makeBattery();
    EXPECT_TRUE(b->canReceiveCharge());
}

TEST(SimpleBatteryTest, NameIsPreserved)
{
    auto b = makeBattery("vessel_a_battery");
    EXPECT_EQ(b->name(), "vessel_a_battery");
}

// ============================================================
// receiveLoad — voltage drain
// ============================================================

TEST(SimpleBatteryTest, ReceiveLoadDrainsVoltage)
{
    auto b = makeBattery();
    const float voltageBefore = b->voltage();
    b->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(b->voltage(), voltageBefore);
}

TEST(SimpleBatteryTest, ReceiveLoadFormula)
{
    // voltage -= (currentA * dt) / capacity_ah
    // 48 - (10 * 1.0) / 100 = 48 - 0.1 = 47.9
    auto b = makeBattery();
    b->receiveLoad(10.0f, 1.0f);
    EXPECT_NEAR(b->voltage(), 47.9f, 1e-4f);
}

TEST(SimpleBatteryTest, ReceiveLoadClampsToVoltageMin)
{
    auto b = makeBattery();
    // Apply enormous load to force below voltage_min
    b->receiveLoad(100000.0f, 100.0f);
    EXPECT_FLOAT_EQ(b->voltage(), 36.0f);
}

TEST(SimpleBatteryTest, ReceiveZeroLoadDoesNotDrain)
{
    auto b = makeBattery();
    const float before = b->voltage();
    b->receiveLoad(0.0f, 1.0f);
    EXPECT_FLOAT_EQ(b->voltage(), before);
}

TEST(SimpleBatteryTest, ReceiveLoadWithZeroDtDoesNotDrain)
{
    auto b = makeBattery();
    const float before = b->voltage();
    b->receiveLoad(100.0f, 0.0f);
    EXPECT_FLOAT_EQ(b->voltage(), before);
}

TEST(SimpleBatteryTest, LargerCurrentDrainsMoreVoltage)
{
    auto b1 = makeBattery("b1");
    auto b2 = makeBattery("b2");
    b1->receiveLoad(5.0f, 1.0f);
    b2->receiveLoad(20.0f, 1.0f);
    EXPECT_GT(b1->voltage(), b2->voltage());
}

TEST(SimpleBatteryTest, LargerDtDrainsMoreVoltage)
{
    auto b1 = makeBattery("b1");
    auto b2 = makeBattery("b2");
    b1->receiveLoad(10.0f, 0.1f);
    b2->receiveLoad(10.0f, 1.0f);
    EXPECT_GT(b1->voltage(), b2->voltage());
}

TEST(SimpleBatteryTest, SmallerCapacityDrainsVoltageMorePerTick)
{
    // Same load, same dt — smaller capacity drains more
    auto big   = makeBattery("big",   200.0f);
    auto small = makeBattery("small",  50.0f);
    big->receiveLoad(10.0f, 1.0f);
    small->receiveLoad(10.0f, 1.0f);
    EXPECT_GT(big->voltage(), small->voltage());
}

// ============================================================
// receiveCharge — voltage recovery
// ============================================================

TEST(SimpleBatteryTest, ReceiveChargeIncreasesVoltage)
{
    auto b = makeBattery();
    b->receiveLoad(10.0f, 10.0f); // drain first
    const float drained = b->voltage();
    b->receiveCharge(10.0f, 10.0f);
    EXPECT_GT(b->voltage(), drained);
}

TEST(SimpleBatteryTest, ReceiveChargeClampsToNominal)
{
    auto b = makeBattery();
    // Try to charge beyond nominal
    b->receiveCharge(100000.0f, 100.0f);
    EXPECT_FLOAT_EQ(b->voltage(), 48.0f);
}

TEST(SimpleBatteryTest, ReceiveChargeAfterDepletionRecovers)
{
    auto b = makeBattery();
    b->receiveLoad(100000.0f, 100.0f); // force to voltage_min
    EXPECT_FLOAT_EQ(b->voltage(), 36.0f);
    b->receiveCharge(10.0f, 10.0f);
    EXPECT_GT(b->voltage(), 36.0f);
}

// ============================================================
// getStateOfCharge
// ============================================================

TEST(SimpleBatteryTest, SocIsOneAtNominalVoltage)
{
    auto b = makeBattery();
    EXPECT_NEAR(b->getStateOfCharge(), 1.0f, 1e-4f);
}

TEST(SimpleBatteryTest, SocIsZeroAtVoltageMin)
{
    auto b = makeBattery();
    b->receiveLoad(100000.0f, 100.0f);
    EXPECT_NEAR(b->getStateOfCharge(), 0.0f, 1e-4f);
}

TEST(SimpleBatteryTest, SocIsHalfwayAtMidVoltage)
{
    // voltage_min=36, voltage_nominal=48 → midpoint=42
    // soc = (42 - 36) / (48 - 36) = 6/12 = 0.5
    auto b = makeBattery();
    // Drain to midpoint: 48 - (I * dt / 100) = 42 → I*dt = 600
    b->receiveLoad(600.0f, 1.0f);
    EXPECT_NEAR(b->getStateOfCharge(), 0.5f, 1e-3f);
}

TEST(SimpleBatteryTest, SocDecreasesAsVoltageDrops)
{
    auto b = makeBattery();
    const float soc1 = b->getStateOfCharge();
    b->receiveLoad(10.0f, 1.0f);
    const float soc2 = b->getStateOfCharge();
    b->receiveLoad(10.0f, 1.0f);
    const float soc3 = b->getStateOfCharge();
    EXPECT_GT(soc1, soc2);
    EXPECT_GT(soc2, soc3);
}

TEST(SimpleBatteryTest, SocIsClampedBetweenZeroAndOne)
{
    auto b = makeBattery();
    b->receiveLoad(100000.0f, 100.0f);
    EXPECT_GE(b->getStateOfCharge(), 0.0f);
    EXPECT_LE(b->getStateOfCharge(), 1.0f);

    b->receiveCharge(100000.0f, 100.0f);
    EXPECT_GE(b->getStateOfCharge(), 0.0f);
    EXPECT_LE(b->getStateOfCharge(), 1.0f);
}

TEST(SimpleBatteryTest, SocWithEqualMinAndNominalReturnsZero)
{
    // Edge case: voltage_min == voltage_nominal → avoid division by zero
    auto b = makeBattery("edge", 100.0f, 1.0f, 48.0f, 48.0f);
    EXPECT_FLOAT_EQ(b->getStateOfCharge(), 0.0f);
}

// ============================================================
// PowerLevel transitions
// ============================================================
// Thresholds for voltage_min=36:
//   NORMAL   : v > 41.4
//   WARN     : 37.8 < v <= 41.4
//   CRITICAL : 36.0 < v <= 37.8
//   DEPLETED : v <= 36.0

TEST(SimpleBatteryTest, PowerLevelNormalAtFullCharge)
{
    auto b = makeBattery();
    EXPECT_EQ(b->powerLevel(), PowerLevel::NORMAL);
}

TEST(SimpleBatteryTest, PowerLevelWarnJustBelowNormalThreshold)
{
    // Drain to just below 41.4V (36 * 1.15)
    // 48 - (I * 1.0 / 100) = 41.3 → I = 670
    auto b = makeBattery();
    b->receiveLoad(670.0f, 1.0f);
    EXPECT_NEAR(b->voltage(), 41.3f, 0.1f);
    EXPECT_EQ(b->powerLevel(), PowerLevel::WARN);
}

TEST(SimpleBatteryTest, PowerLevelCriticalJustBelowWarnThreshold)
{
    // Drain to just below 37.8V (36 * 1.05)
    // 48 - (I * 1.0 / 100) = 37.7 → I = 1030
    auto b = makeBattery();
    b->receiveLoad(1030.0f, 1.0f);
    EXPECT_NEAR(b->voltage(), 37.7f, 0.1f);
    EXPECT_EQ(b->powerLevel(), PowerLevel::CRITICAL);
}

TEST(SimpleBatteryTest, PowerLevelDepletedAtVoltageMin)
{
    auto b = makeBattery();
    b->receiveLoad(100000.0f, 100.0f);
    EXPECT_FLOAT_EQ(b->voltage(), 36.0f);
    EXPECT_EQ(b->powerLevel(), PowerLevel::DEPLETED);
}

TEST(SimpleBatteryTest, IsDepletedFalseWhenNormal)
{
    auto b = makeBattery();
    EXPECT_FALSE(b->isDepleted());
}

TEST(SimpleBatteryTest, IsDepletedTrueAtVoltageMin)
{
    auto b = makeBattery();
    b->receiveLoad(100000.0f, 100.0f);
    EXPECT_TRUE(b->isDepleted());
}

TEST(SimpleBatteryTest, PowerLevelTransitionsNormalToWarnToCriticalToDepleted)
{
    auto b = makeBattery();
    EXPECT_EQ(b->powerLevel(), PowerLevel::NORMAL);

    // Drain to WARN zone (between 37.8 and 41.4)
    while (b->powerLevel() == PowerLevel::NORMAL) {
        b->receiveLoad(10.0f, 1.0f);
    }
    EXPECT_EQ(b->powerLevel(), PowerLevel::WARN);

    // Drain to CRITICAL zone (between 36.0 and 37.8)
    while (b->powerLevel() == PowerLevel::WARN) {
        b->receiveLoad(10.0f, 1.0f);
    }
    EXPECT_EQ(b->powerLevel(), PowerLevel::CRITICAL);

    // Drain to DEPLETED
    while (b->powerLevel() == PowerLevel::CRITICAL) {
        b->receiveLoad(10.0f, 1.0f);
    }
    EXPECT_EQ(b->powerLevel(), PowerLevel::DEPLETED);
    EXPECT_TRUE(b->isDepleted());
}

// ============================================================
// availablePowerW
// ============================================================

TEST(SimpleBatteryTest, AvailablePowerWDecreasesWithVoltage)
{
    auto b = makeBattery();
    const float before = b->availablePowerW();
    b->receiveLoad(10.0f, 1.0f);
    EXPECT_LT(b->availablePowerW(), before);
}

TEST(SimpleBatteryTest, AvailablePowerWIsZeroWhenDepleted)
{
    auto b = makeBattery();
    b->receiveLoad(100000.0f, 100.0f);
    // At voltage_min, soc=0 → availablePowerW = voltage * 0 * capacity = 0
    EXPECT_NEAR(b->availablePowerW(), 0.0f, 1e-3f);
}

// ============================================================
// Multi-vessel isolation — key scalability requirement
// ============================================================

TEST(MultivesselTest, TwoBatteriesAreFullyIndependent)
{
    auto vessel_a = makeBattery("vessel_a_battery");
    auto vessel_b = makeBattery("vessel_b_battery");

    // Drain vessel A heavily
    vessel_a->receiveLoad(500.0f, 1.0f);

    // Vessel B should be unaffected
    EXPECT_FLOAT_EQ(vessel_b->voltage(), 48.0f);
    EXPECT_EQ(vessel_b->powerLevel(), PowerLevel::NORMAL);
    EXPECT_FALSE(vessel_b->isDepleted());
}

TEST(MultivesselTest, HundredVesselsAllIndependent)
{
    constexpr int kVesselCount = 100;
    std::vector<std::unique_ptr<TestBattery>> batteries;
    batteries.reserve(kVesselCount);

    for (int i = 0; i < kVesselCount; ++i) {
        batteries.push_back(makeBattery("vessel_" + std::to_string(i)));
    }

    // Drain only even-indexed vessels
    for (int i = 0; i < kVesselCount; i += 2) {
        batteries[i]->receiveLoad(100000.0f, 100.0f);
    }

    // Even vessels depleted, odd vessels untouched
    for (int i = 0; i < kVesselCount; ++i) {
        if (i % 2 == 0) {
            EXPECT_TRUE(batteries[i]->isDepleted())
                << "vessel_" << i << " should be depleted";
        } else {
            EXPECT_FALSE(batteries[i]->isDepleted())
                << "vessel_" << i << " should NOT be depleted";
            EXPECT_FLOAT_EQ(batteries[i]->voltage(), 48.0f)
                << "vessel_" << i << " voltage should be unchanged";
        }
    }
}

TEST(MultivesselTest, VesselsWithDifferentConfigsAreIndependent)
{
    // Vessel A: small battery, low voltage_min
    auto vessel_a = makeBattery("vessel_a", 50.0f, 1.0f, 24.0f, 36.0f);
    // Vessel B: large battery, high voltage_min
    auto vessel_b = makeBattery("vessel_b", 200.0f, 1.0f, 42.0f, 54.0f);

    vessel_a->receiveLoad(10.0f, 1.0f);
    vessel_b->receiveLoad(10.0f, 1.0f);

    // Different capacities → different voltage drops
    // vessel_a: 36 - (10 * 1 / 50) = 35.8
    // vessel_b: 54 - (10 * 1 / 200) = 53.95
    EXPECT_NEAR(vessel_a->voltage(), 35.8f, 1e-3f);
    EXPECT_NEAR(vessel_b->voltage(), 53.95f, 1e-3f);
}

TEST(MultivesselTest, BatteryPriorityOrderPreservedAcrossVessels)
{
    // Each vessel has two batteries — primary should deplete first
    auto primary_a   = makeBattery("vessel_a_primary",   100.0f);
    auto secondary_a = makeBattery("vessel_a_secondary", 100.0f);
    auto primary_b   = makeBattery("vessel_b_primary",   100.0f);
    auto secondary_b = makeBattery("vessel_b_secondary", 100.0f);

    // Only primaries receive load
    primary_a->receiveLoad(100000.0f, 100.0f);
    primary_b->receiveLoad(100000.0f, 100.0f);

    EXPECT_TRUE(primary_a->isDepleted());
    EXPECT_TRUE(primary_b->isDepleted());
    EXPECT_FALSE(secondary_a->isDepleted());
    EXPECT_FALSE(secondary_b->isDepleted());
}

// ============================================================
// Charge / discharge symmetry
// ============================================================

TEST(SimpleBatteryTest, FullDrainThenFullChargeRestoresVoltage)
{
    auto b = makeBattery();
    b->receiveLoad(100000.0f, 100.0f);
    EXPECT_FLOAT_EQ(b->voltage(), 36.0f);

    b->receiveCharge(100000.0f, 100.0f);
    EXPECT_FLOAT_EQ(b->voltage(), 48.0f);
}

TEST(SimpleBatteryTest, SmallChargeAfterSmallDrainRestoresVoltage)
{
    auto b = makeBattery();
    b->receiveLoad(10.0f, 1.0f);   // drain: 48 - 0.1 = 47.9
    b->receiveCharge(10.0f, 1.0f); // charge: 47.9 + 0.1 = 48.0
    EXPECT_NEAR(b->voltage(), 48.0f, 1e-4f);
}

TEST(SimpleBatteryTest, PowerLevelRecoverFromWarnToNormal)
{
    auto b = makeBattery();
    b->receiveLoad(670.0f, 1.0f); // drain to WARN
    EXPECT_EQ(b->powerLevel(), PowerLevel::WARN);

    b->receiveCharge(100000.0f, 100.0f); // charge back to full
    EXPECT_EQ(b->powerLevel(), PowerLevel::NORMAL);
}

} // namespace lotusim::gazebo

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}