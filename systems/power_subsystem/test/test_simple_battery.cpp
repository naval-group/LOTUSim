/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/power_provider/simple_battery.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <memory>
#include <string>

using lotusim::gazebo::SimpleBattery;

namespace {

/**
 * @brief The <lotusim_power> element a model.sdf would give the battery.
 */
sdf::ElementPtr batterySdf(float capacity_ah, float initial_soc)
{
    auto element = std::make_shared<sdf::Element>();
    element->SetName("lotusim_power");
    auto add = [&](const std::string& name, float value) {
        auto child = std::make_shared<sdf::Element>();
        child->SetName(name);
        child->AddValue("float", std::to_string(value), true);
        element->InsertElement(child);
    };
    add("capacity_ah", capacity_ah);
    add("initial_soc", initial_soc);
    add("voltage_min", 36.0f);
    add("voltage_nominal", 48.0f);
    return element;
}

std::unique_ptr<SimpleBattery> makeBattery(float capacity_ah, float initial_soc)
{
    auto logger = std::make_shared<spdlog::logger>(
        "test_simple_battery",
        std::make_shared<spdlog::sinks::null_sink_mt>());
    // The battery only stores the node; it never touches it.
    return std::make_unique<SimpleBattery>(
        "battery",
        "vessel",
        batterySdf(capacity_ah, initial_soc),
        nullptr,
        logger);
}

}  // namespace

TEST(SimpleBattery, OneHourAtTenAmpsDrawsTenAmpHours)
{
    auto battery = makeBattery(100.0f, 1.0f);
    battery->receiveLoad(10.0f, 3600.0f);
    EXPECT_NEAR(battery->getStateOfCharge(), 0.9f, 1e-5f);
}

TEST(SimpleBattery, IntegratesSmallStepsInAmpHours)
{
    // 10 A for one hour in 0.2 s steps, as the plugin calls it.
    auto battery = makeBattery(100.0f, 1.0f);
    for (int i = 0; i < 18000; ++i) {
        battery->receiveLoad(10.0f, 0.2f);
    }
    EXPECT_NEAR(battery->getStateOfCharge(), 0.9f, 1e-3f);
}

TEST(SimpleBattery, NegativeCurrentCharges)
{
    auto battery = makeBattery(100.0f, 0.5f);
    battery->receiveLoad(-20.0f, 3600.0f);
    EXPECT_NEAR(battery->getStateOfCharge(), 0.7f, 1e-5f);
}

TEST(SimpleBattery, ClampsBetweenEmptyAndFull)
{
    auto battery = makeBattery(100.0f, 0.5f);
    battery->receiveLoad(-1000.0f, 3600.0f);
    EXPECT_FLOAT_EQ(battery->getStateOfCharge(), 1.0f);
    battery->receiveLoad(1000.0f, 3600.0f);
    EXPECT_FLOAT_EQ(battery->getStateOfCharge(), 0.0f);
    EXPECT_TRUE(battery->isDepleted());
}

TEST(SimpleBattery, VoltageFollowsStateOfCharge)
{
    auto battery = makeBattery(100.0f, 1.0f);
    battery->receiveLoad(50.0f, 3600.0f);
    // halfway between voltage_min 36 V and voltage_nominal 48 V
    EXPECT_NEAR(battery->voltage(), 42.0f, 1e-4f);
}

// Same reason as physics_engine_interface's tests: gz-sim's dependency chain
// can crash during static teardown, so exit without running it.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
