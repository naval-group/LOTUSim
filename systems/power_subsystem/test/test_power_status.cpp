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

#include <cstdlib>
#include <map>
#include <memory>
#include <string>

#include "power_subsystem/default_platform_power_manager.hpp"

using lotusim::gazebo::DefaultPlatformPowerManager;

namespace {

using Fields = std::map<std::string, std::string>;

void addChild(const sdf::ElementPtr& parent, const std::string& name, const std::string& value)
{
    auto child = std::make_shared<sdf::Element>();
    child->SetName(name);
    child->AddValue("string", value, true);
    child->SetParent(parent);
    parent->InsertElement(child);
}

/// A <model> with one <lotusim_power> per entry, in priority order.
sdf::ElementPtr modelSdf(const std::vector<Fields>& providers)
{
    auto model = std::make_shared<sdf::Element>();
    model->SetName("model");
    for (const auto& fields : providers) {
        auto power = std::make_shared<sdf::Element>();
        power->SetName("lotusim_power");
        for (const auto& [name, value] : fields) {
            addChild(power, name, value);
        }
        // GetNextElement walks the parent's children, so the parent is set.
        power->SetParent(model);
        model->InsertElement(power);
    }
    return model;
}

const Fields kGenerator{
    {"name", "gen"}, {"type", "simple_generator"}, {"fuel_type", "diesel"},
    {"fuel_capacity", "10"}, {"fuel_level_start", "10"}};

Fields battery(const std::string& name, const std::string& soc)
{
    return {{"name", name}, {"type", "simple_battery"}, {"capacity_ah", "100"},
            {"initial_soc", soc}};
}

std::string activeProvider(const std::vector<Fields>& providers)
{
    static auto node = rclcpp::Node::make_shared("test_power_status");
    static int vessel = 0;
    DefaultPlatformPowerManager manager(
        1, "vessel" + std::to_string(vessel++), node, modelSdf(providers));
    return manager.activeProviderName();
}

}  // namespace

TEST(PowerStatus, GeneratorOnlyVessel)
{
    // Used to index m_batteries[-1] in the status timer, and crash.
    EXPECT_EQ(activeProvider({kGenerator}), "gen");
}

TEST(PowerStatus, EveryBatteryEmptyAtStartup)
{
    EXPECT_EQ(activeProvider({battery("main", "0"), kGenerator}), "gen");
}

TEST(PowerStatus, ActiveBattery)
{
    EXPECT_EQ(activeProvider({battery("main", "0.5"), kGenerator}), "main");
}

TEST(PowerStatus, NoProvider)
{
    EXPECT_EQ(activeProvider({battery("main", "0")}), "");
}

// gz-sim's dependency chain can crash during static teardown (see
// physics_engine_interface's tests), so exit without running it.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
